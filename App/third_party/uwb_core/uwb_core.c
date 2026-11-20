#include "uwb_internal.h"
#include <float.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(DBL_MANT_DIG == 53 && DBL_MAX_EXP == 1024, "E1 needs IEEE binary64");

UwbConfig uwb_research_config(void)
{
    return (UwbConfig){
        .windows_ms = {200, 400}, .min_samples = 3, .quality_floor = 230,
        .center_deg = 20, .cluster_width_deg = 40, .consensus = .55, .adc_weight = .5,
        .iir_alpha = {.18, .35, .5}, .adaptive_center_deg = 30, .adaptive_direction_deg = 50
    };
}

static bool in_range(double value, double lo, double hi)
{
    return value >= lo && value <= hi;
}

bool uwb_core_init(UwbCore *core, const UwbConfig *config)
{
    if (!core || !config) return false;
    const UwbConfig p = *config;
    if (p.windows_ms[0] < 50 || p.windows_ms[1] > 2000 || p.windows_ms[0] >= p.windows_ms[1] ||
        !p.min_samples || p.min_samples > UWB_CAPACITY || p.quality_floor > 255 ||
        !in_range(p.center_deg, 1, 179) || !in_range(p.cluster_width_deg, 1, 180) ||
        !in_range(p.consensus, .5, 1) || !in_range(p.adc_weight, 0, 1) ||
        !in_range(p.adaptive_center_deg, 1, 179) || !in_range(p.adaptive_direction_deg, p.adaptive_center_deg, 179))
        return false;
    for (unsigned i = 0; i < 3; ++i) {
        if (!in_range(p.iir_alpha[i], .01, 1)) return false;
        for (unsigned j = 0; j < i; ++j)
            if (p.iir_alpha[i] == p.iir_alpha[j]) return false;
    }
    core->config = p;
    uwb_core_reset(core);
    return true;
}

static void reset_bundle(UwbCore *core)
{
    core->bundle.head = core->bundle.count = 0;
    core->fw_head = core->fw_count = 0;
    core->firmware_direction = UWB_CENTER;
    core->has_bundle_valid = false;
    memset(core->iir, 0, sizeof core->iir);
    memset(core->cluster_history, 0, sizeof core->cluster_history);
}

static void reset_estimators(UwbCore *core)
{
    reset_bundle(core);
    uwb_adaptive_reset(core);
    core->v1_mode = (UwbModeState){.mode = UWB_LOST};
    core->v1_direction = UWB_UNKNOWN;
    core->good.head = core->good.count = 0;
    core->v2_recovering = false;
    core->v2_anchor.ref.direction = UWB_UNKNOWN;
    core->v2_reversal.direction = UWB_UNKNOWN;
    core->v3_reference.direction = UWB_UNKNOWN;
}

void uwb_core_reset(UwbCore *core)
{
    const UwbConfig config = core->config;
    memset(core, 0, sizeof *core);
    core->config = config;
    reset_estimators(core);
}

static void baseline(UwbCore *core, UwbPoint point)
{
    UwbDecision *out = core->decisions;
    const UwbConfig *p = &core->config;
    const bool usable = isfinite(point.phase);
    for (unsigned i = UWB_RAW; i <= UWB_IIR_2; ++i) out[i] = uwb_empty_decision();
    if (!usable) return;

    uwb_set_phase(&out[UWB_RAW], point.phase);
    out[UWB_RAW].direction = uwb_direction(point.phase, p);
    core->has_bundle_valid = true;
    core->bundle_valid_tick = point.tick;
    core->fw5[core->fw_head] = point;
    core->fw_head = (uint8_t)((core->fw_head + 1) % 5);
    if (core->fw_count < 5) ++core->fw_count;
    if (core->fw_count == 5) {
        UwbWorkspace *work = &core->work;
        work->count = 5;
        for (unsigned i = 0; i < 5; ++i) {
            work->points[i] = &core->fw5[(core->fw_head + i) % 5];
            work->weights[i] = 1;
        }
        const double phase = uwb_circular(work, p, UWB_UNKNOWN).phase;
        uwb_set_phase(&out[UWB_FW5], phase);
        if (!isfinite(phase)) core->firmware_direction = UWB_UNKNOWN;
        else if (phase < -p->center_deg) core->firmware_direction = UWB_LEFT;
        else if (phase > p->center_deg) core->firmware_direction = UWB_RIGHT;
        else if (fabs(phase) < p->center_deg * .75) core->firmware_direction = UWB_CENTER;
    }
    out[UWB_FW5].direction = core->firmware_direction;
    for (unsigned i = UWB_V3_ONLY ? 2 : 0; i < 3; ++i) {
        UwbDecision *d = &out[UWB_IIR_0 + i];
        const double phase = uwb_iir(&core->iir[i], point.phase, p->iir_alpha[i]);
        uwb_set_phase(d, phase);
        d->direction = uwb_direction(phase, p);
    }
}

static unsigned selected_window(const UwbConfig *p)
{
    return abs((int)p->windows_ms[0] - 200) <= abs((int)p->windows_ms[1] - 200) ? 0 : 1;
}

bool uwb_core_method_enabled(const UwbCore *core, UwbMethod method)
{
    if (!core || (unsigned)method >= UWB_METHOD_COUNT) return false;
#if UWB_V3_ONLY
    const unsigned offset = selected_window(&core->config) * 4;
    return method == UWB_RAW || method == UWB_FW5 || method == UWB_IIR_2 ||
           (unsigned)method == UWB_WEIGHTED_VOTE_0 + offset ||
           (unsigned)method == UWB_WEIGHTED_CLUSTER_0 + offset ||
           method == UWB_ADAPTIVE || method == UWB_V1 || method == UWB_V3;
#else
    return true;
#endif
}

static void analyze_windows(UwbCore *core, UwbStep *step)
{
    const UwbConfig *p = &core->config;
    const unsigned chosen = selected_window(p);
    const uint32_t tick = step->obs->tick_ms;
    UwbWorkspace *work = &core->work;
    const unsigned first = UWB_V3_ONLY ? chosen : 0;
    const unsigned end = UWB_V3_ONLY ? chosen + 1 : 2;
    for (unsigned i = first; i < end; ++i) {
        const uint16_t ms = p->windows_ms[i];
        const unsigned start = UWB_VOTE_0 + i * 4;
        for (unsigned weighted = UWB_V3_ONLY ? 1 : 0; weighted < 2; ++weighted) {
            uwb_view(work, &core->bundle, tick, ms, p, weighted != 0);
            UwbDecision vote = uwb_vote(work, p);
            UwbCluster cluster = uwb_cluster(work, p);
            UwbPhaseHistory *previous = &core->cluster_history[i][weighted];
            if (cluster.decision.direction != UWB_UNKNOWN) {
                const double deviation = cluster.decision.deviation_deg;
                if (previous->ready && tick > previous->tick && tick - previous->tick <= UWB_RESET_GAP)
                    cluster.slope = uwb_wrap(deviation - previous->phase) * 1000.0 / (tick - previous->tick);
                *previous = (UwbPhaseHistory){tick, deviation, true};
            } else previous->ready = false;
            if (!isfinite(step->raw_phase)) {
                vote.direction = cluster.decision.direction = UWB_UNKNOWN;
                vote.reasons = cluster.decision.reasons = step->obs->center_valid ? UWB_REASON_INVALID : UWB_REASON_CENTER;
                cluster.slope = NAN;
            }
            vote.window_ms = cluster.decision.window_ms = ms;
            core->decisions[start + weighted] = vote;
            core->decisions[start + 2 + weighted] = cluster.decision;
            if (i != chosen || !weighted) continue;

            step->vote = &core->decisions[start + weighted];
            step->cluster = cluster;
            step->window_ms = ms;
            step->accepted_count = work->count;
            step->accepted = uwb_circular(work, p, UWB_UNKNOWN);
            step->winners = vote.direction == UWB_UNKNOWN ? (UwbCircular){NAN, NAN, 0} :
                uwb_circular(work, p, vote.direction);
            for (uint16_t j = 0; j < work->count; ++j) {
                const UwbDirection dir = uwb_direction(work->points[j]->phase, p);
                if (dir == UWB_LEFT) ++step->left_count;
                if (dir == UWB_RIGHT) ++step->right_count;
                if (dir == vote.direction) ++step->winner_count;
            }
            for (uint16_t j = 0; j < core->bundle.count; ++j)
                if (tick - uwb_ring_at(&core->bundle, j)->tick <= ms) ++step->observed_count;
        }
    }
}

static bool update_recovery(UwbRecovery *good, uint32_t tick, bool valid, uint32_t gap)
{
    if (!valid || gap > UWB_LOST_MS) good->head = good->count = 0;
    if (!valid) return false;
    while (good->count && tick - good->ticks[good->head] > 200) {
        good->head = (uint16_t)((good->head + 1) % UWB_CAPACITY);
        --good->count;
    }
    good->ticks[(good->head + good->count++) % UWB_CAPACITY] = tick;
    return good->count >= 3 && tick - good->ticks[good->head] >= 60;
}

UwbStatus uwb_core_observe(UwbCore *core, const UwbObservation *observation)
{
    if (!core || !observation) return UWB_BAD_INPUT;
    const UwbObservation input = *observation;
    if ((input.center_valid && (input.center_mdeg < -180000 || input.center_mdeg > 180000)) ||
        (core->has_observation && input.arrival_ms < core->last.arrival_ms)) return UWB_BAD_INPUT;
    const bool seen = core->has_observation;
    const UwbObservation *previous = &core->last;
    if (seen && input.sequence == previous->sequence && input.tick_ms == previous->tick_ms) return UWB_DUPLICATE;
    const bool restart = seen && (input.sequence < previous->sequence || input.tick_ms < previous->tick_ms);
    const bool center_changed = seen && (input.center_valid != previous->center_valid ||
        (input.center_valid && input.center_mdeg != previous->center_mdeg));
    const uint32_t gap = seen && !restart && !center_changed ? input.tick_ms - previous->tick_ms : 0;
    const bool bundle_reset = core->has_bundle_valid && input.tick_ms - core->bundle_valid_tick > UWB_RESET_GAP;
    const uint32_t tick = input.tick_ms;
    const uint16_t window = core->config.windows_ms[1];
    const bool usable = input.pdoa_valid && input.center_valid;
    const bool valid = usable && !(input.reason_flags & UWB_RF_HARD_MASK) && input.sts_q > core->config.quality_floor;

    if (!restart && !center_changed) {
        if (!bundle_reset && core->bundle.count == UWB_CAPACITY &&
            tick - uwb_ring_at(&core->bundle, 0)->tick <= window) return UWB_CAPACITY_EXCEEDED;
        if (valid && gap <= UWB_LOST_MS && core->good.count == UWB_CAPACITY &&
            tick - core->good.ticks[core->good.head] <= 200) return UWB_CAPACITY_EXCEEDED;
    }
    if (restart || center_changed) reset_estimators(core);
    else if (bundle_reset) reset_bundle(core);
    if (restart) core->has_range = core->has_prior_range = false;
    if (gap > UWB_RESET_GAP) uwb_adaptive_reset(core);
    const uint32_t missing = seen && !restart && !center_changed && gap <= UWB_RESET_GAP &&
        input.sequence > previous->sequence ? input.sequence - previous->sequence - 1 : 0;
    core->last = input;
    core->has_observation = true;
    core->has_pair = false;

    const double phase = usable ? uwb_wrap(input.raw_pdoa / 2048.0 * 180.0 / UWB_PI - input.center_mdeg / 1000.0) : NAN;
    const UwbPoint point = {phase, tick, 0, input.sts_q, input.reason_flags};
    UwbStep step = {.obs = &input, .cfg = &core->config, .raw_phase = phase, .gap = gap, .valid = valid};
    if (core->has_range && tick >= core->range.tick_ms && tick - core->range.tick_ms <= 1500) {
        step.range_available = true;
        step.range_age = tick - core->range.tick_ms;
        step.range_delta = core->has_prior_range ? (double)core->range.distance_mm - core->prior_range.distance_mm : 0;
        step.range_jump = fabs(step.range_delta) > 500;
    }
    step.recovered = update_recovery(&core->good, tick, valid, gap);
    uwb_ring_prune(&core->bundle, tick, window);
    uwb_ring_push(&core->bundle, point);
    baseline(core, point);
    analyze_windows(core, &step);
    uwb_adaptive_step(core, &step, point, missing);
    uwb_v1_step(core, &step);
#if !UWB_V3_ONLY
    uwb_v2_step(core, &step);
#endif
    uwb_v3_step(core, &step);
    for (unsigned i = 0; i < UWB_METHOD_COUNT; ++i) {
        UwbDecision *d = &core->decisions[i];
        if (d->direction != UWB_UNKNOWN && d->origin == UWB_CURRENT_NONE) d->origin = UWB_CURRENT_OBSERVATION;
    }
    return UWB_ACCEPTED;
}

UwbStatus uwb_core_observe_range(UwbCore *core, const UwbRangeObservation *range)
{
    if (!core || !range || (range->valid && range->distance_mm < 0)) return UWB_BAD_INPUT;
    if (!core->has_observation || range->sequence != core->last.sequence || range->tick_ms < core->last.tick_ms ||
        range->tick_ms - core->last.tick_ms > 1500) return UWB_UNPAIRED;
    if (core->has_pair) return UWB_DUPLICATE;
    core->paired_range = *range;
    core->has_pair = true;
    if (range->valid) {
        core->has_prior_range = core->has_range && range->tick_ms - core->range.tick_ms <= 2000;
        core->prior_range = core->range;
        core->range = *range;
        core->has_range = true;
    }
    return UWB_ACCEPTED;
}

const UwbDecision *uwb_core_decision(const UwbCore *core, UwbMethod method)
{
    return uwb_core_method_enabled(core, method) && core->has_observation ? &core->decisions[method] : NULL;
}

static void invalidate(UwbDecision *d, UwbMode mode, uint32_t reason)
{
    d->direction = UWB_UNKNOWN;
    uwb_set_phase(d, NAN);
    d->mode = mode;
    d->origin = UWB_CURRENT_NONE;
    d->boundary = false;
    d->reasons = reason;
}

bool uwb_core_read(const UwbCore *core, UwbMethod method, uint64_t now, UwbTrackingResult *result)
{
    if (!result || !uwb_core_method_enabled(core, method) || (core->has_observation && now < core->last.arrival_ms))
        return false;
    *result = (UwbTrackingResult){.decision = uwb_empty_decision(), .distance_mm = -1};
    UwbDecision *d = &result->decision;
    if (!core->has_observation) {
        invalidate(d, UWB_RECOVERY, UWB_REASON_WAITING);
        return true;
    }
    *d = core->decisions[method];
    const UwbObservation *last = &core->last;
    const uint64_t age = now - last->arrival_ms;
    result->has_observation = true;
    result->last_arrival_ms = last->arrival_ms;
    result->observation_tick_ms = last->tick_ms;
    result->sequence = last->sequence;
    result->center_valid = last->center_valid;
    result->tag_lost = age > UWB_LOST_MS;
    if (!last->center_valid) invalidate(d, UWB_LOST, UWB_REASON_CENTER);
    else if (result->tag_lost) invalidate(d, UWB_LOST, UWB_REASON_GAP);
    else if (!last->pdoa_valid || (last->reason_flags & UWB_RF_HARD_MASK) || last->sts_q <= core->config.quality_floor)
        invalidate(d, UWB_LOST, !last->pdoa_valid ? UWB_REASON_INVALID : UWB_REASON_QUALITY);
    else if (d->origin == UWB_CONTINUITY_HOLD) {
        d->held_ms = (uint16_t)(d->held_ms + age);
        if (d->held_ms > d->grace_ms) invalidate(d, UWB_MODE_NONE, UWB_REASON_HOLD_EXPIRED);
    }
    if (core->has_pair && core->paired_range.valid && !result->tag_lost) {
        result->distance_valid = true;
        result->distance_mm = core->paired_range.distance_mm;
    }
    result->direction_valid = d->direction != UWB_UNKNOWN;
    result->degraded = d->mode == UWB_DEGRADED || d->mode == UWB_DEGRADED_TRACKING || d->mode == UWB_SOFT_HOLD;
    return true;
}
