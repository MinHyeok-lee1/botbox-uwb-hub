#include "uwb_internal.h"

void uwb_adaptive_reset(UwbCore *core)
{
    core->adaptive.head = core->adaptive.count = 0;
    core->adaptive_mode = (UwbModeState){.mode = UWB_HOLD};
    core->adaptive_direction = UWB_UNKNOWN;
    core->adaptive_iir.ready = false;
    core->adaptive_previous.ready = false;
    core->adaptive_offset.ready = false;
    core->adaptive_trend = 0;
}

static unsigned adaptive_rank(UwbMode mode)
{
    return (unsigned)(mode - UWB_FAST);
}

static void advance_mode(UwbModeState *state, UwbMode desired, uint32_t tick,
                         uint16_t dwell, uint16_t wait, bool immediate, uint32_t *reasons)
{
    if (!state->ready) {
        state->since = tick;
        state->ready = true;
    }
    if (desired == state->mode && !immediate) {
        state->candidate = UWB_MODE_NONE;
        return;
    }
    if (desired != state->candidate) {
        state->candidate = desired;
        state->candidate_at = tick;
    }
    if (immediate || (tick - state->since >= dwell && tick - state->candidate_at >= wait)) {
        state->mode = desired;
        state->since = tick;
        state->candidate = UWB_MODE_NONE;
    } else *reasons |= UWB_REASON_TRANSITION;
}

void uwb_adaptive_step(UwbCore *core, UwbStep *step, UwbPoint point, uint32_t missing)
{
    UwbRing *ring = &core->adaptive;
    UwbWorkspace *work = &core->work;
    const UwbConfig *p = step->cfg;
    const uint32_t tick = point.tick;
    const double previous = ring->count ? uwb_ring_at(ring, (uint16_t)(ring->count - 1))->phase : NAN;
    point.missing = missing;
    uwb_ring_prune(ring, tick, 600);
    uwb_ring_push(ring, point);

    uwb_view(work, ring, tick, 600, p, false);
    const uint16_t usable = work->count;
    const UwbCircular all = uwb_circular(work, p, UWB_UNKNOWN);
    uwb_view(work, ring, tick, 200, p, false);
    const UwbCircular short_stats = uwb_circular(work, p, UWB_UNKNOWN);
    uwb_view(work, ring, tick, 400, p, false);
    const UwbCluster dominant = uwb_cluster(work, p);

    double loss = 0;
    unsigned warnings = 0;
    for (uint16_t i = 0; i < ring->count; ++i) {
        const UwbPoint *sample = uwb_ring_at(ring, i);
        loss += sample->missing;
        if (sample->reason & UWB_RF_ADC_WARNING) ++warnings;
    }
    step->adc = (double)warnings / ring->count;
    step->loss = loss / (ring->count + loss);
    step->valid_ratio = (double)usable / ring->count;
    step->jump = isfinite(previous) && isfinite(point.phase) ? fabs(uwb_wrap(point.phase - previous)) : 0;
    UwbPhaseHistory *feature = &core->adaptive_previous;
    if (isfinite(short_stats.phase)) {
        if (feature->ready && tick > feature->tick) {
            const double rate = uwb_wrap(short_stats.phase - feature->phase) * 1000.0 / (tick - feature->tick);
            core->adaptive_trend = .7 * core->adaptive_trend + .3 * rate;
        }
        *feature = (UwbPhaseHistory){tick, short_stats.phase, true};
    }
    step->trend = core->adaptive_trend;
    if (step->valid) uwb_iir(&core->adaptive_iir, point.phase, .5);

    const double sigma = isnan(short_stats.sigma) ? 180 : short_stats.sigma;
    UwbMode desired = UWB_FAST;
    UwbDecision decision = uwb_empty_decision();
    bool hard = false;
    if (!step->valid || step->valid_ratio < .6) {
        desired = UWB_HOLD;
        hard = true;
        decision.reasons = !step->obs->center_valid ? UWB_REASON_CENTER :
            !isfinite(point.phase) ? UWB_REASON_INVALID : UWB_REASON_QUALITY;
    } else if (usable < p->min_samples) {
        desired = UWB_HOLD;
        hard = true;
        decision.reasons = UWB_REASON_SAMPLES;
    } else if (sigma >= 35 || step->jump > 70 || step->loss > .2 || step->range_jump) {
        desired = UWB_LONG;
        decision.reasons = step->range_jump ? UWB_REASON_RANGE_JUMP : UWB_REASON_DISPERSION;
    } else if (sigma >= 18 || fabs(step->trend) > 150 || step->loss > .05 || step->adc > .5) {
        desired = UWB_MEDIUM;
        decision.reasons = UWB_REASON_DISPERSION;
    } else if (sigma > 7 || fabs(step->trend) > 60 || step->adc > 0) desired = UWB_SHORT;

    UwbModeState *mode = &core->adaptive_mode;
    const uint16_t wait = adaptive_rank(desired) < adaptive_rank(mode->mode) ? 400 : 100;
    advance_mode(mode, desired, tick, 200, wait, hard, &decision.reasons);
    static const uint16_t windows[] = {0, 160, 320, 600, 0};
    decision.window_ms = windows[adaptive_rank(mode->mode)];
    decision.processing = mode->mode;
    decision.occupancy = dominant.decision.occupancy;
    decision.sigma_deg = all.sigma;
    uwb_view(work, ring, tick, decision.window_ms, p, true);
    decision.sample_count = work->count;
    bool accepted = !hard;
    if (mode->mode == UWB_FAST && core->adaptive_iir.ready) {
        const UwbIir *iir = &core->adaptive_iir;
        uwb_set_phase(&decision, uwb_wrap(atan2(iir->y, iir->x) * 180.0 / UWB_PI));
        decision.sample_count = 1;
        decision.sigma_deg = short_stats.sigma;
    } else if (mode->mode == UWB_SHORT) {
        const UwbCircular stats = uwb_circular(work, p, UWB_UNKNOWN);
        uwb_set_phase(&decision, stats.phase);
        decision.sigma_deg = stats.sigma;
        accepted = accepted && work->count >= p->min_samples && stats.coherence > .2;
    } else if (mode->mode == UWB_MEDIUM || mode->mode == UWB_LONG) {
        const UwbDecision cluster = uwb_cluster(work, p).decision;
        uwb_set_phase(&decision, cluster.phase_deg);
        decision.occupancy = cluster.occupancy;
        decision.sigma_deg = cluster.sigma_deg;
        accepted = accepted && cluster.reasons == 0;
        decision.reasons |= cluster.reasons;
    } else accepted = false;

    const double d = decision.deviation_deg;
    UwbPhaseHistory *offset = &core->adaptive_offset;
    const bool persistent_candidate = isfinite(d) && fabs(d) > p->adaptive_direction_deg + 10 &&
                                      all.sigma < 12 && dominant.decision.occupancy >= .85;
    if (persistent_candidate) {
        if (!offset->ready || fabs(uwb_wrap(d - offset->phase)) > 15)
            *offset = (UwbPhaseHistory){tick, d, true};
    } else offset->ready = false;
    decision.persistent_offset = offset->ready && tick - offset->tick >= 2000;

    const double quality = uwb_clamp01((point.q - (double)p->quality_floor) / (256.0 - p->quality_floor));
    double score = uwb_clamp01(.4 * all.coherence + .25 * quality + .2 * step->valid_ratio + .15 * (1 - step->loss));
    if (step->adc) score *= 1 - .35 * step->adc;
    if (step->range_jump) score *= .5;
    if (decision.persistent_offset) {
        score = fmin(score, .55);
        decision.reasons |= UWB_REASON_PERSISTENT;
        if (step->adc >= .25 || step->loss > .05 || !step->range_available) {
            accepted = false;
            decision.reasons |= UWB_REASON_SUPPORT;
        }
    }
    UwbDirection direction = core->adaptive_direction;
    if (!accepted || !isfinite(d) || mode->mode == UWB_HOLD) direction = UWB_UNKNOWN;
    else if (fabs(d) <= p->adaptive_center_deg) direction = UWB_CENTER;
    else if (d >= p->adaptive_direction_deg) direction = UWB_LEFT;
    else if (d <= -p->adaptive_direction_deg) direction = UWB_RIGHT;
    else if (!((direction == UWB_LEFT && d > 0) || (direction == UWB_RIGHT && d < 0) || direction == UWB_CENTER))
        direction = UWB_UNKNOWN;
    if (accepted && direction == UWB_UNKNOWN) decision.reasons |= UWB_REASON_HYSTERESIS;
    decision.direction = core->adaptive_direction = direction;
    decision.mode = direction == UWB_UNKNOWN ? UWB_HOLD : mode->mode;
    decision.score = direction == UWB_UNKNOWN ? fmin(score, .4) : score;
    core->decisions[UWB_ADAPTIVE] = decision;
}

static double agreement_ratio(const UwbDirection directions[6])
{
    unsigned counts[4] = {0}, largest = 0, total = 0;
    for (unsigned i = 0; i < 6; ++i) {
        if (directions[i] == UWB_UNKNOWN) continue;
        const unsigned count = ++counts[directions[i]];
        if (count > largest) largest = count;
        ++total;
    }
    return total ? (double)largest / total : 0;
}

void uwb_v1_step(UwbCore *core, UwbStep *s)
{
    const UwbDecision *adaptive = &core->decisions[UWB_ADAPTIVE];
    const UwbDecision *light = &core->decisions[UWB_IIR_2], *vote = s->vote, *cluster = &s->cluster.decision;
    const UwbConfig *p = s->cfg;
    const UwbDirection raw = core->decisions[UWB_RAW].direction;
    const UwbDirection methods[] = {raw, core->decisions[UWB_FW5].direction, light->direction,
                                    vote->direction, cluster->direction, adaptive->direction};
    const double agreement = agreement_ratio(methods);
    const double sigma = isnan(s->accepted.sigma) ? 180 : s->accepted.sigma;
    const bool warning = s->obs->sts_q <= p->quality_floor + 5 || s->loss > .05 || s->adc > .5 || s->range_jump;
    s->conflict = uwb_opposite(vote->direction, adaptive->direction) || uwb_opposite(vote->direction, cluster->direction);
    s->coherent_change = vote->direction != UWB_UNKNOWN && vote->occupancy >= .8 &&
        s->winner_count >= p->min_samples && raw == vote->direction &&
        sigma < 25 && !s->range_jump && s->loss <= .05;
    const bool severe = s->conflict && !s->coherent_change;
    const bool unstable = sigma > 35 || (sigma > 20 && warning) || s->loss > .1 || s->range_jump ||
                          (s->jump > 80 && agreement < .8) || adaptive->persistent_offset;
    UwbModeState *mode = &core->v1_mode;
    UwbMode desired = UWB_NORMAL;
    UwbDecision d = uwb_empty_decision();
    if (!s->valid) {
        desired = UWB_LOST;
        d.reasons = !s->obs->center_valid ? UWB_REASON_CENTER :
            s->obs->pdoa_valid ? UWB_REASON_QUALITY : UWB_REASON_INVALID;
    } else if (s->gap > UWB_LOST_MS || mode->mode == UWB_LOST || mode->mode == UWB_RECOVERY) {
        desired = !s->recovered ? UWB_RECOVERY : severe ? UWB_DEGRADED : unstable ? UWB_ROBUST : UWB_NORMAL;
        d.reasons = UWB_REASON_RECOVERY;
    } else if (severe) {
        desired = UWB_DEGRADED;
        d.reasons = UWB_REASON_OPPOSITE;
    } else if (unstable) {
        desired = UWB_ROBUST;
        d.reasons = adaptive->persistent_offset ? UWB_REASON_PERSISTENT : UWB_REASON_DISPERSION;
    } else if (sigma <= 5 && fabs(s->trend) < 15 && agreement >= .8 && s->adc < .1 && s->loss == 0 &&
               s->range_available && !s->range_jump && light->direction == vote->direction) desired = UWB_FAST;
    else if (s->coherent_change && fabs(s->trend) > 15) d.reasons = UWB_REASON_TREND;
    if (s->range_jump) d.reasons |= UWB_REASON_RANGE_JUMP;
    if (s->gap > UWB_LOST_MS) d.reasons |= UWB_REASON_GAP;

    const bool safety = desired != UWB_FAST && desired != UWB_NORMAL;
    const uint16_t wait = desired == UWB_FAST ? 300 : mode->mode == UWB_RECOVERY ? 0 : 120;
    advance_mode(mode, desired, s->obs->tick_ms, 80, wait, safety && desired != mode->mode, &d.reasons);
    d.mode = d.processing = mode->mode;
    d.occupancy = vote->occupancy;
    if (d.mode == UWB_FAST) {
        d.direction = light->direction;
        uwb_set_phase(&d, light->phase_deg);
        d.sample_count = 1;
    } else if (d.mode == UWB_NORMAL) {
        d.direction = vote->direction;
        uwb_set_phase(&d, s->winners.phase);
        d.sample_count = vote->sample_count;
        d.window_ms = s->window_ms;
        d.reasons |= vote->reasons;
    } else if (d.mode == UWB_ROBUST) {
        d.direction = adaptive->direction;
        uwb_set_phase(&d, adaptive->phase_deg);
        d.sample_count = adaptive->sample_count;
        d.window_ms = adaptive->window_ms;
        d.occupancy = adaptive->occupancy;
        d.reasons |= adaptive->reasons;
    }
    if (!s->valid || severe) d.direction = UWB_UNKNOWN;
    if (uwb_opposite(d.direction, core->v1_direction) && !s->coherent_change && agreement < .67) {
        d.direction = UWB_UNKNOWN;
        d.reasons |= UWB_REASON_OPPOSITE;
    }
    if (d.direction == UWB_UNKNOWN) uwb_set_phase(&d, NAN);
    d.score = uwb_clamp01(.35 * s->accepted.coherence + .25 * agreement + .2 * s->valid_ratio + .2 * (1 - s->loss));
    d.persistent_offset = adaptive->persistent_offset;
    if (d.persistent_offset) d.score = fmin(d.score, .55);
    d.sigma_deg = s->accepted.sigma;
    core->v1_direction = d.direction;
    core->decisions[UWB_V1] = d;
}

#if !UWB_V3_ONLY
void uwb_v2_step(UwbCore *core, const UwbStep *s)
{
    const UwbDecision *v1 = &core->decisions[UWB_V1], *cluster = &s->cluster.decision, *vote = s->vote;
    const UwbConfig *p = s->cfg;
    const uint32_t tick = s->obs->tick_ms;
    const UwbDirection raw = core->decisions[UWB_RAW].direction;
    const UwbTrackAnchor anchor = core->v2_anchor;
    const UwbDirection side = anchor.ref.direction;
    const unsigned opposing_samples = side == UWB_LEFT ? s->right_count : side == UWB_RIGHT ? s->left_count : 0;
    const bool clear_opposite = (uwb_opposite(side, vote->direction) && vote->occupancy >= p->consensus) ||
        (uwb_opposite(side, cluster->direction) && cluster->occupancy >= p->consensus) ||
        (uwb_opposite(side, raw) && fabs(s->raw_phase) > p->center_deg + 10) || opposing_samples >= 2;
    if (s->gap > UWB_LOST_MS || !s->valid) {
        core->v2_anchor.ref.direction = UWB_UNKNOWN;
        core->v2_reversal.direction = UWB_UNKNOWN;
        core->v2_recovering = true;
    }
    UwbDecision d = uwb_empty_decision();
    d.processing = v1->mode;
    d.score = v1->score;
    d.occupancy = cluster->occupancy;
    d.sigma_deg = cluster->sigma_deg;
    d.window_ms = v1->window_ms ? v1->window_ms : s->window_ms;
    d.sample_count = s->accepted_count;
    d.reasons = UWB_REASON_SUPPORT;
    if (!s->valid) {
        d.mode = UWB_LOST;
        d.reasons = v1->reasons;
    } else if (core->v2_recovering && !s->recovered) {
        d.mode = UWB_RECOVERY;
        d.reasons = UWB_REASON_RECOVERY | (s->gap > UWB_LOST_MS ? UWB_REASON_GAP : 0);
    } else {
        core->v2_recovering = false;
        if (clear_opposite || s->range_jump) {
            core->v2_anchor.ref.direction = UWB_UNKNOWN;
            d.reasons = s->range_jump ? UWB_REASON_RANGE_JUMP : UWB_REASON_OPPOSITE;
        }
        if (v1->direction != UWB_UNKNOWN) {
            UwbReference *reverse = &core->v2_reversal;
            const bool reversing = uwb_opposite(side, v1->direction) || reverse->direction == v1->direction;
            if (reversing && reverse->direction != v1->direction) *reverse = (UwbReference){v1->direction, tick};
            if (reversing && tick - reverse->tick < 80) {
                d.mode = UWB_RECOVERY;
                d.reasons = UWB_REASON_OPPOSITE | UWB_REASON_RECOVERY;
            } else {
                d.direction = v1->direction;
                uwb_set_phase(&d, v1->phase_deg);
                reverse->direction = UWB_UNKNOWN;
                const bool supported = s->accepted_count >= p->min_samples && v1->score >= .6 &&
                    !s->range_jump && !clear_opposite && s->loss <= .1 &&
                    !uwb_opposite(d.direction, cluster->direction) && !uwb_opposite(d.direction, vote->direction);
                d.mode = supported ? UWB_TRACKING : UWB_DEGRADED_TRACKING;
                d.reasons = supported ? 0 : d.reasons | UWB_REASON_SUPPORT;
                if (supported) core->v2_anchor = (UwbTrackAnchor){{d.direction, tick}, s->obs->sts_q, s->adc};
                else d.score = fmin(d.score, .55);
            }
        } else if (core->v2_anchor.ref.direction != UWB_UNKNOWN) {
            const uint32_t age = tick - anchor.ref.tick;
            const bool quality_drop = s->obs->sts_q < anchor.q - 8 || s->adc > anchor.adc + .25 || s->loss > .05;
            const bool cluster_support = cluster->direction == side && cluster->occupancy >= p->consensus;
            const bool sample_support = raw == side && vote->direction == side;
            const bool stable = cluster_support && cluster->occupancy >= .8 && cluster->sigma_deg <= 15 && s->valid_ratio >= .9;
            d.grace_ms = quality_drop ? 80 : stable ? 240 : 160;
            const bool support = age <= d.grace_ms && s->accepted_count >= p->min_samples &&
                s->obs->sts_q > p->quality_floor + 2 && s->valid_ratio >= .7 && fabs(s->trend) < 120 &&
                s->range_available && s->range_age <= 250 && !s->range_jump && !clear_opposite &&
                (cluster_support || sample_support) && !s->conflict;
            if (support) {
                d.direction = side;
                uwb_set_phase(&d, cluster_support ? cluster->phase_deg : s->raw_phase);
                d.origin = UWB_CONTINUITY_HOLD;
                d.held_ms = (uint16_t)age;
                d.mode = age <= 80 ? UWB_SOFT_HOLD : UWB_DEGRADED_TRACKING;
                d.reasons = UWB_REASON_HOLD;
                d.score = fmin(d.score, age <= 80 ? .55 : .45);
            } else {
                d.reasons = age > d.grace_ms ? UWB_REASON_HOLD_EXPIRED : UWB_REASON_SUPPORT;
                if (age > 240) core->v2_anchor.ref.direction = UWB_UNKNOWN;
            }
        }
    }
    if (d.direction == UWB_UNKNOWN) d.score = fmin(d.score, .4);
    core->decisions[UWB_V2] = d;
}
#endif

static uint32_t boundary_rejection(const UwbStep *s, const UwbDecision *v1, UwbReference reference)
{
    const UwbConfig *p = s->cfg;
    const UwbDecision *cluster = &s->cluster.decision;
    const double signed_phase = cluster->deviation_deg;
    const double half_band = fmin(fmin(p->center_deg / 2, (180 - p->center_deg) / 2), 8);
    const UwbDirection side = signed_phase > 0 ? UWB_LEFT : UWB_RIGHT;
    const unsigned same = side == UWB_LEFT ? s->left_count : s->right_count;
    const unsigned opposite = side == UWB_LEFT ? s->right_count : s->left_count;
    const bool reference_opposes = uwb_opposite(reference.direction, side) &&
        s->obs->tick_ms - reference.tick <= s->window_ms && same > 0;
    if (s->conflict || (signed_phase != 0 && isfinite(signed_phase) && opposite > 0) || reference_opposes)
        return UWB_REASON_OPPOSITE;
    if (s->accepted_count < p->min_samples || (double)s->accepted_count / s->observed_count < .8)
        return UWB_REASON_SAMPLES;
    if (s->obs->sts_q <= p->quality_floor + 2 || s->adc > .25 || s->loss > .05 || s->range_jump)
        return s->range_jump ? UWB_REASON_RANGE_JUMP : UWB_REASON_QUALITY;
    if (cluster->reasons || cluster->occupancy + 1e-12 < fmax(p->consensus, .8) ||
        !isfinite(v1->sigma_deg) || v1->sigma_deg > 15 || !isfinite(cluster->sigma_deg) || cluster->sigma_deg > 12)
        return UWB_REASON_DISPERSION | UWB_REASON_CLUSTER;
    if (!isfinite(signed_phase) || fabs(fabs(signed_phase) - p->center_deg) > half_band ||
        !isfinite(s->cluster.slope) || fabs(s->cluster.slope) > 120 || fabs(s->trend) > 120)
        return UWB_REASON_TREND;
    return 0;
}

void uwb_v3_step(UwbCore *core, const UwbStep *s)
{
    UwbDecision d = core->decisions[UWB_V1];
    UwbReference *ref = &core->v3_reference;
    const uint32_t tick = s->obs->tick_ms;
    if (!s->valid || s->gap > UWB_LOST_MS || d.mode == UWB_LOST || d.mode == UWB_RECOVERY) {
        ref->direction = UWB_UNKNOWN;
        if (!s->valid || s->gap > UWB_LOST_MS) {
            d.direction = UWB_UNKNOWN;
            uwb_set_phase(&d, NAN);
            d.mode = !s->valid ? UWB_LOST : UWB_RECOVERY;
        }
    } else if (d.direction != UWB_UNKNOWN) {
        *ref = (UwbReference){d.mode == UWB_FAST || d.mode == UWB_NORMAL ? d.direction : UWB_UNKNOWN, tick};
    } else if (d.mode == UWB_NORMAL && s->vote->reasons == UWB_REASON_VOTE) {
        const uint32_t rejected = boundary_rejection(s, &d, *ref);
        if (rejected) {
            d.reasons |= rejected;
            ref->direction = UWB_UNKNOWN;
        } else {
            const UwbDecision *cluster = &s->cluster.decision;
            const double threshold = s->cfg->center_deg;
            const double half_band = fmin(fmin(threshold / 2, (180 - threshold) / 2), 8);
            const double hysteresis = fmin(half_band / 2, 3);
            const double magnitude = fabs(cluster->deviation_deg);
            const UwbDirection side = cluster->deviation_deg > 0 ? UWB_LEFT : UWB_RIGHT;
            d.reasons = UWB_REASON_BOUNDARY;
            if (magnitude <= threshold - hysteresis) d.direction = UWB_CENTER;
            else if (magnitude >= threshold + hysteresis) d.direction = side;
            else if ((ref->direction == UWB_CENTER || ref->direction == side) && tick - ref->tick <= s->window_ms) {
                d.direction = ref->direction;
                d.origin = UWB_BOUNDARY_REFERENCE;
            }
            if (d.direction != UWB_UNKNOWN && d.origin != UWB_BOUNDARY_REFERENCE)
                *ref = (UwbReference){d.direction, tick};
            else d.reasons |= UWB_REASON_HYSTERESIS;
            d.mode = d.processing = UWB_BOUNDARY;
            d.boundary = true;
            uwb_set_phase(&d, cluster->phase_deg);
            if (d.direction == UWB_UNKNOWN) d.deviation_deg = NAN;
            d.window_ms = s->window_ms;
            d.sample_count = s->accepted_count;
            d.occupancy = cluster->occupancy;
        }
    } else ref->direction = UWB_UNKNOWN;
    core->decisions[UWB_V3] = d;
}
