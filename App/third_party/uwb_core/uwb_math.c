#include "uwb_internal.h"

UwbDecision uwb_empty_decision(void)
{
    return (UwbDecision){
        .phase_deg = NAN, .deviation_deg = NAN, .sigma_deg = NAN,
        .score = NAN, .occupancy = NAN
    };
}

void uwb_set_phase(UwbDecision *decision, double phase)
{
    decision->phase_deg = phase;
    decision->deviation_deg = -phase;
}

double uwb_wrap(double value)
{
    return fmod(fmod(value + 180.0, 360.0) + 360.0, 360.0) - 180.0;
}

double uwb_clamp01(double value)
{
    return fmin(1.0, fmax(0.0, value));
}

UwbDirection uwb_direction(double phase, const UwbConfig *config)
{
    if (!isfinite(phase)) return UWB_UNKNOWN;
    if (phase < -config->center_deg) return UWB_LEFT;
    if (phase > config->center_deg) return UWB_RIGHT;
    return UWB_CENTER;
}

bool uwb_opposite(UwbDirection a, UwbDirection b)
{
    return (a == UWB_LEFT && b == UWB_RIGHT) || (a == UWB_RIGHT && b == UWB_LEFT);
}

double uwb_weight(const UwbPoint *point, const UwbConfig *config)
{
    if (!isfinite(point->phase) || (point->reason & UWB_RF_HARD_MASK)) return 0;
    double weight = fmax(0.0, fmin(point->q, 256.0) - config->quality_floor) / (256.0 - config->quality_floor);
    if (point->reason & UWB_RF_ADC_WARNING) weight *= config->adc_weight;
    return weight;
}

double uwb_iir(UwbIir *state, double phase, double alpha)
{
    const double radians = phase * UWB_PI / 180.0;
    const double x = cos(radians), y = sin(radians);
    state->x = (1.0 - alpha) * (state->ready ? state->x : x) + alpha * x;
    state->y = (1.0 - alpha) * (state->ready ? state->y : y) + alpha * y;
    state->ready = true;
    return hypot(state->x, state->y) < 1e-8 ? NAN : uwb_wrap(atan2(state->y, state->x) * 180.0 / UWB_PI);
}

static UwbCircular from_sums(double x, double y, double weight)
{
    if (weight == 0) return (UwbCircular){NAN, NAN, 0};
    x /= weight;
    y /= weight;
    const double coherence = fmin(1.0, hypot(x, y));
    if (coherence < 1e-8) return (UwbCircular){NAN, NAN, coherence};
    return (UwbCircular){uwb_wrap(atan2(y, x) * 180.0 / UWB_PI),
                         sqrt(-2.0 * log(coherence)) * 180.0 / UWB_PI, coherence};
}

UwbCircular uwb_circular(const UwbWorkspace *work, const UwbConfig *config, UwbDirection keep)
{
    double weight = 0, x = 0, y = 0;
    for (uint16_t i = 0; i < work->count; ++i) {
        const double phase = work->points[i]->phase;
        if (keep != UWB_UNKNOWN && uwb_direction(phase, config) != keep) continue;
        const double radians = phase * UWB_PI / 180.0;
        weight += work->weights[i];
        x += work->weights[i] * cos(radians);
        y += work->weights[i] * sin(radians);
    }
    return from_sums(x, y, weight);
}

UwbDecision uwb_vote(const UwbWorkspace *work, const UwbConfig *config)
{
    double weights[4] = {0};
    UwbDirection order[] = {UWB_LEFT, UWB_CENTER, UWB_RIGHT};
    for (uint16_t i = 0; i < work->count; ++i)
        weights[uwb_direction(work->points[i]->phase, config)] += work->weights[i];
    for (unsigned i = 1; i < 3; ++i) {
        const UwbDirection candidate = order[i];
        unsigned j = i;
        while (j > 0 && weights[candidate] > weights[order[j - 1]]) {
            order[j] = order[j - 1];
            --j;
        }
        order[j] = candidate;
    }
    const double first = weights[order[0]], second = weights[order[1]];
    const double total = first + second + weights[order[2]];
    UwbDecision decision = uwb_empty_decision();
    decision.sample_count = work->count;
    decision.occupancy = total > 0 ? first / total : 0;
    if (work->count < config->min_samples) decision.reasons = UWB_REASON_SAMPLES;
    else if (first == second || decision.occupancy + 1e-12 < config->consensus)
        decision.reasons = UWB_REASON_VOTE;
    else decision.direction = order[0];
    return decision;
}

static uint16_t arc_index(uint16_t index, uint16_t count)
{
    return index < count ? index : (uint16_t)(index - count);
}

UwbCluster uwb_cluster(UwbWorkspace *work, const UwbConfig *config)
{
    const uint16_t count = work->count;
    UwbCluster result = {.decision = uwb_empty_decision(), .slope = NAN};
    UwbDecision *decision = &result.decision;
    decision->sample_count = count;
    decision->occupancy = 0;
    if (count < config->min_samples) {
        decision->reasons = UWB_REASON_SAMPLES;
        return result;
    }

    double total = 0;
    for (uint16_t i = 0; i < count; ++i) {
        const double angle = uwb_wrap(work->points[i]->phase) + 180.0;
        uint16_t j = i;
        while (j > 0 && angle < work->angles[work->order[j - 1]]) {
            work->order[j] = work->order[j - 1];
            --j;
        }
        work->order[j] = i;
        work->angles[i] = angle;
        total += work->weights[i];
    }

    uint16_t best = 0;
    for (uint16_t start = 0; start < count; ++start) {
        const double first = work->angles[work->order[start]];
        double weight = 0;
        uint16_t end = start;
        for (; end < start + count; ++end) {
            const uint16_t index = work->order[arc_index(end, count)];
            const double angle = work->angles[index] + (end >= count ? 360.0 : 0.0);
            if (angle - first > config->cluster_width_deg + 1e-9) break;
            weight += work->weights[index];
        }
        work->arc_counts[start] = (uint16_t)(end - start);
        work->arc_weights[start] = weight;
        if (weight > work->arc_weights[best]) best = start;
    }

    const double best_weight = work->arc_weights[best];
    result.members = work->arc_counts[best];
    double rival = 0;
    for (uint16_t offset = result.members; offset < count; ++offset) {
        const uint16_t start = arc_index((uint16_t)(best + offset), count);
        if (work->arc_counts[start] <= count - offset) rival = fmax(rival, work->arc_weights[start]);
    }

    double x = 0, y = 0;
    for (uint16_t offset = 0; offset < result.members; ++offset) {
        const uint16_t index = work->order[arc_index((uint16_t)(best + offset), count)];
        const double radians = work->points[index]->phase * UWB_PI / 180.0;
        x += work->weights[index] * cos(radians);
        y += work->weights[index] * sin(radians);
    }
    const UwbCircular stats = from_sums(x, y, best_weight);
    decision->occupancy = best_weight / total;
    decision->sigma_deg = stats.sigma;
    decision->score = decision->occupancy * stats.coherence;
    uwb_set_phase(decision, stats.phase);
    if (result.members < config->min_samples) decision->reasons = UWB_REASON_SAMPLES | UWB_REASON_CLUSTER;
    else if (decision->occupancy + 1e-12 < config->consensus || !isfinite(stats.phase) ||
             (best_weight - rival) / total < 1.0 / 6.0) decision->reasons = UWB_REASON_CLUSTER;
    else decision->direction = uwb_direction(stats.phase, config);
    return result;
}

const UwbPoint *uwb_ring_at(const UwbRing *ring, uint16_t index)
{
    return &ring->points[(ring->head + index) % UWB_CAPACITY];
}

void uwb_ring_prune(UwbRing *ring, uint32_t tick, uint16_t window)
{
    while (ring->count && tick - ring->points[ring->head].tick > window) {
        ring->head = (uint16_t)((ring->head + 1) % UWB_CAPACITY);
        --ring->count;
    }
}

void uwb_ring_push(UwbRing *ring, UwbPoint point)
{
    if (ring->count == UWB_CAPACITY) {
        ring->head = (uint16_t)((ring->head + 1) % UWB_CAPACITY);
        --ring->count;
    }
    ring->points[(ring->head + ring->count++) % UWB_CAPACITY] = point;
}

void uwb_view(UwbWorkspace *work, const UwbRing *ring, uint32_t tick, uint16_t window,
              const UwbConfig *config, bool weighted)
{
    work->count = 0;
    for (uint16_t i = 0; i < ring->count; ++i) {
        const UwbPoint *point = uwb_ring_at(ring, i);
        if (tick - point->tick > window || !isfinite(point->phase)) continue;
        const double weight = weighted ? uwb_weight(point, config) : 1;
        if (weight > 0) {
            work->points[work->count] = point;
            work->weights[work->count++] = weight;
        }
    }
}
