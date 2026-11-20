#ifndef UWB_INTERNAL_H
#define UWB_INTERNAL_H

#include "uwb_core.h"
#include <math.h>
#include <stddef.h>

#ifndef UWB_V3_ONLY
#define UWB_V3_ONLY 0
#endif

#define UWB_PI 3.14159265358979323846
#define UWB_RESET_GAP 1000u

typedef struct { double phase, sigma, coherence; } UwbCircular;
typedef struct {
    UwbDecision decision;
    uint16_t members;
    double slope;
} UwbCluster;
typedef struct {
    const UwbObservation *obs;
    const UwbConfig *cfg;
    const UwbDecision *vote;
    UwbCluster cluster;
    UwbCircular accepted, winners;
    uint16_t accepted_count, observed_count, winner_count;
    uint16_t left_count, right_count, window_ms;
    double raw_phase, adc, loss, valid_ratio, jump, trend, range_delta;
    uint32_t gap, range_age;
    bool valid, recovered, range_available, range_jump, conflict, coherent_change;
} UwbStep;

UwbDecision uwb_empty_decision(void);
void uwb_set_phase(UwbDecision *decision, double phase);
double uwb_wrap(double value);
double uwb_clamp01(double value);
UwbDirection uwb_direction(double phase, const UwbConfig *config);
bool uwb_opposite(UwbDirection a, UwbDirection b);
double uwb_weight(const UwbPoint *point, const UwbConfig *config);
double uwb_iir(UwbIir *state, double phase, double alpha);
UwbCircular uwb_circular(const UwbWorkspace *work, const UwbConfig *config, UwbDirection keep);
UwbDecision uwb_vote(const UwbWorkspace *work, const UwbConfig *config);
UwbCluster uwb_cluster(UwbWorkspace *work, const UwbConfig *config);
const UwbPoint *uwb_ring_at(const UwbRing *ring, uint16_t index);
void uwb_ring_prune(UwbRing *ring, uint32_t tick, uint16_t window);
void uwb_ring_push(UwbRing *ring, UwbPoint point);
void uwb_view(UwbWorkspace *work, const UwbRing *ring, uint32_t tick, uint16_t window,
              const UwbConfig *config, bool weighted);
void uwb_adaptive_reset(UwbCore *core);
void uwb_adaptive_step(UwbCore *core, UwbStep *step, UwbPoint point, uint32_t missing);
void uwb_v1_step(UwbCore *core, UwbStep *step);
#if !UWB_V3_ONLY
void uwb_v2_step(UwbCore *core, const UwbStep *step);
#endif
void uwb_v3_step(UwbCore *core, const UwbStep *step);

#endif
