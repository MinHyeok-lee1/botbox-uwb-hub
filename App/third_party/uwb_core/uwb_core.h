#ifndef UWB_CORE_H
#define UWB_CORE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UWB_CAPACITY 128u
#define UWB_LOST_MS 250u

typedef enum { UWB_UNKNOWN, UWB_LEFT, UWB_CENTER, UWB_RIGHT } UwbDirection;
typedef enum {
    UWB_MODE_NONE, UWB_FAST, UWB_SHORT, UWB_MEDIUM, UWB_LONG, UWB_HOLD,
    UWB_NORMAL, UWB_ROBUST, UWB_DEGRADED, UWB_LOST, UWB_RECOVERY,
    UWB_TRACKING, UWB_SOFT_HOLD, UWB_DEGRADED_TRACKING, UWB_BOUNDARY
} UwbMode;
typedef enum {
    UWB_RAW, UWB_FW5, UWB_IIR_0, UWB_IIR_1, UWB_IIR_2,
    UWB_VOTE_0, UWB_WEIGHTED_VOTE_0, UWB_CLUSTER_0, UWB_WEIGHTED_CLUSTER_0,
    UWB_VOTE_1, UWB_WEIGHTED_VOTE_1, UWB_CLUSTER_1, UWB_WEIGHTED_CLUSTER_1,
    UWB_ADAPTIVE, UWB_V1, UWB_V2, UWB_V3, UWB_METHOD_COUNT
} UwbMethod;
typedef enum {
    UWB_CURRENT_NONE, UWB_CURRENT_OBSERVATION, UWB_CONTINUITY_HOLD, UWB_BOUNDARY_REFERENCE
} UwbOrigin;
typedef enum {
    UWB_ACCEPTED, UWB_DUPLICATE, UWB_UNPAIRED, UWB_BAD_INPUT, UWB_CAPACITY_EXCEEDED
} UwbStatus;

enum {
    UWB_REASON_CENTER = 1u << 0, UWB_REASON_INVALID = 1u << 1,
    UWB_REASON_QUALITY = 1u << 2, UWB_REASON_SAMPLES = 1u << 3,
    UWB_REASON_VOTE = 1u << 4, UWB_REASON_CLUSTER = 1u << 5,
    UWB_REASON_DISPERSION = 1u << 6, UWB_REASON_RANGE_JUMP = 1u << 7,
    UWB_REASON_OPPOSITE = 1u << 8, UWB_REASON_TRANSITION = 1u << 9,
    UWB_REASON_RECOVERY = 1u << 10, UWB_REASON_HYSTERESIS = 1u << 11,
    UWB_REASON_HOLD = 1u << 12, UWB_REASON_HOLD_EXPIRED = 1u << 13,
    UWB_REASON_SUPPORT = 1u << 14, UWB_REASON_BOUNDARY = 1u << 15,
    UWB_REASON_TREND = 1u << 16, UWB_REASON_GAP = 1u << 17,
    UWB_REASON_PERSISTENT = 1u << 18, UWB_REASON_WAITING = 1u << 19
};
enum { UWB_RF_HARD_MASK = 7, UWB_RF_ADC_WARNING = 8 };

typedef struct {
    uint16_t windows_ms[2], min_samples, quality_floor;
    double center_deg, cluster_width_deg, consensus, adc_weight;
    double iir_alpha[3], adaptive_center_deg, adaptive_direction_deg;
} UwbConfig;

typedef struct {
    uint64_t arrival_ms;
    uint32_t tick_ms, sequence;
    int32_t center_mdeg;
    int16_t raw_pdoa, sts_q;
    uint16_t reason_flags;
    bool pdoa_valid, center_valid;
} UwbObservation;

/* Adapter supplies a distance already associated with this observation. */
typedef struct {
    uint32_t tick_ms, sequence;
    int32_t distance_mm;
    bool valid;
} UwbRangeObservation;

typedef struct {
    double phase_deg, deviation_deg, sigma_deg, score, occupancy;
    uint32_t reasons;
    uint16_t window_ms, sample_count, held_ms, grace_ms;
    UwbDirection direction;
    UwbMode mode, processing;
    UwbOrigin origin;
    bool boundary, persistent_offset;
} UwbDecision;

typedef struct {
    UwbDecision decision;
    uint64_t last_arrival_ms;
    uint32_t observation_tick_ms, sequence;
    int32_t distance_mm;
    bool has_observation, distance_valid, direction_valid, center_valid, tag_lost, degraded;
} UwbTrackingResult;

/* Storage layout is exposed for static allocation. Inspect it via the public API. */
typedef struct {
    double phase;
    uint32_t tick, missing;
    int16_t q;
    uint16_t reason;
} UwbPoint;
typedef struct { UwbPoint points[UWB_CAPACITY]; uint16_t head, count; } UwbRing;
typedef struct { double x, y; bool ready; } UwbIir;
typedef struct { uint32_t tick; double phase; bool ready; } UwbPhaseHistory;
typedef struct { UwbMode mode, candidate; uint32_t since, candidate_at; bool ready; } UwbModeState;
typedef struct { uint32_t ticks[UWB_CAPACITY]; uint16_t head, count; } UwbRecovery;
typedef struct { UwbDirection direction; uint32_t tick; } UwbReference;
typedef struct {
    UwbReference ref;
    int16_t q;
    double adc;
} UwbTrackAnchor;
typedef struct {
    const UwbPoint *points[UWB_CAPACITY];
    double weights[UWB_CAPACITY];
    double angles[UWB_CAPACITY];
    double arc_weights[UWB_CAPACITY];
    uint16_t order[UWB_CAPACITY], arc_counts[UWB_CAPACITY], count;
} UwbWorkspace;
typedef struct {
    UwbConfig config;
    UwbObservation last;
    UwbRangeObservation range, prior_range, paired_range;
    bool has_observation, has_range, has_prior_range, has_pair, has_bundle_valid;
    uint32_t bundle_valid_tick;
    UwbRing bundle, adaptive;
    UwbPoint fw5[5];
    uint8_t fw_count, fw_head;
    UwbDirection firmware_direction, adaptive_direction, v1_direction;
    UwbIir iir[3], adaptive_iir;
    UwbPhaseHistory cluster_history[2][2], adaptive_previous, adaptive_offset;
    double adaptive_trend;
    UwbModeState adaptive_mode, v1_mode;
    UwbRecovery good;
    bool v2_recovering;
    UwbTrackAnchor v2_anchor;
    UwbReference v2_reversal, v3_reference;
    UwbDecision decisions[UWB_METHOD_COUNT];
    UwbWorkspace work;
} UwbCore;

UwbConfig uwb_research_config(void);
bool uwb_core_init(UwbCore *core, const UwbConfig *config);
/* Clears observations and estimator history, retaining the current configuration. */
void uwb_core_reset(UwbCore *core);
UwbStatus uwb_core_observe(UwbCore *core, const UwbObservation *observation);
UwbStatus uwb_core_observe_range(UwbCore *core, const UwbRangeObservation *range);
/* Reports availability for this build profile and the initialized configuration. */
bool uwb_core_method_enabled(const UwbCore *core, UwbMethod method);
/* Returns the raw estimator result, including when a later read would time out. */
const UwbDecision *uwb_core_decision(const UwbCore *core, UwbMethod method);
/* Poll from the caller's monotonic clock, even when observations stop. */
bool uwb_core_read(const UwbCore *core, UwbMethod method, uint64_t now_ms, UwbTrackingResult *result);

#ifdef __cplusplus
}
#endif
#endif
