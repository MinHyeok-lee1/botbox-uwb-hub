#include "uwb_core.h"
#include <inttypes.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

static UwbCore core;
static unsigned rows, numeric_checks, categorical_checks, failures, queries, pairs;
static double numeric_max[4], total_us, max_us;

static double now_us(void)
{
#ifdef _WIN32
    static double scale;
    LARGE_INTEGER value;
    if (!scale) {
        QueryPerformanceFrequency(&value);
        scale = 1e6 / (double)value.QuadPart;
    }
    QueryPerformanceCounter(&value);
    return (double)value.QuadPart * scale;
#else
    struct timespec value;
    timespec_get(&value, TIME_UTC);
    return (double)value.tv_sec * 1e6 + (double)value.tv_nsec / 1000;
#endif
}

static void check_int(const char *field, unsigned method, uint64_t actual, uint64_t expected)
{
    ++categorical_checks;
    if (actual == expected) return;
    if (++failures <= 24)
        fprintf(stderr, "D %u tick %" PRIu32 " seq %" PRIu32 " %s/%u: C=%" PRIu64 " JS=%" PRIu64 "\n",
                rows, core.last.tick_ms, core.last.sequence, field, method, actual, expected);
}

static void check_float(unsigned method, unsigned field, double actual, double expected)
{
    static const double tolerance[] = {1e-8, 3e-6, 1e-10, 1e-10};
    ++numeric_checks;
    if (isnan(actual) && isnan(expected)) return;
    if (isfinite(actual) && isfinite(expected)) {
        double error = fabs(actual - expected);
        if (field == 0) error = fmin(error, fabs(360 - error));
        numeric_max[field] = fmax(error, numeric_max[field]);
        if (error <= tolerance[field]) return;
    }
    if (++failures <= 24)
        fprintf(stderr, "D %u seq %" PRIu32 " numeric/%u/%u: C=%.17g JS=%.17g\n",
                rows, core.last.sequence, method, field, actual, expected);
}

static bool check_estimates(char *line)
{
    char directions[UWB_METHOD_COUNT + 1];
    unsigned modes[5], hold, boundary, reference, held_ms, grace_ms, persistent;
    int numeric_at = 0;
    if (sscanf(line, "E %17s %u %u %u %u %u %u %u %u %u %u %u %n", directions,
               &modes[0], &modes[1], &modes[2], &modes[3], &modes[4], &hold, &boundary, &reference,
               &held_ms, &grace_ms, &persistent, &numeric_at) != 12 || strlen(directions) != UWB_METHOD_COUNT)
        return false;
    for (unsigned i = 0; i < UWB_METHOD_COUNT; ++i)
        if (uwb_core_method_enabled(&core, (UwbMethod)i))
            check_int("direction", i, core.decisions[i].direction, (unsigned)(directions[i] - '0'));
    check_int("adaptive.mode", UWB_ADAPTIVE, core.decisions[UWB_ADAPTIVE].mode, modes[0]);
    check_int("adaptive.processing", UWB_ADAPTIVE, core.decisions[UWB_ADAPTIVE].processing, modes[1]);
    for (unsigned i = 2; i < 5; ++i)
        if (uwb_core_method_enabled(&core, (UwbMethod)(UWB_V1 + i - 2)))
            check_int("mode", UWB_V1 + i - 2, core.decisions[UWB_V1 + i - 2].mode, modes[i]);
    check_int("persistent", UWB_ADAPTIVE, core.decisions[UWB_ADAPTIVE].persistent_offset, persistent);
    if (uwb_core_method_enabled(&core, UWB_V2)) {
        check_int("hold", UWB_V2, core.decisions[UWB_V2].origin == UWB_CONTINUITY_HOLD, hold);
        check_int("held_ms", UWB_V2, core.decisions[UWB_V2].held_ms, held_ms);
        check_int("grace_ms", UWB_V2, core.decisions[UWB_V2].grace_ms, grace_ms);
    }
    check_int("boundary", UWB_V3, core.decisions[UWB_V3].boundary, boundary);
    check_int("reference", UWB_V3, core.decisions[UWB_V3].origin == UWB_BOUNDARY_REFERENCE, reference);
    char *start = line + numeric_at;
    for (unsigned method = 0; method < UWB_METHOD_COUNT; ++method) {
        const UwbDecision *d = &core.decisions[method];
        const double values[] = {d->phase_deg, d->sigma_deg, d->score, d->occupancy};
        /* Dashboard baseline/cluster score has a different UI field name. */
        const unsigned fields = method <= UWB_IIR_2 ? 1 : method < UWB_ADAPTIVE ? 2 : 4;
        for (unsigned field = 0; field < fields; ++field) {
            char *end = start;
            const double expected = strtod(start, &end);
            if (start == end) return false;
            if (uwb_core_method_enabled(&core, (UwbMethod)method))
                check_float(method, field, values[field], expected);
            start = end;
        }
    }
    return true;
}

static bool observation(char *line)
{
    UwbObservation d;
    int raw, q;
    unsigned reason, valid, center;
    if (sscanf(line, "D %" SCNu32 " %" SCNu32 " %" SCNu64 " %d %u %d %u %" SCNd32 " %u",
               &d.tick_ms, &d.sequence, &d.arrival_ms, &raw, &valid, &q, &reason, &d.center_mdeg, &center) != 9)
        return false;
    d.raw_pdoa = (int16_t)raw;
    d.sts_q = (int16_t)q;
    d.reason_flags = (uint16_t)reason;
    d.pdoa_valid = valid != 0;
    d.center_valid = center != 0;
    const double start = now_us();
    const UwbStatus status = uwb_core_observe(&core, &d);
    const double duration = now_us() - start;
    total_us += duration;
    max_us = fmax(max_us, duration);
    ++rows;
    check_int("observe", 0, status, UWB_ACCEPTED);
    return status == UWB_ACCEPTED;
}

static bool range(char *line)
{
    UwbRangeObservation p;
    unsigned valid, status;
    if (sscanf(line, "P %" SCNu32 " %" SCNu32 " %" SCNd32 " %u %u",
               &p.tick_ms, &p.sequence, &p.distance_mm, &valid, &status) != 5) return false;
    p.valid = valid != 0;
    check_int("range", 0, uwb_core_observe_range(&core, &p), status);
    ++pairs;
    return true;
}

static bool query(char *line)
{
    uint64_t now, arrival;
    uint32_t sequence, tick;
    int32_t distance;
    unsigned method, direction, mode, origin, ready, lost, center, range_valid, direction_valid, degraded;
    if (sscanf(line, "Q %" SCNu64 " %u %u %u %u %u %u %u %u %" SCNd32 " %" SCNu32 " %" SCNu32 " %" SCNu64 " %u %u",
               &now, &method, &direction, &mode, &origin, &ready, &lost, &center, &range_valid, &distance,
               &sequence, &tick, &arrival, &direction_valid, &degraded) != 15)
        return false;
    UwbTrackingResult r;
    if (!uwb_core_method_enabled(&core, (UwbMethod)method)) {
        return !uwb_core_read(&core, (UwbMethod)method, now, &r) &&
            uwb_core_decision(&core, (UwbMethod)method) == NULL;
    }
    if (!uwb_core_read(&core, (UwbMethod)method, now, &r)) return false;
    check_int("read.direction", method, r.decision.direction, direction);
    check_int("read.mode", method, r.decision.mode, mode);
    check_int("read.origin", method, r.decision.origin, origin);
    check_int("read.ready", method, r.has_observation, ready);
    check_int("read.tag_lost", method, r.tag_lost, lost);
    check_int("read.center", method, r.center_valid, center);
    check_int("read.distance_valid", method, r.distance_valid, range_valid);
    check_int("read.distance", method, (uint32_t)r.distance_mm, (uint32_t)distance);
    check_int("read.sequence", method, r.sequence, sequence);
    check_int("read.tick", method, r.observation_tick_ms, tick);
    check_int("read.last_arrival", method, r.last_arrival_ms, arrival);
    check_int("read.direction_valid", method, r.direction_valid, direction_valid);
    check_int("read.degraded", method, r.degraded, degraded);
    ++queries;
    return true;
}

static bool capacity_and_status(void)
{
    static UwbCore instance;
    UwbConfig p = uwb_research_config();
    p.windows_ms[0] = 2000;
    if (uwb_core_init(&instance, &p)) return false;
    p = uwb_research_config();
    p.windows_ms[0] = 100;
    p.windows_ms[1] = 220;
    if (!uwb_core_init(&instance, &p) || !uwb_core_method_enabled(&instance, UWB_WEIGHTED_VOTE_1) ||
        !uwb_core_method_enabled(&instance, UWB_WEIGHTED_CLUSTER_1)) return false;
    const bool full = uwb_core_method_enabled(&instance, UWB_V2);
    if (uwb_core_method_enabled(&instance, UWB_WEIGHTED_VOTE_0) != full ||
        uwb_core_method_enabled(&instance, UWB_IIR_1) != full) return false;
    p = uwb_research_config();
    if (!uwb_core_init(&instance, &p)) return false;
    UwbObservation d = {.center_valid = true, .pdoa_valid = true, .sts_q = 250};
    for (unsigned i = 0; i < UWB_CAPACITY; ++i) {
        d.sequence = i;
        if (uwb_core_observe(&instance, &d) != UWB_ACCEPTED) return false;
    }
    if (uwb_core_observe(&instance, &d) != UWB_DUPLICATE) return false;
    ++d.sequence;
    if (uwb_core_observe(&instance, &d) != UWB_CAPACITY_EXCEEDED || instance.last.sequence != UWB_CAPACITY - 1)
        return false;
    d.tick_ms = 401;
    d.arrival_ms = UINT64_C(4294967296);
    if (uwb_core_observe(&instance, &d) != UWB_ACCEPTED) return false;
    d.arrival_ms--;
    if (uwb_core_observe(&instance, &d) != UWB_BAD_INPUT) return false;
    const UwbRangeObservation unpaired = {.sequence = 1000};
    return uwb_core_observe_range(&instance, &unpaired) == UWB_UNPAIRED &&
           uwb_core_decision(&instance, UWB_METHOD_COUNT) == NULL &&
           !core.has_observation;
}

int main(void)
{
    char line[4096];
    const UwbConfig config = uwb_research_config();
    if (!uwb_core_init(&core, &config) || !capacity_and_status()) {
        fprintf(stderr, "Core contract self-check failed\n");
        return 1;
    }
    unsigned line_number = 0;
    while (fgets(line, sizeof line, stdin)) {
        ++line_number;
        bool ok;
        switch (line[0]) {
        case 'X': uwb_core_reset(&core); ok = true; break;
        case 'D': ok = observation(line); break;
        case 'E': ok = core.has_observation && check_estimates(line); break;
        case 'P': ok = range(line); break;
        case 'Q': ok = query(line); break;
        default: ok = false; break;
        }
        if (!ok) {
            fprintf(stderr, "Invalid replay record %u (%c)\n", line_number, line[0]);
            return 2;
        }
    }
    printf("profile=%s observations=%u range=%u queries=%u categorical=%u numeric=%u mismatches=%u\n",
           uwb_core_method_enabled(&core, UWB_V2) ? "full" : "v3-only",
           rows, pairs, queries, categorical_checks, numeric_checks, failures);
    printf("max_error_phase_deg=%.9g sigma_deg=%.9g score=%.9g occupancy=%.9g\n",
           numeric_max[0], numeric_max[1], numeric_max[2], numeric_max[3]);
    printf("host_core_bytes=%zu host_mean_us=%.3f host_max_us=%.3f\n",
           sizeof core, rows ? total_us / rows : 0, max_us);
    return failures || ferror(stdin) || rows == 0 ? 1 : 0;
}
