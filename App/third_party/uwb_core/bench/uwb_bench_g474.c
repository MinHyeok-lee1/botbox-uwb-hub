#include "uwb_bench_g474.h"
#include "uwb_core.h"
#include "stm32g474xx.h"
#include <stdbool.h>

enum { WARMUP_SAMPLES = 64, TIMED_SAMPLES = 1024, PERIOD_MS = 40 };

volatile UwbBenchG474Report uwb_bench_g474_report;
static UwbCore core;

static uint32_t cycles(void)
{
    __DSB();
    __ISB();
    return DWT->CYCCNT;
}

static int16_t raw_phase(unsigned scenario, unsigned index)
{
    static const int16_t stable[] = {0, 5, -7, 3};
    static const int16_t dispersed[] = {0, 5600, 35, -32, -5800, 70, 42, 5900};
    if (scenario == UWB_BENCH_STABLE) return stable[index % 4];
    if (scenario == UWB_BENCH_DISPERSED) return dispersed[index % 8];
    return index % 2 == 0 ? 3217 : -3217; /* Two equal groups near ±90 degrees. */
}

static bool run_case(unsigned scenario, UwbBenchCaseResult *result)
{
    const UwbConfig config = uwb_research_config();
    if (!uwb_core_init(&core, &config)) return false;
    for (unsigned i = 0; i < WARMUP_SAMPLES + TIMED_SAMPLES; ++i) {
        const uint32_t tick = i * PERIOD_MS;
        const UwbObservation observation = {
            .arrival_ms = tick, .tick_ms = tick, .sequence = i,
            .raw_pdoa = raw_phase(scenario, i), .sts_q = 250,
            .pdoa_valid = true, .center_valid = true
        };
        uint32_t elapsed = 0;
        UwbStatus status;
        if (i < WARMUP_SAMPLES) {
            status = uwb_core_observe(&core, &observation);
        } else {
            const uint32_t start = cycles();
            status = uwb_core_observe(&core, &observation);
            elapsed = cycles() - start; /* The 32-bit counter may wrap once. */
        }
        const UwbRangeObservation range = {
            .tick_ms = tick + 7, .sequence = i, .distance_mm = 2000, .valid = true
        };
        if (status != UWB_ACCEPTED || uwb_core_observe_range(&core, &range) != UWB_ACCEPTED) return false;
        if (i < WARMUP_SAMPLES) continue;

        result->total_cycles += elapsed;
        if (elapsed > result->max_cycles) result->max_cycles = elapsed;
        ++result->samples;
        const UwbDecision *v1 = uwb_core_decision(&core, UWB_V1);
        const UwbDecision *adaptive = uwb_core_decision(&core, UWB_ADAPTIVE);
        result->v1_fast += v1->mode == UWB_FAST;
        result->v1_sigma_over_35 += v1->sigma_deg > 35;
        result->adaptive_cluster_holds += adaptive->direction == UWB_UNKNOWN &&
            (adaptive->reasons & UWB_REASON_CLUSTER) != 0;
    }
    result->mean_cycles = (uint32_t)(result->total_cycles / result->samples);
    return true;
}

void uwb_bench_g474_run(uint32_t cpu_hz)
{
    uwb_bench_g474_report = (UwbBenchG474Report){
        .status = UWB_BENCH_RUNNING, .cpu_hz = cpu_hz,
        .core_storage_bytes = sizeof(UwbCore), .result_bytes = sizeof(UwbTrackingResult)
    };
    if (!cpu_hz) {
        uwb_bench_g474_report.status = UWB_BENCH_BAD_CLOCK;
        return;
    }
    const uint32_t demcr = CoreDebug->DEMCR;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    const uint32_t control = DWT->CTRL;
    if (control & DWT_CTRL_NOCYCCNT_Msk) {
        CoreDebug->DEMCR = demcr;
        uwb_bench_g474_report.status = UWB_BENCH_NO_COUNTER;
        return;
    }
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    const uint32_t initial = cycles();
    for (unsigned i = 0; i < 64; ++i) __NOP();
    uint32_t status = UWB_BENCH_DONE;
    if (cycles() == initial) status = UWB_BENCH_NO_COUNTER;
    if (status == UWB_BENCH_DONE) {
        uint32_t overhead = UINT32_MAX;
        for (unsigned i = 0; i < 64; ++i) {
            const uint32_t start = cycles();
            const uint32_t elapsed = cycles() - start;
            if (elapsed < overhead) overhead = elapsed;
        }
        uwb_bench_g474_report.timer_overhead_min_cycles = overhead;
        for (unsigned i = 0; i < UWB_BENCH_CASE_COUNT; ++i) {
            UwbBenchCaseResult result = {0};
            if (!run_case(i, &result)) {
                status = UWB_BENCH_CORE_REJECTED;
                break;
            }
            uwb_bench_g474_report.cases[i] = result;
            const bool expected = i == UWB_BENCH_STABLE ? result.v1_fast == result.samples :
                i == UWB_BENCH_DISPERSED ? result.v1_sigma_over_35 == result.samples && result.adaptive_cluster_holds == 0 :
                result.adaptive_cluster_holds == result.samples;
            if (!expected) status = UWB_BENCH_WRONG_CASE;
        }
    }
    DWT->CTRL = control;
    CoreDebug->DEMCR = demcr;
    uwb_bench_g474_report.status = status;
}
