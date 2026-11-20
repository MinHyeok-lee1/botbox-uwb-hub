#ifndef UWB_BENCH_G474_H
#define UWB_BENCH_G474_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { UWB_BENCH_STABLE, UWB_BENCH_DISPERSED, UWB_BENCH_AMBIGUOUS, UWB_BENCH_CASE_COUNT };
enum { UWB_BENCH_IDLE, UWB_BENCH_RUNNING, UWB_BENCH_DONE, UWB_BENCH_NO_COUNTER,
       UWB_BENCH_BAD_CLOCK, UWB_BENCH_CORE_REJECTED, UWB_BENCH_WRONG_CASE };

typedef struct {
    uint64_t total_cycles;
    uint32_t samples, mean_cycles, max_cycles;
    uint32_t v1_fast, v1_sigma_over_35, adaptive_cluster_holds;
} UwbBenchCaseResult;

typedef struct {
    uint32_t status, cpu_hz, timer_overhead_min_cycles, core_storage_bytes, result_bytes;
    UwbBenchCaseResult cases[UWB_BENCH_CASE_COUNT];
} UwbBenchG474Report;

/* Inspect after status leaves RUNNING. cpu_hz is the configured core clock. */
extern volatile UwbBenchG474Report uwb_bench_g474_report;
void uwb_bench_g474_run(uint32_t cpu_hz);

#ifdef __cplusplus
}
#endif
#endif
