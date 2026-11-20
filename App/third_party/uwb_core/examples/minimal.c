/* Standalone example with decoded and validated exchange data. */
#include "uwb_core.h"

#include <inttypes.h>
#include <stdio.h>

static UwbCore core;

static bool submit_exchange(const UwbObservation *observation, const UwbRangeObservation *range)
{
    if (uwb_core_observe(&core, observation) != UWB_ACCEPTED) return false;
    return uwb_core_observe_range(&core, range) == UWB_ACCEPTED;
}

static int fail(const char *message)
{
    fprintf(stderr, "%s\n", message);
    return 1;
}

int main(void)
{
    const UwbConfig config = uwb_research_config();
    UwbTrackingResult result;
    if (!uwb_core_init(&core, &config) || !uwb_core_method_enabled(&core, UWB_V3))
        return fail("초기화 실패");
    if (!uwb_core_read(&core, UWB_V3, 0, &result) ||
        result.has_observation || result.decision.mode != UWB_RECOVERY || result.tag_lost)
        return fail("최초 관측 대기 상태 확인 실패");

    /* The adapter has already extended the u8 SN and associated this exchange with CAL. */
    const unsigned count = 8;
    const uint32_t period_ms = 40;
    const uint32_t range_offset_ms = 7; /* Synthetic fixture, not a wire timing assumption. */
    for (unsigned i = 0; i < count; ++i) {
        const uint32_t tick = i * period_ms;
        const UwbObservation observation = {
            .arrival_ms = 1000u + tick + range_offset_ms,
            .tick_ms = tick, .sequence = 253u + i,
            .center_mdeg = 12000, .raw_pdoa = -1024, .sts_q = 250, .reason_flags = 0,
            .center_valid = true, .pdoa_valid = true
        };
        const UwbRangeObservation range = {
            .tick_ms = observation.tick_ms + range_offset_ms, .sequence = observation.sequence,
            .distance_mm = 2000, .valid = true
        };
        if (!submit_exchange(&observation, &range) ||
            !uwb_core_read(&core, UWB_V3, observation.arrival_ms, &result))
            return fail("관측 또는 동일 교환 거리 입력 실패");
        if (i == 0 && (!result.distance_valid || result.direction_valid ||
                      result.decision.mode != UWB_RECOVERY))
            return fail("초기 거리와 방향 유효성 확인 실패");
    }
    if (!result.distance_valid || result.distance_mm != 2000 || !result.direction_valid ||
        result.decision.direction != UWB_LEFT || result.tag_lost)
        return fail("안정 관측 결과 확인 실패");
    printf("seq=%" PRIu32 ", 거리=%" PRId32 " mm, 방향=LEFT, 상대 편차=%.3f deg\n",
           result.sequence, result.distance_mm, result.decision.deviation_deg);

    const uint64_t last_arrival_ms = result.last_arrival_ms;
    if (!uwb_core_read(&core, UWB_V3, last_arrival_ms + UWB_LOST_MS, &result) ||
        !result.direction_valid || result.tag_lost)
        return fail("250 ms 경계 확인 실패");
    if (!uwb_core_read(&core, UWB_V3, last_arrival_ms + UWB_LOST_MS + 1u, &result) ||
        !result.tag_lost || result.direction_valid || result.distance_valid ||
        result.decision.mode != UWB_LOST)
        return fail("수신 공백 처리 확인 실패");
    puts("251 ms 수신 공백에서 LOST 확인");

    /* A confirmed CAL generation change also resets history when the angle stays equal. */
    uwb_core_reset(&core);
    if (!uwb_core_read(&core, UWB_V3, last_arrival_ms + UWB_LOST_MS + 1u, &result) ||
        result.has_observation || result.direction_valid || result.distance_valid ||
        result.decision.mode != UWB_RECOVERY)
        return fail("센터 세대 변경 후 재획득 대기 확인 실패");
    return 0;
}
