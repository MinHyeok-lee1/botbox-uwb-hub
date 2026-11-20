/* Independent link-time size probe. It is never a board firmware target. */
#include "uwb_core.h"
#include <stddef.h>

static UwbCore instance;
static UwbTrackingResult result;
static volatile int keep;

int main(void)
{
    const UwbConfig config = uwb_research_config();
    const UwbObservation observation = {.center_valid = true, .pdoa_valid = true, .sts_q = 250};
    const UwbRangeObservation range = {.valid = true};
    keep = uwb_core_init(&instance, &config);
    keep = uwb_core_observe(&instance, &observation);
    keep = uwb_core_observe_range(&instance, &range);
    keep = uwb_core_decision(&instance, UWB_V1) != NULL;
    keep = uwb_core_read(&instance, UWB_V3, 251, &result);
    for (;;) {}
}
