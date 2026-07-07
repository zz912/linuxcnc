#include "hm2_trace_trigger.h"

int hm2_trace_trigger_init(struct hm2_trace_trigger *trigger)
{
    if (!trigger) {
        return -1;
    }

    trigger->config.servo_threshold_pct = 70;
    trigger->config.nic_threshold_pct = 50;

    trigger->freeze_requested = false;
    trigger->frozen = false;

    trigger->reason = HM2_TRACE_TRIGGER_NONE;

    return 0;
}

void hm2_trace_trigger_cleanup(struct hm2_trace_trigger *trigger)
{
    (void)trigger;
}
