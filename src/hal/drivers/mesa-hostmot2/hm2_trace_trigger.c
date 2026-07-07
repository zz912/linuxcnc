#include "hm2_trace_trigger.h"
#include "hm2_trace.h"

int hm2_trace_trigger_init(struct hm2_trace_trigger *trigger)
{
    if (!trigger) {
        return -1;
    }

    trigger->config.latency_threshold_pct = 10;

    trigger->freeze_requested = false;
    trigger->frozen = false;

    trigger->reason = HM2_TRACE_TRIGGER_NONE;
    trigger->ratio_pct = 0;

    return 0;
}

void hm2_trace_trigger_cleanup(struct hm2_trace_trigger *trigger)
{
    (void)trigger;
}

void hm2_trace_trigger_eval(
    struct hm2_trace_trigger *trigger,
    const struct hm2_trace *trace,
    uint32_t servo_period_ns)
{
    uint32_t ratio;

    if (servo_period_ns == 0)
        return;

    ratio = (trace->total_tmax_ns * 100) / servo_period_ns;

    trigger->ratio_pct = ratio;

    if (ratio >= trigger->config.latency_threshold_pct) {
        trigger->freeze_requested = true;
        trigger->reason = HM2_TRACE_TRIGGER_LATENCY;
    } else {
        trigger->freeze_requested = false;
        trigger->reason = HM2_TRACE_TRIGGER_NONE;
    }
}
