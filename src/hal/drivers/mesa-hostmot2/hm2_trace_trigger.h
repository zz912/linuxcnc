#ifndef HM2_TRACE_TRIGGER_H
#define HM2_TRACE_TRIGGER_H

#include <stdbool.h>
#include <stdint.h>

struct hm2_trace;

enum hm2_trace_trigger_reason {
    HM2_TRACE_TRIGGER_NONE = 0,
    HM2_TRACE_TRIGGER_LATENCY,
};

struct hm2_trace_trigger_config {
    unsigned int latency_threshold_pct;
};


struct hm2_trace_trigger {
    struct hm2_trace_trigger_config config;

    bool freeze_requested;

    enum hm2_trace_trigger_reason reason;

    uint32_t ratio_pct;
};

int hm2_trace_trigger_init(struct hm2_trace_trigger *trigger);
void hm2_trace_trigger_cleanup(struct hm2_trace_trigger *trigger);

void hm2_trace_trigger_eval(
    struct hm2_trace_trigger *trigger,
    const struct hm2_trace *trace,
    uint32_t servo_period_ns);

#endif
