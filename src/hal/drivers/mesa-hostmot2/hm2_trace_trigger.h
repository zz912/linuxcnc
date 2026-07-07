#ifndef HM2_TRACE_TRIGGER_H
#define HM2_TRACE_TRIGGER_H

#include <stdbool.h>

enum hm2_trace_trigger_reason {
    HM2_TRACE_TRIGGER_NONE = 0,
    HM2_TRACE_TRIGGER_SERVO,
    HM2_TRACE_TRIGGER_NIC,
};

struct hm2_trace_trigger_config {
    unsigned int servo_threshold_pct;
    unsigned int nic_threshold_pct;
};

struct hm2_trace_trigger {
    struct hm2_trace_trigger_config config;

    bool freeze_requested;
    bool frozen;

    enum hm2_trace_trigger_reason reason;
};

int hm2_trace_trigger_init(struct hm2_trace_trigger *trigger);
void hm2_trace_trigger_cleanup(struct hm2_trace_trigger *trigger);

#endif
