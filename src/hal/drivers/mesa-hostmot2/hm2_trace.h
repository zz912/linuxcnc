#ifndef HM2_TRACE_H
#define HM2_TRACE_H

#include <rtapi_stdint.h>

#define HM2_TRACE_RING_SIZE 4096

struct hm2_trace_entry {
    rtapi_s64 timestamp;
    rtapi_u16 event;
    rtapi_u16 cpu;
    rtapi_u32 value;
};

struct hm2_trace {
    int enabled;
    int frozen;

    rtapi_u32 head;
    rtapi_u32 size;

    struct hm2_trace_entry *ring;
};

int hm2_trace_init(struct hm2_trace *trace);
void hm2_trace_cleanup(struct hm2_trace *trace);

#endif
