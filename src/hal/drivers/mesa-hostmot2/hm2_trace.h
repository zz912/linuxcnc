#ifndef HM2_TRACE_H
#define HM2_TRACE_H

#include <rtapi_stdint.h>

#define HM2_TRACE_RING_SIZE 4096

enum hm2_trace_event {
    HM2_TRACE_READ_ENTER = 0,
    HM2_TRACE_READ_EXIT,

    HM2_TRACE_RECV_ENTER,
    HM2_TRACE_RECV_EXIT,

   HM2_TRACE_WRITE_ENTER,
   HM2_TRACE_WRITE_EXIT,
};

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
    rtapi_u64 samples_written;

    struct hm2_trace_entry *ring;

    uint64_t read_start_ns;
    uint64_t write_start_ns;

    uint32_t total_runtime_ns;
    uint32_t total_tmax_ns;

    uint32_t read_runtime_ns;
    uint32_t write_runtime_ns;

    uint32_t read_tmax_ns;
    uint32_t write_tmax_ns;
};

int hm2_trace_init(struct hm2_trace *trace);
void hm2_trace_cleanup(struct hm2_trace *trace);

void hm2_trace_log(
    struct hm2_trace *trace,
    enum hm2_trace_event event,
    rtapi_u32 value);

void hm2_trace_dump(const struct hm2_trace *trace);

#ifdef HM2_TRACE_DISABLE

#define HM2_TRACE(trace, event, value) \
    do { } while (0)

#else

#define HM2_TRACE(trace, event, value) \
    hm2_trace_log((trace), (event), (value))

#endif

#endif
