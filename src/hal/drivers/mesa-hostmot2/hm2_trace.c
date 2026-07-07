#include <errno.h>

#include <rtapi.h>
#include <rtapi_slab.h>

#include "hm2_trace.h"

int hm2_trace_init(struct hm2_trace *trace)
{
    trace->enabled = 1;
    trace->frozen = 0;

    trace->head = 0;
    trace->size = HM2_TRACE_RING_SIZE;

    trace->ring = rtapi_kzalloc(
        sizeof(struct hm2_trace_entry) * trace->size,
        RTAPI_GFP_KERNEL);

    if (trace->ring == NULL)
        return -ENOMEM;

    return 0;
}

void hm2_trace_cleanup(struct hm2_trace *trace)
{
    if (trace->ring != NULL) {
        rtapi_kfree(trace->ring);
        trace->ring = NULL;
    }
}

static const char *hm2_trace_event_name(enum hm2_trace_event event)
{
    switch (event) {

    case HM2_TRACE_READ_ENTER:
        return "READ_ENTER";

    case HM2_TRACE_READ_EXIT:
        return "READ_EXIT";

    case HM2_TRACE_RECV_ENTER:
        return "RECV_ENTER";

    case HM2_TRACE_RECV_EXIT:
        return "RECV_EXIT";

    case HM2_TRACE_WRITE_ENTER:
        return "WRITE_ENTER";

    case HM2_TRACE_WRITE_EXIT:
        return "WRITE_EXIT";

    default:
        return "UNKNOWN";
    }
}

void hm2_trace_dump(const struct hm2_trace *trace)
{
    rtapi_u32 i;

    if (!trace || !trace->ring)
        return;

    rtapi_print(
        "HM2 trace:\n"
        "  head=%u size=%u\n"
        "  read_runtime=%u ns    read_tmax=%u ns\n"
        "  write_runtime=%u ns   write_tmax=%u ns\n"
        "  total_runtime=%u ns   total_tmax=%u ns\n",
        trace->head,
        trace->size,
        trace->read_runtime_ns,
        trace->read_tmax_ns,
        trace->write_runtime_ns,
        trace->write_tmax_ns,
        trace->total_runtime_ns,
        trace->total_tmax_ns);

    rtapi_print("Ring buffer:\n");

    for (i = 0; i < 32 && i < trace->size; i++) {
        const struct hm2_trace_entry *e = &trace->ring[i];

        rtapi_print("[%4u] ts=%lld %-12s cpu=%u value=%u\n",
                    i,
                    (long long)e->timestamp,
                    hm2_trace_event_name(e->event),
                    e->cpu,
                    e->value);
    }
}

static inline rtapi_s64 hm2_trace_timestamp(void)
{
    return rtapi_get_time();
}

void hm2_trace_log(
    struct hm2_trace *trace,
    enum hm2_trace_event event,
    rtapi_u32 value)
{
    struct hm2_trace_entry *entry;
    uint64_t now;
    uint64_t runtime;

    if (!trace->enabled)
        return;

    if (trace->frozen)
        return;

    now = hm2_trace_timestamp();

    switch (event) {

    case HM2_TRACE_READ_ENTER:
        trace->read_start_ns = now;
        break;

    case HM2_TRACE_READ_EXIT:
        runtime = now - trace->read_start_ns;

        trace->read_runtime_ns = (uint32_t)runtime;

        if (trace->read_runtime_ns > trace->read_tmax_ns)
            trace->read_tmax_ns = trace->read_runtime_ns;

        break;

    case HM2_TRACE_WRITE_ENTER:
        trace->write_start_ns = now;
        break;

    case HM2_TRACE_WRITE_EXIT:
        runtime = now - trace->write_start_ns;

        trace->write_runtime_ns = (uint32_t)runtime;

        if (trace->write_runtime_ns > trace->write_tmax_ns)
            trace->write_tmax_ns = trace->write_runtime_ns;

        trace->total_runtime_ns =
            trace->read_runtime_ns + trace->write_runtime_ns;

        if (trace->total_runtime_ns > trace->total_tmax_ns)
            trace->total_tmax_ns = trace->total_runtime_ns;

        break;

    default:
        break;
    }

    entry = &trace->ring[trace->head];

    entry->timestamp = now;
    entry->event = event;
    entry->cpu = 0;
    entry->value = value;

    trace->head++;

    if (trace->head >= trace->size)
        trace->head = 0;
}
