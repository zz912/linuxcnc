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

 void hm2_trace_dump(const struct hm2_trace *trace)
 {
     rtapi_u32 i;

     if (!trace || !trace->ring)
         return;

     rtapi_print("HM2 trace dump (head=%u size=%u)\n",
                 trace->head,
                 trace->size);

     for (i = 0; i < 32 && i < trace->size; i++) {
         const struct hm2_trace_entry *e = &trace->ring[i];

         rtapi_print("[%4u] ts=%lld event=%u cpu=%u value=%u\n",
                     i,
                     (long long)e->timestamp,
                     e->event,
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

    if (!trace->enabled)
        return;

    if (trace->frozen)
        return;

    entry = &trace->ring[trace->head];

    entry->timestamp = hm2_trace_timestamp();
    entry->event = event;
    entry->cpu = 0;
    entry->value = value;

    trace->head++;

    if (trace->head >= trace->size)
        trace->head = 0;
}
