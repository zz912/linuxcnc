#include <errno.h>

#include <rtapi_slab.h>

#include "hm2_trace.h"

int hm2_trace_init(struct hm2_trace *trace)
{
    trace->enabled = 0;
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
