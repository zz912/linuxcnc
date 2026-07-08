#include "hm2_trace_export.h"

#include "hm2_trace.h"
#include "hm2_trace_trigger.h"

#include <rtapi.h>

#include <string.h>

static int export_comp_id;

static int export_shmem_id = -1;

static struct hm2_trace_shmem *export_shmem = NULL;

#define HM2_TRACE_EXPORT_VERSION 1

int hm2_trace_export_init(int comp_id)
{
    export_comp_id = comp_id;

    export_shmem_id = rtapi_shmem_new(
        HM2_TRACE_SHMEM_KEY,
        export_comp_id,
        sizeof(struct hm2_trace_shmem));

    if (export_shmem_id < 0)
        return export_shmem_id;

    if (rtapi_shmem_getptr(export_shmem_id, (void **)&export_shmem) < 0)
        return -1;

    export_shmem->version = HM2_TRACE_EXPORT_VERSION;
    export_shmem->valid = 0;
    export_shmem->generation = 0;

    memset(export_shmem->ring, 0, sizeof(export_shmem->ring));

    return 0;
 }

void hm2_trace_export_cleanup(void)
{
    if (export_shmem_id >= 0)
        rtapi_shmem_delete(export_shmem_id, export_comp_id);

    export_shmem = NULL;
    export_shmem_id = -1;
}

int hm2_trace_export(
    struct hm2_trace *trace,
    const struct hm2_trace_trigger *trigger)
{
    (void)trigger;

    if (export_shmem == NULL)
        return -1;

    export_shmem->generation++;

    export_shmem->read_runtime_ns = trace->read_runtime_ns;
    export_shmem->write_runtime_ns = trace->write_runtime_ns;
    export_shmem->total_runtime_ns = trace->total_runtime_ns;

    export_shmem->read_tmax_ns = trace->read_tmax_ns;
    export_shmem->write_tmax_ns = trace->write_tmax_ns;
    export_shmem->total_tmax_ns = trace->total_tmax_ns;

    export_shmem->ring_head = trace->head;
    export_shmem->ring_size = HM2_TRACE_RING_SIZE;

    memcpy(export_shmem->ring,
           trace->ring,
           sizeof(export_shmem->ring));

    /*
     * Publish data only after the shared memory image
     * has been fully updated.
     */

    export_shmem->trigger_reason = trigger->reason;
    export_shmem->trigger_ratio_pct = trigger->ratio_pct;

    export_shmem->valid = 1;
    export_shmem->generation++;

    return 0;
}
