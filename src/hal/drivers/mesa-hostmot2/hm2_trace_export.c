#include "hm2_trace_export.h"

#include "hm2_trace.h"

#include <rtapi.h>

static int export_comp_id;

static int export_shmem_id = -1;

static void *export_shmem = NULL;

#define HM2_TRACE_SHMEM_KEY  0x484D3254  /* "HM2T" */

struct hm2_trace_shmem {
    uint32_t version;
    uint32_t valid;

    uint32_t read_runtime_ns;
    uint32_t write_runtime_ns;
    uint32_t total_runtime_ns;

    uint32_t read_tmax_ns;
    uint32_t write_tmax_ns;
    uint32_t total_tmax_ns;

};

int hm2_trace_export_init(int comp_id)
{
    export_comp_id = comp_id;

    export_shmem_id = rtapi_shmem_new(
        HM2_TRACE_SHMEM_KEY,
        export_comp_id,
        sizeof(struct hm2_trace_shmem));

    if (export_shmem_id < 0)
        return export_shmem_id;

    if (rtapi_shmem_getptr(export_shmem_id, &export_shmem) < 0)
        return -1;

    ((struct hm2_trace_shmem *)export_shmem)->version = 1;
    ((struct hm2_trace_shmem *)export_shmem)->valid = 0;

    return 0;
 }

void hm2_trace_export_cleanup(void)
{
    if (export_shmem_id >= 0)
        rtapi_shmem_delete(export_shmem_id, export_comp_id);

    export_shmem = NULL;
    export_shmem_id = -1;
}

int hm2_trace_export(struct hm2_trace *trace)
{
    struct hm2_trace_shmem *shmem = export_shmem;

    if (shmem == NULL)
        return -1;

    shmem->read_runtime_ns = trace->read_runtime_ns;
    shmem->write_runtime_ns = trace->write_runtime_ns;
    shmem->total_runtime_ns = trace->total_runtime_ns;

    shmem->read_tmax_ns = trace->read_tmax_ns;
    shmem->write_tmax_ns = trace->write_tmax_ns;
    shmem->total_tmax_ns = trace->total_tmax_ns;

    shmem->valid = 1;

    return 0;
}
