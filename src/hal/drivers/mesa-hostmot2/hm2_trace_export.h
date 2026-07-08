#ifndef HM2_TRACE_EXPORT_H
#define HM2_TRACE_EXPORT_H

#define HM2_TRACE_SHMEM_KEY 0x130CF407

#include <stdint.h>

#include "hm2_trace.h"

struct hm2_trace;
struct hm2_trace_trigger;

struct hm2_trace_shmem {
    uint32_t version;
    uint32_t generation;

    uint32_t ring_head;
    uint32_t ring_size;
    uint64_t samples_written;

    uint32_t trigger_reason;
    uint32_t trigger_ratio_pct;

    uint32_t read_runtime_ns;
    uint32_t write_runtime_ns;
    uint32_t total_runtime_ns;

    uint32_t read_tmax_ns;
    uint32_t write_tmax_ns;
    uint32_t total_tmax_ns;

    struct hm2_trace_entry ring[HM2_TRACE_RING_SIZE];
};

int hm2_trace_export_init(int comp_id);
void hm2_trace_export_cleanup(void);

int hm2_trace_export(
    struct hm2_trace *trace,
    const struct hm2_trace_trigger *trigger);

#endif
