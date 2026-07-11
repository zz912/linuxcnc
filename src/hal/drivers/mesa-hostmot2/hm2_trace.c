/*
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * HostMot2 Trace Infrastructure
 *
 * This file is part of the HostMot2 trace infrastructure.
 *
 * The HostMot2 tracer is a generic realtime diagnostic framework
 * for collecting timestamped trace events, runtime statistics and
 * frozen snapshots for offline analysis.
 *
 * Architecture and design documentation:
 *     hm2_tracer-readme.md
 *
 * Copyright (C) 2026 zz912
 *
 * Originally developed by zz912 with implementation assistance
 * from OpenAI ChatGPT.
 */

#include <errno.h>

#include <rtapi.h>
#include <rtapi_slab.h>

#include "hm2_trace.h"

int hm2_trace_init(struct hm2_trace *trace)
{
    trace->enabled = 1;
    trace->frozen = 0;

    trace->trigger_active = 0;
    trace->export_pending = 0;
    trace->trigger_index = 0;
    trace->remaining_after_trigger = 0;

    trace->head = 0;
    trace->trigger_head = 0;
    trace->size = HM2_TRACE_RING_SIZE;
    trace->samples_written = 0;

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

static inline rtapi_s64 hm2_trace_timestamp(void)
{
    return rtapi_get_time();
}

void hm2_trace_log(
    struct hm2_trace *trace,
    enum hm2_trace_event event)
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

    if (trace->trigger_active) {

        if (trace->remaining_after_trigger ==
            HM2_TRACE_RING_SIZE / 2)
            trace->trigger_index =
                (trace->head + trace->size - 1) % trace->size;

        if (trace->remaining_after_trigger > 0) {
            trace->remaining_after_trigger--;

            if (trace->remaining_after_trigger == 0){
                trace->frozen = 1;
                trace->export_pending = 1;
            }
        }
    }

    trace->head++;

    trace->samples_written++;

    if (trace->head >= trace->size)
        trace->head = 0;
}
