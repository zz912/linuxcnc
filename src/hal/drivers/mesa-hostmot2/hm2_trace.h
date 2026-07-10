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

#ifndef HM2_TRACE_H
#define HM2_TRACE_H

#include <rtapi_stdint.h>

#define HM2_TRACE_RING_SIZE 4096

enum hm2_trace_event {
    HM2_TRACE_CYCLE_START = 0, // event for calculate start cycle

    HM2_TRACE_READ_ENTER,
    HM2_TRACE_READ_EXIT,

    HM2_TRACE_RECV_ENTER,
    HM2_TRACE_RECV_EXIT,

   HM2_TRACE_WRITE_ENTER,
   HM2_TRACE_WRITE_EXIT,
};

struct hm2_trace_entry {
    rtapi_s64 timestamp;
    rtapi_u16 event;
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

void hm2_trace_log(struct hm2_trace *trace,
                   enum hm2_trace_event event);

void hm2_trace_dump(const struct hm2_trace *trace);

#ifdef HM2_TRACE_DISABLE

#define HM2_TRACE(trace, event) \
    do { } while (0)

#else

#define HM2_TRACE(trace, event) \
    hm2_trace_log((trace), (event))

#endif

#endif
