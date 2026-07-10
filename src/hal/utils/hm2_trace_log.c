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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <stdint.h>
#include <limits.h>

#include "rtapi.h"
#include "hal.h"

#include "../drivers/mesa-hostmot2/hm2_trace_export.h"
#include "../drivers/mesa-hostmot2/hm2_trace_trigger.h"
 
static const char *hm2_trace_event_name(uint16_t event)
{
    switch (event) {
    
    case HM2_TRACE_CYCLE_START:  // event for calculate start cycle
        return "CYCLE_START";

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

static const char *hm2_trace_trigger_name(uint32_t reason)
{
    switch (reason) {

    case HM2_TRACE_TRIGGER_NONE:
        return "NONE";

    case HM2_TRACE_TRIGGER_LATENCY:
        return "LATENCY";

    default:
        return "UNKNOWN";
    }
}

#define HM2_TRACE_INFO(fmt, ...) \
    printf("[HM2.TRACE][INFO] " fmt "\n", ##__VA_ARGS__)

#define HM2_TRACE_WARNING(fmt, ...) \
    fprintf(stderr, "[HM2.TRACE][WARNING] " fmt "\n", ##__VA_ARGS__)

#define HM2_TRACE_ERROR(fmt, ...) \
    fprintf(stderr, "[HM2.TRACE][ERROR] " fmt "\n", ##__VA_ARGS__)

static volatile int running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    running = 0;
}

static FILE *open_log_file(char *path, size_t path_size)
{
    char timestamp[64];
    time_t now;
    struct tm tm;
    const char *home;

    home = getenv("HOME");
    if (home == NULL)
        return NULL;

    now = time(NULL);
    localtime_r(&now, &tm);

    strftime(
        timestamp,
        sizeof(timestamp),
        "%Y_%m_%d-%H_%M_%S",
        &tm);

    snprintf(
        path,
        path_size,
        "%s/hm2_trace_%s.log",
        home,
        timestamp);

    return fopen(path, "w");
}

int main(void)
{
    int comp_id;
    int shmem_id;
    struct hm2_trace_shmem *shmem;

    comp_id = hal_init("hm2-trace-log");
    if (comp_id < 0) {
        HM2_TRACE_ERROR("hal_init failed");
        return 1;
    }

    shmem_id = rtapi_shmem_new(
        HM2_TRACE_SHMEM_KEY,
        comp_id,
        sizeof(struct hm2_trace_shmem));

    if (shmem_id < 0) {
        HM2_TRACE_ERROR("rtapi_shmem_new failed");
        hal_exit(comp_id);
        return 1;
    }

    if (rtapi_shmem_getptr(
            shmem_id,
            (void **)&shmem) < 0) {

        HM2_TRACE_ERROR("rtapi_shmem_getptr failed");
        hal_exit(comp_id);
        return 1;
    }

    hal_ready(comp_id);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    uint32_t last_generation = shmem->generation;

    HM2_TRACE_INFO("Waiting for trigger...");

    while (running) {

        if (last_generation != UINT32_MAX &&
            shmem->generation == last_generation) {
            usleep(100000);
            continue;
        }

        last_generation = shmem->generation;

        char path[512];
        FILE *log;

        log = open_log_file(path, sizeof(path));
        if (log == NULL) {
            HM2_TRACE_ERROR("Cannot create log file");
            break;
        }

        fprintf(log, "HM2 TRACE\n\n");

        fprintf(log, "version           : %u\n", shmem->version);
        fprintf(log, "generation        : %u\n", shmem->generation);

        fprintf(log, "trigger_reason    : %s\n",
                hm2_trace_trigger_name(shmem->trigger_reason));
        fprintf(log, "trigger_ratio     : %u %%\n", shmem->trigger_ratio_pct);

        fprintf(log, "\n");

        fprintf(log, "read_runtime_ns   : %u\n", shmem->read_runtime_ns);
        fprintf(log, "write_runtime_ns  : %u\n", shmem->write_runtime_ns);
        fprintf(log, "total_runtime_ns  : %u\n", shmem->total_runtime_ns);

        fprintf(log, "\n");

        fprintf(log, "read_tmax_ns      : %u\n", shmem->read_tmax_ns);
        fprintf(log, "write_tmax_ns     : %u\n", shmem->write_tmax_ns);
        fprintf(log, "total_tmax_ns     : %u\n", shmem->total_tmax_ns);

        fprintf(log, "\n");

        fprintf(log, "ring_size         : %u\n", shmem->ring_size);
        fprintf(log, "ring_head         : %u\n", shmem->ring_head);

        fprintf(log, "\n");
        fprintf(log, "Ring buffer:\n");
        fprintf(log, "------------\n");

        fprintf(log, " idx        timestamp    snapshot(ns)      cycle(ns)      event(ns)  event\n");
        fprintf(log, "---- ---------------- --------------- --------------- --------------- ------------\n");


        {
            uint32_t count;
            uint32_t start;
            int64_t base_timestamp = 0;
            int64_t cycle_timestamp = 0;
            int64_t previous_timestamp = 0;
            int cycle_start_found = 0;
            int first_event = 1;
            uint32_t i;

            if (shmem->samples_written < shmem->ring_size) {
                count = (uint32_t)shmem->samples_written;
                start = 0;
            } else {
                count = shmem->ring_size;
                start = shmem->ring_head;
            }

            if (count > 0)
                base_timestamp = shmem->ring[start].timestamp;

            for (i = 0; i < count; i++) {
                uint32_t idx = (start + i) % shmem->ring_size;
                const struct hm2_trace_entry *e = &shmem->ring[idx];

                if (e->event == HM2_TRACE_CYCLE_START) {
                    fprintf(log, "\n");
                    cycle_timestamp = e->timestamp;
                    cycle_start_found = 1;
                }

                fprintf(log,
                        "%4u %16lld %+15lld ",
                        idx,
                        (long long)e->timestamp,
                        (long long)(e->timestamp - base_timestamp));

                if (cycle_start_found) {
                    fprintf(log,
                            "%+15lld ",
                            (long long)(e->timestamp - cycle_timestamp));
                } else {
                    fprintf(log,
                            "%15s ",
                            "n/a");
                }

                if (first_event) {
                    fprintf(log,
                            "%15s ",
                            "n/a");
                } else {
                    fprintf(log,
                            "%+15lld ",
                            (long long)(e->timestamp -
                                        previous_timestamp));
                }

                fprintf(log,
                        "%-12s\n",
                        hm2_trace_event_name(e->event));

                previous_timestamp = e->timestamp;
                first_event = 0;
            }
        }

        fclose(log);

        HM2_TRACE_INFO("Trace written: %s", path);

    }

    hal_exit(comp_id);

    return 0;
}
