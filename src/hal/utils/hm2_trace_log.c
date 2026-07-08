#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include "rtapi.h"
#include "hal.h"

#include "../drivers/mesa-hostmot2/hm2_trace_export.h"

static volatile int running = 1;

static void signal_handler(int sig)
{
    (void)sig;
    running = 0;
}

static FILE *open_log_file(void)
{
    char path[512];
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
        sizeof(path),
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
        fprintf(stderr, "hal_init failed\n");
        return 1;
    }

    shmem_id = rtapi_shmem_new(
        HM2_TRACE_SHMEM_KEY,
        comp_id,
        sizeof(struct hm2_trace_shmem));

    if (shmem_id < 0) {
        fprintf(stderr, "rtapi_shmem_new failed\n");
        hal_exit(comp_id);
        return 1;
    }

    if (rtapi_shmem_getptr(
            shmem_id,
            (void **)&shmem) < 0) {

        fprintf(stderr, "rtapi_shmem_getptr failed\n");
        hal_exit(comp_id);
        return 1;
    }

    hal_ready(comp_id);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    uint32_t last_generation = shmem->generation;

    while (running) {

        if (shmem->generation == last_generation) {
            usleep(100000);
            continue;
        }

        last_generation = shmem->generation;

        FILE *log;

        log = open_log_file();
        if (log == NULL) {
            fprintf(stderr, "cannot create log file\n");
            break;
        }

        fprintf(log, "HM2 TRACE\n\n");

        fprintf(log, "version           : %u\n", shmem->version);
        fprintf(log, "generation        : %u\n", shmem->generation);

        fprintf(log, "trigger_reason    : %u\n", shmem->trigger_reason);
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

        fclose(log);

        printf("Trace written.\n");

    }

    hal_exit(comp_id);

    return 0;
}
