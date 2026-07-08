#include <stdio.h>
#include <stdlib.h>

#include "rtapi.h"
#include "hal.h"

#include "../drivers/mesa-hostmot2/hm2_trace_export.h"

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

    printf("HM2 trace logger attached.\n");
    printf("version    = %u\n", shmem->version);
    printf("valid      = %u\n", shmem->valid);
    printf("generation = %u\n", shmem->generation);
    printf("ring_size  = %u\n", shmem->ring_size);
    printf("ring_head  = %u\n", shmem->ring_head);

    getchar();

    hal_exit(comp_id);

    return 0;
}
