#include "hm2_trace_export.h"

#include "hm2_trace.h"

#include <rtapi.h>

static int export_comp_id;

int hm2_trace_export_init(int comp_id)
{
    export_comp_id = comp_id;

    return 0;
 }

void hm2_trace_export_cleanup(void)
{

}

int hm2_trace_export(struct hm2_trace *trace)
{
    (void)trace;

    return 0;
}
