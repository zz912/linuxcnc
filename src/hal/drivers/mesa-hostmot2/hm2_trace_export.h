#ifndef HM2_TRACE_EXPORT_H
#define HM2_TRACE_EXPORT_H

struct hm2_trace;
struct hm2_trace_trigger;

int hm2_trace_export_init(int comp_id);
void hm2_trace_export_cleanup(void);

int hm2_trace_export(
    struct hm2_trace *trace,
    const struct hm2_trace_trigger *trigger);

#endif
