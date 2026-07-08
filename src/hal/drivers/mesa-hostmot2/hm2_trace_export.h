#ifndef HM2_TRACE_EXPORT_H
#define HM2_TRACE_EXPORT_H

struct hm2_trace;

int hm2_trace_export_init(int comp_id);
void hm2_trace_export_cleanup(void);

/*
 * Export a frozen trace capture.
 */
int hm2_trace_export(struct hm2_trace *trace);

#endif
