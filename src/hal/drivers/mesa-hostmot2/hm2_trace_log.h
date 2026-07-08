#ifndef HM2_TRACE_LOG_H
#define HM2_TRACE_LOG_H

struct hm2_trace;

int hm2_trace_log_init(void);
void hm2_trace_log_cleanup(void);

void hm2_trace_log_write(
    const struct hm2_trace *trace);

#endif
