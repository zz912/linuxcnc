# HostMot2 Trace Infrastructure

## Purpose

This document describes the design of the HostMot2 trace
infrastructure.

It is intended for developers maintaining or extending the
tracer. The document describes the intended architecture,
design rules and communication between the realtime and
userspace components.

The document is maintained together with the source code and
should always describe the intended behaviour of the tracer.


---

# 1. Motivation

The HostMot2 tracer was originally created to investigate
occasional latency spikes in the HostMot2 Ethernet driver.

During development it evolved from a simple latency profiler
into a generic realtime tracing infrastructure.

The tracer is intended to become a permanent part of HostMot2
and provide a reusable diagnostic framework for future
development and debugging.


---

# 2. Design Goals

The tracer is designed around the following principles.

* Generic diagnostic infrastructure.
* Minimal impact on realtime execution.
* No dynamic memory allocation while tracing.
* Lock-free communication between realtime and userspace.
* Snapshot-based operation.
* Simple extension with additional tracepoints.
* Simple extension with additional trigger conditions.
* Human-readable log output.


---

# 3. Architecture

```
                    REALTIME

                  HostMot2
                     │
     ┌───────────────┼────────────────┐
     │               │                │
     ▼               ▼                ▼

 HM2_TRACE()   hm2_trigger_eval()   hm2_trace_export()
     │               ▲                ▲
     │               │                │
     ▼               │                │
 hm2_trace ──────────┘────────────────┘
          │
          └──── supplies data

                     │
                     ▼

                Shared Memory

================ USERSPACE =================

              hm2_trace_log

                     │
                     ▼

                Trace Log File
```

Each layer has exactly one responsibility.

| Component | Responsibility |
|-----------|----------------|
| Tracepoints | Mark interesting execution points inside the HostMot2 driver. |
| hm2_trace | Collect trace events and runtime statistics. |
| hm2_trigger | Decide when a snapshot should be captured. |
| hm2_trace_export | Export a frozen snapshot into shared memory. |
| hm2_trace_log | Convert a snapshot into a human-readable log file. |


---

# 4. Design Rules

The tracer follows the rules below.

Realtime components:

* collect trace information
* update runtime statistics
* evaluate trigger conditions
* export snapshots
* write shared memory
* never read shared memory

Userspace components:

* read shared memory
* generate log files
* never modify shared memory

General rules:

* Shared memory is the only communication channel between
  realtime and userspace.
* Each layer has exactly one responsibility.
* Trace collection is independent of trigger evaluation.
* Trigger evaluation is independent of snapshot export.
* Snapshot export is independent of log formatting.
* Log formatting is performed entirely in userspace.


---

# 5. Snapshot

A snapshot represents a frozen image of the tracer state at the
moment a trigger condition becomes true.

A snapshot contains:

* snapshot metadata
* trigger information
* runtime statistics
* runtime maxima
* ring buffer metadata
* complete trace ring buffer

The logger operates exclusively on snapshots.

It never accesses internal HostMot2 structures.


---

# 6. Trace Events

Trace events are timestamped records stored inside the realtime
ring buffer.

Each event contains:

* timestamp
* event identifier

The event identifier defines the meaning of the tracepoint.

The userspace logger converts event identifiers into readable
text.


---

# 7. Trigger

The trigger continuously evaluates the collected runtime
information.

When a trigger condition becomes true, the current tracer state
is frozen and exported as a snapshot.

The trigger mechanism is independent of the exported data and
may support multiple trigger conditions.


---

# 8. Shared Memory

Shared memory provides the interface between realtime and
userspace.

Its purpose is to transfer a complete snapshot without exposing
internal HostMot2 data structures.

The shared memory layout defines the public interface between
the realtime tracer and the userspace logger.


---

# 9. Userspace Logger

The logger waits for new snapshots.

Whenever a new snapshot becomes available it:

* reads shared memory
* creates a log file
* converts event identifiers into text
* generates relative timestamps
* formats a human-readable report

The logger never modifies snapshot data.


The generated event table contains the following columns.

| Column | Description |
|--------|-------------|
| idx | Ring buffer index. |
| timestamp | Absolute realtime timestamp stored in the trace event. |
| snapshot(ns) | Time elapsed from the first event contained in the exported snapshot. |
| cycle(ns) | Time elapsed from the most recent `CYCLE_START` event. Events preceding the first `CYCLE_START` are reported as `n/a`. |
| event(ns) | Time elapsed from the previous trace event. The first event of the snapshot is reported as `n/a`. |
| event | Human-readable event name. |

+The userspace logger inserts an empty line before every
+`CYCLE_START` event to improve readability of individual
+servo cycles.

---

# 10. Extending the Tracer

The tracer is intended to grow without changing its overall
architecture.

Examples of future extensions include:

* additional tracepoints
* additional tracepoint categories
* additional trigger conditions
* additional exported statistics
* additional userspace analysis tools



---

# TODO

Tato kapitola slouží pouze během vývoje. Před začleněním traceru
do hlavní větve musí být prázdná.

* vyřešit posunutí dat z ring bufferu vůči snapshotu
* zvýraznit šipkou data v ring bufferu, která odpovídají okamžiku snapshotu
* zvážit změnu architektury s ohledem na předchozí bod – orchestrátor by měl být `hm2_trace.c`, nikoliv `hostmot2.c`
* přidat do `loadrt hm2_eth` konfigurační volbu pro aktivaci traceru (existuje příprava `HM2_TRACE_DISABLE`)
* doplnit uživatelskou dokumentaci HostMot2 (`*.adoc`) o postup spuštění loggeru (`loadusr hm2-trace-log`)