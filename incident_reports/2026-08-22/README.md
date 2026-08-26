# 2026-08-22 incident artifacts

This directory accompanies
[`INCIDENT_REPORT_2026-08-22_ARM_DROP.md`](../INCIDENT_REPORT_2026-08-22_ARM_DROP.md).

## Raw evidence

The raw trajectory remains outside Git because it is 21 MB and contains
high-volume hardware telemetry:

```text
telemetry/trajectories/20260821T164545Z/
trajectory_20260821T164602.042250Z_39104975-0ad0-4c26-b246-4ebc69346862.jsonl
```

`trace.tsv` is a compact extract of its incident events.

## Source-code provenance limitation

The unsafe fine-movement executor was working-tree code at the time of the
incident and was never committed. It is not present in the current tree,
reachable Git history, reflog, or recoverable unreachable blobs. The latest
committed controller before the incident was `abd6880`, but that commit does
not contain the fine-movement path and must not be represented as the incident
implementation.

`control_flow.txt` is therefore explicitly a non-executable reconstruction
from the raw events and the incident analysis, not an exact source snapshot.
This distinction preserves the evidence boundary instead of inventing code.

Regenerate the generic event trace with:

```bash
python incident_reports/extract_incident_trace.py \
  telemetry/trajectories/20260821T164545Z/trajectory_20260821T164602.042250Z_39104975-0ad0-4c26-b246-4ebc69346862.jsonl
```
