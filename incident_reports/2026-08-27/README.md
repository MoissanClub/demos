# 2026-08-27 incident artifacts

This directory accompanies
[`INCIDENT_REPORT_2026-08-27_ARM_SDK_HANDOFF_RECURRENCE.md`](../INCIDENT_REPORT_2026-08-27_ARM_SDK_HANDOFF_RECURRENCE.md).

## Exact source provenance

The incident implementation remains in the working tree. The two responsible
files at the time these artifacts were assembled have these SHA-256 values:

```text
4c0e70178f62fe67368348a2bf84be538fe23248d69a17572f29c917ad7b590d  g1_standalone_arm_sequence.py
82499550f5d86e4fc1937d265270654209e08b6c2cec30c1f9ad5ffebd3e2923  handshake/standalone_arm.py
```

They were uncommitted at incident time, so the hashes—not the mutable original
paths alone—are the source identity. The complete locally authored runtime,
support, documentation, and test files have now been copied byte-for-byte under
`code/`. `MANIFEST.sha256` verifies every snapshot. These are archival incident
artifacts and must not be executed against a robot.

`control_flow.txt` also contains the exact relevant excerpts with irrelevant
setup removed for quicker review.

External dependencies are identified in `SOURCE_PROVENANCE.md`; they are not
duplicated here because their Git revisions remain available in their own
repositories.

## Raw evidence

```text
telemetry/standalone_arm/sequence_20260826T233757Z.jsonl
telemetry/standalone_arm/sequence_20260826T233832Z.jsonl
```

Raw-file SHA-256 values are recorded in the incident report. The two TSV files
contain compact incident-window traces. Regenerate generic event traces with:

```bash
python incident_reports/extract_incident_trace.py \
  telemetry/standalone_arm/sequence_20260826T233757Z.jsonl

python incident_reports/extract_incident_trace.py \
  telemetry/standalone_arm/sequence_20260826T233832Z.jsonl
```
