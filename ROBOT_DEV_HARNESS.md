# Robot Development Evidence Harness

`robot_dev_harness` is a motion-agnostic development harness for synchronized
robot telemetry and video evidence. It intentionally contains no motor-command,
DDS publisher, handshake-policy, or robot-specific control code.

## Artifact layout

Each invocation creates a unique immutable run under:

```text
artifacts/robot_dev_runs/YYYYMMDDTHHMMSS.ffffffZ_<slug>/
  manifest.json
  telemetry/<stream>.jsonl
  video/camera_opencv_videoN_<run_id>.avi
  video/frame_timestamps.jsonl
  evidence/<event>_<timestamp>_<frame_index>.jpg
  verification.md
  checksums.sha256
```

`manifest.json` records the project, purpose, operator safety confirmation, Git
commit and dirty status, devices, command invocation, stream counts, run result,
and start/end times. Every telemetry row carries UTC and monotonic timestamps,
a per-stream sequence number, source, validity, and schema version.

The camera uses OpenCV's V4L2 backend and records MJPEG in an AVI container.
Each frame is timestamped on the host immediately after acquisition with the
same monotonic clock used for telemetry. `frame_timestamps.jsonl` maps those
timestamps to contiguous video frame indices. The negotiated and measured frame
rates are preserved separately.

Original telemetry and video are not modified after capture. Evidence frames
are derived before finalization and identify their exact source frame. Finalize
the run only after analysis artifacts are written; finalization flushes all
streams and computes SHA-256 checksums.

The artifact root is intentionally ignored by Git because video runs can be
large. Copy or synchronize completed run directories to the project's durable
artifact storage before reimaging PC2; the included checksums verify the copy.

## Read-only rehearsal

This command records `/dev/video6`, G1 low state, and sport-mode state without
constructing a robot command publisher:

```bash
/home/dwei/miniconda3/envs/g1brainco/bin/python \
  record_robot_dev_run.py \
  --project robot-development \
  --slug read-only-rehearsal \
  --purpose "Validate synchronized camera and telemetry capture" \
  --duration-seconds 5 \
  --network-interface eth0 \
  --camera-device /dev/video6 \
  --confirm-area-clear
```

The script verifies free disk space before capture, waits for the first camera
frame, records for a bounded duration, extracts baseline and final evidence
frames, and preserves partial results as `incomplete` if any component fails.

## Library interfaces

- `RunArtifacts` owns run creation, timestamped JSONL streams, manifests,
  lifecycle state, and checksums.
- `OpenCVMjpegCamera` owns V4L2 capture, per-frame host timestamps, and measured
  cadence.
- `extract_nearest_frame` maps a telemetry event's `monotonic_ns` to the closest
  video frame and reports the signed timing offset.
- `format_visual_review` renders verification findings only when every finding
  includes one or more exact synchronized frame indices and UTC timestamps.
- `LegacyTelemetryAdapter` lets existing repository subscribers write into the
  generic run schema without changing their control behavior.
- `EvidenceSession` coordinates telemetry-source and camera readiness, records
  lifecycle events, and finalizes partial runs without containing motion code.
- `EvidenceBackedCommandTransport` queues command intent before invoking a
  separately reviewed project transport exactly once; it refuses to invoke the
  transport when evidence capture is not ready and records transport failures.

Future projects should add telemetry sources through `RunArtifacts.record()`
and keep their project-specific event detection outside this package. Movement
commands must remain in separately reviewed control code. Starting evidence
capture does not authorize robot motion.

Visual verification must state which frames support each observation. Cite the
event-aligned start/end frames (and intermediate frames when needed), including
both frame index and UTC timestamp. Scope wording to what those cited frames
show; a few event frames do not by themselves establish that every intervening
video frame was reviewed.

## Chinese recording announcements

The reusable audio-announcement hook is implemented in the evidence-session lifecycle. Every
video-recording transition must produce these exact Chinese TTS announcements:

- after video capture becomes active and the first frame is confirmed:
  `机器人开始移动` ("robot starts moving");
- whenever an active video capture stops, including normal completion, abort,
  or cleanup after failure: `机器人停止移动` ("robot stops moving").

Acceptance requirements:

- Keep the hook project-neutral; do not import handshake policy into
  `robot_dev_harness`.
- Initialize and health-check the configured audio backend before a physical
  session becomes command-ready.
- Do not let TTS RPC latency block the camera thread or real-time control loop.
- Timestamp the announcement request, backend acceptance/result, and any error
  in the run's `events` telemetry stream.
- Never omit the stop announcement merely because the run aborted. Emit it once
  if and only if video capture became active.
- Prevent duplicate start/stop announcements during idempotent cleanup.
- Treat a missing or failed start announcement as an evidence-readiness failure
  for physical-motion sessions. Preserve the partial run and do not construct a
  command publisher.
- If the stop announcement fails after motion, preserve the run as incomplete,
  record the failure, and continue safe controller cleanup without delay.
- Add dry-run/fake-backend tests covering normal completion, startup failure,
  abort, duplicate finalization, and audio-backend failure.

The first read-only PC2 rehearsal succeeded with both Unitree TTS calls returning
zero. Evidence is stored under:

```text
artifacts/robot_dev_runs/20260830T042807.515987Z_chinese-recording-announcement-rehearsal/
```
