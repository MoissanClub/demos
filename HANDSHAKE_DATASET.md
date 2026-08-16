# G1 BrainCo Handshake Dataset Guide

This guide describes the data written by `g1_brainco_handshake_demo.py`. It is
for people exploring recordings and for agents generating validators,
visualizations, feature extraction, or statistical analysis code.

## Dataset unit and storage

Each finalized `.jsonl` file is one tactile handshake trajectory. It starts
when contact makes the tactile controller leave `open_wait`, includes
`closing`, `hold`, and `releasing`, and ends when the controller returns to
`open_wait`. Vision-only invitations and idle `open_wait` data are not logged.
A run can therefore produce zero, one, or many files.

```text
# Local
telemetry/trajectories/<run_id>/trajectory_<UTC>_<trajectory_id>.jsonl

# Hugging Face
trajectories/<run_id>/trajectory_<UTC>_<trajectory_id>.jsonl
```

The configured repository is currently `davidwei79/g1-handshake-data` and is
created as private. Files are written as `.tmp` and atomically renamed when
finalized. Treat remaining `.tmp` files as incomplete.

## Record envelope and time

JSON Lines means every line is one complete JSON object, not one element of a
surrounding JSON array. Every record has:

```json
{
  "timestamp_monotonic_ns": 10470727347808,
  "stream": "controller.decision",
  "data": {},
  "optional_stream_metadata": "..."
}
```

| Field | Type | Meaning |
|---|---:|---|
| `timestamp_monotonic_ns` | integer | PC2 monotonic receipt/decision time in nanoseconds. |
| `stream` | string | Record type and primary discriminator. |
| `data` | object, array, or scalar | Stream-specific payload. |
| other top-level fields | varies | Stream metadata such as `topic` or `touch_metric`. |

Use the `trajectory.metadata` timestamp as `start_ns` and calculate relative
time as `(timestamp_monotonic_ns - start_ns) / 1e9`. Monotonic time supports
durations and alignment within one boot; it is not Unix time and cannot be
compared across reboots. UTC appears only in lifecycle metadata.

Sources are asynchronous. Sort each stream by timestamp and use an as-of or
nearest-time join with an explicit tolerance; do not assume adjacent lines are
simultaneous. Sampling rates are observed, not guaranteed. Serial latency, DDS
delivery, CPU load, and recorder queue pressure affect them.

## Stream inventory

| Stream | Cardinality | Purpose |
|---|---:|---|
| `trajectory.metadata` | one, first | Identity and controller configuration. |
| `brainco.touch` | repeated | Raw per-finger tactile readings. |
| `brainco.motor` | repeated when reads succeed | Measured BrainCo motor state. |
| `controller.decision` | repeated | State-machine output and derived signals. |
| `controller.command` | zero or more | Finger commands sent or simulated. |
| `controller.event` | zero or more | Arm requests occurring inside the trajectory. |
| `unitree.lowstate` | repeated when DDS is available | IMU and all G1 motors. |
| `trajectory.summary` | one, last | Outcome and dropped-sample count. |

Code must tolerate missing optional streams.

## `trajectory.metadata`

The first record's `data` has:

| Field | Meaning |
|---|---|
| `trajectory_id` | UUID, also in the filename and summary. |
| `started_at_utc` | ISO-8601 UTC start time. |
| `run_id` | Process run, such as `20260816T013113Z`. |
| `hand_side` | `left` or `right`. |
| `brainco_device` | SDK description, firmware, hardware type, serial, and SKU. |
| `port`, `slave_id` | Serial path and Modbus ID (normally 126 left, 127 right). |
| `dry_run`, `enable_arm` | Runtime mode flags. |
| `control_parameters` | Tactile parameters listed below. |

Current control parameters are:

```text
start_threshold, stop_threshold, release_threshold, release_seconds,
hold_duration, max_close, step, period, thumb_scale,
arm_raise_guard_seconds
```

Vision tuning and post-handshake lowering delay are not yet copied into this
metadata. Preserve the command line separately when those settings matter.
SDK enums serialize as `{"name":"...","int_value":6}`; use `int_value` for
numeric processing and retain `name` for display.

## `brainco.touch`

For the current Revo2 capacitive hand, `data` is five finger objects in order:

```text
0 thumb, 1 index, 2 middle, 3 ring, 4 pinky
```

Observed fields per finger:

```text
normal_force1, normal_force2, normal_force3
tangential_force1, tangential_force2, tangential_force3
tangential_direction1, tangential_direction2, tangential_direction3
self_proximity1, self_proximity2, mutual_proximity
status, description
```

These are raw SDK values. Do not label them newtons or degrees without
hardware-specific calibration. Direction value `65535` commonly behaves like
an unsigned sentinel; do not plot it automatically as a physical angle.
`description` is redundant display text and should be omitted from numeric
tables.

Top-level `touch_metric` is the maximum absolute value across all fingers and
all six normal/tangential force fields. This drives the tactile state machine.
Top-level `controller_state_before` is the state immediately before that
sample's update.

Array-pressure hardware has a different payload: 25 raw values, five per
finger in the same finger order, with axes `Fx, Fy, Fz, Mx, My`. The controller
decodes signed 16-bit force axes, divides by 100, and derives
`max(abs(Fx), abs(Fy), abs(Fz))` across fingers, in newtons according to the SDK
example. Branch on metadata hardware type instead of assuming one schema.

## `brainco.motor`

`data` is serialized `MotorStatusData`:

```text
positions[6], speeds[6], currents[6], states[6], description
```

Motor order is:

```text
0 thumb, 1 thumb_aux, 2 index, 3 middle, 4 ring, 5 pinky
```

Positions are approximately `0` fully open through `1000` fully closed, and
commands use the same ordering/range. Treat speed and current as raw SDK values
unless calibration is supplied. State entries are enum objects with `name` and
`int_value`. This timestamp is serial-read completion and is not exactly
simultaneous with the neighboring tactile sample.

## `controller.decision`

This is the easiest stream for state overlays and segmentation.

| Data field | Type | Meaning |
|---|---:|---|
| `state` | string | State after update: `open_wait`, `closing`, `hold`, `releasing`. |
| `close_value` | integer | Current normalized target, `0..1000`. |
| `command_close` | integer/null | New target this update; null means no new command. |
| `trigger_arm` | boolean | Legacy tactile request; vision mode uses separate arm policy. |
| `release_arm` | boolean | Hand-open confirmation/timeout reached. |
| `entered_hold` | boolean | True only on transition into `hold`. |
| `event` | string/null | Transition reason below. |
| `touch_metric` | number | Derived contact metric. |
| `hand_is_open` | boolean | All measured positions satisfy open threshold. |
| `vision_state` | string/null | `no_hand` or `hand_present`. |
| `vision_score` | number/null | Largest depth-gated changed-region fraction in the image ROI, `0..1`; not probability. |

Events:

| Event | Meaning |
|---|---|
| `contact_started` | `open_wait -> closing`. |
| `pressure_limit_reached` | `closing -> hold` at pressure threshold. |
| `max_close_reached` | `closing -> hold` at close limit. |
| `release_during_closing` | Confirmed low contact while closing. |
| `release_during_hold` | Confirmed low contact while holding. |
| `hold_timeout` | Maximum hold duration elapsed. |
| `hand_open_confirmed` | `releasing -> open_wait` after measured open. |
| `hand_open_timeout` | Same transition after bounded timeout. |

`controller_state_before` on the touch row and decision `state` describe
opposite sides of the same update, making exact transitions identifiable.

## `controller.command` and `controller.event`

Finger command example:

```json
{"kind":"finger_positions","positions":[50,50,50,50,50,50],"reason":"state_machine"}
```

Positions use the six-motor order above. Reasons include `state_machine`, a
transition event, and `idle_open`. In dry-run mode they are intentions, not
proof of movement. Compare with measured positions for lag/tracking error.

Known arm request payloads include:

```json
{"event":"arm_action_requested","action":"shake hand"}
{"event":"arm_release_requested","action":"release arm"}
{"event":"arm_raise_requested","reason":"vision_or_handshake"}
{"event":"arm_lower_requested","reason":"post_handshake_complete"}
```

Absence does not prove no arm action occurred. Vision may raise the arm before
tactile contact, and delayed lowering may happen after trajectory finalization.
Infer actual motion primarily from Unitree joints and treat events as requests.

## `unitree.lowstate`

Top-level `topic` is normally `rt/lowstate`. `data` contains:

```text
version_raw[2], mode_pr_raw, mode_machine_raw, tick_raw, imu, motors[]
```

The IMU normally has `quaternion[4]`, `gyroscope[3]`, `accelerometer[3]`,
`rpy[3]`, and `temperature`. Preserve quaternion ordering as recorded; confirm
the installed Unitree message definition before assuming `xyzw` or `wxyz`.

Each motor has:

| Field | Unit/meaning |
|---|---|
| `index`, `joint_name` | DDS index and recorder-added label. |
| `mode_raw`, `motor_state_raw` | Raw codes. |
| `position_rad` | radians. |
| `velocity_rad_s` | radians/second. |
| `acceleration_raw` | Raw `ddq`; units not asserted here. |
| `estimated_torque_nm` | N·m. |
| `temperature_raw` | Raw two-element array. |
| `voltage_raw` | Raw SDK voltage field. |

Joint mapping:

```text
 0 left_hip_pitch          1 left_hip_roll
 2 left_hip_yaw            3 left_knee
 4 left_ankle_pitch        5 left_ankle_roll
 6 right_hip_pitch         7 right_hip_roll
 8 right_hip_yaw           9 right_knee
10 right_ankle_pitch      11 right_ankle_roll
12 waist_yaw              13 waist_roll
14 waist_pitch            15 left_shoulder_pitch
16 left_shoulder_roll     17 left_shoulder_yaw
18 left_elbow             19 left_wrist_roll
20 left_wrist_pitch       21 left_wrist_yaw
22 right_shoulder_pitch   23 right_shoulder_roll
24 right_shoulder_yaw     25 right_elbow
26 right_wrist_roll       27 right_wrist_pitch
28 right_wrist_yaw
```

Indices 29+ remain as `reserved_<index>` and should stay in immutable raw data.
For this right-hand demo, indices 22-28 are most relevant. Vibration can only
be estimated as a proxy (for example rolling RMS of velocity or high-pass
position); no direct vibration-strength field exists.

## `trajectory.summary`

The final `data` object has `trajectory_id`, `result` (`success` or `aborted`),
`reason`, `dropped_samples_total`, and `finished_at_utc`. Common reasons are
`hand_open_confirmed`, `hand_open_timeout`, and `controller_exit`.

The dropped count is cumulative across trajectories in the process run, not
necessarily local to this file. Zero means no recorder queue overflow was
observed; it does not prove that upstream DDS or serial delivery lost nothing.

## Recommended derived tables

Keep raw JSONL immutable. Derive:

- `trajectory`: one row per file with metadata, outcome, duration, stream
  counts/rates, gaps, and dropped count.
- `controller`: `trajectory_id, t_s, state, event, touch_metric, close_value,
  command_close, hand_is_open, vision_state, vision_score`.
- `touch_long`: one row per sample/finger with all force, direction,
  proximity, status, and derived metric fields.
- `hand_motor_long`: one row per sample/motor with position, raw speed/current,
  and state enum.
- `unitree_motor_long`: one row per DDS sample/joint with position, velocity,
  torque, raw acceleration/temperatures/voltage/mode/state.

Do not forward-fill into a higher apparent rate without documenting it. For
joint/tactile comparison, nearest-time match with a declared maximum tolerance
or resample both to a declared common grid.

## Minimal loader

```python
import json
from pathlib import Path

path = Path("trajectory_example.jsonl")
rows = [json.loads(line) for line in path.open(encoding="utf-8")]
metadata = next(r for r in rows if r["stream"] == "trajectory.metadata")
start_ns = metadata["timestamp_monotonic_ns"]

by_stream = {}
for row in rows:
    row["t_s"] = (row["timestamp_monotonic_ns"] - start_ns) / 1e9
    by_stream.setdefault(row["stream"], []).append(row)
for values in by_stream.values():
    values.sort(key=lambda r: r["timestamp_monotonic_ns"])
```

Process large collections line-by-line; `unitree.lowstate` dominates file size
because every row contains every motor.

## Recommended visualization

Use a shared relative-time axis with:

1. Touch metric and per-finger force signals.
2. Commanded hand position and six measured positions.
3. Right-arm joint positions (indices 22-28).
4. Right-arm velocities and estimated torques.
5. Vision score/state, labeled as depth-gated scene change rather than confidence.

Shade controller states and mark non-null decision events. Above the plots,
show result, duration, device serial/firmware, thresholds, stream counts,
effective rates, maximum gaps, and dropped samples.

## Validation checklist

1. First and last streams are metadata and summary.
2. Metadata, summary, and filename trajectory UUIDs match.
3. At least one touch and one decision record exist.
4. Successful state progression includes `closing`, `releasing`, and final
   `open_wait`; `hold` may be bypassed by an early release.
5. Result, reason, and dropped count exist.
6. Sort timestamps; report non-monotonic values and per-stream gaps/rates.
7. Check array lengths before indexing because SDK/robot variants can differ.
8. Check finite/plausible numeric values without modifying raw sentinels.
9. Record interpolation, filtering, baseline removal, tolerances, and unit
   conversions as derived-data provenance.

## Known limitations

- No explicit schema-version field exists yet.
- Images are not recorded; only vision state/score appears during trajectories.
- Vision score is depth-gated scene change, not semantic hand confidence.
- Idle invitation and delayed lowering events can be outside file boundaries.
- Human identity, comfort/success ratings, and annotations are not collected.
- Raw SDK fields may change with SDK/firmware; group using device metadata.
- Queue writes are non-blocking: overload drops samples instead of delaying
  robot control.
