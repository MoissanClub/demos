# Session Handoff

Last updated: 2026-08-20 (Asia/Shanghai)

## Resume point

The project now follows two parallel tracks documented in `PROJECT_PLAN.md`:

1. Vision track
   - Detect a human hand and estimate `(x, y, z)` using RealSense color/depth.
   - Track the hand with a bounded inverse-kinematics arm controller.
2. Handshake track
   - Validate and analyze the existing trajectory recordings.
   - Derive and safely stage a bounded arm oscillation during `hold`.

The trajectory validator is now implemented. The next handshake deliverable is
a synchronized derived training table, followed by right-arm trajectory
visualization and analysis during `hold`.

## Validation baseline

Run the streaming validator with:

```bash
python validate_handshake_trajectories.py telemetry/trajectories
```

The 2026-08-20 baseline found 38 local trajectory files: 37 valid successful
episodes and one incomplete episode. The incomplete file is the expected
2026-08-16 09:46 Asia/Shanghai recording:

```text
telemetry/trajectories/20260816T014604Z/
trajectory_20260816T014618.574298Z_1a6bae15-49b9-43a0-85f2-d79fc3c9fb51.jsonl
```

It contains malformed JSON at line 3104 and no `trajectory.summary`. Preserve
it as immutable raw evidence; do not repair it in place.

The extended validation reports complete `open_wait` state cycles for all 37
successful episodes and an incomplete cycle for the truncated episode. Vision
signals are present in 24 of the 38 trajectories. Per-stream frequency and
10%-of-period missing-sample coverage are included in the JSON report.

## Next step: synchronized training table

Keep each JSONL trajectory as immutable asynchronous raw evidence. Create a
separate derived table on a uniform training grid; do not require an original
sample from every modality at the exact grid timestamp.

Initial design:

1. Measure BrainCo rates and gaps across the validated collection, then select
   an initial grid frequency. Start by evaluating 5 Hz and 10 Hz because the
   observed BrainCo streams are the limiting control-relevant source.
2. Align continuous signals with an explicitly documented method:
   - interpolate joint positions only between sufficiently close samples;
   - use nearest-neighbor or cautious interpolation for velocity and torque;
   - use nearest-neighbor or previous-value hold for tactile and hand-motor
     data, subject to strict age limits;
   - use previous-value hold for controller and vision state, with freshness
     limits; and
   - encode events as nearest-grid flags or state intervals.
3. Add modality masks and sample ages at every grid point, including
   `touch_valid`, `touch_age_ms`, `hand_motor_valid`, `hand_motor_age_ms`,
   `unitree_valid`, `unitree_age_ms`, `vision_valid`, and `vision_age_ms`.
4. Reject or mask a training window when any required control signal exceeds
   its freshness limit. Never present a forward-filled stale value as fresh.
5. Retain higher-rate Unitree data separately when deriving arm-motion features
   such as amplitude, frequency, velocity RMS, and peak estimated torque.
6. Record grid frequency, alignment methods, tolerances, age limits,
   interpolation, filtering, and source trajectory ID as derived-data
   provenance.

Exit criterion: every retained training row has synchronized values or an
explicit invalid mask for each required modality, and the derived table can be
reproduced without modifying the raw JSONL trajectory.

## Current conclusion about arm-oscillation data

The existing recording structure is sufficient to reconstruct the measured
joint trajectory `q(t)`, isolate `hold`, identify participating joints, and
derive a new smooth bounded oscillation offline. It is not sufficient by itself
to safely replay a trajectory on hardware because it records measured state,
not the high-level controller's original joint commands. Before commanding new
motion, confirm the Unitree control mode and command interface, joint and motion
limits, controller behavior, and safety-stop paths.

Recent recordings contain `unitree.lowstate` at roughly 400-500 Hz, including
joint position, velocity, estimated torque, raw acceleration, motor state,
temperature, and voltage. They also contain tactile data and controller state
markers. Some five-second `hold` segments show meaningful shoulder-pitch,
elbow, and wrist-pitch motion; others are nearly stationary.

## Relevant right-arm fields

Each `unitree.lowstate` row has a record-level `timestamp_monotonic_ns` and a
`data.motors[]` array. Prefer selecting by `joint_name`; the current indices are:

| Joint | Index | Position field |
|---|---:|---|
| Right shoulder pitch | 22 | `data.motors[22].position_rad` |
| Right elbow | 25 | `data.motors[25].position_rad` |
| Right wrist pitch | 27 | `data.motors[27].position_rad` |

Other useful per-motor fields are `velocity_rad_s`, `acceleration_raw`,
`estimated_torque_nm`, `mode_raw`, `motor_state_raw`, `temperature_raw`, and
`voltage_raw`.

## Exact next actions

1. Implement and compare 5 Hz and 10 Hz synchronized derived tables with
   per-modality validity masks and sample-age fields.
2. Choose freshness limits from the observed frequency and gap distributions.
3. Plot the right-arm joints during `hold`, aligned with tactile and controller
   states.
4. Separate stationary holds from oscillatory or human-driven motion and
   estimate amplitude, frequency, phase, and joint coupling.
5. Propose a newly generated bounded oscillation; do not blindly replay a
   recorded measured trajectory.
6. Decide how validation reports and rejected/incomplete episodes should be
   stored without modifying raw recordings.
7. In the vision track, prototype observe-only human-hand XYZ localization.

## Repository state at handoff

- The two-track roadmap was committed as `aa738ab` (`document two-track
  handshake roadmap`).
- The validator has ten focused tests. All 50 automated tests pass with
  OpenCV installed in the active Python environment.
- Raw recordings, logs, dependencies, and caches are intentionally untracked.

At the start of the next session, read this file and `PROJECT_PLAN.md`, inspect
`git status`, and continue with the synchronized training table unless the user
changes priority.
