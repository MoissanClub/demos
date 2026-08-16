# Session Handoff

Last updated: 2026-08-16 (Asia/Shanghai)

## Resume point

The project now follows two parallel tracks documented in `PROJECT_PLAN.md`:

1. Vision track
   - Detect a human hand and estimate `(x, y, z)` using RealSense color/depth.
   - Track the hand with a bounded inverse-kinematics arm controller.
2. Handshake track
   - Validate and analyze the existing trajectory recordings.
   - Derive and safely stage a bounded arm oscillation during `hold`.

The immediate deliverable is validated trajectory analysis plus an observe-only
3D hand-localization prototype.

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

1. Implement an episode validator and run it over all trajectory JSONL files.
2. Investigate the trajectory from the 2026-08-16 09:46 session, which appears
   to lack `trajectory.summary`.
3. Plot the right-arm joints during `hold`, aligned with tactile and controller
   states.
4. Separate stationary holds from oscillatory or human-driven motion and
   estimate amplitude, frequency, phase, and joint coupling.
5. Propose a newly generated bounded oscillation; do not blindly replay a
   recorded measured trajectory.
6. In the vision track, prototype observe-only human-hand XYZ localization.

## Repository state at handoff

- The two-track roadmap was committed as `aa738ab` (`document two-track
  handshake roadmap`).
- The automated suite passed: 40 tests.
- Raw recordings, logs, dependencies, and caches are intentionally untracked.

At the start of the next session, read this file and `PROJECT_PLAN.md`, inspect
`git status`, and continue with the validator unless the user changes priority.
