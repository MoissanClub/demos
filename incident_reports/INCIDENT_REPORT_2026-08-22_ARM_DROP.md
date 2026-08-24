# Incident Report: G1 Arm Drop During Fine-Movement Test

## Summary

On 2026-08-22 (China Standard Time), a hardware test of the G1/BrainCo
handshake demo caused the robot's raised right arm to drop suddenly after the
operator touched the robotic hand. The custom fine-movement controller took
control through Unitree's `rt/arm_sdk` interface while the high-level arm action
was still physically moving. Its velocity guard then detected a right-elbow
velocity of 13.135 rad/s and aborted. The abort immediately released custom arm
control, producing an uncontrolled-looking handoff back to Unitree's controller.

No injury or hardware damage was reported in the available conversation or
logs. Hardware testing of the fine-movement path must remain suspended until
the corrective actions in this report are implemented and reviewed.

## Incident details

- **Date/time:** 2026-08-22 00:46:02 CST (2026-08-21 16:46:02 UTC)
- **Trajectory ID:** `39104975-0ad0-4c26-b246-4ebc69346862`
- **Run ID:** `20260821T164545Z`
- **System:** Unitree G1 with right BrainCo hand
- **Operating mode:** vision, high-level arm action, and custom fine movement enabled
- **Fine-movement request:** 0.02 rad right-elbow out-and-back movement, with a
  0.5 s start delay and 1.0 s movement duration
- **Observed behavior:** vision caused the arm to rise; contact then caused the
  arm to drop suddenly
- **Software outcome:** custom movement aborted on the elbow-velocity guard
- **Reported human/hardware impact:** none reported

## Evidence and timeline

The source evidence is the local trajectory file under
`telemetry/trajectories/20260821T164545Z/`. Raw telemetry is intentionally not
committed because it contains device identifiers and high-volume hardware data.

Times below are relative to the trajectory start.

| Relative time | Recorded event |
| ---: | --- |
| 0.000 s | Trajectory metadata recorded with vision, high-level arm control, and fine movement enabled. |
| 0.186 s | Fine movement was requested and marked as waiting for the arm raise. |
| 0.191 s | `minor_arm_movement_started` was emitted by the executor. |
| 0.241 s | `minor_arm_movement_released_after_arm_raise` was recorded. |
| 0.354 s | Executor aborted: right-elbow velocity exceeded the limit at 13.135 rad/s. |
| 5.365 s | Trajectory summary recorded `success` because the hand reopened. |

The event ordering also exposes an observability issue: the executor's
`started` event was recorded before the controller's `released_after_arm_raise`
event. The latter was written after starting the executor thread, so it does not
form a reliable causal ordering in telemetry.

The later run beginning at 2026-08-22 00:48:14 CST did not reproduce or test
this path because `execute_minor_arm_movement` was false.

## Technical cause

The immediate cause was an unsafe transfer of authority between two arm
controllers:

1. Vision requested Unitree's high-level `shake hand` action.
2. The program treated a successful return from `ExecuteAction()` as proof that
   the physical arm had reached and settled at its raised pose.
3. In reality, the arm was still moving under the high-level controller.
4. On contact, the custom executor captured a single transient `LowState`
   snapshot and used it as a fixed base pose.
5. The executor began commanding all arm joints (indices 15 through 28), not
   only the right elbow. Each joint was assigned the frozen measured position,
   zero desired velocity, zero feed-forward torque, `kp=30`, and `kd=1.5` while
   the arm-SDK authority weight ramped upward.
6. The right elbow reached 13.135 rad/s, exceeding the 1 rad/s safety limit.
7. The exception path attempted to publish arm-SDK weight zero immediately,
   abruptly returning authority to the Unitree controller.

The visible drop is consistent with two target discontinuities: first from a
moving high-level trajectory to a frozen full-arm target, and then from that
custom target back to Unitree's target when the guard aborted. The log contains
no intentional arm-lowering request at the time of the drop.

## Root cause

The root cause was using RPC completion as the gate for physical controller
handoff. `ExecuteAction()` returning successfully established that the action
request returned; it did not establish that the arm had reached a known pose,
stopped moving, or was safe for `rt/arm_sdk` takeover.

## Contributing factors

- The custom executor froze and controlled all 14 arm joints even though the
  planned motion concerned only the right elbow.
- The captured base pose came from one sample while the arm was moving.
- Desired joint velocity changed immediately to zero during takeover.
- Abort handling released authority immediately instead of performing a
  controlled transition to a verified safe controller/pose.
- No measured-pose or sustained-settling gate existed before takeover.
- The trajectory summary used `success` for hand-state completion even though
  the arm sub-operation aborted, which can mislead incident review.
- Commanded arm samples and authority weights were not recorded, limiting exact
  tracking-error and handoff reconstruction.
- Controller events were recorded across threads without causal sequencing.

## Actions taken

- The operator was told not to repeat the hardware fine-movement test.
- The incident trajectory was inspected and compared with the implementation.
- The velocity guard successfully detected the excessive elbow speed and
  stopped the custom trajectory, although its release behavior was unsafe.
- The unsafe path and its current limitations are documented here.

## Required corrective actions

Before any further hardware fine-movement test:

1. Replace the RPC-completion gate with a measured-state gate covering every
   joint that will be controlled.
2. Require joint position to be inside a defined raised-pose envelope and joint
   velocity to remain below a conservative threshold for a sustained settling
   interval.
3. Add a timeout that cancels fine movement without taking arm-SDK authority if
   the raised pose never settles.
4. Capture the custom controller's base pose only after settling is confirmed.
5. Design and test a bumpless handoff: initial custom targets and velocities
   must match the measured state and motion at takeover.
6. Implement safe abort behavior that maintains a stable pose and transfers
   authority through a controlled, verified transition. Do not immediately set
   authority weight to zero as a generic exception response.
7. Minimize the commanded joint set where supported, or explicitly validate the
   full-arm target and controller interaction if the SDK requires full-arm
   commands.
8. Record each commanded position, velocity, gain, authority weight, measured
   state, and guard decision with monotonic sequence numbers.
9. Make the overall trajectory result fail or become `partial/arm_aborted` when
   an enabled arm operation aborts.
10. Add deterministic tests for a still-moving arm, settling timeout, abort
    during blend-in, telemetry loss, state exit, and safe handoff ordering.
11. Review Unitree's documented `rt/arm_sdk` ownership and handoff semantics
    before selecting final gains or running on hardware.

## Return-to-test criteria

Hardware testing may resume only after the corrective implementation is code
reviewed and the following stages pass in order:

1. Unit tests and simulated moving-state/abort scenarios.
2. Command logging only, with no arm-SDK publication.
3. Robot clear of people and obstacles, hand open, emergency-stop operator
   present, and fine-movement amplitude set to the minimum bounded value.
4. Verification from measured telemetry that the arm was settled before
   takeover, the handoff was continuous, the movement tracked within agreed
   tolerances, and abort/release remained stable.

Human-contact testing is explicitly out of scope until those results are
reviewed and a separate approval is recorded.

## Status

**Open / hardware fine-movement testing suspended.** The incident is understood
at the software-control level, but the safe handoff redesign has not yet been
implemented or validated on hardware.
