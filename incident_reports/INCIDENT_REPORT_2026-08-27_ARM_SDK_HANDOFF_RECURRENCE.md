# Incident Report: G1 Arm Drop During Settled Arm-SDK Handoff

## Summary

On 2026-08-27 China Standard Time, two minimum-amplitude physical tests of the
standalone G1 arm-controller handoff caused the raised right arm to drop
suddenly. The behavior visibly resembled the 2026-08-22 arm-drop incident.

Both tests waited for Unitree's high-level `shake hand` RPC to return and then
required sustained measured settling before publishing to `rt/arm_sdk`.
Nevertheless, the arm became unstable while the arm-SDK authority weight was
increasing. The measured right elbow departed sharply from an almost unchanged
command and reached approximately 15 rad/s. This demonstrates that RPC return
plus kinematic settling is not a sufficient ownership-handoff gate.

No injury or hardware damage was reported in the available session. All
physical `rt/arm_sdk` testing is suspended.

## Incident details

- **Date:** 2026-08-27 CST (runs began on 2026-08-26 UTC)
- **Robot:** Unitree G1 with BrainCo hand
- **Operating state at preflight:** FSM 501, mode 0
- **High-level action:** modern `G1ArmActionClient` action `shake hand`
- **Custom interface:** `rt/arm_sdk`, 250 Hz target rate
- **Commanded joints:** both arms, indices 15 through 28
- **Authority weight:** joint 29 `q`
- **Requested movement:** one 0.005 rad right-elbow out-and-back trajectory
- **Safety setup reported by operator:** gantry attached and emergency-stop
  ready via the required command acknowledgements
- **Observed behavior:** arm raised and settled, then dropped suddenly during
  arm-SDK authority blend-in

## Evidence

The raw JSONL files are intentionally uncommitted and must be preserved:

```text
telemetry/standalone_arm/sequence_20260826T233757Z.jsonl
telemetry/standalone_arm/sequence_20260826T233832Z.jsonl
```

SHA-256:

```text
08d3a9767aa9d058d70745e1963152e9bbfbf9090d8ba76d01115f28376e3753  sequence_20260826T233757Z.jsonl
5d8ff4b5fa98345ae326e25229e0b6cece90aa1bc8910c597c970c7c8db827a8  sequence_20260826T233832Z.jsonl
```

File sizes:

```text
24,699,893 bytes  sequence_20260826T233757Z.jsonl
 6,885,764 bytes  sequence_20260826T233832Z.jsonl
```

Both recordings reported zero dropped telemetry samples and no recorder write
error.

## Preconditions that passed

The tests incorporated the principal software mitigations from the first
incident:

1. The high-level action request completed before arm-SDK publication.
2. Every arm joint was required to remain below 0.10 rad/s for at least 0.5
   seconds.
3. The settled pose was checked against a four-run reviewed pose center with a
   0.01 rad tolerance.
4. Measured high-level-action displacement was approximately 0.799 rad.
5. Arm-SDK targets began at measured joint positions.
6. Authority weight was ramped rather than switched immediately.
7. Targets were limited to 0.5 rad/s convergence.
8. The command stream used Unitree's G1 motion-mode arm gains, motor mode 1,
   joint 29 authority weight, CRC, and a 250 Hz deadline scheduler.
9. The requested elbow excursion was reduced to 0.005 rad.
10. Every computed and published command was recorded with a sequence number.

In the second run, `ExecuteAction("shake hand")` returned code zero and the arm
then settled for 0.519 seconds. That run conclusively shows the recurrence was
not caused by accepting an RPC timeout as success.

## First recurrence: `20260826T233757Z`

Relative to `movement_started`:

| Time | Recorded event |
| ---: | --- |
| -0.772 s | `shake hand` returned 3104 after the arm physically raised. |
| -0.263 s | Sustained settling completed. |
| 0.000 s | Arm-SDK publication began. |
| 0.556 s | Authority reached 1.0; elbow remained near 0.1832 rad. |
| 0.624 s | State-limit violation triggered controlled release. |
| 0.650 s | Elbow measured 0.5363 rad at 14.518 rad/s while its command remained near 0.1838 rad. |
| 1.137 s | Arm-SDK weight reached zero. |
| 2.570 s | High-level `release arm` returned code zero. |
| 10.587 s | Safe-return verification failed at joint 24. |

Peak values observed during the handoff interval included:

- Right elbow: 15.117 rad/s.
- Right wrist pitch: 12.401 rad/s.
- Right shoulder pitch estimated torque: 28.75 Nm.
- Right wrist roll: 5.202 rad/s.
- Right shoulder pitch position range: 0.510 rad.
- Right elbow position range: 0.731 rad.

The run ended aborted because safe-return joint 24 remained 0.01645 rad outside
the required pose envelope.

## Second recurrence: `20260826T233832Z`

Relative to `movement_started`:

| Time | Recorded event |
| ---: | --- |
| -0.769 s | `shake hand` returned code zero. |
| -0.248 s | Sustained settling completed after 0.519 s. |
| 0.000 s | Arm-SDK publication began. |
| 0.404 s | Elbow remained near 0.1832 rad at authority weight 0.801. |
| 0.408 s | State-limit violation triggered controlled release. |
| 0.446 s | Elbow measured 0.2372 rad at 7.115 rad/s while its command remained near 0.1832 rad. |
| 0.493 s | Elbow measured 0.3344 rad at 12.390 rad/s. |
| 0.917 s | Controlled release ended and weight was zero. |
| 2.362 s | High-level `release arm` returned code zero. |
| 2.960 s | Safe-return settling succeeded. |

Peak values observed during the interval included:

- Right elbow: 14.668 rad/s.
- Right wrist pitch: 13.873 rad/s.
- Right shoulder pitch: 6.113 rad/s.
- Right wrist roll: 4.232 rad/s.
- Right elbow position range: 0.808 rad.

The run ended aborted on state limits, although the later high-level return
restored the reviewed safe pose.

## Command-versus-motion finding

The planned 0.005 rad trajectory did not cause the observed displacement. At
the onset of instability, the right-elbow command was still approximately
0.1832 rad while the measured joint accelerated away from it:

```text
Settled elbow position:       approximately 0.1832 rad
Command near failure:         approximately 0.1832 to 0.1838 rad
Requested trajectory offset:  at most 0.0050 rad
Measured excursion:           approximately 0.73 to 0.81 rad
Measured peak velocity:       approximately 15 rad/s
```

The failure therefore cannot be explained as execution of the requested
trajectory or ordinary tracking overshoot.

## Technical assessment

The evidence is consistent with an unsupported or incompletely understood
controller-ownership transition between Unitree's high-level arm-action
controller and `rt/arm_sdk`.

`ExecuteAction("shake hand")` returning establishes RPC completion. Sustained
zero velocity establishes a stationary arm. Neither observation establishes
that the high-level controller has relinquished the arm, that the pose is a
supported arm-SDK takeover state, or that the two controllers share identical
targets, gains, reference frames, and transition semantics.

The second recurrence occurred after a code-zero RPC return and measured
settling, so additional fixed delay cannot address the missing ownership
evidence. Sport-mode telemetry also changed between mode 0 and mode 1 around
the handoff, but the meaning and causal role of that transition are not yet
established.

This assessment is intentionally narrower than a definitive firmware root
cause. Exact internal controller ownership and blend semantics require Unitree
documentation or vendor analysis.

## Abort-path finding

The controlled weight ramp prevented an immediate software switch to zero, but
it did not stabilize the already divergent arm. Large oscillations continued
during and after the ramp. In the first run, the later high-level return failed
the reviewed safe-return envelope.

Consequently, the current controlled-release algorithm is not a validated
recovery mechanism for this controller-transition failure. Falling back to the
last command while reducing authority is insufficient once the firmware-level
controllers have diverged.

## Revised safety decision

The following transition is prohibited:

```text
high-level shake hand action
-> RPC return
-> measured settling
-> rt/arm_sdk authority takeover
```

Waiting longer after RPC return or settling is not an accepted corrective
action.

All physical `rt/arm_sdk` publication is suspended. The existing
`--execute-arm-sdk` path must not be used until it is structurally disabled or
redesigned and a new return-to-test review is completed.

## Required corrective direction

Future work must avoid transferring directly from the high-level handshake
action into arm-SDK control. Candidate architectures require separate review:

1. Use only Unitree's predefined high-level raise and release actions, with no
   custom arm-SDK movement.
2. Start from documented Regular mode without a held high-level arm action and
   use one continuous, Unitree-supported `rt/arm_sdk` controller for raise,
   bounded movement, return, and authority release, following the ownership
   model used by Unitree XR teleoperation.

Before any new arm-SDK hardware test:

1. Obtain authoritative controller-ownership and weight semantics for this G1
   firmware.
2. Explain the observed sport-mode 0/1 transition.
3. Determine whether the entire 29-motor `LowCmd` initialization used by the
   official controller is required on `rt/arm_sdk`.
4. Redesign abort handling for a divergent controller transition; do not rely
   on the current weight ramp as a safe recovery.
5. Add a hard software disable preventing accidental use of the failed
   high-level-action-to-arm-SDK path.
6. Validate any replacement entirely offline and command-only before a newly
   approved gantry-attached test.

## Status

**Open / physical arm-SDK testing suspended.** The recurrence disproves the
previous handoff gate. Raw evidence is preserved, but internal firmware
ownership behavior and a safe replacement architecture remain unresolved.
