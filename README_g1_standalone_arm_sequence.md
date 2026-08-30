# G1 Standalone Bounded Arm Sequence

> **Incident hold:** Do not run `--execute-arm-sdk`. Two minimum-amplitude
> trials on 2026-08-27 reproduced a sudden arm drop after the high-level
> `shake hand` action. The implementation below is retained for forensic
> analysis and command-only work; its physical publication path is unsafe.

## Replacement architecture

The next implementation will remove high-level arm actions from the arm-SDK
sequence. One continuous `rt/arm_sdk` controller will start from the measured
Regular-mode pose, acquire authority while holding that fixed pose, raise the
arm smoothly, perform the bounded shake, return smoothly to the initial pose,
verify settling, and release authority. No modern or legacy handshake action
may overlap that session.

The continuous controller now has a fail-closed gravity-feedforward layer. It
loads the checked-out `xr_teleoperate` G1-29 URDF, reduces it to the exact
joint-15-through-28 ordering, and computes Pinocchio RNEA with zero desired
acceleration. Each torque must be finite and remain within the software bound
(5 Nm for shoulder/elbow joints and 1.5 Nm for wrists). Commanded torque is
scaled by the arm-SDK authority weight, reaches the bounded RNEA value at full
authority, and returns to zero during release. The URDF SHA-256, joint mapping,
bounds, and every commanded torque are recorded. A mapping, model, non-finite,
or torque-bound failure aborts before the next command.

This is offline-validated code, not approval for another physical trial. The
next physical step remains one newly reviewed, gantry-attached, compensated
zero-offset authority cycle. Do not use the raise candidate before its results
have been compared with the three uncompensated baselines.

The roadmap and return-to-test gates are in `PROJECT_PLAN.md`; the current
resume point and incident evidence are in `SESSION_HANDOFF.md` and
`incident_reports/2026-08-27/`.

`g1_standalone_arm_sequence.py` isolates the arm-controller handoff from vision,
tactile sensing, and BrainCo hand manipulation. Its intended sequence is:

```text
high-level shake hand action
-> measured post-action settling
-> one bounded arm-SDK out-and-back movement
-> controlled arm-SDK authority release
-> high-level release arm action
```

The current implementation covers offline planning, read-only post-action pose
capture, command-only rehearsal, and an explicitly gated arm-SDK publication
mode. Hand manipulation is the next extension after the arm-only stages pass.

## Safety boundary

- Keep the robot attached to the reviewed gantry for every physical mode.
- Establish the exclusion zone and assign a dedicated emergency-stop operator.
- Use no human contact.
- Do not infer physical settling from high-level RPC completion. The script
  waits for sustained measured settling after the RPC returns.
- Every high-level action must return Unitree code zero. Nonzero RPC or arm
  service codes abort the sequence and are recorded; an unchanged pose is not
  accepted as a successful capture.
- Pose capture and `--dry-run-arm-sdk` do not create an `rt/arm_sdk` publisher.
- Publication follows Unitree's G1 motion-mode arm-controller pattern: a 250 Hz
  `rt/arm_sdk` stream, joint 29 as the native/user arm-command blend weight,
  arm motor mode 1, measured-state velocity clipping, and Unitree's
  shoulder/elbow and wrist gains. Legs and waist remain under the native motion
  controller.
- Do not use `--execute-arm-sdk` until Unitree ownership semantics for the exact
  robot and firmware, the full commanded joint set, gains, and authority
  transitions are reviewed.
- The script reports every joint that violates settling, rather than only the
  first failure.

## 1. Offline trajectory

This mode imports no Unitree SDK and cannot move the robot:

```bash
python g1_standalone_arm_sequence.py --offline-plan-only
```

### Parameterized Cartesian command planning

`plan_g1_cartesian_arm.py` is the reusable, offline-only Cartesian command
interface. It cannot publish DDS commands. Every plan requires explicit
world-frame bounds for both hands; it rejects an initial pose or target outside
those reviewed workspaces. It also enforces the Cartesian displacement norm,
maximum joint offset, maximum joint velocity, duration, sample rate, IK
residuals, joint limits, and bounded RNEA feedforward.

```bash
python plan_g1_cartesian_arm.py \
  --initial-arm-q Q15 Q16 Q17 Q18 Q19 Q20 Q21 Q22 Q23 Q24 Q25 Q26 Q27 Q28 \
  --right-delta-m DX DY DZ \
  --left-workspace-min-m LX_MIN LY_MIN LZ_MIN \
  --left-workspace-max-m LX_MAX LY_MAX LZ_MAX \
  --right-workspace-min-m RX_MIN RY_MIN RZ_MIN \
  --right-workspace-max-m RX_MAX RY_MAX RZ_MAX \
  --maximum-displacement-m 0.02 \
  --maximum-joint-offset-rad 0.40 \
  --maximum-joint-velocity-rad-s 0.075 \
  --duration-seconds 2 \
  --sample-rate-hz 250 \
  --summary-only
```

Workspace values must come from a separately reviewed runtime plan; do not use
placeholder or broad bounds to make a rejected target pass. The retired
`--execute-cartesian-10cm-right-x-test` flag remains hard-disabled after its
one successful verification. This planning interface does not re-authorize it.

### Absolute Cartesian coordinate workflow

For an absolute world-frame coordinate, first create a new immutable request.
The request fixes the target, both reviewed workspaces, trajectory duration,
sample rate, Cartesian displacement limit, joint-offset limit, and joint-speed
limit. Existing request files are never overwritten.

```bash
python create_g1_cartesian_request.py \
  --attempt-id UNIQUE_ATTEMPT_ID \
  --output reviewed_request.json \
  --right-target-m X Y Z \
  --left-workspace-min-m LX_MIN LY_MIN LZ_MIN \
  --left-workspace-max-m LX_MAX LY_MAX LZ_MAX \
  --right-workspace-min-m RX_MIN RY_MIN RZ_MIN \
  --right-workspace-max-m RX_MAX RY_MAX RZ_MAX \
  --maximum-displacement-m 0.01 \
  --maximum-joint-offset-rad 0.05 \
  --maximum-joint-velocity-rad-s 0.02 \
  --duration-seconds 8
```

The command prints the request's canonical SHA-256. Plan it against a recent
read-only 14-joint arm snapshot without importing Unitree DDS:

```bash
python plan_g1_cartesian_request.py \
  --request reviewed_request.json \
  --expect-request-sha256 SHA256 \
  --initial-arm-q Q15 Q16 Q17 Q18 Q19 Q20 Q21 Q22 Q23 Q24 Q25 Q26 Q27 Q28
```

`run_g1_reviewed_cartesian_test.py` is the corresponding evidence-backed
runtime. It recomputes IK from a fresh planning snapshot, requires the live
execution pose and both hand endpoints to remain continuous with that snapshot,
records command intent before transport, moves out and returns, settles, and
releases authority. It accepts only the exact request hash compiled into
`AUTHORIZED_REQUEST_SHA256`, and it is currently hard-disabled by
`PHYSICAL_EXECUTION_ENABLED = False`. A reviewer must authorize one exact hash
for one attempt and disable it immediately afterward. Creating or planning a
request never authorizes physical movement.

## Read-only mode and service preflight

Before another physical attempt, query the current locomotion FSM and arm
action service without requesting motion:

```bash
python g1_standalone_arm_sequence.py \
  --probe-preflight \
  --network-interface eth0
```

The preflight requires fresh sport-mode DDS state or a successful read-only
locomotion FSM query and accepts only Unitree's documented combinations: FSM
500, FSM 501, or FSM 801 with mode 0 or 3. Arm action-list discovery is
advisory: this robot has physically executed the modern `shake hand` action
even though its API-version and action-list discovery endpoints have returned
3103 or 3104. Every physical mode runs the same preflight automatically and
refuses to request an action if the FSM/telemetry gate fails. It never requests
an FSM or controller transition.

The modern `G1ArmActionClient` backend is selected by default. Legacy sport
tasks 2 and 3 are available only through an explicit
`--high-level-backend legacy-sport` override. Task 3's safe-return semantics
remain unverified, so the legacy backend is not part of the active test plan.

## 2. Capture the stable post-action pose

This physically invokes Unitree's high-level `shake hand` and `release arm`
actions, but it never creates an arm-SDK publisher:

```bash
python g1_standalone_arm_sequence.py \
  --capture-post-action-pose \
  --network-interface eth0 \
  --confirm-gantry-attached \
  --confirm-estop-ready
```

After the raise RPC returns, the script requires all arm-joint velocities to
remain at or below 0.10 rad/s for 0.5 seconds. It then records one second of
settled samples and prints fourteen median centers for joints 15 through 28.
After the high-level release action, it also requires measured return settling
and prints observed safe-return centers.

Repeat this read-only capture several times. Review cross-run deviations before
selecting an envelope; do not approve centers from one run alone.

## 3. Command-only rehearsal

After reviewing fourteen post-action centers, compute the complete arm-SDK
sequence without creating a publisher:

```bash
python g1_standalone_arm_sequence.py \
  --dry-run-arm-sdk \
  --network-interface eth0 \
  --confirm-gantry-attached \
  --confirm-estop-ready \
  --post-action-pose-rad \
  Q15 Q16 Q17 Q18 Q19 Q20 Q21 \
  Q22 Q23 Q24 Q25 Q26 Q27 Q28
```

The rehearsal must contain contiguous `arm_sdk_command` sequence numbers,
`blend_in`, `trajectory`, and `release` phases, `published=false` on every
command, controlled completion, zero dropped samples, a measured command rate
near 250 Hz with bounded schedule lag, and successful high-level return. This
documents the failed historical sequence; it is not the replacement design.
Per-command records are queued without console printing so terminal I/O cannot
consume the four-millisecond control period. Every incoming low-state sample
still updates the safety monitor, while full 35-motor low-state disk records are
rate-limited to 100 Hz to reduce JSON serialization contention. Inspect the
generated JSONL file under `telemetry/standalone_arm/`.

## Publication gate

`--execute-arm-sdk` exists in the preserved failed implementation, but its
acknowledgements do not authorize use. The next code change must hard-disable
that path. Any replacement publisher requires a separate interface and a new
written return-to-test review after the continuous arm-SDK design passes its
offline and command-only gates.
