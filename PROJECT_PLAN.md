# Vision- and Touch-Guided Adaptive Handshake Plan

## Purpose and document boundaries

This is the durable engineering roadmap for extending the Unitree G1 and
BrainCo tactile handshake into a safe, vision- and touch-guided learned
interaction. It records the target control architecture, milestones,
dependencies, safety gates, exit criteria, and unresolved decisions.

Use the other project documents for information that changes more frequently:

- [`README.md`](README.md) describes the research goal and public roadmap.
- [`README_g1_brainco_handshake_demo.md`](README_g1_brainco_handshake_demo.md)
  documents current behavior, configuration, operation, and troubleshooting.
- [`HANDSHAKE_DATASET.md`](HANDSHAKE_DATASET.md) is the authoritative recording
  schema and analysis reference.
- [`SESSION_HANDOFF.md`](SESSION_HANDOFF.md) records current findings, exact next
  actions, and the latest repository state.
- Git history is the implementation and revision log.

## Objective

Build a learned handshake policy that can use vision, tactile feedback, and
robot state as observations and produce custom arm and hand movement while
preserving Unitree's native lower-body balance controller.

The target runtime architecture is:

```text
                         PC2
              ┌───────────────────────┐
RealSense ────>│                       │
BrainCo touch >│ observations          │
G1 LowState ──>│                       │
              │ vision + touch policy │
              │                       │
              │       actions         │
              └───────┬───────┬───────┘
                      │       │
              arm q targets   finger targets
                      │       │
              safety/filter   BrainCo SDK
                      │       │
                      ▼       ▼
                 rt/arm_sdk  hand
                      │
                      ▼
                 G1 arm joints

                         PC1
              ┌───────────────────────┐
              │ Unitree Regular      │
              │ Motion Controller    │
              │                      │
              │ legs / balance /     │
              │ locomotion remain    │
              │ under native control │
              └───────────────────────┘
```

The key architectural decision is to run custom upper-body control from PC2
through `rt/arm_sdk` while the G1 remains in Unitree Regular/Motion Control
mode. Do not use whole-body `rt/lowcmd` for the normal learned-policy runtime.
This follows the split-control pattern demonstrated by Unitree's
`xr_teleoperate --motion` path: custom arm control coexists with the native
lower-body motion controller.

Handshake motion uses complementary representations at different layers:

```text
6D Cartesian hand-pose intent
        ↓
constrained, posture-aware IK
        ↓
smooth bounded joint trajectory
        ↓
joint PD + bounded Pinocchio RNEA feedforward
        ↓
rt/arm_sdk
```

Explicit reviewed joint-space poses remain authoritative for acquisition,
initial hold, return, abort, and recovery.

The intended development sequence is deliberately incremental. The high-level
Unitree handshake action remains characterization evidence only; it must never
be used as the source state for an `rt/arm_sdk` takeover:

```text
measured arm pose in Regular/Motion mode, with no high-level arm action active
                ↓
continuous rt/arm_sdk authority acquisition at one fixed reviewed pose
                ↓
full arm-SDK raise -> bounded shake -> return -> controlled release
                ↓
deterministic BrainCo hand integration with a nonhuman fixture
                ↓
vision-guided approach + tactile handshake
                ↓
learned policy proposes bounded arm/hand actions
```

## Control architecture decisions

### Robot operating mode

For the target runtime:

- Run application and policy code on PC2.
- Keep the G1 in Unitree Regular/Motion Control mode.
- Use `rt/arm_sdk` for custom arm joint commands.
- Leave legs and native self-balancing/locomotion under Unitree's controller.
- Leave waist/body ownership under Unitree initially unless separate ownership
  is explicitly documented and validated.
- Control BrainCo fingers through the BrainCo SDK, independently of
  `rt/arm_sdk`.
- Subscribe to `rt/lowstate` for measured G1 joint and IMU state.

Debug/development whole-body low-level mode and `rt/lowcmd` are not the normal
runtime architecture because they can transfer responsibility for lower-body
joints away from Unitree's native balance controller.

### `rt/arm_sdk` actuator contract

The custom arm actuator should support:

- joint position targets `q`;
- optional joint velocity targets `dq`;
- bounded model-based feed-forward torque `tau` after separate offline and
  compensated zero-offset validation;
- conservative, explicit `kp` and `kd` gains;
- arm-SDK authority/blending weight with gradual acquisition and release;
- current arm state from `rt/lowstate`;
- joint-limit, velocity, acceleration, workspace, timeout, and stale-state
  enforcement;
- controlled cancellation and safe return/release behavior.

Initial policy deployment should use position-dominant control: learned or
heuristic `q_target`, zero desired velocity, fixed conservative gains, and
validated model-based feedforward. Feedforward is computed by Pinocchio RNEA
in the deterministic actuator layer; it is not a learned policy output. Learned
torque control remains out of scope for the first policy.

### Control-rate separation

Do not require neural inference to run at the arm command rate. Use separate
loops, approximately:

```text
camera / vision          ~30 Hz
BrainCo tactile          measured sustainable rate
policy inference         ~10-30 Hz initially
arm command/interpolate  high-rate loop, target ~250 Hz if validated
hand command             measured sustainable rate
Unitree balance          native/internal
```

The arm actuator interpolates or filters policy targets between inference
updates and rejects stale targets.

## Safety principles

- Robot control must not depend on disk, camera, network, model inference, or
  upload availability.
- Hand opening, arm release, cancellation, timeout, and safe return take
  priority over recording, speech, perception, and learned behavior.
- Every failure path should attempt to open the hand before releasing or safely
  returning the arm when physically appropriate.
- Learned outputs remain behind deterministic position, velocity, acceleration,
  pressure, torque, temperature, workspace, timeout, and stale-data limits.
- Never send unconstrained neural-network output directly to DDS or the BrainCo
  actuator.
- Acquire and release `rt/arm_sdk` authority gradually. Capture the initial
  measured pose before acquisition and hold that fixed target during blend-in;
  do not chase the changing measured position.
- Never transition directly from `ExecuteAction("shake hand")`, `release arm`,
  or a legacy sport handshake task into `rt/arm_sdk`. Two gantry-attached tests
  on 2026-08-27 produced sudden arm drops despite RPC completion, measured
  settling, target continuity, command clipping, and authority blending.
- Raw telemetry is immutable. Derived signals must be identified as derived.
- Samples use monotonic receipt timestamps; device clocks are not assumed to be
  synchronized.
- Estimated torque or motor current must not be described as direct joint
  friction.
- Biometric data remains separate from tactile trajectory data. Identity
  recognition requires a concrete feature, explicit enrollment, and consent.
- Physical validation proceeds incrementally: offline, dry run, arm-only,
  open hand, nonhuman fixture, then conservative human contact.

## Milestone status

Implementation does not imply that a physical exit criterion has been met.

| Milestone | Status | Remaining evidence or work |
|---|---|---|
| Controller safety hardening and hardware-independent state tests | Implemented; physical validation open | Complete interrupted-handshake trials, simulated failure audit, and stable baseline tag. |
| Read-only BrainCo and Unitree telemetry probe | Implemented; characterization partial | Publish measured fields, units, rates, latency, jitter, firmware, and unresolved semantics. |
| JSONL trajectory lifecycle and schema | Implemented | Validate every lifecycle outcome and reconcile incomplete recordings. |
| Non-blocking trajectory recording | Implemented; scale validation open | Demonstrate 50 valid local episodes, bounded control timing, dropped-sample reporting, and recorder-failure isolation. |
| Hugging Face upload | Initial post-run upload implemented | Verify privacy and idempotence; decide whether a separate durable spool/uploader is required. |
| Episode validator | Implemented; first baseline complete | Extend with plots and telemetry analysis; decide how rejected data is quarantined. |
| Depth-gated visual invitation | Interim implementation | It detects depth-gated scene change, not a semantic human hand or hand position. |
| `rt/arm_sdk` actuator layer | Implemented; gantry-attached raise/return verified | Extend cancellation/failure trials and characterize tracking/torque margins over the approved workspace. |
| Full deterministic arm raise/shake/return | Verified under gantry | Extend cancellation/failure trials and characterize repeatability before closing Track 1C. |
| Native self-balance with custom arm control | Regular-mode coexistence verified under gantry; broader validation open | Repeat dynamic and cancellation trials, quantify lower-body/IMU response, and validate with the gantry slack before closing 1C. |
| Semantic hand detection and 3D localization | Planned | Prototype and validate in observe-only mode. |
| Bounded inverse-kinematics approach | Implemented; one handshake-height target physically verified | Add collision/posture objectives, multi-target workspace coverage, moving-target replay, and loss/cancellation tests. |
| Staged validation without gantry | Planned | Requires reviewed balance/control evidence, a defined test envelope, and explicit approval. |
| Integrated safety supervisor | Planned | Integrate before learned actions can command hardware. |
| Learned vision+touch arm/hand policy | Future | Requires validated actuator layer, data, action representation, deterministic bounds, and evaluation protocol. |

### Verified arm-control baseline (2026-09-02)

The current implementation has passed the earlier compensated-control and
first-raise gates:

- the robot remains in Unitree Regular/Motion Control mode while custom arm
  commands use only `rt/arm_sdk`;
- Pinocchio RNEA feedforward uses the checked-out G1 model with a verified
  14-joint SDK/model mapping and conservative per-joint torque bounds;
- the absolute Cartesian interface accepts a reviewed right-hand position and
  orientation, validates the rotation, enforces workspace/motion bounds, and
  chooses the valid deterministic multi-start IK solution with the least
  predicted peak joint speed;
- attempt `handshake-height-10s-b` raised the right hand over 10 seconds,
  settled, held, returned over 10 seconds, released authority, and restored
  `(FSM 501, mode 0)`; and
- the measured settled hand position was
  `[0.264704, -0.139741, 0.102549] m`, `0.007791 m` from the reviewed target,
  with `0.20402 rad/s` peak active-arm speed.

This closes the Track 1A exit criterion for the current gantry-attached robot
configuration. A later own-IK run also closed the bounded Track 1B oscillation
gate: two raised-cosine vertical cycles completed between the verified raise
and return, followed by authority release and restoration of `(FSM 501, mode
0)`. It does not close Track 1C slack-gantry balance validation, autonomous
Cartesian tracking, nonhuman contact, or any human-contact gate. The immutable
requests, telemetry/video evidence, and exact analyses are identified in
`SESSION_HANDOFF.md`.

## Track 1: Custom G1 arm-control substrate

### 1A. Implement `rt/arm_sdk` actuator

Add a dedicated module, preferably:

```text
handshake/g1_arm_controller.py
```

Do not embed the continuous DDS implementation into the already-large
`controller.py`.

The module should own:

1. `rt/arm_sdk` command publishing.
2. `rt/lowstate` subscription or access to a shared LowState source.
3. Verified active-arm joint mapping for the installed G1 configuration.
4. Current arm `q`/`dq` access.
5. Capture of a fixed initial target from measured joint positions before
   authority acquisition; the target must not track measured motion during the
   blend.
6. Gradual arm-SDK authority ramp from 0 to 1.
7. Gradual controlled authority release from 1 to 0.
8. Position, velocity, acceleration, and workspace limits.
9. High-rate interpolation/filtering of lower-rate targets.
10. Timeout, stale-state, cancellation, and safe-return behavior.
11. An explicit phase machine for acquire, raise, settle, shake, return,
    settle, and release, with telemetry-backed transition guards.

The existing `ArmActionRunner` and `G1ArmActionClient` remain useful only as
baseline/reference behavior. The arm-SDK path must neither invoke nor overlap
predefined Unitree actions such as `"shake hand"` or `"release arm"`.

Authority-only tests on 2026-08-27 established this observed transition on the
current robot and firmware:

```text
(FSM 501, mode 0) -> arm-SDK authority -> (FSM 501, mode 1)
-> authority weight 0 -> verified (FSM 501, mode 0)
```

Mode 1 is accepted while arm-SDK authority is active. Release is not complete
until mode 0 is observed again. These empirical meanings must not be assumed
for other firmware without verification.

Three successful zero-offset authority cycles produced small, repeatable joint
motion. With `tau = 0`, PD position error must supply the torque needed to hold
against gravity; this is the leading explanation. Unitree's official
`xr_teleoperate` G1 path supplies Pinocchio RNEA output to the arm controller.
Use the checked-out, robot-matched model with verified joint mapping and torque
signs, finite-value checks, conservative per-joint bounds, authority-consistent
blending, and command telemetry. Raw estimated/measured torque is not an
approved substitute for model-based feedforward.

The original pre-raise validation sequence was completed in this order:

1. Verify model configuration, 14-joint ordering, RNEA signs, and bounds
   offline, including NaN, mapping, stale-state, and model-load failures.
2. Generate and review every position and feedforward-torque command for a
   compensated zero-offset acquire/hold/release cycle without publishing.
3. Run a newly approved gantry-attached compensated zero-offset cycle and
   compare its joint motion, torque, FSM transition, and release against the
   three uncompensated baselines.
4. Review and physically verify bounded raises only after the compensated
   results are repeatable and accepted.

Exit criterion: while the robot remains in Regular/Motion Control mode, the
controller can acquire arm authority without a discontinuity, hold the measured
pose, execute one tiny bounded arm movement, return, and release authority while
Unitree continues to own the lower body.

Status: met for the current gantry-attached configuration. Continue regression
coverage and failure-path testing as the motion envelope expands.

### 1B. Deterministic programmable handshake motion

Before introducing a learned arm policy, implement a deterministic trajectory
through the new actuator.

Required sequence:

```text
capture and hold fixed initial pose
      ↓
acquire arm-SDK authority
      ↓
smooth arm-SDK raise and settle
      ↓
small smooth arm oscillation
      ↓
smooth return and settle
      ↓
controlled arm-SDK authority release
```

Start with the smallest joint set capable of producing a natural handshake.
Generate smooth sinusoidal or otherwise bounded trajectories offline first.
Use low amplitude and frequency and increase only after measured tracking and
balance behavior are understood.

Record commanded `q_target` separately from measured `q` and `dq`.

Exit criterion: a repeatable subtle oscillation operates only during the
approved handshake state, tracks its target within defined bounds, stops on all
abort conditions, and returns safely.

### 1C. Native balance validation with custom arm control

This milestone verifies locally that Unitree's native lower-body stabilization
remains active and effective while `rt/arm_sdk` controls the arms.

Prerequisites:

1. Complete 1A and initial 1B tests.
2. Confirm Regular/Motion Control mode on the installed firmware.
3. Confirm that commands are published only through `rt/arm_sdk`, not
   whole-body `rt/lowcmd`.
4. Define conservative torso attitude, angular-rate, foot/contact, arm-command,
   and recovery-step limits.
5. Use a reviewed, load-rated gantry/fall-arrest setup, clear exclusion zone,
   and emergency-stop operator.

Required tests:

- native standing with arm-SDK authority at zero;
- arm-SDK enabled while commanding the measured pose only;
- one minimum-amplitude arm movement;
- bounded repeated arm oscillation;
- controlled nonhuman contact/disturbance within an approved envelope;
- cancellation and authority release during each stage.

Record torso/IMU state, available foot/contact state, lower-body joint response,
arm targets, arm measured state, authority weight, and controller transitions.

Exit criterion: with the gantry slack and non-load-bearing during nominal
trials, native stabilization or bounded recovery remains repeatable while
custom arm control is active, and cancellation/authority release remains
controlled in every tested event.

### 1D. Staged validation without gantry

Removing the gantry is a separate approval gate, not an automatic continuation
of 1C.

Prerequisites:

1. Complete and review the 1C exit evidence.
2. Resolve every unexplained balance, stepping, controller-ownership, command,
   release, or telemetry anomaly.
3. Define a smaller initial motion and disturbance envelope, test surface,
   clearance area, spotter roles, emergency-stop procedure, and abort criteria.
4. Record explicit approval for the exact robot configuration, firmware,
   controller version, and test procedure.

Validation order:

1. Unsupported standing with arm-SDK disabled and no contact.
2. Arm-SDK enabled with zero motion and no contact.
3. One minimum-amplitude arm movement with the robotic hand open and no contact.
4. Controlled nonhuman-fixture trials within the approved balance envelope.
5. Conservative human-contact testing only after separate review and explicit
   approval of the preceding results.

Exit criterion: the robot completes the approved unsupported trials without an
unplanned step, fall-arrest intervention, unsafe balance excursion, controller
conflict, or uncontrolled arm-authority transition. Any failure returns the
program to gantry-attached testing and closes the without-gantry gate pending
review.

## Track 2: Vision-guided approach

### 2A. Human-hand detection and 3D localization

The existing depth-gated visual invitation is an interim readiness signal. It
must not be treated as semantic hand detection or as a probability estimate.

Required work:

1. Detect a human hand rather than generic scene change.
2. Fuse detection with RealSense depth to estimate `(x, y, z)` in camera
   coordinates.
3. Calibrate and transform the target into the robot torso/arm frame.
4. Add confidence gating, temporal filtering, persistence, ambiguity handling,
   and explicit target-loss behavior.
5. Measure accuracy, safe range, latency, update rate, jitter, occlusion, and
   multiple-hand behavior.
6. Complete validation with overlays and structured logs before commanding
   robot motion.

Exit criterion: a human hand is localized reliably within a defined safe
interaction volume, with uncertainty, ambiguity, staleness, and loss reported
explicitly.

### 2B. Cartesian intent and bounded inverse-kinematics approach

Cartesian and joint-space control are complementary layers, not alternatives.
The interaction layer specifies a 6D target (position and orientation) relative
to the detected hand. Three-dimensional position alone cannot define palm
orientation or a natural elbow and wrist posture. Constrained IK converts the
target into joint positions, and the deterministic actuator executes them with
joint PD and model-based feedforward.

Current implementation supports an immutable absolute world-frame target with
an optional reviewed right-hand orientation, workspace and motion bounds,
deterministic multi-start IK, least-maximum-joint-speed selection, smooth
time-parameterized execution, synchronized evidence, controlled return, and
authority release. A fixed handshake-height target has been physically
verified. Moving-target tracking, collision-aware secondary objectives, and
perception-driven target updates remain open.

Required work:

1. Define a safe 6D end-effector handshake pose relative to the detected hand,
   including approach direction, palm orientation, and standoff distance.
2. Solve IK within verified joint, velocity, acceleration, workspace, and
   collision constraints, with secondary objectives for joint-limit margin,
   preferred elbow/wrist posture, self-collision, and continuity from the
   previous solution.
3. Convert IK output into a smooth time-parameterized joint trajectory; do not
   publish raw per-frame IK solutions. Feed the trajectory through the same
   `rt/arm_sdk` safety and interpolation layer used by deterministic motion.
4. Add target-loss, stale-telemetry, timeout, cancellation, IK-failure, and
   safe-return paths.
5. Validate in order: offline solutions and plots, recorded-target replay,
   command-only dry run, operator-confirmed arm-only motion, then autonomous
   tracking.

Exit criterion: the arm follows a slowly moving test hand inside the approved
workspace and returns safely for every tested loss and failure condition.

## Track 3: Tactile handshake and data

### 3A. Trajectory validation and analysis

Required work:

1. Run the episode validator over all existing JSONL recordings.
2. Classify episodes as successful, aborted, rejected, or incomplete.
3. Report stream completeness, rates, timestamp gaps, dropped samples, field
   ranges, required transitions, and safe-open/arm-release evidence.
4. Plot tactile signals, commanded and measured finger positions, active-arm
   joints, estimated torque, and controller states and events.
5. Publish the telemetry field, rate, unit, and firmware report.
6. Characterize the current high-level Unitree `shake hand` action as a baseline
   but do not treat its opaque trajectory as the future policy actuator.
7. Preserve JSONL as immutable raw evidence. Evaluate Parquet only as a derived
   analysis and training representation.

Exit criterion: every existing episode has a reproducible validation result,
and telemetry quality and active-arm motion are understood well enough to set
defensible control bounds.

### 3B. Extend recording for custom/policy arm control

For each active trajectory, record at minimum:

```text
policy/trajectory target arm q
measured arm q and dq
arm-SDK authority weight
BrainCo hand command and measured state
raw tactile observations
vision observation/state
IMU/body state
observation timestamp
policy inference timestamp
command timestamp
controller state/events
```

Keep commanded and measured quantities distinct. Record policy/model identity,
configuration, and action-filter version as episode metadata once learned
components are introduced.

Exit criterion: actuator tracking error, observation-to-action latency, policy
latency, tactile response, and body response can be reconstructed for every
accepted episode.

## Track 4: Learned vision+touch policy

Learning begins only after the deterministic actuator and safety substrate are
validated.

### Observation space

Candidate observations are:

```text
camera image/features
BrainCo tactile state
BrainCo measured finger positions
G1 arm q/dq
optional IMU/body state
behavioral state / previous action
```

### Action space

Start with bounded position targets:

```text
G1 arm joint q_target
BrainCo finger position targets
```

Do not initially learn `kp`, `kd`, feed-forward torque, whole-body joints, or
balance commands.

The learned policy may run at a lower rate than the arm command loop. A
deterministic actuator layer performs interpolation, rate limiting, clipping,
and stale-action handling.

### Learning sequence

1. Use deterministic trajectories to establish safe control bounds and collect
   richer active-handshake data.
2. Evaluate contact-state and interaction-state estimation offline.
3. Evaluate comfort/pressure prediction and handshake-quality metrics.
4. Train/evaluate bounded next-hand and arm target prediction offline.
5. Shadow mode: run policy inference live but record proposed actions without
   executing them.
6. Execute policy actions only through deterministic constraints and only in a
   restricted interaction state/workspace.
7. Expand action authority gradually based on measured evidence.

End-to-end unconstrained arm control, learned whole-body control, and learned
balance control are out of scope for the initial work.

## Integrated runtime behavior

After the independent tracks meet their exit criteria, the target interaction
is:

```text
detect/localize hand
        ↓
bounded approach through IK or learned target proposal
        ↓
tactile contact
        ↓
controlled BrainCo closing
        ↓
bounded learned/deterministic arm-hand interaction
        ↓
open hand
        ↓
safe arm return and authority release
```

Vision governs approach and readiness. Tactile state governs contact, grasp,
and release. The learned policy proposes movement only inside approved states
and bounds. A shared safety supervisor may stop any subsystem, and no learned
component may bypass it.

Integration exit criteria:

- Every active state can transition to safe return.
- Loss, ambiguity, stale telemetry, timeout, cancellation, command failure, and
  model failure are tested across subsystem boundaries.
- Recording, upload, speech, perception, and model-inference failures do not
  prevent safe hand opening or arm return.
- Operator cancellation and emergency stop remain available throughout the
  interaction.
- Unitree lower-body balance remains under the native motion controller during
  normal policy operation.

## Data, evaluation, and learning milestones

### Recording and validation

- Retain completed, aborted, rejected, and incomplete episodes with explicit
  outcomes and reasons.
- Keep credentials outside source control and verify dataset visibility before
  uploading participant data.
- Do not upload raw faces, names, voices, or biometric embeddings with tactile
  trajectories unless separately approved.
- Collect at least 50 validated local episodes before treating the recording
  format as stable.
- Do not choose a larger learning target solely by episode count; decide based
  on behavioral diversity, participant/session coverage, action diversity, and
  validation performance.
- Split train, validation, and test data by participant and recording session,
  not randomly by frame.

### Human quality labels

Candidate post-episode labels are comfort, grip firmness, arm-motion quality,
successful contact, unexpected behavior, and an optional note. The operator
interface and consent/retention policy remain unresolved.

### Policy evaluation

Evaluate at least:

- task success;
- contact acquisition and release quality;
- pressure/comfort violations;
- arm tracking error;
- action smoothness and rate-limit activation;
- observation-to-action latency;
- model inference latency and stale-action frequency;
- body attitude/balance response;
- abort frequency and safe-return success;
- generalization by participant and session.

## Optional identity-based personalization

Face or identity recognition is not required for hand localization or readiness.
Add it only for a concrete personalization feature, with:

- explicit enrollment and consent;
- pseudonymous identifiers and an explicit `unknown` result;
- protected embeddings and deletion/re-enrollment procedures;
- separation from shareable trajectory data; and
- no identity-based grip changes without explicit preference data and safety
  validation.

## Open decisions

| Decision | Current direction | Status |
|---|---|---|
| Hand side for first dataset | Right hand only | Open |
| Initial recording threshold | 50 validated local episodes before schema freeze | Proposed |
| Larger collection target | Decide from diversity and validation needs, not a fixed count alone | Open |
| Immutable raw format | Current JSONL schema | Implemented; validation open |
| Derived numeric format | Evaluate Parquet after validator work | Proposed |
| Standard training format | Convert a stable schema to LeRobot v3 | Proposed |
| Dataset visibility | Private until policies are settled | Must verify |
| Upload architecture | Post-run upload exists; durable spool is under consideration | Open |
| Camera storage | Decide before vision-policy training; current tactile dataset may remain image-free | Open |
| Identity capability | Hand detection before optional identity recognition | Proposed |
| First learned motor action | Bounded arm `q_target` + BrainCo finger targets | Proposed |
| Arm control runtime | Regular/Motion mode + `rt/arm_sdk` | Selected; gantry-attached coexistence verified |
| Handshake motion representation | 6D Cartesian intent -> constrained posture-aware IK -> smooth bounded joint trajectory | Selected |
| Low-level arm control | Joint PD + bounded Pinocchio RNEA feedforward | Selected; physically validated for bounded raise/return |
| Whole-body `rt/lowcmd` | Not used for normal learned-policy runtime | Selected |
| Waist control | Leave native initially | Selected; revisit only with evidence |
| Learned torque/gains | Out of scope initially | Selected |

## Open technical questions

- Which BrainCo motor fields and units are exposed by the installed SDK and
  firmware?
- At what rate can tactile and motor state be read concurrently without serial
  timeouts?
- What `kp`/`kd` values and command rate are appropriate for conservative
  `rt/arm_sdk` position control as the tested workspace expands beyond the
  verified baseline?
- How should arm-SDK authority and safe return behave during cancellation and
  injected failures in every phase?
- How broadly can the verified handshake-height pose and orientation be varied
  while preserving tracking, clearance, and low body disturbance?
- Which subset of arm joints produces the most natural low-disturbance shake?
- How much lower-body compensation is observed for increasingly dynamic arm
  trajectories while remaining within the approved envelope?
- What localization accuracy and sampling rate are required for a safe approach?
- What 6D handshake-pose convention and secondary IK objectives produce a
  natural elbow and wrist posture across the approved interaction volume?
- Should the first learned action be Cartesian deltas through deterministic IK
  or bounded joint-space action chunks after the approach phase?
- What observation history/chunk length is appropriate for tactile handshake
  dynamics?
- Should raw and processed datasets share a repository?
- What participant consent, retention, deletion, and licensing policies apply?
- What operator interface will collect quality labels and emergency input?

## Immediate next implementation steps

1. Complete Track 1C cancellation and stale-telemetry failure trials in every
   phase of the verified raise/oscillate/return sequence.
2. Quantify repeatability, tracking error, IMU/lower-body response, authority
   release, and safe return over the approved workspace and oscillation envelope.
3. Complete Track 1C slack-gantry balance trials for the
   deterministic raise/shake/return sequence.
4. In parallel with Track 1C work, prototype semantic hand detection and
   RealSense 3D localization in observe-only mode. Do not use perception output
   to command the arm until its accuracy, uncertainty, staleness, and loss
   behavior pass Track 2A.

## References

- [BrainCo RevoHand SDK examples](https://github.com/BrainCoTech/brainco-hand-sdk)
- [Unitree G1 developer documentation](https://support.unitree.com/home/en/G1_developer)
- [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [Unitree XR Teleoperate](https://github.com/unitreerobotics/xr_teleoperate)
- [XR Teleoperate motion-control notes](https://github.com/unitreerobotics/xr_teleoperate/wiki/Motion)
- [Hugging Face upload guide](https://huggingface.co/docs/huggingface_hub/guides/upload)
- [LeRobot Dataset v3](https://huggingface.co/docs/lerobot/main/lerobot-dataset-v3)
