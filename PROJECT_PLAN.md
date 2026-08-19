# Vision-Guided Adaptive Handshake Plan

## Purpose and document boundaries

This is the durable engineering roadmap for extending the Unitree G1 and
BrainCo tactile handshake into a safe, vision-guided, adaptive interaction. It
records milestones, dependencies, safety gates, exit criteria, and unresolved
decisions.

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

Develop two independently testable tracks that converge into a safely bounded
handshake:

```text
Vision track                              Handshake track
Human-hand detection + XYZ                Trajectory validation + analysis
        ↓                                       ↓
Bounded IK hand tracking                  Bounded hold-state arm oscillation
        └──────────────────┼─────────────────╯
                           ↓
              Integrated safety supervisor
```

The longer-term goal is to use recorded interaction data to improve grip,
timing, arm motion, and social behavior without allowing learned components to
bypass deterministic safety constraints.

## Safety principles

- Robot control must not depend on disk, camera, network, or upload availability.
- Hand opening, arm release, cancellation, timeout, and safe return take priority
  over recording, speech, perception, and learned behavior.
- Every failure path should attempt to open the hand before releasing or safely
  returning the arm.
- Learned outputs remain behind deterministic position, velocity, acceleration,
  pressure, torque, temperature, workspace, timeout, and stale-data limits.
- Raw telemetry is immutable. Derived signals must be identified as derived.
- Samples use monotonic receipt timestamps; device clocks are not assumed to be
  synchronized.
- Estimated torque or motor current must not be described as direct joint
  friction.
- Biometric data remains separate from tactile trajectory data. Identity
  recognition requires a concrete feature, explicit enrollment, and consent.
- Physical validation proceeds incrementally: offline, dry run, arm-only, open
  hand, then conservative human contact.

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
| Semantic hand detection and 3D localization | Planned | Prototype and validate in observe-only mode. |
| Bounded inverse-kinematics approach | Planned | Requires calibrated coordinates, verified limits, target-loss handling, and staged hardware validation. |
| Bounded hold-state arm oscillation | Planned | Requires trajectory analysis and confirmation of the supported Unitree control mode. |
| Integrated safety supervisor | Planned | Integrate only after both tracks independently meet their exit criteria. |
| Learning and personalization | Future | Requires validated data, labels, deterministic bounds, and consent controls. |

## Track 1: Vision and arm tracking

### 1A. Human-hand detection and 3D localization

The existing depth-gated visual invitation is an interim readiness signal. It
must not be treated as semantic hand detection or as a probability estimate.

Required work:

1. Detect a human hand rather than generic scene change.
2. Fuse the detection with RealSense depth to estimate `(x, y, z)` in camera
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

### 1B. Bounded inverse-kinematics arm tracking

Required work:

1. Define a safe end-effector handshake pose relative to the detected hand.
2. Solve IK within verified joint, velocity, acceleration, workspace, and
   collision constraints.
3. Smooth and rate-limit the target and commanded trajectory.
4. Add target-loss, stale-telemetry, timeout, cancellation, IK-failure, and
   safe-return paths.
5. Validate in order: offline solutions and plots, command-only dry run,
   operator-confirmed arm-only motion, then autonomous tracking.

Exit criterion: the arm follows a slowly moving test hand inside the approved
workspace and returns safely for every tested loss and failure condition.

## Track 2: Handshake behavior

### 2A. Trajectory validation and analysis

Required work:

1. Implement an episode validator and run it over all existing JSONL recordings.
2. Classify episodes as successful, aborted, rejected, or incomplete.
3. Report stream completeness, rates, timestamp gaps, dropped samples, field
   ranges, required transitions, and safe-open/arm-release evidence.
4. Plot tactile signals, commanded and measured finger positions, active-arm
   joints, estimated torque, and controller states and events.
5. Publish the telemetry field, rate, unit, and firmware report.
6. Characterize the current high-level arm action during `hold` and distinguish
   commanded behavior from human-driven or stationary motion.
7. Preserve JSONL as immutable raw evidence. Evaluate Parquet only as a derived
   analysis and training representation.

Exit criterion: every existing episode has a reproducible validation result,
and telemetry quality and active-arm motion are understood well enough to set
defensible control bounds.

### 2B. Bounded hold-state arm oscillation

The oscillation must not be implemented by repeatedly invoking the fixed
high-level `shake hand` action. Custom joint commands must not overlap an
incompatible high-level controller.

Prerequisites:

1. Confirm active-arm joint names, indices, units, limits, and state rate.
2. Characterize the measured trajectory of the high-level action.
3. Identify the smallest joint set that can produce a natural vertical shake.
4. Verify the supported control mode and safe transitions into and out of
   bounded joint control.

Required behavior:

- Generate and plot a smooth, low-amplitude trajectory offline first.
- Enforce hard position, velocity, acceleration, torque, pressure, duration,
  workspace, and telemetry-freshness limits.
- Stop on tactile release, state exit, excessive pressure or torque, stale data,
  timeout, cancellation, emergency stop, or command failure.
- Validate in order: offline trajectory, live command logging, arm-only motion,
  open robotic hand, then a loose human handshake at minimum amplitude and
  duration.

Exit criterion: a repeatable subtle oscillation operates only during `hold`,
stops before hand-first release, and cannot interfere with safe arm lowering.

## Track integration

Integrate only after both tracks independently satisfy their exit criteria:

```text
detect hand -> estimate XYZ -> bounded IK approach -> tactile contact
-> controlled closing -> bounded hold oscillation -> open hand
-> safe arm return
```

Vision governs approach and readiness. The tactile state machine governs grasp,
hold, and release. A shared safety supervisor may stop either subsystem, and
neither subsystem may bypass it.

Integration exit criteria:

- Every active state can transition to safe return.
- Loss, ambiguity, stale telemetry, timeout, cancellation, and command failure
  are tested across subsystem boundaries.
- Recording, upload, speech, and perception failures do not prevent safe hand
  opening or arm return.
- Operator cancellation and emergency stop remain available throughout the
  interaction.

## Data, evaluation, and learning milestones

### Recording and validation

- Retain completed, aborted, rejected, and incomplete episodes with explicit
  outcomes and reasons.
- Keep credentials outside source control and verify dataset visibility before
  uploading participant data.
- Do not upload raw faces, names, voices, or biometric embeddings with tactile
  trajectories.
- Collect 50 validated local episodes before treating the recording format as
  stable.
- Collect 300–500 carefully labeled episodes before deciding whether the data
  supports learned control.
- Split train, validation, and test data by participant and recording session,
  not randomly by frame.

### Human quality labels

Candidate post-episode labels are comfort, grip firmness, arm-motion quality,
successful contact, unexpected behavior, and an optional note. The operator
interface and consent/retention policy remain unresolved.

### Learning sequence

1. Compare a contact-state estimator with the deterministic threshold state
   machine without deploying it as the safety authority.
2. Evaluate comfort and pressure prediction.
3. Evaluate a bounded next-hand command that can only make small close, hold,
   or open decisions through deterministic constraints.
4. Consider bounded timing and amplitude adaptation of a fixed, verified arm
   trajectory only after hand models are reliable.

End-to-end unconstrained arm control is out of scope for the initial learning
work.

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
| Initial recording threshold | 50 validated local episodes | Proposed |
| Larger collection target | 300–500 labeled episodes | Proposed |
| Immutable raw format | Current JSONL schema | Implemented; validation open |
| Derived numeric format | Evaluate Parquet after validator work | Proposed |
| Standard training format | Convert a stable schema to LeRobot v3 | Proposed |
| Dataset visibility | Private until policies are settled | Must verify |
| Upload architecture | Post-run upload exists; durable spool is under consideration | Open |
| Camera storage | No images in the initial trajectory dataset | Proposed |
| Identity capability | Hand detection before optional identity recognition | Proposed |
| First ML task | Contact state and comfort/pressure prediction | Proposed |
| Arm control | Fixed high-level action before bounded adaptation | Proposed |

## Open technical questions

- Which BrainCo motor fields and units are exposed by the installed SDK and
  firmware?
- At what rate can tactile and motor state be read concurrently without serial
  timeouts?
- Which Unitree state and command interfaces are supported in the robot's
  current operating mode?
- Does estimated arm torque remain meaningful while the high-level action
  service is active?
- What are the verified active-arm and torso joint indices and limits?
- What preparation pose is safest on the installed firmware?
- What localization accuracy and sampling rate are required for a safe approach?
- Should raw and processed datasets share a repository?
- What participant consent, retention, deletion, and licensing policies apply?
- What operator interface will collect quality labels and emergency input?

## References

- [BrainCo RevoHand SDK examples](https://github.com/BrainCoTech/brainco-hand-sdk)
- [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [Hugging Face upload guide](https://huggingface.co/docs/huggingface_hub/guides/upload)
- [LeRobot Dataset v3](https://huggingface.co/docs/lerobot/main/lerobot-dataset-v3)
