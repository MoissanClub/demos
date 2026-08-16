# Vision-Guided Adaptive Handshake Plan

## 1. Project objective

Extend the existing Unitree G1 and BrainCo tactile handshake demo along two
parallel development tracks that converge into a safe, vision-guided handshake:

1. Record synchronized tactile, hand motor, arm joint, and event data for each handshake.
2. Validate and upload completed trajectories to a private Hugging Face dataset.
3. Use the collected data to develop safer, more natural hand-pressure control and bounded arm-motion adaptation.
4. Detect a human hand and estimate its three-dimensional position.
5. Track the hand with a safely bounded inverse-kinematics arm controller.
6. Optionally recognize consenting, enrolled people for explicitly defined personalization features.

The two tracks remain independently testable until their safety and exit
criteria are satisfied:

```text
Vision track                              Handshake track
Human-hand detection + XYZ                Trajectory validation + analysis
        ↓                                       ↓
Bounded IK hand tracking                  Bounded hold-state arm oscillation
        └──────────────────┬─────────────────┘
                           ↓
              Integrated safety supervisor
```

## 2. Guiding principles

- Physical control must not depend on disk, camera, network, or upload availability.
- Robot control has higher priority than recording and uploading.
- Every failure path should attempt to open the hand and release or safely return the arm.
- Record raw telemetry before creating derived signals.
- Do not label estimated torque or motor current as direct joint friction. Friction may be estimated later from position, velocity, torque, current, and temperature.
- Use monotonic timestamps at data receipt; do not assume device clocks are synchronized.
- Store every episode locally before attempting an upload.
- Keep learned outputs behind deterministic position, velocity, pressure, torque, timeout, and stale-data limits.
- Keep biometric data separate from the tactile trajectory dataset.
- Begin with face detection. Add identity recognition only if a concrete feature requires it.

## 3. Current baseline

The current implementation is a standalone Python program that:

- Communicates directly with one BrainCo hand over Modbus.
- Reads tactile values from capacitive or array-pressure hardware.
- Runs an `open_wait -> closing -> hold -> open_wait` state machine.
- Optionally invokes Unitree's high-level `shake hand` and `release arm` actions.
- Defaults to the right hand when no hand is selected.

Before extending it, address these known baseline issues:

- Ctrl-C cleanup may occur outside the coroutine and skip hand reopening.
- Unexpected SDK or serial failures can close Modbus without first opening the hand or releasing the arm.
- The Unitree arm client can be called concurrently by the action thread and immediate-release path.
- Partial `--port` or `--slave-id` overrides can implicitly select mismatched left-hand defaults.
- Threshold ordering is not validated.
- The README disagrees with the implementation about the default maximum closure.
- The control state machine has no hardware-independent automated tests.

## 4. Phase 0: Stabilize the current controller

### Tasks

1. Move safe hand-open and arm-release behavior into a reliable finalization path.
2. Handle task cancellation as well as `KeyboardInterrupt`.
3. Serialize all calls to the Unitree arm action client.
4. Add stale-tactile-data and control-loop timeout handling.
5. Validate configuration before opening hardware:
   - `release_threshold < start_threshold < stop_threshold`
   - nonnegative durations
   - valid hand port and slave combinations
   - bounded close command and thumb scale
6. Extract the state transition logic from hardware I/O.
7. Add unit tests for normal contact, early release, hold timeout, repeated contact, bad telemetry, and cancellation.
8. Reconcile README defaults with the implementation.
9. Tag the stable version as the recording baseline.

### Exit criteria

- Ten normal handshakes succeed.
- Ten interrupted handshakes return safely.
- Every simulated failure path invokes safe cleanup.
- State-machine tests pass without connected hardware.

## 5. Phase 1: Telemetry discovery

Create a read-only telemetry probe before integrating recording into the controller.

### BrainCo hand investigation

Inspect the installed SDK results from:

- `get_touch_sensor_status()`
- `get_array_pressure_touch_data()`, when supported
- `get_motor_status()`
- device and firmware information APIs

For every field, document:

- Field name and Python type
- Unit or raw scaling
- Observed minimum and maximum
- Sustainable sampling frequency
- Read latency and jitter
- Whether motor-status polling reduces tactile sampling rate

### Unitree arm investigation

Subscribe read-only to G1 state while retaining the proven high-level action controller. Inspect:

- Joint position
- Joint velocity
- Estimated torque
- Motor current, if exposed
- Motor temperature
- Motor state and error code
- IMU orientation and angular velocity
- Robot/control mode
- The mapping between joint array indices and named joints

Do not switch to low-level arm control during this phase.

### Timing investigation

Measure tactile, hand-motor, and Unitree DDS rates independently. Timestamp samples on receipt with `time.monotonic_ns()` and also record wall-clock UTC at the episode level.

### Deliverable

Create a telemetry report containing available fields, units, rates, latencies, firmware versions, and unresolved semantics.

### Exit criteria

- Every selected feature has an understood name and shape.
- Unknown units are explicitly marked as raw rather than guessed.
- Target recording rates are based on measured hardware behavior.

## 6. Phase 2: Define the episode lifecycle

Use an explicit lifecycle for every handshake:

```text
IDLE
  ↓ contact detected
CONTACT_DETECTED
  ↓ arm action requested
CLOSING
  ↓ pressure or close limit reached
HOLDING
  ↓ release or hold timeout
RELEASING
  ↓ hand open and arm released
COMPLETE

Any active state
  ↓ exception, emergency stop, invalid data, or operator rejection
ABORTED
```

Create the episode UUID when recording begins. Keep a two-second in-memory ring buffer so an episode can include:

- One to two seconds before first contact
- The complete handshake
- One to two seconds after release

Completed and aborted episodes must both remain inspectable. Aborted episodes must contain an explicit reason.

## 7. Phase 3: Dataset schema

Preserve raw asynchronous streams and generate an aligned table as a derived artifact. Use a schema version from the first recording.

### Episode metadata

```yaml
episode_id:
schema_version:
started_at_utc:
robot_id:
hand_side:
brainco_serial:
brainco_firmware:
unitree_firmware:
software_git_commit:
result: success | aborted | rejected
abort_reason:
operator_rating:
comfort_rating:
person_id: anonymous_or_consented_pseudonym
environment:
planned_condition:
control_parameters:
  start_threshold:
  stop_threshold:
  release_threshold:
  release_seconds:
  max_close:
  step:
  period:
  hold_duration:
  thumb_scale:
  arm_action:
  arm_release_action:
```

### Tactile stream

Preserve per-finger raw values rather than only the maximum metric:

```text
timestamp_ns
episode_time_s
finger
normal_force_1
normal_force_2
normal_force_3
tangential_force_1
tangential_force_2
tangential_force_3
sensor_status
touch_metric_derived
```

For array-pressure hardware, preserve all raw force and moment axes and the documented scale.

### Hand motor stream

```text
timestamp_ns
commanded_positions[6]
measured_positions[6]
measured_velocities[6], if available
motor_current[6], if available
estimated_torque[6], if available
temperature[6], if available
```

### Arm and robot stream

```text
timestamp_ns
joint_position[N]
joint_velocity[N]
estimated_torque[N]
motor_current[N], if available
motor_temperature[N]
motor_state[N]
imu_orientation
imu_angular_velocity
robot_mode
```

Record all joints in the raw stream even if the first model uses only the active arm.

### Event stream

```text
timestamp_ns
event
state_before
state_after
touch_metric
close_command
arm_action
reason
```

Expected events include:

- `contact_started`
- `closing_started`
- `pressure_limit_reached`
- `max_close_reached`
- `arm_action_requested`
- `release_detected`
- `episode_completed`
- `episode_aborted`
- `emergency_stop`

### Human quality labels

Collect a short post-episode assessment:

- Comfort: 1-5
- Grip firmness: too soft / good / too hard
- Arm motion: awkward / acceptable / natural
- Successful contact: yes/no
- Unexpected behavior: yes/no
- Optional note

## 8. Phase 4: Non-blocking recorder

Refactor toward this structure:

```text
handshake/
  controller.py
  brainco_reader.py
  unitree_reader.py
  episode_recorder.py
  schema.py
  uploader.py
  face_detector.py
  readiness_controller.py
  cli.py
tests/
```

Runtime flow:

```text
BrainCo reader ─┐
Unitree reader ─┼─> bounded queues ─> episode recorder ─> local spool
Controller ─────┘
```

### Recorder requirements

- Queue writes cannot block robot control.
- Every sample carries an independent timestamp.
- Queue overflow increments a dropped-sample counter.
- Data flushes incrementally to disk.
- Episodes are written into temporary directories.
- Completion uses an atomic rename or completion marker.
- An interrupted write is recoverable and classified as aborted.
- Numeric streams use Parquet unless telemetry experiments identify a concrete incompatibility.
- Raw streams remain immutable; aligned/resampled tables are derived outputs.

### Exit criteria

- Fifty local episodes record successfully without Hub access.
- Control timing remains within its target bounds while recording.
- Forced recorder failure does not affect safe hand control.

## 9. Phase 5: Validation and review tools

Create an episode validator that reports:

- Stream duration and sample count
- Effective sampling rates
- Dropped samples
- Timestamp gaps and non-monotonic samples
- Tactile ranges
- Joint position, velocity, torque, and temperature ranges
- Required state transitions
- Whether safe open and release commands occurred
- NaN, malformed, or missing features
- Firmware and schema compatibility

Generate per-episode plots containing:

1. Per-finger tactile signals.
2. Commanded and measured finger position.
3. Arm joint position and velocity.
4. Estimated arm torque and motor current, when available.
5. State-machine regions.
6. Contact, pressure-limit, hold, release, and abort markers.

Invalid episodes move to a rejected area and are not silently included in training data.

## 10. Phase 6: Hugging Face dataset upload

Start with a private dataset repository and a local-first spool:

```text
dataset-spool/
  pending/
  uploading/
  uploaded/
  rejected/
```

### Upload workflow

1. Complete or abort the physical episode.
2. Close and validate all local files.
3. Move a valid episode to `pending`.
4. Let a separate uploader process claim the episode.
5. Upload with an idempotent remote path based on the episode UUID.
6. Verify the remote commit.
7. Record the commit hash locally.
8. Move the local episode to `uploaded`.
9. Retry safely after interruption.

Uploading must never happen synchronously in the control loop.

### Format progression

- First 50 episodes: local files only.
- Next 100 episodes: private Hub repository using episode-oriented staging directories.
- After the schema stabilizes: batch conversion to LeRobot Dataset v3.
- Keep old schema versions readable; never silently rewrite historical data.

### Security

- Keep Hugging Face credentials outside source control.
- Do not upload raw faces, names, voices, or biometric embeddings with tactile trajectories.
- Keep the dataset private until consent, licensing, anonymization, and quality policies are settled.

## 11. Phase 7: Initial data collection campaign

Collect controlled variation across:

- Hand side, if both sides will be supported
- Different consenting participants and hand sizes
- Light, normal, and firm contact
- Short and long holds
- Early release
- Failed or missed contact
- Contact concentrated on different fingers
- Robot/person height difference
- Modest approach-angle variation
- Intentional aborts
- Multiple sessions, days, and motor temperatures

Use a session plan and preserve the planned condition in metadata rather than varying all factors randomly.

### Initial target

Collect 300-500 carefully labeled episodes before deciding whether the data supports learned control. Split train, validation, and test sets by participant and recording session, not randomly by frame.

## 12. Phase 8: Learning roadmap

### Model A: Contact-state estimator

Inputs:

- Recent tactile history
- Finger positions and velocities
- Motor current or estimated torque, when available

Outputs:

- No contact
- Initial contact
- Stable grip
- Excessive pressure
- Released

Compare this model against the current threshold state machine before deploying it.

### Model B: Bounded next-hand command

Inputs:

- A 0.5-1.0 second tactile window
- Finger position and velocity
- Previous command

Outputs:

- Small close increment
- Hold
- Open

The deployed output must pass through deterministic constraints:

- Position bounds
- Maximum command change per cycle
- Tactile pressure limit
- Torque/current limit
- Temperature limit
- Release override
- Stale-data timeout
- Emergency-open command

### Model C: Bounded arm adaptation

Only after Models A and B are reliable:

- Learn timing and amplitude adjustments to a fixed, verified trajectory.
- Keep joint position, velocity, acceleration, torque, and workspace limits deterministic.
- Do not initially allow a learned model to generate unconstrained joint commands.

The first useful ML target should be comfort/pressure prediction, not end-to-end arm control.

## 13. Phase 9: Camera and face detection

Face detection answers whether a person is present. Face recognition identifies an enrolled person. Detection is sufficient for readiness behavior.

### Tasks

1. Identify the camera interface and available stream format.
2. Measure resolution, frame rate, latency, and field of view.
3. Run face detection without robot motion.
4. Estimate face center, size, confidence, and persistence.
5. Define a central interaction zone and useful distance range.
6. Reject small, partial, distant, transient, and ambiguous detections.
7. Test multiple-face behavior.
8. Initially log detection metadata without retaining images.

One detected frame must never immediately command arm motion.

## 14. Phase 10: Face-triggered readiness supervisor

Add a supervisor above the handshake controller:

```text
NO_PERSON
  ↓ stable face for configured duration
PERSON_PRESENT
  ↓ robot safe and person inside interaction zone
ARM_PREPARING
  ↓ verified half-raised pose reached
READY_FOR_CONTACT
  ↓ tactile contact
HANDSHAKE_ACTIVE
  ↓ release or timeout
RETURNING
  ↓ safe pose reached
COOLDOWN
```

Every active state must be able to transition to `SAFE_RETURN`.

### Safe-return triggers

- Face disappears for a configured interval
- Multiple faces make the target ambiguous
- Person enters an unsafe proximity zone
- Tactile telemetry becomes stale
- Joint telemetry becomes stale
- Arm action fails or times out
- Readiness pose is not reached
- Emergency stop
- Operator cancellation

Use a separately verified half-raised preparation pose. Do not reuse a full handshake action if it begins shaking immediately.

## 15. Phase 11: Optional face recognition

Add recognition only after face detection and readiness behavior are reliable and a concrete personalization requirement exists.

### Requirements

1. Explicit enrollment and consent.
2. Pseudonymous person IDs.
3. Encrypted face embeddings rather than raw images where practical.
4. Separation from public or shareable trajectory data.
5. Deletion and re-enrollment procedures.
6. An explicit `unknown` outcome.
7. Several consistent frames before confirming identity.
8. No identity-based grip-force changes without explicit preference data and safety validation.

## 16. Two-track high-level plan

### Track 1: Vision and arm tracking

#### 1A. Human-hand detection and 3D localization

1. Replace generic foreground-motion detection with a detector that identifies
   a human hand.
2. Combine the detection with RealSense depth to estimate a target `(x, y, z)`
   in camera coordinates.
3. Calibrate and transform the target into the robot torso/arm coordinate frame.
4. Stabilize the estimate with confidence gating, temporal filtering, target
   persistence, and explicit target-loss behavior.
5. Measure detection accuracy, range, latency, update rate, and jitter.
6. Complete this stage in observe-only mode with overlays and structured logs;
   it must not command robot motion.

Exit criterion: a human hand can be localized reliably inside a defined safe
interaction volume, with uncertainty and loss reported explicitly.

#### 1B. Arm tracking through bounded inverse kinematics

1. Define a safe end-effector handshake pose relative to the detected hand.
2. Solve IK within verified joint, velocity, acceleration, workspace, and
   collision constraints.
3. Smooth and rate-limit both the target and commanded joint trajectory.
4. Add target-loss, stale-telemetry, timeout, cancellation, IK-failure, and
   safe-return paths.
5. Validate in stages: offline solutions and plots, command-only dry run,
   operator-confirmed arm-only motion, then autonomous tracking.

Exit criterion: the arm follows a slowly moving test hand inside the approved
workspace and returns safely for every loss or failure condition.

### Track 2: Handshake behavior

#### 2A. Trajectory validation and data analysis

1. Implement an episode validator and run it over all existing recordings.
2. Classify episodes as successful, aborted, rejected, or incomplete.
3. Report stream completeness, sampling rates, timestamp gaps, dropped samples,
   field ranges, required transitions, and safe-open/arm-release evidence.
4. Plot tactile signals, commanded and measured finger positions, active-arm
   joints, estimated torque, and controller states.
5. Publish the telemetry field/rate/unit report and characterize the current
   high-level handshake action.
6. Treat JSONL as the immutable raw format for now; evaluate Parquet as a
   derived analysis/training representation.
7. Diagnose the unfinalized trajectory from the 2026-08-16 09:46 session.

Exit criterion: every existing episode has a reproducible validation result,
and the active arm motion and telemetry quality are understood well enough to
define control bounds.

#### 2B. Bounded hold-state arm oscillation

1. Use the analyzed arm trajectory to select the smallest appropriate joint set.
2. Generate and plot a smooth, low-amplitude oscillation offline.
3. Enforce hard position, velocity, acceleration, torque, duration, pressure,
   and telemetry-freshness limits.
4. Stop immediately on tactile release, state exit, excessive pressure or
   torque, stale data, timeout, cancellation, or command failure.
5. Validate in stages: offline trajectory, live command logging, arm-only
   motion, open robotic hand, and finally a loose human handshake at minimum
   amplitude and duration.

Exit criterion: a repeatable subtle oscillation operates only during `hold`,
stops before hand-first release, and cannot interfere with safe arm lowering.

### Track integration

Integrate the tracks only after each subsystem meets its independent exit
criteria:

```text
detect hand -> estimate XYZ -> bounded IK approach -> tactile contact
-> controlled closing -> bounded hold oscillation -> open hand
-> safe arm return
```

Vision governs approach and readiness. The tactile state machine governs grasp,
hold, and release. A shared safety supervisor can stop either subsystem, and
neither subsystem may bypass it.

## 17. Immediate execution plan

The next deliverable is **validated trajectory analysis plus an observe-only
3D hand-localization prototype**. Work proceeds in parallel where practical.

### Handshake/data work

1. Implement the episode validator.
2. Run it over all current trajectories and investigate the incomplete 09:46
   recording.
3. Generate plots and the telemetry field/rate/unit report, emphasizing the
   right-arm joints during the current high-level action.
4. Use the results to propose the oscillating joint set, initial amplitude,
   frequency, duration, and hard safety bounds.
5. Implement only an offline oscillation trajectory generator and plots at this
   stage; do not send new joint commands to hardware yet.

### Vision work

1. Establish and measure the RealSense color/depth baseline.
2. Select and prototype a human-hand detector.
3. Fuse the detected hand with depth and log filtered `(x, y, z)` coordinates.
4. Define the camera-to-robot calibration procedure and the initial safe
   interaction volume.
5. Evaluate accuracy, jitter, latency, occlusion, multiple-hand ambiguity, and
   target loss in observe-only mode.

### Planning and data decisions

1. Update completed milestones as validation evidence is produced.
2. Preserve JSONL recordings as immutable raw evidence.
3. Decide the derived Parquet layout only after the validator exposes the
   actual stream shapes and analysis requirements.
4. Continue collecting toward fifty validated real episodes after validation
   and labeling tools are available.

### Next step: non-Chinese greeting support

The G1 built-in `TtsMaker()` service currently produces acceptable Mandarin but
does not provide reliable English or general multilingual pronunciation. Add a
language-independent greeting path using `AudioClient.PlayStream()`:

1. Add `greeting_audio_file` and `greeting_language` to `handshake_config.json`.
2. Load and validate a local WAV asset before starting robot motion.
3. Convert or require the exact PCM sample format accepted by the Unitree audio service.
4. Stream the audio asynchronously when the controller enters `hold`.
5. Retain built-in TTS for Mandarin and use audio-file playback for other languages.
6. Define deterministic fallback behavior when the file is missing, malformed, or playback fails.
7. Add unit tests for language routing, audio validation, one-shot playback, and fallback behavior.
8. Add documented English and Mandarin example configurations.
9. Keep generated audio licensing and voice-consent metadata with each distributable asset.

Exit criterion: Mandarin TTS and at least one English WAV greeting both play
clearly on the robot without blocking hand control or affecting safe cleanup.

### Future step: controlled hold-state arm shake

Add a subtle arm oscillation while the handshake controller is in `hold`. This
must not be implemented by repeatedly invoking the fixed high-level `shake hand`
action, and custom joint commands must not run concurrently with an incompatible
high-level arm controller.

Prerequisites:

1. Complete the Unitree read-only telemetry probe.
2. Confirm the active arm's joint names, indices, units, limits, and state rate.
3. Record the joint trajectory produced by the current high-level handshake action.
4. Identify which single joint or minimal joint set produces a natural vertical shake.
5. Verify the supported control mode and safe transition between high-level and bounded joint control.

Initial motion envelope for hardware validation:

- One verified arm joint, or the smallest safe coupled joint set
- Sinusoidal trajectory with smooth acceleration
- Approximately 1-2 degrees of amplitude
- Approximately 1-1.5 Hz
- Maximum duration of 2-3 seconds
- Existing joint position, velocity, acceleration, torque, and workspace limits remain authoritative

Configuration should eventually include:

```yaml
hold_shake_enabled: false
hold_shake_amplitude_degrees: 1.0
hold_shake_frequency_hz: 1.0
hold_shake_duration_seconds: 2.0
```

The oscillation must stop immediately when:

- The controller exits `hold`
- Tactile release begins
- Pressure or estimated torque exceeds a safety limit
- Tactile or joint telemetry becomes stale
- The configured shake duration expires
- An operator cancels or emergency stop occurs
- Any arm command or state validation fails

Rollout order:

1. Generate and plot the trajectory without hardware.
2. Run dry-run command logging against live joint telemetry.
3. Test arm-only motion with no person and no hand contact.
4. Test with an open robotic hand.
5. Test a loose human handshake at minimum amplitude and duration.
6. Increase parameters only within the verified motion envelope.

Exit criterion: a repeatable, subtle hold-state shake stops before the hand-first
`releasing` sequence and cannot interfere with measured-open confirmation or arm lowering.

## 18. Decisions to make

These decisions should be recorded here as they are resolved:

| Decision | Current recommendation | Status |
|---|---|---|
| Hand side for first dataset | Right hand only | Open |
| First recording target | 50 local, then 300-500 labeled | Proposed |
| Numeric storage | Parquet raw streams | Proposed |
| Standard dataset format | Convert stable schema to LeRobot v3 | Proposed |
| Hub visibility | Private | Proposed |
| Upload timing | After episode validation, separate process | Proposed |
| Camera storage | No images in initial dataset | Proposed |
| Face capability | Detection before recognition | Proposed |
| First ML task | Contact state and comfort/pressure prediction | Proposed |
| Arm control | Fixed high-level action before bounded adaptation | Proposed |

## 19. Open technical questions

- Which BrainCo motor fields and units are exposed by the installed SDK and firmware?
- At what rate can tactile and motor status be read concurrently without serial timeouts?
- Which Unitree state topic and message type are available in the robot's current operating mode?
- Does estimated arm torque remain meaningful while using the high-level action service?
- What are the exact G1 joint indices for the active arm and torso?
- What half-raised arm pose or supported action is safest on this firmware?
- Which camera and transport interface should supply face frames?
- What sampling rates are required for useful pressure learning?
- Should raw asynchronous streams and aligned training data live in one repository or separate raw/processed repositories?
- What participant consent and dataset retention policy will apply?
- What operator interface will collect comfort labels and emergency cancellation?

## 20. References

- [BrainCo RevoHand SDK examples](https://github.com/BrainCoTech/brainco-hand-sdk)
- [Unitree SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [Hugging Face upload guide](https://huggingface.co/docs/huggingface_hub/guides/upload)
- [LeRobot Dataset v3](https://huggingface.co/docs/lerobot/main/lerobot-dataset-v3)

## 21. Plan revision log

| Date | Revision | Summary |
|---|---:|---|
| 2026-08-04 | 0.1 | Initial phased project plan created from design brainstorming. |
| 2026-08-04 | 0.2 | Implemented Phase 0 software hardening and tests; physical validation remains open. |
| 2026-08-04 | 0.3 | Added a configurable, non-blocking speaker greeting on entry to the hold state. |
| 2026-08-04 | 0.4 | Switched the initial greeting to Mandarin for compatibility with the G1 built-in TTS service. |
| 2026-08-04 | 0.5 | Documented non-Chinese greeting support through configurable PCM/WAV streaming as the next audio step. |
| 2026-08-04 | 0.6 | Added a hand-first releasing state with measured-open confirmation before lowering the arm. |
| 2026-08-04 | 0.7 | Added a future bounded hold-state arm oscillation plan gated on telemetry and controller-mode validation. |
| 2026-08-16 | 0.8 | Added the Phase 1 read-only BrainCo and Unitree telemetry discovery probe. |
| 2026-08-16 | 0.9 | Reorganized future work into parallel vision/IK and handshake-analysis/oscillation tracks with a shared immediate plan. |
