# Session Handoff

Last updated: 2026-09-02 (Asia/Shanghai)

## Current pause point: immutable own-IK trace replay verified

The operator confirmed the robot was under the gantry, the arm workspace was
clear, and a dedicated emergency-stop operator was present. The USB verifier
camera recorded the complete moving arm and upper body. The successful guarded
run is:

```text
artifacts/robot_dev_runs/20260901T182607.243046Z_trace-replay-f4fa-25pct-slow-a/
```

The selected immutable source was:

```text
telemetry/trajectories/20260816T004441Z/trajectory_20260816T004450.063057Z_f4fa0507-ba92-4675-902c-168294565857.jsonl
SHA-256: f272f4f03756ca7405603d69709d698843f56d9943be38d16307f2aa0baa191a
```

It was selected because its source commands contain a complete arm raise and
release, measured oscillation, a BrainCo close/open command cycle, a successful
lifecycle, and zero dropped samples. The reviewed replay request is:

```text
reviewed_cartesian_requests/trace_replay_f4fa_25pct_slow_a.json
SHA-256: 8b65ac8e55112019660df9dc49dcd4b09922ac94b407dffd357e1a2d2caa0ec0
```

The request scales the source Cartesian offsets to 25%, stretches time by 4x,
detrends the sampled trace to an exact closed endpoint, and solves every
waypoint through the project's own IK with bounded cubic interpolation and
RNEA feedforward. The physical run completed the normalized BrainCo close
command ramp, 10-second raise, 14-second trace, source-timed hand open,
10-second return, authority release, and final fail-safe hand open. It did not
complete a full-hand physical close: the measured state progressed from
`[1000,510,490,490,500,500]` to `[1000,950,950,490,500,500]`, so only channels
1 and 2 moved substantially while channels 0 and 3--5 stayed at their open
values. Video likewise shows the fingers largely open. The later open command
returned all six channels exactly to the captured reference.

Measured trace peak-to-peak motion was `[1.798, 1.439, 6.877] mm` against
requested `[2.635, 1.303, 7.982] mm`. Peak active joint velocity was
`0.210155 rad/s`, below the `0.25 rad/s` controller limit. The raised hand was
`7.564 mm` from the requested center. Return settled within
`[1.110, 0.494, 0.035] mm` of the initial hand position and `0.003356 rad`
maximum joint residual. Independent postflight passed `(FSM 501, mode 0)` and
found all six BrainCo motors idle at the exact captured open reference. The arm
trace replay therefore passed, while the hand-close criterion remains open.

Three preceding attempts aborted safely before any arm command publication:
the first exposed an incorrect BrainCo baud rate, the second an optional TTS
failure, and the third the current Revo2 calibration's requirement for the
positions-and-speeds API. Each attempt has timestamped video analysis and
checksum-protected evidence. The successful run's `verification.md` contains
the complete telemetry assessment and a sentence-level timestamp/frame/evidence
video review.

Physical execution is hard-disabled again in
`run_g1_reviewed_trace_replay.py`: `PHYSICAL_EXECUTION_ENABLED = False` and
`AUTHORIZED_REQUEST_SHA256 = None`. The full offline suite passes 186 tests
with pytest importlib mode.

The immediate next engineering step is to identify the installed Revo2
six-channel unit mapping and calibrated range, verify each channel individually
with conservative no-contact commands, and achieve a measured and visibly
confirmed full-hand close/open cycle before repeating the combined trace.
Repeatability and arm cancellation trials follow that correction.

## Current pause point: own-IK handshake-height run verified

The operator confirmed a clear arm workspace, standard mode under the gantry,
and a dedicated emergency-stop operator. The front USB verifier camera was
validated at 640x480 MJPG and measured approximately 30 fps with the full right
arm and upper body visible. Independent preflight and postflight checks passed
`(FSM 501, mode 0)`.

The Cartesian interface now accepts an optional reviewed right-hand rotation,
validates it as an SO(3) matrix, includes it in the immutable request hash, and
uses deterministic five-candidate IK selection for the least predicted peak
joint speed. The reviewed handshake-height target was:

```text
[0.2649462353, -0.1376723351, 0.1100564379] m
duration: 10.0 s
maximum planned joint speed: 0.16 rad/s
```

Attempt A (`handshake-height-10s-a`) moved smoothly to handshake height and
held, but was marked aborted because the measured joints did not enter the
overly strict 0.010 rad settle band within 30 seconds. Controlled authority
release and native return completed; postflight passed. This attempt must not
be relabeled as successful.

Attempt B (`handshake-height-10s-b`) used the controller's existing 0.030 rad
tracking band as its joint settle tolerance and completed the raise, settle,
one-second hold, 10-second controlled return, return settle, authority release,
and native-control handoff. The immutable request is:

```text
reviewed_cartesian_requests/handshake_height_10s_b.json
SHA-256: e2d25bbe33dbeb7644fdaf6a6b3d0655ece8f0efed5c6b73e3aec86e976e8d12
```

Measured settled right-hand position was
`[0.264704, -0.139741, 0.102549] m`, a `0.007791 m` Cartesian error from the
requested point. Peak active arm speed was `0.20402 rad/s`; peak release speed
was `0.21936 rad/s`, both below the `0.25 rad/s` measured-velocity abort limit.
The return settled within `[0.002119, 0.001108, 0.000138] m` of the initial
right-hand point and `0.008281 rad` maximum joint residual. Video review showed
a smooth forward raise, stable handshake-level hold, and smooth return with no
visible contact or oscillation.

Local evidence and checksum-protected analysis:

```text
artifacts/robot_dev_runs/20260901T165655.503579Z_handshake-height-10s-a/
artifacts/robot_dev_runs/20260901T170015.199657Z_handshake-height-10s-b/
```

Physical execution is hard-disabled again:
`PHYSICAL_EXECUTION_ENABLED = False` and `AUTHORIZED_REQUEST_SHA256 = None`.
The offline suite passes 177 tests with pytest importlib mode. The default
pytest importer can encounter duplicate archived test module names through
stale `__pycache__`; use `python -m pytest -q --import-mode=importlib`.

## Current pause point: forward scenario campaign stopped at scenario 2

The operator confirmed the robot was secured, the arm space was clear, and a
dedicated emergency-stop operator was present. Ten positive-world-X forward
scenarios were defined in
`reviewed_cartesian_requests/forward_scenario_suite.json`. Runtime planning now
uses deterministic multi-start IK and selects the valid solution with the least
predicted maximum joint speed. Physical execution remains hard-disabled.

Scenario 1 (`forward-01-1cm-a`) passed. It measured `0.009931 m` forward with
only `-0.000272 / -0.000011 m` lateral/vertical deviation, returned within
`0.000165 / -0.000067 / -0.000054 m`, and passed synchronized video review and
independent `(501, 0)` postflight.

Scenario 2 (`forward-02-2cm-a`) reached `0.019696 m` forward but failed the
tightened `0.005 rad` joint settle gate after its 30-second timeout. No normal
return phase began. Controlled abort release completed, native control returned
the arm, synchronized video showed smooth unobstructed motion and return, and
independent postflight passed `(501, 0)`. The campaign stopped as designed;
scenarios 3--10 were not physically executed. Full telemetry and video findings
are appended to each run's `verification.md` and protected by regenerated
checksums:

```text
artifacts/robot_dev_runs/20260831T172733.272821Z_forward-01-1cm-a/
artifacts/robot_dev_runs/20260831T172941.860239Z_forward-02-2cm-a/
```

## Current pause point: physical right-arm motion completed; target did not pass

One guarded attempt, `right-004-neg020-neg010-21s-b`, physically moved the
right arm outbound and returned it after the original 20-second request failed
its offline `0.0200 rad/s` trajectory-velocity gate. The replacement retained
the same endpoint, workspaces, and limits but used 21-second outbound and return
phases. Its canonical request is:

```text
reviewed_cartesian_requests/right_004_-020_-010_21s.json
SHA-256: bacb3dc04332adc685132d7333e81f6b72c02835751d70a7cf26691320cb51b9
```

The controller completed outbound, return, settle, authority release, and
native return. An independent read-only postflight passed `(FSM 501, mode 0)`.
The exact target assessment did not pass: maximum measured right-hand
displacement was `[0.028431, 0.031754, 0.003393] m` relative to the initial
endpoint, rather than fully achieving the requested absolute endpoint. Return
settled to `[0.000281, 0.001754, -0.000403] m` Cartesian residual and
`0.007934 rad` maximum joint residual.

All 12,057 command records were contiguous at `250.003 Hz`; maximum active
velocity was `0.171806 rad/s`, maximum release velocity was `0.302194 rad/s`,
and maximum active torque was `2.50 Nm`. Video recorded 1,474 frames at
`30.013 fps`; sampled synchronized frames showed smooth unobstructed outbound
and return motion with no visible contact. Zero evidence records dropped.

The artifact final status is `incomplete` solely because the stop-recording
Unitree TTS call returned code 3104 after motion and release completed. Checksums
pass and the detailed assessment is in:

```text
artifacts/robot_dev_runs/20260831T170801.300133Z_right-004-neg020-neg010-21s-b/
telemetry/standalone_arm/sequence_20260831T170924Z.jsonl
```

Physical execution is hard-disabled again. Do not repeat this attempt. Before
another physical request, investigate why measured Cartesian tracking did not
reach the planned endpoint and separately diagnose TTS return code 3104.

## Current pause point: exact Cartesian request ready; camera disconnected

The operator approved one future right-arm attempt to the pelvis-fixed endpoint
`[0.04, -0.20, -0.10] m`, accepting a narrowly expanded workspace for this
specific target. The trajectory is 20 seconds outbound and 20 seconds return at
250 Hz. The immutable request is:

```text
reviewed_cartesian_requests/right_004_-020_-010_20s.json
SHA-256: db03793cbced9f920fb953d918693138158095fafd124e8ed4175a04d115fde1
attempt: right-004-neg020-neg010-20s-a
```

Latest offline plan from the 2026-08-31 read-only pose:

- displacement: `0.045663 m`;
- maximum joint change: `0.209644 rad`;
- maximum planned joint velocity: `0.019654 rad/s`;
- right endpoint translation residual: `0.004507 m`;
- right orientation residual: `0.014102 rad`;
- bounded RNEA torque output; focused and full offline tests pass.

The operator moved the verifier camera to ground level behind the robot's right
arm, looking toward that arm, and confirmed the robot was secured, the path was
clear, and an emergency-stop operator was ready. During camera gating,
`/dev/video6` initially negotiated 30 fps through `v4l2-ctl`, then disappeared.
OpenCV could not open it and subsequently no `/dev/video*` node existed. No
publisher was constructed and no motion command was issued.

Physical execution remains hard-disabled in
`run_g1_reviewed_cartesian_test.py`: `PHYSICAL_EXECUTION_ENABLED = False` and
`AUTHORIZED_REQUEST_SHA256 = None`. Do not enable either gate until the camera
is reconnected and the new view is inspected.

### Tomorrow's exact resume sequence

1. Reconnect/power the verifier camera and enumerate `/dev/video*`; do not
   assume it returns as `/dev/video6`.
2. Capture and inspect a read-only frame from the exact OpenCV backend/device
   that the evidence session will use. Require the full right-arm path and
   clearance to be visible from the new ground-level view.
3. Confirm again that the robot is secured, the path/exclusion zone is clear,
   and a dedicated emergency-stop operator is ready; yesterday's confirmation
   is not a substitute for a fresh physical-session check.
4. Run the normal read-only preflight immediately before the attempt and
   require sustained `(FSM 501, mode 0)`, `mode_machine=5`, and stationary arms.
5. Verify the request's canonical SHA-256 and rerun the offline plan from the
   fresh pose. Reject if the current pose/target leaves the request workspace,
   if any gate fails, or if the camera cannot record reliably.
6. Only after reviewing the fresh plan, set `AUTHORIZED_REQUEST_SHA256` to the
   exact hash above and `PHYSICAL_EXECUTION_ENABLED = True`. Run exactly one
   attempt with every CLI confirmation and the verified camera device.
7. Immediately restore both hard-disable values after the process exits,
   regardless of result. Then verify checksums, analyze phase timing/telemetry,
   inspect exact synchronized visual frames, and run an independent read-only
   `(501, 0)` postflight before deciding whether the result passed.

The full offline suite passed 144/144 immediately before this handoff, and
`git diff --check` passed. BrainCo hardware remained disconnected and is not
required for this arm-only, no-contact attempt.

## 2026-08-31 continuation: preflight and command timing

The normal read-only preflight passed with 107 sport-state samples all at
`(FSM 501, mode 0)`, 148 low-state samples all at `mode_machine=5`, zero record
drops, and no write error. Maximum arm velocity was `0.03682 rad/s` and maximum
arm position span was `0.0000839 rad`. Evidence:

```text
telemetry/standalone_arm/sequence_20260830T222022Z.jsonl
```

No publisher or robot command was constructed. The DDS reader message printed
after the success result occurred during shutdown and did not invalidate the
completed recording.

Offline analysis localized retry B's `238.76 Hz` aggregate command rate to
relative-sleep scheduling outside the main trajectories. The acquire, outbound,
return, and release trajectories were already approximately 250 Hz; the old
settle, raised-hold, and internal-return phases were only 129--137 Hz. A typical
full evidence command is 1,281 bytes and serialized in approximately 57 us on
PC2, so JSON serialization was not the limiting factor.

The pending worktree changes use absolute-deadline scheduling for settle,
raised-hold, and internal-return commands; preserve the complete pre-send
command record; remove one redundant synchronous mapping copy; add phase-aware
timing to `analyze_g1_cartesian_run.py`; and add deterministic timing/evidence
regressions. The full offline suite passes 132/132 and `git diff --check` passes.
Physical execution remains hard-disabled.

BrainCo touch telemetry is the next hardware gate. The existing standalone
BrainCo code already reads and records `brainco.touch` and `brainco.motor`, but
no BrainCo serial device was present at the 2026-08-31 pause point: no
`/dev/serial`, `/dev/ttyUSB*`, or `/dev/ttyACM*` entry was available, and no
BrainCo/stark process was running. Before live routing can be validated, the
operator must connect/enable the BrainCo USB device and confirm exclusive serial
ownership. Do not enable touch sensors, move the hand, command contact, or
enable a physical arm attempt merely to satisfy this telemetry gate.

## Reusable absolute Cartesian coordinate solution

The pending worktree now supports an absolute world-frame hand coordinate, not
only a relative delta. `CartesianPositionCommand` holds the current hand
orientation, keeps the untargeted hand fixed, and enforces the reviewed
workspace, Cartesian displacement, joint-offset, trajectory-velocity, joint
limit, IK residual, model mapping, and RNEA torque gates.

`create_g1_cartesian_request.py` writes a new immutable request containing the
exact target, both hand workspaces, and all motion limits, and prints its
canonical SHA-256. `plan_g1_cartesian_request.py` verifies that hash and solves
the request offline without importing Unitree DDS. The evidence-backed runtime
`run_g1_reviewed_cartesian_test.py` now accepts such a request, recomputes the
plan from a fresh read-only snapshot, requires planning-to-execution joint and
endpoint continuity, moves out and returns, settles, and releases authority.

The physical runtime remains doubly disabled: `PHYSICAL_EXECUTION_ENABLED` is
false and `AUTHORIZED_REQUEST_SHA256` is unset. Even when reviewed code changes
the first gate, only the single compiled exact request hash can pass the second.
The CLI also requires the same expected hash plus fresh area-clear, emergency
stop, `(501, 0)`, and plan-review confirmations.

An end-to-end offline workflow based on the latest read-only pose successfully
planned a 1 mm right-hand world-X target. It requested endpoint
`[0.0147186622, -0.2366612566, -0.1070986541] m`, held the left hand fixed,
required only `0.0028298 rad` maximum joint offset and `0.0006633 rad/s`
maximum planned joint velocity, and produced `0.0000251 m` right translation
residual. This was planning validation only and did not authorize or publish
motion. The full offline suite passes 140/140 and `git diff --check` passes.

The next required operator input is the desired absolute world-frame target
coordinate (which hand and X/Y/Z in meters). A physical session additionally
requires a newly reviewed workspace around the current pose, camera view,
exclusion zone, emergency-stop operator, fresh `(501, 0)` preflight, and
single-attempt authorization for the resulting request hash.

## Chinese recording announcements implemented

`EvidenceSession` now requires the injected recording announcer to say
`机器人开始移动` after the first video frame and before a session becomes
command-ready. It says `机器人停止移动` exactly once after every active recording
stops, including failure cleanup. Unitree TTS requests and results are recorded
in the run event stream; a start-announcement failure prevents publisher
readiness.

The read-only rehearsal completed successfully with both TTS calls returning
zero, 95 video frames at measured 29.999 fps, and valid checksums:

```text
artifacts/robot_dev_runs/20260830T042807.515987Z_chinese-recording-announcement-rehearsal/
```

The Cartesian physical attempt remains hard-disabled. A repeat must use a new
attempt ID and fresh physical-session safety confirmation; do not reuse retry B.

## Current pause point: recorded 1 cm Cartesian test passed

Guarded retry `20260830-right-x-1cm-b` completed the full right-arm Cartesian
out-and-return cycle with synchronized telemetry and `/dev/video6`. The exact
physical flag was immediately hard-disabled after the attempt. An independent
read-only postflight confirmed healthy native control in `(FSM 501, mode 0)`.
No remote-control state change or reboot is required before tomorrow's normal
read-only preflight.

### Verified result

- requested right-hand world-X displacement: `0.010000 m`;
- measured maximum X displacement: `0.009658 m`;
- lateral/vertical deviation at maximum X: `-0.000226 / -0.000035 m`;
- maximum active-control arm velocity: `0.07977 rad/s` at joint 28;
- maximum release-phase velocity: `0.25617 rad/s` at joint 26, below the
  separate `0.50 rad/s` release gate;
- maximum active-control torque: `2.25 Nm` at joint 23;
- return-settled Cartesian residual: `[0.000366, -0.000070, -0.000055] m`;
- return-settled maximum joint residual: `0.003871 rad`;
- 5,306 contiguous pre-send command records, 2,284 low-state samples, 1,001
  sport-state samples, and zero evidence-record drops;
- 690 synchronized video frames at measured `29.985 fps`;
- successful outbound settle, return settle, authority release, native return,
  and independent `(501, 0)` postflight;
- visual event-frame review showed smooth outbound/return motion with no person,
  obstacle contact, collision, jerk, or unexpected motion.

Touch telemetry was unavailable and no touch interaction was commanded; this
was an arm-only test. IMU peaks were modest: `0.04013 rad/s` gyroscope norm and
`0.3028 m/s^2` acceleration change from the first sample.

Authoritative local evidence (large generated artifacts, intentionally ignored
by Git but protected by `checksums.sha256`):

```text
artifacts/robot_dev_runs/20260830T041734.493329Z_20260830-right-x-1cm-b/
telemetry/standalone_arm/sequence_20260830T041835Z.jsonl
```

The run's `verification.md` has the final pass assessment. All checksums pass.

### Important implementation state

- `run_g1_reviewed_cartesian_test.py` is hard-disabled after retry B.
- Heavy Pinocchio model loading and IK now finish from a fresh read-only
  planning snapshot before starting physical telemetry/video capture.
- The physical session then requires planning-to-execution continuity within
  `0.01 rad` per joint and `0.005 m` at both hand endpoints before publisher
  construction.
- The reusable evidence harness records command intent before transport and
  synchronizes telemetry/video on the host monotonic clock.
- `analyze_g1_cartesian_run.py` provides repeatable phase-aware telemetry and
  video-event analysis.

### Tomorrow's resume sequence

1. Improve the harness: the Visual review section of verification.md should explain the specific frame(s) that the mentioned observation were made.
2. Run only the normal read-only preflight and require `(501, 0)`.
3. Run the full offline suite and `git diff --check`.
4. Verify the retry-B artifact checksums and review its `verification.md`.
5. Improve command-loop evidence performance before another physical attempt:
   retry B averaged `238.76 Hz` rather than the configured `250 Hz`, with a
   `14.95 ms` maximum interval. Preserve evidence-first ordering while reducing
   synchronous command serialization or payload size; validate timing offline.
6. Add or explicitly route BrainCo touch telemetry before any human-contact or
   handshake test. Do not infer touch from arm or IMU data.
7. Keep physical execution disabled until a new exact target, runtime plan,
   safety confirmation, camera view, and single-attempt authorization are
   reviewed. Do not progress directly to human contact or repeated shaking.
8. Implement the project-neutral recording-announcement feature request in
   `ROBOT_DEV_HARNESS.md`: say `机器人开始移动` after the first video frame and
   `机器人停止移动` whenever an active recording stops. The audio lifecycle must
   be evidence-recorded, nonblocking for control, failure-aware, and tested
   before it is used in another physical session.

## 2026-08-30 recorded 1 cm attempt A: no command published

Guarded attempt `20260830-right-x-1cm-a` aborted during initial observation
before publisher construction or any `rt/arm_sdk` command. The evidence run has
no command stream and only the `initial_observe` telemetry events. Pinocchio
model initialization occurred after the evidence session started and delayed
Python DDS callbacks for about six seconds; sport-mode telemetry consequently
aged to 5.759 seconds and the controller refused to proceed. The camera also
fell to 1.33 fps during that CPU/GIL-heavy initialization. The abort-release
path completed (with zero acquired authority), and a separate read-only
post-abort preflight confirmed `(501, 0)`.

Evidence:

```text
artifacts/robot_dev_runs/20260830T041341.506911Z_20260830-right-x-1cm-a/
telemetry/standalone_arm/sequence_20260830T041406Z.jsonl
```

The one-shot attempt was immediately hard-disabled. The corrected harness now
loads the model and solves IK from a fresh read-only planning snapshot before
starting the physical evidence session. After fresh telemetry/video startup it
requires the live arm pose to remain within 0.01 rad per joint and 0.005 m at
both modeled hand endpoints before constructing a publisher. No motion limits
or target values were relaxed.

## 2026-08-30 continuation: offline Cartesian interface

The normal read-only resume preflight observed `(FSM 4, mode 0)`, not the
required Regular-mode `(501, 0)`, and aborted before constructing an arm
publisher or issuing motion. Diagnostic evidence is in
`telemetry/standalone_arm/sequence_20260830T034513Z.jsonl`. Do not run a
physical arm test while FSM 4 remains active. A future physical session must
first establish the intended robot/controller mode through the operator's
normal Unitree procedure and then pass a fresh read-only `(501, 0)` preflight.

Offline work resumed successfully:

- the focused controller, IK, and standalone suites passed 53/53 before edits;
- `handshake/cartesian_command.py` now provides an offline-only parameterized
  dual-hand Cartesian delta interface;
- every command requires reviewed world-frame workspaces for both the initial
  and target hand endpoints;
- the interface enforces displacement norm, joint-offset, trajectory-velocity,
  duration, sample-rate, IK-residual, model-joint, and RNEA torque gates;
- `plan_g1_cartesian_arm.py` exposes those parameters without importing Unitree
  DDS or constructing a publisher;
- the retired `--execute-cartesian-10cm-right-x-test` flag remains
  hard-disabled;
- the expanded focused suite passes 56/56.

The reusable `robot_dev_harness` continuation now implements the timestamped
run-artifact recorder described in `HANDSHAKE_SYSTEM_PROMPT.md`. A read-only
PC2 rehearsal captured synchronized low state, sport state, and `/dev/video6`
at a measured 30.009 fps with per-frame host monotonic timestamps, zero dropped
telemetry records, and valid SHA-256 checksums. See `ROBOT_DEV_HARNESS.md` for
the project-neutral interface. Do not connect the new command interface to
physical publication until `(501, 0)` is restored, a
fresh target and workspace are reviewed, synchronized telemetry/video capture
passes its pre-command health checks, the operator confirms the physical test
session is safe, and exactly one guarded attempt is explicitly authorized.

## Current authoritative handoff: successful Cartesian verification

This section supersedes the older same-day notes below. The robot ended the
session healthy in Regular mode `(FSM 501, mode 0)`; a final read-only preflight
confirmed that state after the successful physical run. No reboot is required
before resuming tomorrow, although the normal read-only preflight remains the
first step.

### Physically verified result

The operator visibly confirmed the right arm moving forward and returning. The
successful run requested a 0.10 m right-hand world-X displacement and recorded:

- measured maximum right-hand X displacement: **0.09555 m**;
- lateral displacement at maximum X: 0.00017 m;
- vertical displacement at maximum X: 0.00637 m;
- maximum active arm velocity: 0.1810 rad/s;
- maximum active arm torque: 2.3125 Nm;
- final right-hand Cartesian residual: 0.00176 m in X;
- final maximum joint residual: 0.01547 rad;
- 6,094 contiguous commands and 2,361 low-state samples;
- successful outbound settle, return settle, authority release, and verified
  native-controller return to `(501, 0)`.

Authoritative local evidence (intentionally not committed because it is 28 MB):

```text
telemetry/continuous_arm/cartesian_10cm_lead020_visible_20260829.jsonl
telemetry/standalone_arm/preflight_after_cartesian_10cm_success_20260829.jsonl
```

### Implementation now in the repository

- `handshake/cartesian_arm_ik.py` provides bounded dual-arm Pinocchio IK and a
  smooth quintic joint trajectory with bounded RNEA feedforward.
- The reviewed 10 cm solve keeps the left arm fixed. Its live endpoint used a
  0.34914 rad maximum joint offset, 0.00721 m model translation residual, and
  0.01532 rad rotation residual.
- `g1_standalone_arm_sequence.py` now matches XR Teleoperate by publishing
  desired joint velocity `dq=0`; motion uses desired position `q`, RNEA torque,
  and the upstream G1 gain map.
- `handshake/continuous_arm.py` adds measured-state-following command limiting.
  The successful configuration capped the commanded joint target at 0.020 rad
  ahead of measured state while retaining a 0.030 rad tracking gate.
- Motion and native-release velocity gates are separate. Native takeover may
  move faster than commanded motion without preventing authority release.
- A fault after authority acquisition invokes `abort_release()`, holding the
  measured pose while ramping arm-SDK weight to zero and verifying `(501, 0)`.
- Runtime IK is calculated from the live initial pose before constructing the
  publisher. A failed solve therefore publishes nothing.

The one-shot `--execute-cartesian-10cm-right-x-test` mode is hard-disabled after
successful verification. Legacy authority and raise modes also remain disabled.
Do not remove these guards casually.

### Failed iterations retained as evidence

The initial open-loop attempts moved only about 2 cm before shoulder joint 22
exceeded tracking error. One required a reboot because authority remained in
mode 1. The first feedback-limited attempt reached 2.34 cm before exceeding the
0.25 rad/s active-motion velocity gate; its new abort path successfully returned
to `(501, 0)`. Reducing command lead from 0.025 to 0.020 rad produced the full
successful cycle above.

### Tomorrow's resume sequence

1. Run the read-only preflight and require `(501, 0)`:

   ```bash
   /home/dwei/miniconda3/envs/g1brainco/bin/python \
     g1_standalone_arm_sequence.py --probe-preflight \
     --network-interface eth0
   ```

2. Run the offline test suite and `git diff --check`. The focused controller,
   IK, and standalone suites ended today with 53/53 passing.
3. Review the successful telemetry summary above and preserve the raw JSONL.
4. Next engineering step: extract the verified Cartesian move into a reusable,
   parameterized Cartesian command interface while keeping displacement,
   workspace, joint-offset, tracking, velocity, torque, and authority-release
   gates. Start offline; do not immediately re-enable the one-shot physical flag.
5. Before any new physical target, require a fresh runtime plan review and an
   operator at the emergency stop. Re-enable exactly one guarded mode for one
   attempt, then disable it immediately and inspect telemetry.

## Archived earlier 2026-08-29 handoff

The material below records earlier intermediate states and is retained for
incident history. Where it conflicts with the current authoritative handoff
above, follow the current handoff.

## 2026-08-29 end-of-session handoff

The robot must be rebooted before resuming. The last physical run ended during
authority release and the last recorded state was `(FSM 501, mode 1)`. Do not
construct an arm publisher or issue a recovery command before a post-reboot
read-only check shows sustained `(501, 0)` and stationary arms.

### What was established

The continuous arm path now reproduces the relevant G1-29 `xr_teleoperate`
pattern:

- `rt/arm_sdk` at 250 Hz;
- `mode_pr=0` and live low-state `mode_machine` (observed value 5);
- motors 0--28 initialized from their measured positions with the upstream
  strong/weak/wrist gain map;
- exact SDK-to-Pinocchio ordering for joints 15--28;
- full bounded gravity RNEA rather than authority-scaled torque;
- immediate full authority, matching XR rather than a custom ramp;
- fixed measured pose for zero-offset acquisition and hold;
- gradual weight release followed by verified `(501, 0)` return.

The exact-XR acquisition and full-authority hold in
`telemetry/continuous_arm/exact_xr_visible_check_20260829.jsonl` succeeded. The
operator reported no visible motion. The run completed acquisition, the
zero-offset trajectory, raised settling, one-second hold, return, and return
settling. It aborted only during authority release at weight 0.720 because
joint 20 moved toward the native controller's pose and exceeded the outgoing
arm-SDK target by 0.01 rad. Recorded joint-20 departure was approximately
0.0095 rad with 0.104 rad/s peak velocity. This was an invalid release-phase
tracking rule, not an acquisition or hold instability.

The controller has since been corrected: target-tracking error remains enforced
during acquisition, motion, and hold, but is not enforced during
`authority_release`, where the native controller is expected to blend toward
its own pose. Velocity, torque, telemetry freshness, FSM, finite-value, and RNEA
bounds remain enforced during release. A regression test covers native pose
blending during release.

All 104 offline tests pass after this correction. Both legacy physical modes
and the exact-XR physical flag are currently hard-disabled in
`g1_arm_sdk_raise.py`; leave them disabled until the resume gates below pass.

### Offline Cartesian layer

`handshake/cartesian_arm_ik.py` and `plan_g1_cartesian_arm.py` implement an
offline-only version of the `G1_29_ArmIK` pattern:

```text
dual 6D hand targets -> bounded posture-continuous Pinocchio IK
-> 14 joint targets -> bounded gravity RNEA
```

It has no Unitree SDK or DDS imports. A 1 mm right-hand target produced a
0.0028 rad maximum joint step and approximately 0.000038 m translation
residual. Cartesian output is not authorized for hardware yet.

### Resume sequence

1. Confirm the robot has been rebooted and is secured on the reviewed gantry
   with an exclusion zone and dedicated physical emergency-stop operator.
2. Run only the read-only preflight and require sustained `(501, 0)`, live
   `mode_machine=5`, stationary arms, and normal wrist telemetry:

   ```bash
   /home/dwei/miniconda3/envs/g1brainco/bin/python \
     g1_standalone_arm_sequence.py \
     --probe-preflight \
     --network-interface eth0
   ```

3. Re-run the 104-test offline suite and `git diff --check`.
4. Review the release-phase tracking fix in `handshake/continuous_arm.py` and
   its regression in `tests/test_continuous_arm.py`.
5. Only while the operator is visibly watching both wrists, temporarily enable
   exactly one `--execute-xr-pattern-authority-test` zero-offset run. Do not
   enable `--execute-authority-test` or `--execute-cycle`.
6. Accept only if acquisition/hold remain visually stationary, release reaches
   weight zero, `arm_released` is recorded, final FSM is `(501, 0)`, recording
   has no drops/errors, and all measured safety limits pass.
7. Re-disable physical execution immediately after the attempt and analyze its
   telemetry before considering any Cartesian or joint displacement.

The next physical check remains zero-offset. Do not command a Cartesian move,
raise candidate, hand action, human contact, or repeated oscillation in the
next session.

## Resume point

A second compensated zero-offset attempt using XR-pattern `LowCmd`
initialization was rejected on 2026-08-29. Live state changed from the earlier
read-only `(501, 1)` observation to `(501, 0)`, so the guarded run proceeded.
It copied live `mode_machine=5`, initialized motors 0--28 from measured state
with the upstream gain map, and used tighter limits, but aborted when joint 27
exceeded 0.25 rad/s. Evidence is in
`telemetry/continuous_arm/xr_pattern_refusal_mode1.jsonl`. The XR-pattern
physical mode is now hard-disabled. No more physical arm-SDK testing is
authorized; secure the robot physically and escalate the repeated wrist
transition to Unitree/vendor support before another publisher is considered.

On 2026-08-29 the compensated zero-offset check was physically run and rejected.
`telemetry/continuous_arm/authority_20260828T173734Z.jsonl` aborted at 0.770
authority after right wrist roll reached 0.966 rad/s; right wrist yaw reached
0.724 rad/s at the same transition. The final recorded FSM state remained
`(501, 1)` because the fail-closed safety path did not attempt software release.
Both physical modes in `g1_arm_sdk_raise.py` are now hard-disabled. Do not rerun
or issue a software recovery command. First physically secure the robot and
verify its current controller state through a separately reviewed read-only
diagnostic.

The 2026-08-29 continuation implemented the first compensated-control slice:
`handshake/arm_feedforward.py` loads the checked-out `xr_teleoperate` G1-29
URDF, verifies its exact 14-joint reduced-model ordering, computes Pinocchio
RNEA, rejects non-finite or out-of-bound output, and exposes model provenance.
The continuous controller and `rt/arm_sdk` sink now carry per-joint torque,
scale it with authority weight, and record it. All 95 offline tests pass.

No physical command was published during this continuation. The immediate next
step is to review the selected 5 Nm shoulder/elbow and 1.5 Nm wrist software
bounds and the command telemetry, then prepare explicit approval for one
gantry-attached compensated zero-offset authority cycle. A raise candidate is
still paused.

The active work is milestone 2B's arm-only architecture redesign after the
second arm-drop incident. It intentionally excludes vision, tactile triggers,
BrainCo hand commands, human contact, and repeated oscillation.

**Do not run `--execute-arm-sdk`.** The current implementation preserves the
failed high-level-action-to-arm-SDK handoff for incident analysis and is not an
approved controller. Exact code and traces are committed under
`incident_reports/2026-08-27/`.

The replacement architecture uses `rt/arm_sdk` continuously for the full arm
sequence, starting from documented Regular mode without an active high-level
arm action:

```text
measure initial pose
-> acquire arm-SDK authority while holding that fixed pose
-> smooth arm-SDK raise
-> bounded arm-SDK shake
-> smooth arm-SDK return to the initial pose
-> measured settling
-> controlled arm-SDK authority release
```

The redesign will be implemented in or replace:

- `g1_standalone_arm_sequence.py`
- `handshake/standalone_arm.py`
- `handshake/sport_mode_state.py`
- `tests/test_standalone_arm.py`
- `README_g1_standalone_arm_sequence.md`
- `capture_g1_failure_snapshot.py`
- `diagnose_g1_arm_service.py`

The former harness successfully characterized high-level raise/release poses,
generated a 250 Hz command-only sequence, and met dry-run timing targets. Those
results did not validate controller ownership. Two later minimum-amplitude
publication trials reproduced the arm drop even after RPC return, reviewed-pose
settling, target continuity, velocity clipping, authority blending, and a
0.005 rad requested elbow excursion.

The new design must not call modern `shake hand`/`release arm` or legacy sport
handshake tasks in its nominal sequence. Additional delay or settling is not a
fix because the second failure followed a code-zero raise RPC and 0.519 seconds
of sustained settling.

The synchronized training-table work described below remains useful but is no
longer the immediate priority.

## Latest standalone-arm evidence

The latest recorded attempts are under `telemetry/standalone_arm/` and must be
preserved as raw evidence.

- `sequence_20260824T210345Z.jsonl` predates strict RPC-return validation. The
  raise action returned 3104 and the release returned 7400, but the old runner
  incorrectly labeled the capture successful. Its reported pose centers are
  not approved and must not be used as reviewed command inputs.
- Intermediate attempts correctly aborted. Low-state DDS telemetry was present, while
  the locomotion FSM RPC failed with 3102 (send failure). In the latest run,
  the arm action-list RPC also failed with 3104 (timeout).
- Later evidence demonstrated successful preflight, repeatable high-level
  raise/release, reviewed pose centers, and a 250 Hz command-only rehearsal.
  Those results are retained only as characterization evidence because the
  subsequent cross-controller publication failed twice.
- No further arm-SDK publication is authorized by this handoff.

### 2026-08-27 controller-handoff recurrence

Two gantry-attached publication trials are the authoritative latest evidence:

```text
telemetry/standalone_arm/sequence_20260826T233757Z.jsonl
telemetry/standalone_arm/sequence_20260826T233832Z.jsonl
```

The first reached 15.117 rad/s at the right elbow and failed the final
safe-return envelope. The second followed a code-zero `shake hand` return and
measured settling, then reached 14.668 rad/s at the right elbow before the
state-limit abort; its later high-level return recovered the reviewed pose.
In both runs the commanded elbow remained near 0.183 rad while the measured
joint departed toward approximately 0.9 rad. The requested trajectory was only
0.005 rad, so the motion was a controller-handoff failure rather than execution
of the planned movement.

Committed reports and forensic artifacts:

```text
incident_reports/INCIDENT_REPORT_2026-08-27_ARM_SDK_HANDOFF_RECURRENCE.md
incident_reports/2026-08-27/
```

Safety conclusions:

- RPC return plus measured settling does not prove controller release.
- The transition from a held high-level arm action to `rt/arm_sdk` is prohibited.
- The existing controlled weight-ramp abort did not stabilize the divergent arm.
- All physical arm-SDK publication remains suspended pending a new design and
  return-to-test review.

### 2026-08-26 controller failure and reboot diagnosis

Before the robot reboot, low-state DDS telemetry remained healthy and the
generic `robot_state` RPC remained available, but the remote controller could
not request damping and the locomotion FSM RPC failed with 3102. A sealed
pre-reboot snapshot recorded `ai_sport` service status 1:

```text
telemetry/diagnostics/g1_pre_reboot_20260825T181128Z/
telemetry/diagnostics/g1_pre_reboot_20260825T181128Z.tar.gz
```

Archive SHA-256:

```text
5c8ee9dd1f0e9af606649efcfff400f456de9c441c7ff63d4196b0e0f43af78c
```

After reboot, the matching snapshot recorded `ai_sport` status 0 and the same
locomotion FSM query succeeded. This localizes the prior remote/SDK failure to
the robot's high-level locomotion path rather than PC2 networking or general
DDS. The evidence does not distinguish a service switch/debug transition,
controller-ownership cleanup failure, or service crash as the initiating
cause. The post-reboot bundle and analysis are:

```text
telemetry/diagnostics/g1_post_reboot_20260825T181632Z/
telemetry/diagnostics/g1_post_reboot_20260825T181632Z.tar.gz
```

Archive SHA-256:

```text
fdbfe3913813007025d2e1c6d440ad3a976b95931e5592b54be226993b16b49c
```

The robot subsequently entered standard mode. Read-only preflight observed
FSM 501, mode 0, confirming that the remote-to-`ai_sport` path and locomotion
RPC are healthy. The arm action-list call still failed with 3104:

```text
telemetry/standalone_arm/sequence_20260825T181926Z.jsonl
```

Deeper read-only discovery established that the `sport` server API is present
and version-matched at 1.0.0.0, while the `arm` server API-version endpoint
returns 3103 (API not registered). Meanwhile `rt/arm/action/state` publishes
about 14 Hz and reports no held action. Evidence:

```text
telemetry/diagnostics/g1_arm_service_standard_20260825T182100Z.json
```

SHA-256:

```text
cd2eea01281b65726c7ab3aebd351af85c95b5837417b53e37f0fcce9944ef3e
```

The installed official legacy G1 locomotion client exposes arm-task API 7106:
task 2 is `ShakeHand(stage=0)` and task 3 is `ShakeHand(stage=1)`. These are
two handshake stages, not a documented equivalent of the newer explicit
`release arm` action 99. Do not assume task 3 safely lowers or releases the
arm; characterize both stages under the gantry before assigning raise/return
semantics.

The current code rejects every nonzero high-level or service RPC result,
records service failures together, accepts only the documented FSM 500, 501,
or FSM 801 mode 0/3 combinations, requires sustained measured settling, and
requires reviewed post-action and safe-return poses before publication.

## Verification baseline

On 2026-08-26, the `g1brainco` Python environment passed all 65 tests:

```bash
/home/dwei/miniconda3/envs/g1brainco/bin/python -m unittest discover -s tests -v
```

The offline plan command also completed, Python compilation passed, and
`git diff --check` reported no whitespace errors.

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

1. Add a hard software disable for the failed
   `high-level action -> rt/arm_sdk` publication path. Preserve its archived
   incident snapshot, but make the live executable refuse that combination.
2. Obtain or locate authoritative Unitree documentation for Regular-mode
   `rt/arm_sdk` ownership, joint-29 weight semantics, FSM modes 0/1, required
   `LowCmd` initialization, and supported release/recovery behavior.
3. Design the complete arm-SDK raise/shake/return trajectory offline. Define
   joint targets from kinematics and reviewed workspace limits rather than
   replaying the high-level handshake pose.
4. Hold one fixed initial target throughout authority acquisition; do not chase
   measured positions during blend-in.
5. Add a phase state machine with measured gates for authority acquisition,
   raise, raised settling, shake, return, return settling, and release.
6. Redesign abort handling for tracking divergence before taking authority,
   while holding authority, and during release. Do not reuse the failed weight
   ramp without new evidence.
7. Add deterministic tests for FSM changes, target discontinuity, gravity
   departure, torque/velocity violations, stale telemetry, scheduling overrun,
   cancellation, and every abort phase.
8. Generate plots and a complete command-only run with no publisher. Review
   all 14 joint commands, gains, weight, timing, and lower-body state.
9. Keep physical publication suspended. Any future gantry trial requires a new
   written return-to-test approval and begins with authority acquisition and
   return only—no raise or shake.

## Repository state at handoff

- The incident reports and evidence are pushed to `origin/main` through
  `1bad58d` (`preserve incident code and traces`).
- The standalone-arm implementation, documentation, tests, and the 2B roadmap
  update are currently uncommitted.
- The last offline suite contained 78 passing tests, but test success did not
  validate the failed physical controller handoff.
- Raw recordings, logs, dependencies, and caches are intentionally untracked.

At the start of the next session, read this file and `PROJECT_PLAN.md`, inspect
`git status`, review both incident reports, and begin with the hard disable and
offline continuous-arm-SDK redesign. Do not resume physical preflight or
publication unless the user establishes a new return-to-test review.

## Verified own-IK oscillation (2026-09-02)

The guarded request
`reviewed_cartesian_requests/handshake_oscillation_4mm_raised_cosine_f.json`
(`d5be4706c8b33225a65f3fefb78e5a27c97c4be8fda793ee4072037d975df8d7`)
completed a 10-second own-IK raise, two vertical raised-cosine-squared cycles
over 16 seconds, a controlled return, authority release, and restoration of
standard mode `(FSM 501, mode 0)`.

The immutable run artifact is
`artifacts/robot_dev_runs/20260901T173745.730241Z_handshake-oscillation-4mm-raised-cosine-f`.
Measured vertical peak-to-peak motion was `0.007540 m` in cycle 1 and
`0.005959 m` in cycle 2. Peak active-arm speed was `0.179476 rad/s`; all
`14,324` command records were contiguous. Return-settled maximum joint residual
was `0.003320 rad`, and post-native-release maximum joint residual was
`0.001774 rad`. The artifact's effective `incomplete` label is solely from a
non-control stop-recording TTS RPC returning 3104 after the verified motion and
postflight had completed.

The run's `verification.md` contains a sentence-level video review: every
observation cites an exact UTC timestamp and frame index and links its extracted
JPEG in `evidence/`. Physical execution is disabled again in
`run_g1_reviewed_cartesian_test.py`; its authorization hash is `None`.

The next user-selected goal is to score the immutable files under
`telemetry/trajectories/` for a complete raise/release, measured oscillation,
and BrainCo hand close/open cycle, then reproduce the selected Cartesian trace
through the own-IK safety pipeline rather than directly replaying measured
joint state.
