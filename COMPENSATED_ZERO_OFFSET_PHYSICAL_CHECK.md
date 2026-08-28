# Compensated zero-offset physical check

Status: **rejected and suspended** after the 2026-08-29 physical run.

Update at session end: exact XR acquisition and full-authority zero-offset hold
were subsequently demonstrated without operator-visible motion in
`exact_xr_visible_check_20260829.jsonl`. That run aborted only during release
because an outgoing-target tracking rule incorrectly remained active while the
native controller blended back in. The rule is fixed and all 104 offline tests
pass, but the corrected release has not been physically verified. Physical
execution remains disabled, the robot requires a reboot, and the next check is
one final observed zero-offset acquire/hold/release cycle—no Cartesian motion.

## 2026-08-29 result

The physical run in `telemetry/continuous_arm/authority_20260828T173734Z.jsonl`
aborted during authority acquisition at command 118 and authority weight
0.770. Right wrist roll (joint 26) reached 0.966 rad/s and tripped the 1 rad/s
safety threshold on the following state check; right wrist yaw (joint 28)
simultaneously reached 0.724 rad/s. Their maximum pose departures before the
recording ended were 0.0152 rad and 0.0127 rad. All other arm joints stayed
within 0.0004 rad of their fixed targets.

The largest commanded feedforward across all joints was 1.531 Nm. Joint 26's
commanded feedforward was approximately -0.006 Nm at the end, so its motion is
not consistent with executing a large RNEA wrist-roll command. The run did not
reach release: its summary is `aborted`, and the last recorded FSM state is
`(501, 1)`. Recording completed with zero dropped samples and no write error.

This run fails the acceptance criteria. Both `--execute-authority-test` and
`--execute-cycle` are now hard-disabled. Do not repeat the command below. The
robot must be physically secured and its controller state verified before any
further investigation; no software recovery command is authorized by this
document.

## Scope

This check acquires and releases `rt/arm_sdk` authority while holding the one
measured initial arm pose. Every motion offset is zero. It does not call a
high-level arm action, command a raise, manipulate either hand, or make human
contact. `--execute-cycle` is hard-disabled until this check is accepted.

Expected controller transition on this robot and firmware:

```text
(FSM 501, mode 0) -> compensated authority -> (FSM 501, mode 1)
-> authority weight 0 -> verified (FSM 501, mode 0)
```

The controller uses the checked-out G1-29 URDF with SHA-256
`8bbf006633fc50b616f665c7a970780cc296577a0adfd7d28b049e751c238735`.
Pinocchio RNEA uses the fixed measured target pose, zero velocity, and zero
acceleration. Software torque bounds are 5 Nm on shoulders/elbows and 1.5 Nm
on wrists. Torque is multiplied by the authority weight, so it starts and ends
at zero.

## Evidence reviewed before this check

The three successful uncompensated cycles all returned to `(501, 0)`:

| Capture | Maximum observed pose delta | Maximum arm velocity |
| --- | ---: | ---: |
| `authority_20260827T182247Z.jsonl` | 0.0268 rad | 0.3559 rad/s |
| `authority_20260827T182303Z.jsonl` | 0.0231 rad | 0.1841 rad/s |
| `authority_20260827T182315Z.jsonl` | 0.0235 rad | 0.0736 rad/s |

At the first capture's initial pose, model torque includes approximately
`+1.924 Nm` at left shoulder roll and `-1.968 Nm` at right shoulder roll.
The uncompensated telemetry reports the same signs and roughly `+2.375 Nm` and
`-2.375 Nm`, respectively. This is a useful sign sanity check, not calibration
or proof that every joint's feedforward is correct.

## Operator gates

Do not start unless every item is true:

- Robot is attached to the reviewed gantry with feet clear of load-bearing.
- Exclusion zone is empty; there will be no human contact.
- A dedicated operator is holding the physical emergency stop and is not the
  person typing the command.
- Robot is visibly stable in Regular/Motion mode and read-only telemetry shows
  exactly FSM 501, mode 0.
- Both arms are stationary and unobstructed.
- Network interface is `eth0` and no other process publishes `rt/arm_sdk`.
- The printed URDF hash matches the hash above.
- The team accepts that a safety fault does not attempt an automatic release;
  use the physical emergency stop if motion is unexpected. Do not rely on
  Ctrl-C to stabilize unexpected motion.

## Final offline check

Run this first; it cannot create a DDS publisher:

```bash
/home/dwei/miniconda3/envs/g1brainco/bin/python g1_arm_sdk_raise.py \
  --print-plan \
  --raise-offset-rad 0 0 0 0 0 0 0 0 0 0 0 0 0 0
```

Then run the full offline suite:

```bash
/home/dwei/miniconda3/envs/g1brainco/bin/python -m unittest discover -s tests -v
```

## Suspended physical command (do not run)

Only after the gates above are jointly reviewed:

```bash
/home/dwei/miniconda3/envs/g1brainco/bin/python g1_arm_sdk_raise.py \
  --execute-authority-test \
  --network-interface eth0 \
  --hold-seconds 1.0 \
  --confirm-gantry-attached \
  --confirm-estop-ready \
  --confirm-regular-mode-501-0
```

The publisher is constructed only after stationary initial-pose observation,
model loading, joint-order validation, and initial torque computation succeed.

## Stop and acceptance criteria

Trigger the physical emergency stop immediately for visible unexpected motion,
oscillation, increasing motion, collision risk, or loss of balance. Software
also aborts on stale telemetry, unexpected FSM state, arm speed above 1 rad/s,
measured arm torque above 10 Nm, non-finite model output, or feedforward beyond
its per-joint bound.

Accept the run only if telemetry records all of the following:

- zero dropped samples and no recorder error;
- initial state `(501, 0)`, transition only to `(501, 1)`, and verified final
  state `(501, 0)`;
- contiguous command sequence with fixed position and zero desired velocity;
- torque beginning at zero, bounded during acquisition/hold, and ending at
  zero;
- smaller peak pose delta and peak velocity than all three uncompensated
  baselines (strict targets: `<0.0231 rad` and `<0.0736 rad/s`);
- final `arm_released` event and successful summary.

Do not run a raise afterward. First compare the new telemetry against the three
baselines and explicitly accept or reject the compensated result.
