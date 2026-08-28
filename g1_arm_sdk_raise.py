#!/usr/bin/env python3
"""Run the staged continuous G1 rt/arm_sdk authority checks."""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Sequence

from g1_standalone_arm_sequence import ArmSdkCommandSink, LowStateMonitor, _load_sdk_path
from handshake.arm_feedforward import G1ArmGravityFeedforward
from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.continuous_arm import ContinuousArmConfig, ContinuousArmController
from handshake.recording import TelemetryRecorder
from handshake.standalone_arm import ARM_JOINT_INDICES


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--print-plan", action="store_true")
    modes.add_argument("--execute-authority-test", action="store_true")
    modes.add_argument("--execute-xr-pattern-authority-test", action="store_true")
    modes.add_argument("--execute-cycle", action="store_true")
    modes.add_argument("--execute-cartesian-10cm-right-x-test", action="store_true")
    targets = parser.add_mutually_exclusive_group()
    targets.add_argument("--raise-offset-rad", nargs=14, type=float, metavar="Q")
    targets.add_argument("--candidate-capture-scale-010", action="store_true")
    parser.add_argument("--hold-seconds", type=float, default=1.0)
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--g1-urdf", type=Path,
        default=Path("/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"),
    )
    parser.add_argument("--confirm-gantry-attached", action="store_true")
    parser.add_argument("--confirm-estop-ready", action="store_true")
    parser.add_argument("--confirm-regular-mode-501-0", action="store_true")
    parser.add_argument("--confirm-xr-message-pattern-reviewed", action="store_true")
    parser.add_argument("--confirm-cartesian-10cm-plan-reviewed", action="store_true")
    args = parser.parse_args(argv)
    if args.execute_authority_test:
        parser.error(
            "physical authority testing is suspended after the 2026-08-29 wrist-limit abort"
        )
    if args.execute_xr_pattern_authority_test:
        parser.error(
            "exact-XR zero-offset verification completed; physical execution is disabled pending review"
        )
    if args.execute_cartesian_10cm_right_x_test:
        parser.error(
            "the feedback-limited 10 cm Cartesian verification completed; "
            "physical execution is disabled pending review"
        )
    if args.execute_cycle:
        parser.error(
            "raise execution is paused until the compensated zero-offset physical check is accepted"
        )
    authority_test = args.execute_authority_test or args.execute_xr_pattern_authority_test
    physical_cartesian = args.execute_cartesian_10cm_right_x_test
    if not authority_test and not physical_cartesian and not (
        args.raise_offset_rad is not None or args.candidate_capture_scale_010
    ):
        parser.error("select 14 reviewed offsets or --candidate-capture-scale-010")
    if not 0.0 <= args.hold_seconds <= 30.0:
        parser.error("hold duration must be between 0 and 30 seconds")
    if (args.execute_cycle or authority_test or physical_cartesian) and not (
        args.confirm_gantry_attached
        and args.confirm_estop_ready
        and args.confirm_regular_mode_501_0
    ):
        parser.error(
            "execution requires gantry, emergency-stop, and Regular-mode 501/0 confirmations"
        )
    if args.execute_xr_pattern_authority_test and not args.confirm_xr_message_pattern_reviewed:
        parser.error("XR-pattern execution requires --confirm-xr-message-pattern-reviewed")
    if physical_cartesian and not args.confirm_cartesian_10cm_plan_reviewed:
        parser.error("Cartesian execution requires --confirm-cartesian-10cm-plan-reviewed")
    return args


# Ten percent of the mean raised-minus-return displacement from the three clean
# 2026-08-28 captures. The left arm is deliberately fixed, and the right elbow
# is capped at 0.08 rad. This is a staged test candidate, not a reviewed final pose.
CANDIDATE_CAPTURE_SCALE_010 = {
    15: 0.0, 16: 0.0, 17: 0.0, 18: 0.0, 19: 0.0, 20: 0.0, 21: 0.0,
    22: -0.040, 23: 0.016, 24: 0.006, 25: -0.080,
    26: 0.025, 27: -0.040, 28: -0.0045,
}


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output = args.output or Path("telemetry/continuous_arm") / f"authority_{run_id}.jsonl"
    if args.execute_authority_test or args.execute_xr_pattern_authority_test or args.execute_cartesian_10cm_right_x_test:
        offsets = {index: 0.0 for index in ARM_JOINT_INDICES}
    elif args.candidate_capture_scale_010:
        offsets = dict(CANDIDATE_CAPTURE_SCALE_010)
    else:
        offsets = dict(zip(ARM_JOINT_INDICES, args.raise_offset_rad))
    config = ContinuousArmConfig()
    if args.execute_xr_pattern_authority_test or args.execute_cartesian_10cm_right_x_test:
        config = replace(
            config, acquire_seconds=2.0, release_seconds=2.0,
            max_measured_velocity_rad_s=0.25, max_tracking_error_rad=0.03,
            scale_feedforward_by_authority=False, step_to_full_authority=True,
        )
    if args.execute_cartesian_10cm_right_x_test:
        config = replace(
            config, raise_seconds=10.0, return_seconds=10.0,
            settle_timeout_seconds=30.0, max_offset_rad=0.40,
            max_command_lead_rad=0.020,
        )
    cartesian_planner = (
        G1CartesianArmIK(args.g1_urdf) if args.execute_cartesian_10cm_right_x_test else None
    )
    feedforward = (
        cartesian_planner.feedforward if cartesian_planner is not None
        else G1ArmGravityFeedforward(args.g1_urdf)
    )
    if args.print_plan:
        peak_velocity = max(abs(value) for value in offsets.values()) * 1.875 / min(
            config.raise_seconds, config.return_seconds
        )
        print(json.dumps({
            "mode": "print_plan",
            "candidate": "capture_scale_010" if args.candidate_capture_scale_010 else "explicit",
            "config": config.__dict__,
            "offsets_rad": offsets,
            "maximum_offset_rad": max(abs(value) for value in offsets.values()),
            "planned_peak_velocity_rad_s": peak_velocity,
            "publishes_commands": False,
            "feedforward": feedforward.configuration(),
        }, indent=2))
        return 0

    _load_sdk_path()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    ChannelFactoryInitialize(0, args.network_interface)
    recorder = TelemetryRecorder(output, queue_size=8192)
    recorder.start()
    monitor = LowStateMonitor(recorder)
    monitor.start()
    def event(name, details):
        if name != "arm_sdk_command":
            print(f"{name}: {details}")
        recorder.record("continuous_arm.event", {"event": name, **details})

    controller = ContinuousArmController(
        monitor.latest, monitor.latest_sport, None, event, feedforward,
        publish_commands=True, config=config,
    )
    result, reason = "aborted", "initialization_failed"
    try:
        controller.observe_initial_pose()
        if cartesian_planner is not None:
            left_target, right_target = cartesian_planner.forward_kinematics(controller.initial_pose)
            right_target[0, 3] += 0.10
            cartesian_plan = cartesian_planner.plan_trajectory(
                left_target, right_target, controller.initial_pose,
                config.raise_seconds, config.sample_rate_hz,
            )
            endpoint = cartesian_plan["endpoint"]["positions_rad"]
            offsets = {i: endpoint[i] - controller.initial_pose[i] for i in ARM_JOINT_INDICES}
            event("cartesian_plan_reviewed_at_runtime", {
                "right_delta_m": [0.10, 0.0, 0.0],
                "sample_count": cartesian_plan["sample_count"],
                "maximum_joint_velocity_rad_s": cartesian_plan["maximum_joint_velocity_rad_s"],
                "maximum_joint_step_rad": cartesian_plan["endpoint"]["maximum_joint_step_rad"],
                "translation_error_m": cartesian_plan["endpoint"]["translation_error_m"],
                "rotation_error_rad": cartesian_plan["endpoint"]["rotation_error_rad"],
                "offsets_rad": offsets,
            })
        zero_velocity = {index: 0.0 for index in ARM_JOINT_INDICES}
        initial_torque = feedforward(controller.initial_pose, zero_velocity)
        event("arm_feedforward_initialized", {
            **feedforward.configuration(),
            "initial_pose_torque_nm": initial_torque,
        })
        initial_state, initial_state_ns = monitor.latest()
        if initial_state is None or initial_state_ns is None:
            raise RuntimeError("low state disappeared before publisher construction")
        sink = ArmSdkCommandSink(initial_state)
        event("arm_sdk_publisher_initialized", sink.runtime_configuration())
        controller.attach_command_sink(sink)
        controller.raise_arm(offsets)
        deadline = time.monotonic() + args.hold_seconds
        while time.monotonic() < deadline:
            controller.hold_once()
            time.sleep(1.0 / config.sample_rate_hz)
        controller.release_arm()
        result, reason = "success", "raise_return_release_complete"
    except KeyboardInterrupt:
        reason = "operator_cancelled; authority state may require emergency-stop handling"
    except BaseException as exc:
        fault = f"{type(exc).__name__}: {exc}"
        try:
            controller.abort_release()
            reason = f"{fault}; arm authority released after safety fault"
        except BaseException as release_exc:
            reason = f"{fault}; abort release failed: {type(release_exc).__name__}: {release_exc}"
    finally:
        recorder.record("continuous_arm.summary", {"result": result, "reason": reason})
        monitor.close()
        recorder.close()
    print(f"result={result}; reason={reason}; telemetry={output}")
    return 0 if result == "success" else 1


if __name__ == "__main__":
    raise SystemExit(main())
