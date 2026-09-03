#!/usr/bin/env python3
"""Execute one exact-hash own-IK Cartesian trace and BrainCo replay."""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

from g1_standalone_arm_sequence import ArmSdkCommandSink, LowStateMonitor, _load_sdk_path
from handshake.brainco_replay import BrainCoHandReplay
from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.cartesian_command import CartesianWorkspace
from handshake.continuous_arm import ContinuousArmConfig, ContinuousArmController
from handshake.standalone_arm import ARM_JOINT_INDICES
from robot_dev_harness.adapters import LegacyTelemetryAdapter
from robot_dev_harness.commands import EvidenceBackedCommandTransport
from robot_dev_harness.opencv_camera import OpenCVMjpegCamera
from robot_dev_harness.run_artifacts import RunArtifacts
from robot_dev_harness.session import EvidenceSession


PHYSICAL_EXECUTION_ENABLED = True
AUTHORIZED_REQUEST_SHA256 = "8b65ac8e55112019660df9dc49dcd4b09922ac94b407dffd357e1a2d2caa0ec0"


def load_request(path: Path, expected_sha256: str):
    raw = path.read_bytes()
    digest = hashlib.sha256(raw).hexdigest()
    if digest != expected_sha256.lower():
        raise ValueError(f"request hash mismatch: calculated {digest}")
    request = json.loads(raw)
    if request.get("schema_version") != "1.0":
        raise ValueError("unsupported trace replay schema")
    source = Path(request["source"]["path"])
    if hashlib.sha256(source.read_bytes()).hexdigest() != request["source"]["sha256"]:
        raise ValueError("immutable source trace hash mismatch")
    return request, digest


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-reviewed-request", type=Path, required=True)
    parser.add_argument("--expect-request-sha256", required=True)
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--camera-device", default="/dev/video6")
    parser.add_argument("--brainco-port", default=(
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if01-port0"
    ))
    parser.add_argument("--brainco-slave-id", type=lambda value: int(value, 0), default=0x7F)
    parser.add_argument("--brainco-baud", type=int, default=460800)
    parser.add_argument("--artifact-root", type=Path, default=Path("artifacts/robot_dev_runs"))
    parser.add_argument("--g1-urdf", type=Path, default=Path(
        "/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"
    ))
    for flag in (
        "confirm-area-clear", "confirm-estop-ready", "confirm-regular-mode-501-0",
        "confirm-plan-reviewed", "confirm-hand-motion-reviewed",
    ):
        parser.add_argument(f"--{flag}", action="store_true")
    args = parser.parse_args(argv)
    try:
        args.request, args.request_sha256 = load_request(
            args.execute_reviewed_request, args.expect_request_sha256
        )
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        parser.error(f"invalid reviewed trace request: {exc}")
    if not PHYSICAL_EXECUTION_ENABLED:
        parser.error(f"reviewed attempt {args.request['attempt_id']} is currently hard-disabled")
    if args.request_sha256 != AUTHORIZED_REQUEST_SHA256:
        parser.error("physical execution is not authorized for this exact request hash")
    if not all((
        args.confirm_area_clear, args.confirm_estop_ready,
        args.confirm_regular_mode_501_0, args.confirm_plan_reviewed,
        args.confirm_hand_motion_reviewed,
    )):
        parser.error("execution requires all five safety and exact-plan confirmations")
    return args


def plan_request(planner, request, initial):
    arm = request["arm"]
    left, right = planner.forward_kinematics(initial)
    left_workspace = CartesianWorkspace(
        request["workspace_m"]["left"]["minimum"], request["workspace_m"]["left"]["maximum"]
    )
    right_workspace = CartesianWorkspace(
        request["workspace_m"]["right"]["minimum"], request["workspace_m"]["right"]["maximum"]
    )
    left_workspace.require_contains(left[:3, 3], "initial left hand")
    right_workspace.require_contains(right[:3, 3], "initial right hand")
    right[:3, 3] = arm["right_center_m"]
    right[:3, :3] = arm["right_orientation"]
    right_workspace.require_contains(right[:3, 3], "raised center")
    for offset in arm["right_offsets_m"]:
        right_workspace.require_contains(right[:3, 3] + offset, "trace waypoint")
    raise_plan = planner.plan_minimum_peak_speed_trajectory(
        left, right, initial, arm["raise_duration_seconds"], arm["sample_rate_hz"],
        max_joint_step_rad=arm["maximum_joint_offset_rad"],
        max_joint_velocity_rad_s=arm["maximum_raise_velocity_rad_s"], max_candidates=5,
    )
    trace_plan = planner.plan_cartesian_offset_trace(
        left, right, raise_plan["endpoint"]["positions_rad"],
        waypoint_times_seconds=arm["waypoint_times_seconds"],
        right_offsets_m=arm["right_offsets_m"], sample_rate_hz=arm["sample_rate_hz"],
        max_joint_velocity_rad_s=arm["maximum_trace_velocity_rad_s"],
        max_joint_acceleration_rad_s2=arm["maximum_trace_acceleration_rad_s2"],
    )
    return raise_plan, trace_plan


def main(argv=None) -> int:
    args = parse_args(argv)
    request, arm = args.request, args.request["arm"]
    run = RunArtifacts.create(
        root=args.artifact_root, slug=request["attempt_id"], project="handshake",
        purpose="Own-IK replay of an immutable measured Cartesian and hand trace",
        operator_safety_confirmation={
            "area_clear": True, "emergency_stop_ready": True,
            "regular_mode_501_0_confirmed": True, "exact_plan_reviewed": True,
            "hand_motion_reviewed": True,
        },
        worktree=Path(__file__).parent,
        metadata={
            "attempt_id": request["attempt_id"], "request_sha256": args.request_sha256,
            "request": request, "command_argv": list(sys.argv),
            "publishes_robot_commands": True, "publishes_brainco_commands": True,
        },
    )
    adapter = LegacyTelemetryAdapter(run)
    session = controller = planning_monitor = hand = None
    result, reason, plan_summary = "incomplete", "initialization_failed", None
    try:
        planner = G1CartesianArmIK(args.g1_urdf)
        config = replace(
            ContinuousArmConfig(), acquire_seconds=2.0,
            raise_seconds=arm["raise_duration_seconds"],
            return_seconds=arm["return_duration_seconds"], release_seconds=2.0,
            settle_timeout_seconds=30.0, pose_tolerance_rad=0.030,
            max_offset_rad=arm["maximum_joint_offset_rad"],
            max_measured_velocity_rad_s=0.25, max_tracking_error_rad=0.03,
            max_command_lead_rad=0.020, scale_feedforward_by_authority=False,
            step_to_full_authority=True,
        )
        _load_sdk_path()
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(0, args.network_interface)

        def event(name, details):
            if name != "arm_sdk_command" and not run.record(
                "controller-events", "trace-replay-controller", {"event": name, **details}
            ):
                raise RuntimeError(f"could not record controller event {name}")

        planning_monitor = LowStateMonitor(adapter)
        planning_monitor.start()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            if planning_monitor.latest()[0] is not None and planning_monitor.latest_sport()[0] is not None:
                break
            time.sleep(0.01)
        planning_controller = ContinuousArmController(
            planning_monitor.latest, planning_monitor.latest_sport, None, event,
            planner.feedforward, publish_commands=False, config=config,
        )
        planning_pose = planning_controller.observe_initial_pose()
        raise_plan, trace_plan = plan_request(planner, request, planning_pose)
        planning_monitor.close()
        planning_monitor = None
        endpoint = raise_plan["endpoint"]["positions_rad"]
        offsets = {index: endpoint[index] - planning_pose[index] for index in ARM_JOINT_INDICES}
        plan_summary = {
            "raise": {key: value for key, value in raise_plan.items() if key != "samples"},
            "trace": {key: value for key, value in trace_plan.items() if key not in ("samples", "waypoint_errors")},
        }
        if not run.record("events", "trace-replay-planner", {
            "event": "trace_plan_completed_before_physical_session", **plan_summary,
        }):
            raise RuntimeError("could not record trace replay plan")

        monitor = LowStateMonitor(adapter)
        camera = OpenCVMjpegCamera(run, device=args.camera_device, fps=30.0)
        # TTS is deliberately absent: recording synchronization is based on
        # host monotonic timestamps, and a non-control speech RPC must never
        # prevent or invalidate an otherwise reviewed physical run.
        session = EvidenceSession(run, camera, [monitor], announcer=None)
        session.start()
        from handshake.controller import int_to_baudrate, sdk
        is_left = args.brainco_slave_id == 0x7E
        hand = BrainCoHandReplay(
            sdk, args.brainco_port, int_to_baudrate(args.brainco_baud),
            args.brainco_slave_id, event,
            open_positions=(0, 200, 0, 0, 0, 0) if is_left else None,
            close_positions=(0, 200, 1000, 1000, 1000, 1000) if is_left else None,
        )
        hand.start()

        def checked_low_state():
            hand.raise_if_failed()
            return monitor.latest()

        controller = ContinuousArmController(
            checked_low_state, monitor.latest_sport, None, event, planner.feedforward,
            publish_commands=True, config=config,
        )
        execution_pose = controller.observe_initial_pose()
        pose_drift = max(abs(execution_pose[i] - planning_pose[i]) for i in ARM_JOINT_INDICES)
        if pose_drift > 0.01:
            raise RuntimeError(f"planning-to-execution pose drift {pose_drift:.5f} rad exceeds 0.01 rad")
        state, _ = monitor.latest()
        physical_sink = ArmSdkCommandSink(state)
        transport = EvidenceBackedCommandTransport(session, "unitree-arm-sdk", lambda payload: physical_sink(
            payload["positions"], payload["velocities"], payload["feedforward_torques_nm"],
            payload["authority_weight"],
        ))

        def command_sink(positions, velocities, torques, weight):
            transport.send({
                "topic": "rt/arm_sdk", "positions": list(positions),
                "velocities": list(velocities), "feedforward_torques_nm": list(torques),
                "authority_weight": float(weight), "controller_phase": controller.phase,
                "controller_sequence": controller.sequence + 1,
            })

        controller.attach_command_sink(command_sink)
        session.event("physical_publisher_ready", physical_sink.runtime_configuration())
        hand.start_close_ramp(steps=10, period_seconds=0.2)
        controller.raise_arm(offsets)
        source_open_seconds = next(
            item["source_time_seconds"] for item in request["hand"]["source_schedule"]
            if item["positions"] == [0, 0, 0, 0, 0, 0]
        )
        open_delay = (
            source_open_seconds - request["extraction"]["trace_window_seconds"][0]
        ) * request["extraction"]["time_scale"]
        hand.schedule_open(max(0.0, open_delay))
        controller.oscillate(trace_plan)
        hand.open("pre_return_open", wait=True)
        controller.release_arm()
        result, reason = "complete", "trace_ik_hand_close_open_return_release_complete"
    except KeyboardInterrupt:
        result, reason = "aborted", "operator_cancelled"
    except BaseException as exc:
        result, reason = "aborted", f"{type(exc).__name__}: {exc}"
        if controller is not None:
            try:
                controller.abort_release()
                reason += "; abort release completed"
            except BaseException as release_exc:
                reason += f"; abort release failed: {type(release_exc).__name__}: {release_exc}"
    finally:
        if planning_monitor is not None:
            planning_monitor.close()
        if hand is not None:
            try:
                hand.close()
                reason += "; hand fail-safe open completed"
            except BaseException as hand_exc:
                reason += f"; hand fail-safe open failed: {type(hand_exc).__name__}: {hand_exc}"
                if result == "complete":
                    result = "incomplete"
        verification = (
            "# Guarded own-IK trace replay\n\n"
            f"- Attempt: `{request['attempt_id']}`\n"
            f"- Reviewed request SHA-256: `{args.request_sha256}`\n"
            f"- Immutable source SHA-256: `{request['source']['sha256']}`\n"
            f"- Runtime result: **{result}**\n"
            f"- Runtime reason: `{reason}`\n"
            "- Telemetry and timestamped video review: pending post-run analysis.\n"
        )
        if session is None:
            run.finalize(result, reason, verification, metadata={"plan": plan_summary})
        else:
            session.finalize(result, reason, verification, result_metadata={"plan": plan_summary})
            result, reason = session.final_status or result, session.final_reason or reason
    print(f"result={result}; reason={reason}; run={run.directory}")
    return 0 if result == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
