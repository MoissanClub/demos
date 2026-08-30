#!/usr/bin/env python3
"""Execute one exact, reviewed G1 Cartesian test with synchronized evidence."""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import replace
from pathlib import Path

from g1_standalone_arm_sequence import ArmSdkCommandSink, LowStateMonitor, _load_sdk_path
from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.cartesian_command import (
    CartesianDeltaCommand, CartesianWorkspace, G1CartesianCommandInterface,
)
from handshake.continuous_arm import ContinuousArmConfig, ContinuousArmController
from handshake.standalone_arm import ARM_JOINT_INDICES
from robot_dev_harness.adapters import LegacyTelemetryAdapter
from robot_dev_harness.commands import EvidenceBackedCommandTransport
from robot_dev_harness.opencv_camera import OpenCVMjpegCamera
from robot_dev_harness.run_artifacts import RunArtifacts
from robot_dev_harness.session import EvidenceSession


ATTEMPT_ID = "20260830-right-x-1cm-a"
PHYSICAL_EXECUTION_ENABLED = True
RIGHT_DELTA_M = (0.01, 0.0, 0.0)
LEFT_WORKSPACE = CartesianWorkspace(
    (-0.0085, 0.2166, -0.1278), (0.0316, 0.2567, -0.0877),
)
RIGHT_WORKSPACE = CartesianWorkspace(
    (-0.0083, -0.2559, -0.1280), (0.0418, -0.2157, -0.0878),
)
COMMAND = CartesianDeltaCommand(
    right_delta_m=RIGHT_DELTA_M,
    duration_seconds=8.0,
    sample_rate_hz=250.0,
    maximum_displacement_m=0.01,
    maximum_joint_offset_rad=0.05,
    maximum_joint_velocity_rad_s=0.02,
)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-reviewed-attempt", choices=(ATTEMPT_ID,), required=True)
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--camera-device", default="/dev/video6")
    parser.add_argument("--artifact-root", type=Path, default=Path("artifacts/robot_dev_runs"))
    parser.add_argument("--g1-urdf", type=Path, default=Path(
        "/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"
    ))
    parser.add_argument("--confirm-area-clear", action="store_true")
    parser.add_argument("--confirm-estop-ready", action="store_true")
    parser.add_argument("--confirm-regular-mode-501-0", action="store_true")
    parser.add_argument("--confirm-plan-reviewed", action="store_true")
    args = parser.parse_args(argv)
    if not PHYSICAL_EXECUTION_ENABLED:
        parser.error(f"reviewed attempt {ATTEMPT_ID} is currently hard-disabled")
    if not all((
        args.confirm_area_clear, args.confirm_estop_ready,
        args.confirm_regular_mode_501_0, args.confirm_plan_reviewed,
    )):
        parser.error(
            "execution requires area-clear, emergency-stop, Regular-mode, "
            "and exact-plan confirmations"
        )
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    run = RunArtifacts.create(
        root=args.artifact_root,
        slug=ATTEMPT_ID,
        project="handshake",
        purpose="Guarded 1 cm right-arm world-X Cartesian out-and-return test",
        operator_safety_confirmation={
            "area_clear": True,
            "emergency_stop_ready": True,
            "regular_mode_501_0_confirmed": True,
            "exact_plan_reviewed": True,
        },
        worktree=Path(__file__).parent,
        metadata={
            "attempt_id": ATTEMPT_ID,
            "command_argv": list(sys.argv if argv is None else [sys.argv[0], *argv]),
            "publishes_robot_commands": True,
            "right_delta_m": list(RIGHT_DELTA_M),
        },
    )
    adapter = LegacyTelemetryAdapter(run)
    session = None
    controller = None
    result, reason = "incomplete", "initialization_failed"
    plan_summary = None
    try:
        _load_sdk_path()
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(0, args.network_interface)
        monitor = LowStateMonitor(adapter)
        camera = OpenCVMjpegCamera(run, device=args.camera_device, fps=30.0)
        session = EvidenceSession(run, camera, [monitor])
        session.start()

        planner = G1CartesianArmIK(args.g1_urdf)
        config = replace(
            ContinuousArmConfig(),
            acquire_seconds=2.0,
            raise_seconds=COMMAND.duration_seconds,
            return_seconds=COMMAND.duration_seconds,
            release_seconds=2.0,
            settle_timeout_seconds=30.0,
            max_offset_rad=COMMAND.maximum_joint_offset_rad,
            max_measured_velocity_rad_s=0.25,
            max_tracking_error_rad=0.03,
            max_command_lead_rad=0.020,
            scale_feedforward_by_authority=False,
            step_to_full_authority=True,
        )

        def event(name, details):
            # Exact published payloads are already recorded before transport in
            # the commands stream. Avoid duplicating them at 250 Hz here.
            if name == "arm_sdk_command":
                return
            if not run.record("controller-events", "continuous-arm-controller", {
                "event": name, **details,
            }):
                raise RuntimeError(f"could not record controller event {name}")

        controller = ContinuousArmController(
            monitor.latest, monitor.latest_sport, None, event, planner.feedforward,
            publish_commands=True, config=config,
        )
        controller.observe_initial_pose()
        plan = G1CartesianCommandInterface(planner).plan(
            COMMAND, controller.initial_pose, LEFT_WORKSPACE, RIGHT_WORKSPACE,
        )
        endpoint = plan["endpoint"]["positions_rad"]
        offsets = {
            index: endpoint[index] - controller.initial_pose[index]
            for index in ARM_JOINT_INDICES
        }
        plan_summary = {key: value for key, value in plan.items() if key != "samples"}
        session.event("runtime_cartesian_plan_accepted", plan_summary)

        state, state_ns = monitor.latest()
        if state is None or state_ns is None:
            raise RuntimeError("fresh low state disappeared before publisher construction")
        physical_sink = ArmSdkCommandSink(state)

        def publish(payload):
            physical_sink(
                payload["positions"], payload["velocities"],
                payload["feedforward_torques_nm"], payload["authority_weight"],
            )

        evidence_transport = EvidenceBackedCommandTransport(
            session, "unitree-arm-sdk", publish,
        )

        def command_sink(positions, velocities, torques, weight):
            evidence_transport.send({
                "topic": "rt/arm_sdk",
                "positions": list(positions),
                "velocities": list(velocities),
                "feedforward_torques_nm": list(torques),
                "authority_weight": float(weight),
                "controller_phase": controller.phase,
                "controller_sequence": controller.sequence + 1,
            })

        controller.attach_command_sink(command_sink)
        session.event("physical_publisher_ready", physical_sink.runtime_configuration())
        controller.raise_arm(offsets)
        hold_deadline = time.monotonic() + 1.0
        while time.monotonic() < hold_deadline:
            controller.hold_once()
            time.sleep(1.0 / config.sample_rate_hz)
        controller.release_arm()
        result, reason = "complete", "cartesian_out_return_release_complete"
    except KeyboardInterrupt:
        result, reason = "aborted", "operator_cancelled"
    except BaseException as exc:
        result, reason = "aborted", f"{type(exc).__name__}: {exc}"
        if controller is not None:
            try:
                controller.abort_release()
                reason += "; abort release completed"
            except BaseException as release_exc:
                reason += (
                    f"; abort release failed: {type(release_exc).__name__}: {release_exc}"
                )
    finally:
        verification = (
            "# Guarded Cartesian physical test\n\n"
            f"- Attempt: `{ATTEMPT_ID}`\n"
            f"- Runtime result: **{result}**\n"
            f"- Runtime reason: `{reason}`\n"
            "- Visual and telemetry analysis: pending post-run review.\n"
        )
        if session is None:
            run.finalize(result, reason, verification, metadata={"plan": plan_summary})
        else:
            session.finalize(
                result, reason, verification,
                result_metadata={"plan": plan_summary},
            )
            result = session.final_status or result
            reason = session.final_reason or reason
    print(f"result={result}; reason={reason}; run={run.directory}")
    return 0 if result == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
