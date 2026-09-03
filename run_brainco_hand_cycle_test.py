#!/usr/bin/env python3
"""Guarded camera-recorded BrainCo close/open verification without arm commands."""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from handshake.brainco_replay import BrainCoHandReplay
from robot_dev_harness.opencv_camera import OpenCVMjpegCamera
from robot_dev_harness.run_artifacts import RunArtifacts
from robot_dev_harness.session import EvidenceSession


PHYSICAL_EXECUTION_ENABLED = False


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-id", required=True)
    parser.add_argument("--camera-device", default="/dev/video6")
    parser.add_argument("--brainco-port", default=(
        "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA1LW3T-if01-port0"
    ))
    parser.add_argument("--brainco-slave-id", type=lambda value: int(value, 0), default=0x7F)
    parser.add_argument("--brainco-baud", type=int, default=460800)
    parser.add_argument("--artifact-root", type=Path, default=Path("artifacts/robot_dev_runs"))
    parser.add_argument("--confirm-area-clear", action="store_true")
    parser.add_argument("--confirm-estop-ready", action="store_true")
    parser.add_argument("--confirm-hand-motion-reviewed", action="store_true")
    args = parser.parse_args(argv)
    if not PHYSICAL_EXECUTION_ENABLED:
        parser.error("physical hand execution is hard-disabled in source")
    if not all((args.confirm_area_clear, args.confirm_estop_ready,
                args.confirm_hand_motion_reviewed)):
        parser.error("all physical safety confirmations are required")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    run = RunArtifacts.create(
        root=args.artifact_root, slug=args.attempt_id, project="handshake",
        purpose="Hand-only six-channel BrainCo close/open verification",
        operator_safety_confirmation={
            "area_clear": True, "emergency_stop_ready": True,
            "hand_motion_reviewed": True,
        },
        worktree=Path(__file__).parent,
        metadata={"publishes_robot_commands": False, "publishes_brainco_commands": True,
                  "command_argv": list(sys.argv)},
    )
    camera = OpenCVMjpegCamera(run, device=args.camera_device, fps=30.0)
    session = EvidenceSession(run, camera, [], announcer=None)
    hand = None
    result, reason = "incomplete", "initialization_failed"

    def event(name, details):
        if not run.record("controller-events", "brainco-hand-cycle", {
            "event": name, **details,
        }):
            raise RuntimeError(f"could not record hand event {name}")

    try:
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
        hand.start_close_ramp(steps=10, period_seconds=0.2)
        hand.wait_schedulers(10.0)
        event("hand_close_verified", {"positions": list(hand.close_positions)})
        time.sleep(2.0)
        hand.command(
            hand.open_positions, "verified_open", wait=True,
            settle_tolerance=30, settle_timeout_seconds=5.0,
        )
        event("hand_open_verified", {"positions": list(hand.open_positions)})
        time.sleep(1.0)
        result, reason = "complete", "six_channel_close_open_measured_complete"
    except KeyboardInterrupt:
        result, reason = "aborted", "operator_cancelled"
    except BaseException as exc:
        result, reason = "aborted", f"{type(exc).__name__}: {exc}"
    finally:
        if hand is not None:
            try:
                hand.close()
                reason += "; hand fail-safe open completed"
            except BaseException as exc:
                reason += f"; hand fail-safe open failed: {type(exc).__name__}: {exc}"
                if result == "complete":
                    result = "incomplete"
        verification = (
            "# Guarded BrainCo hand-only cycle\n\n"
            f"- Attempt: `{args.attempt_id}`\n"
            f"- Runtime result: **{result}**\n"
            f"- Runtime reason: `{reason}`\n"
            "- Timestamped telemetry and video review: pending post-run analysis.\n"
        )
        session.finalize(result, reason, verification)
        result, reason = session.final_status or result, session.final_reason or reason
    print(f"result={result}; reason={reason}; run={run.directory}")
    return 0 if result == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
