#!/usr/bin/env python3
"""Record a synchronized, read-only robot telemetry and camera evidence run."""
from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

from robot_dev_harness.adapters import LegacyTelemetryAdapter
from robot_dev_harness.run_artifacts import RunArtifacts
from robot_dev_harness.opencv_camera import OpenCVMjpegCamera
from robot_dev_harness.session import EvidenceSession
from g1_recording_announcer import UnitreeRecordingAnnouncer


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--project", default="robot-development")
    parser.add_argument("--slug", required=True)
    parser.add_argument("--purpose", required=True)
    parser.add_argument("--duration-seconds", type=float, default=5.0)
    parser.add_argument("--artifact-root", type=Path, default=Path("artifacts/robot_dev_runs"))
    parser.add_argument("--network-interface", default="eth0")
    parser.add_argument("--camera-device", default="/dev/video6")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--minimum-free-gib", type=float, default=1.0)
    parser.add_argument("--speaker-id", type=int, default=0)
    parser.add_argument("--confirm-area-clear", action="store_true")
    args = parser.parse_args(argv)
    if not 1.0 <= args.duration_seconds <= 600.0:
        parser.error("duration must be between 1 and 600 seconds")
    if not 0.1 <= args.minimum_free_gib <= 100.0:
        parser.error("minimum free space must be between 0.1 and 100 GiB")
    if not args.confirm_area_clear:
        parser.error("read-only physical recording requires --confirm-area-clear")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    args.artifact_root.mkdir(parents=True, exist_ok=True)
    free_bytes = shutil.disk_usage(args.artifact_root).free
    required_bytes = int(args.minimum_free_gib * 1024 ** 3)
    if free_bytes < required_bytes:
        raise RuntimeError(
            f"insufficient artifact storage: {free_bytes} bytes free, "
            f"{required_bytes} required"
        )

    run = RunArtifacts.create(
        root=args.artifact_root,
        slug=args.slug,
        project=args.project,
        purpose=args.purpose,
        operator_safety_confirmation={
            "area_clear": True,
            "confirmation_source": "--confirm-area-clear",
        },
        worktree=Path(__file__).parent,
        metadata={
            "command_argv": list(sys.argv if argv is None else [sys.argv[0], *argv]),
            "publishes_robot_commands": False,
            "requested_duration_seconds": args.duration_seconds,
            "storage_free_bytes_at_start": free_bytes,
        },
    )
    adapter = LegacyTelemetryAdapter(run)
    camera = OpenCVMjpegCamera(
        run, device=args.camera_device, width=args.camera_width,
        height=args.camera_height, fps=args.camera_fps,
    )
    session = None
    result = "incomplete"
    reason = "initialization_failed"
    try:
        from g1_standalone_arm_sequence import LowStateMonitor, _load_sdk_path

        _load_sdk_path()
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(0, args.network_interface)
        monitor = LowStateMonitor(adapter)
        announcer = UnitreeRecordingAnnouncer(run, speaker_id=args.speaker_id)
        session = EvidenceSession(run, camera, [monitor], announcer=announcer)
        session.start()
        session.event("read_only_recording_started", {
            "publishes_robot_commands": False,
        })
        deadline = time.monotonic() + args.duration_seconds
        while time.monotonic() < deadline:
            session.require_ready()
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        session.event("read_only_recording_duration_complete")
        result, reason = "complete", "requested_read_only_capture_complete"
    except KeyboardInterrupt:
        result, reason = "aborted", "operator_cancelled"
    except BaseException as exc:
        result = "incomplete"
        reason = f"{type(exc).__name__}: {exc}"
    finally:
        verification = (
            "# Read-only development run\n\n"
            f"- Result: **{result}**\n"
            f"- Reason: `{reason}`\n"
            "- Robot commands published: **no**\n"
            "- Automated movement verification: not applicable to this capture rehearsal.\n"
        )
        if session is None:
            run.finalize(result, reason, verification)
        else:
            session.finalize(result, reason, verification)
            result = session.final_status or result
            reason = session.final_reason or reason
    print(f"result={result}; reason={reason}; run={run.directory}")
    return 0 if result == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
