#!/usr/bin/env python3
"""Record a synchronized, read-only robot telemetry and camera evidence run."""
from __future__ import annotations

import argparse
import shutil
import sys
import time
from pathlib import Path

from robot_dev_harness.adapters import LegacyTelemetryAdapter
from robot_dev_harness.evidence import extract_nearest_frame, load_frame_timestamps
from robot_dev_harness.run_artifacts import RunArtifacts
from robot_dev_harness.opencv_camera import OpenCVMjpegCamera


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
    monitor = None
    camera_active = False
    result = "incomplete"
    reason = "initialization_failed"
    camera_summary = None
    try:
        from g1_standalone_arm_sequence import LowStateMonitor, _load_sdk_path

        _load_sdk_path()
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize

        ChannelFactoryInitialize(0, args.network_interface)
        monitor = LowStateMonitor(adapter)
        monitor.start()
        camera.start()
        camera_active = True
        run.record("events", "harness", {
            "event": "read_only_recording_started",
            "publishes_robot_commands": False,
        })
        deadline = time.monotonic() + args.duration_seconds
        while time.monotonic() < deadline:
            if not camera.active or camera.error:
                raise RuntimeError(camera.error or "camera stopped before recording deadline")
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        camera_summary = camera.stop()
        camera_active = False
        frame_index_path = run.path("video/frame_timestamps.jsonl")
        frame_rows = load_frame_timestamps(frame_index_path)
        evidence = []
        for event, event_ns in (
            ("recording-baseline", frame_rows[0]["monotonic_ns"]),
            ("recording-final", frame_rows[-1]["monotonic_ns"]),
        ):
            item = extract_nearest_frame(
                run.path(camera_summary["path"]), frame_index_path, event_ns,
                event, run.path("evidence"),
            )
            item["evidence_path"] = str(
                Path(item["evidence_path"]).relative_to(run.directory.resolve())
            )
            evidence.append(item)
        run.record("events", "harness", {
            "event": "read_only_recording_stopped",
            "camera": camera_summary,
            "evidence": evidence,
        })
        result, reason = "complete", "requested_read_only_capture_complete"
    except KeyboardInterrupt:
        result, reason = "aborted", "operator_cancelled"
    except BaseException as exc:
        result = "incomplete"
        reason = f"{type(exc).__name__}: {exc}"
    finally:
        if monitor is not None:
            monitor.close()
        if camera_active and camera.active:
            try:
                camera_summary = camera.stop()
            except BaseException as exc:
                reason += f"; camera cleanup failed: {type(exc).__name__}: {exc}"
        verification = (
            "# Read-only development run\n\n"
            f"- Result: **{result}**\n"
            f"- Reason: `{reason}`\n"
            "- Robot commands published: **no**\n"
            "- Automated movement verification: not applicable to this capture rehearsal.\n"
        )
        run.finalize(
            result, reason, verification,
            metadata={"camera": camera_summary} if camera_summary else None,
        )
    print(f"result={result}; reason={reason}; run={run.directory}")
    return 0 if result == "complete" else 1


if __name__ == "__main__":
    raise SystemExit(main())
