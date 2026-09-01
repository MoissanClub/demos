#!/usr/bin/env python3
"""Extract an immutable Cartesian replay request from measured trajectory evidence."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.standalone_arm import ARM_JOINT_INDICES


SCHEMA_VERSION = "1.0"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--expect-source-sha256", required=True)
    parser.add_argument("--attempt-id", required=True)
    parser.add_argument("--offset-scale", type=float, default=0.25)
    parser.add_argument("--time-scale", type=float, default=4.0)
    parser.add_argument("--g1-urdf", type=Path, default=Path(
        "/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"
    ))
    return parser.parse_args(argv)


def load_source(path: Path):
    states, events, hand_commands = [], [], []
    summary = None
    for line_number, line in enumerate(path.open(encoding="utf-8"), 1):
        try:
            row = json.loads(line)
        except json.JSONDecodeError as exc:
            raise ValueError(f"malformed source JSON at line {line_number}") from exc
        timestamp = int(row["timestamp_monotonic_ns"])
        data = row["data"]
        if row["stream"] == "unitree.lowstate":
            motors = data["motors"]
            states.append((timestamp, {
                index: float(motors[index]["position_rad"])
                for index in ARM_JOINT_INDICES
            }))
        elif row["stream"] == "controller.event":
            events.append((timestamp, data))
        elif row["stream"] == "controller.command" \
                and data.get("kind") == "finger_positions":
            hand_commands.append((timestamp, data))
        elif row["stream"] == "trajectory.summary":
            summary = data
    if summary is None or summary.get("result") != "success":
        raise ValueError("source trace is not a complete successful trajectory")
    return states, events, hand_commands, summary


def main(argv=None) -> int:
    args = parse_args(argv)
    source_sha256 = hashlib.sha256(args.source.read_bytes()).hexdigest()
    if source_sha256 != args.expect_source_sha256.lower():
        raise ValueError("source trace SHA-256 does not match the reviewed value")
    if not 0.05 <= args.offset_scale <= 1.0:
        raise ValueError("offset scale must be between 0.05 and 1.0")
    if not 1.0 <= args.time_scale <= 10.0:
        raise ValueError("time scale must be between 1.0 and 10.0")
    states, events, hand_commands, summary = load_source(args.source)
    action_time = next(
        timestamp for timestamp, data in events
        if data.get("event") == "arm_action_requested"
    )
    release_time = next(
        timestamp for timestamp, data in events
        if data.get("event") == "arm_release_requested"
    )
    if (release_time - action_time) / 1e9 < 19.0:
        raise ValueError("source trace does not contain the reviewed full arm action")

    planner = G1CartesianArmIK(args.g1_urdf)
    sample_times = np.arange(0.0, 20.0001, 0.05)
    source_positions = []
    source_rotations = []
    source_joints = []
    for time_seconds in sample_times:
        _, joints = min(
            states, key=lambda item: abs(item[0] - (action_time + time_seconds * 1e9))
        )
        _, right = planner.forward_kinematics(joints)
        source_positions.append(right[:3, 3])
        source_rotations.append(right[:3, :3])
        source_joints.append(joints)
    source_positions = np.asarray(source_positions)
    center_mask = (sample_times >= 3.0) & (sample_times <= 5.0)
    center_position = np.median(source_positions[center_mask], axis=0)
    center_rotation = source_rotations[np.flatnonzero(center_mask)[len(np.flatnonzero(center_mask)) // 2]]

    trace_source_times = np.arange(5.5, 9.0001, 0.25)
    raw_offsets = np.asarray([
        source_positions[np.argmin(np.abs(sample_times - timestamp))] - center_position
        for timestamp in trace_source_times
    ])
    # Remove source endpoint drift so the replay is exactly closed and can
    # hand continuously back to the reviewed raised pose before return.
    fractions = np.linspace(0.0, 1.0, len(raw_offsets))[:, None]
    closed_offsets = raw_offsets - (
        raw_offsets[0] + fractions * (raw_offsets[-1] - raw_offsets[0])
    )
    scaled_offsets = closed_offsets * args.offset_scale
    replay_times = (trace_source_times - trace_source_times[0]) * args.time_scale

    initial_joints = source_joints[0]
    hand_schedule = []
    for timestamp, data in hand_commands:
        relative = (timestamp - action_time) / 1e9
        if relative <= 2.2 or data.get("positions") == [0, 0, 0, 0, 0, 0]:
            hand_schedule.append({
                "source_time_seconds": relative,
                "positions": list(map(int, data["positions"])),
                "reason": str(data.get("reason")),
            })

    request = {
        "schema_version": SCHEMA_VERSION,
        "attempt_id": args.attempt_id,
        "source": {
            "path": str(args.source),
            "sha256": source_sha256,
            "trajectory_id": summary["trajectory_id"],
            "action_duration_seconds": (release_time - action_time) / 1e9,
        },
        "extraction": {
            "raised_center_window_seconds": [3.0, 5.0],
            "trace_window_seconds": [5.5, 9.0],
            "source_waypoint_period_seconds": 0.25,
            "endpoint_detrending": "linear_cartesian_offset",
            "offset_scale": args.offset_scale,
            "time_scale": args.time_scale,
        },
        "arm": {
            "raise_duration_seconds": 10.0,
            "return_duration_seconds": 10.0,
            "right_center_m": list(map(float, center_position)),
            "right_orientation": [list(map(float, row)) for row in center_rotation],
            "waypoint_times_seconds": list(map(float, replay_times)),
            "right_offsets_m": [list(map(float, row)) for row in scaled_offsets],
            "sample_rate_hz": 250.0,
            "maximum_displacement_m": 0.38,
            "maximum_joint_offset_rad": 0.9,
            "maximum_raise_velocity_rad_s": 0.16,
            "maximum_trace_velocity_rad_s": 0.10,
            "maximum_trace_acceleration_rad_s2": 0.25,
            "initial_reference_positions_rad": {
                str(index): float(initial_joints[index]) for index in ARM_JOINT_INDICES
            },
        },
        "hand": {
            "source_schedule": hand_schedule,
            "replay_policy": "close_during_raise_then_open_at_scaled_source_phase",
            "maximum_position": 500,
        },
        "workspace_m": {
            "left": {"minimum": [-0.03, 0.19, -0.15], "maximum": [0.06, 0.28, -0.06]},
            "right": {"minimum": [-0.02, -0.27, -0.14], "maximum": [0.29, -0.11, 0.14]},
        },
    }
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(request, output, indent=2, sort_keys=True)
        output.write("\n")
    print(json.dumps({
        "output": str(args.output),
        "source_sha256": source_sha256,
        "right_center_m": request["arm"]["right_center_m"],
        "raw_cartesian_peak_to_peak_m": list(map(float, np.ptp(raw_offsets, axis=0))),
        "replay_cartesian_peak_to_peak_m": list(map(float, np.ptp(scaled_offsets, axis=0))),
        "replay_duration_seconds": float(replay_times[-1]),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
