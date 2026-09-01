#!/usr/bin/env python3
"""Analyze one finalized G1 Cartesian evidence run without commanding hardware."""
from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.standalone_arm import ARM_JOINT_INDICES
from robot_dev_harness.evidence import load_frame_timestamps, nearest_frame


def rows(path: Path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def vector_norm(values):
    return math.sqrt(sum(float(value) ** 2 for value in values))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--g1-urdf", type=Path, default=Path(
        "/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"
    ))
    args = parser.parse_args(argv)

    commands = rows(args.run / "telemetry/commands.jsonl")
    controller_events = rows(args.run / "telemetry/controller-events.jsonl")
    lowstate = rows(args.run / "telemetry/unitree.lowstate.jsonl")
    sport = rows(args.run / "telemetry/unitree.sportmodestate.jsonl")
    frames = load_frame_timestamps(args.run / "video/frame_timestamps.jsonl")
    if not commands or not lowstate or not controller_events:
        raise RuntimeError("run lacks commands, low state, or controller events")

    event_times = {}
    for row in controller_events:
        data = row["data"]
        name = data.get("event")
        phase = data.get("phase")
        if name == "phase_started" and phase:
            event_times[f"{phase}_started"] = row["monotonic_ns"]
        elif name == "phase_finished" and phase:
            event_times[f"{phase}_finished"] = row["monotonic_ns"]
        elif name in {"arm_raised", "arm_released"}:
            event_times[name] = row["monotonic_ns"]

    execution_initial = [
        row["data"]["initial_pose"] for row in controller_events
        if row["data"].get("event") == "phase_finished"
        and row["data"].get("phase") == "initial_observe"
        and row["monotonic_ns"] < event_times["authority_acquire_started"]
    ][-1]
    initial_positions = {int(index): float(value) for index, value in execution_initial.items()}
    planner = G1CartesianArmIK(args.g1_urdf)
    _, initial_right = planner.forward_kinematics(initial_positions)
    initial_xyz = initial_right[:3, 3]

    samples = []
    maximum_gyro = 0.0
    maximum_accel_delta = 0.0
    baseline_accel = lowstate[0]["data"]["value"]["imu"]["accelerometer"]
    for row in lowstate:
        value = row["data"]["value"]
        motors = value["motors"]
        positions = {index: float(motors[index]["position_rad"]) for index in ARM_JOINT_INDICES}
        _, right = planner.forward_kinematics(positions)
        delta = right[:3, 3] - initial_xyz
        velocity_values = {
            index: abs(float(motors[index]["velocity_rad_s"])) for index in ARM_JOINT_INDICES
        }
        torque_values = {
            index: abs(float(motors[index]["estimated_torque_nm"])) for index in ARM_JOINT_INDICES
        }
        samples.append({
            "monotonic_ns": row["monotonic_ns"],
            "positions": positions,
            "delta": tuple(map(float, delta)),
            "maximum_velocity_rad_s": max(velocity_values.values()),
            "maximum_velocity_joint": max(velocity_values, key=velocity_values.get),
            "maximum_torque_nm": max(torque_values.values()),
            "maximum_torque_joint": max(torque_values, key=torque_values.get),
        })
        imu = value["imu"]
        maximum_gyro = max(maximum_gyro, vector_norm(imu["gyroscope"]))
        maximum_accel_delta = max(
            maximum_accel_delta,
            vector_norm(a - b for a, b in zip(imu["accelerometer"], baseline_accel)),
        )

    outbound_end = event_times.get(
        "return_started", event_times.get("authority_release_started", samples[-1]["monotonic_ns"])
    )
    outbound = [sample for sample in samples if sample["monotonic_ns"] <= outbound_end]
    maximum_x_sample = max(outbound, key=lambda sample: sample["delta"][0])
    active = [
        sample for sample in samples
        if event_times["authority_acquire_started"] <= sample["monotonic_ns"]
        < event_times["authority_release_started"]
    ]
    release = [
        sample for sample in samples
        if event_times["authority_release_started"] <= sample["monotonic_ns"]
    ]
    active_velocity_sample = max(active, key=lambda sample: sample["maximum_velocity_rad_s"])
    release_velocity_sample = max(release, key=lambda sample: sample["maximum_velocity_rad_s"])
    active_torque_sample = max(active, key=lambda sample: sample["maximum_torque_nm"])
    return_reference_event = (
        "return_settle_finished"
        if "return_settle_finished" in event_times
        else "authority_release_started"
    )
    return_settled_sample = min(
        samples,
        key=lambda sample: abs(
            sample["monotonic_ns"] - event_times[return_reference_event]
        ),
    )
    raised_reference_event = (
        "arm_raised" if "arm_raised" in event_times else "raised_settle_started"
    )
    raised_sample = min(
        samples,
        key=lambda sample: abs(sample["monotonic_ns"] - event_times[raised_reference_event]),
    )
    raised_xyz = tuple(
        float(initial_xyz[i] + raised_sample["delta"][i]) for i in range(3)
    )
    manifest = json.loads((args.run / "manifest.json").read_text(encoding="utf-8"))
    right_target = manifest.get("metadata", {}).get("right_target_m")
    raised_target_residual = (
        tuple(float(raised_xyz[i] - right_target[i]) for i in range(3))
        if right_target is not None else None
    )
    final_sample = samples[-1]
    oscillation_motion = None
    if "oscillate_started" in event_times and "oscillate_finished" in event_times:
        oscillation_samples = [
            sample for sample in samples
            if event_times["oscillate_started"] <= sample["monotonic_ns"]
            <= event_times["oscillate_finished"]
        ]
        oscillation_center = min(
            samples,
            key=lambda sample: abs(sample["monotonic_ns"] - event_times["oscillate_started"]),
        )["delta"]
        relative = [
            tuple(sample["delta"][axis] - oscillation_center[axis] for axis in range(3))
            for sample in oscillation_samples
        ]
        oscillation_motion = {
            "sample_count": len(relative),
            "right_hand_minimum_from_center_m": [min(row[axis] for row in relative) for axis in range(3)],
            "right_hand_maximum_from_center_m": [max(row[axis] for row in relative) for axis in range(3)],
            "right_hand_peak_to_peak_m": [
                max(row[axis] for row in relative) - min(row[axis] for row in relative)
                for axis in range(3)
            ],
        }
        oscillation_request = (
            manifest.get("metadata", {}).get("request", {})
            .get("command", {}).get("oscillation")
        )
        if oscillation_request is not None:
            period_seconds = 1.0 / float(oscillation_request["frequency_hz"])
            cycle_count = int(round(
                float(oscillation_request["duration_seconds"]) / period_seconds
            ))
            cycles = []
            for cycle_index in range(cycle_count):
                cycle_start = event_times["oscillate_started"] + int(
                    cycle_index * period_seconds * 1e9
                )
                cycle_end = cycle_start + int(period_seconds * 1e9)
                cycle_rows = [
                    sample for sample in oscillation_samples
                    if cycle_start <= sample["monotonic_ns"] <= cycle_end
                ]
                cycle_relative_z = [
                    sample["delta"][2] - oscillation_center[2]
                    for sample in cycle_rows
                ]
                cycles.append({
                    "cycle_index": cycle_index + 1,
                    "minimum_z_from_center_m": min(cycle_relative_z),
                    "maximum_z_from_center_m": max(cycle_relative_z),
                    "peak_to_peak_z_m": max(cycle_relative_z) - min(cycle_relative_z),
                })
            oscillation_motion["cycles"] = cycles
    return_settled_joint_residual = max(
        abs(return_settled_sample["positions"][index] - initial_positions[index])
        for index in ARM_JOINT_INDICES
    )
    post_release_joint_residual = max(
        abs(final_sample["positions"][index] - initial_positions[index])
        for index in ARM_JOINT_INDICES
    )
    command_sequences = [row["sequence"] for row in commands]
    command_times = [row["monotonic_ns"] for row in commands]
    command_intervals = [
        (current - previous) / 1e9
        for previous, current in zip(command_times, command_times[1:])
    ]
    command_times_by_phase = defaultdict(list)
    for row in commands:
        command_times_by_phase[row["data"]["controller_phase"]].append(
            row["monotonic_ns"]
        )
    command_phase_timing = {}
    for phase, timestamps in command_times_by_phase.items():
        intervals = [
            (current - previous) / 1e9
            for previous, current in zip(timestamps, timestamps[1:])
        ]
        command_phase_timing[phase] = {
            "count": len(timestamps),
            "mean_rate_hz": (
                (len(timestamps) - 1) / ((timestamps[-1] - timestamps[0]) / 1e9)
                if len(timestamps) > 1 else None
            ),
            "maximum_interval_seconds": max(intervals) if intervals else None,
        }
    fsm_states = sorted({
        (int(row["data"]["value"]["fsm_id"]), int(row["data"]["value"]["fsm_mode"]))
        for row in sport
    })

    event_frames = {}
    for event, timestamp in event_times.items():
        frame = nearest_frame(frames, timestamp)
        event_frames[event] = {
            "frame_index": frame["frame_index"],
            "frame_timestamp_utc": frame["timestamp_utc"],
            "offset_ms": (frame["monotonic_ns"] - timestamp) / 1e6,
        }

    report = {
        "run": str(args.run),
        "commands": {
            "count": len(commands),
            "contiguous": command_sequences == list(range(len(commands))),
            "mean_rate_hz": (len(commands) - 1) / ((command_times[-1] - command_times[0]) / 1e9),
            "maximum_interval_seconds": max(command_intervals),
            "phase_timing": command_phase_timing,
        },
        "measured_motion": {
            "completed_return": "return_settle_finished" in event_times,
            "return_reference_event": return_reference_event,
            "maximum_right_x_displacement_m": maximum_x_sample["delta"][0],
            "right_y_at_maximum_x_m": maximum_x_sample["delta"][1],
            "right_z_at_maximum_x_m": maximum_x_sample["delta"][2],
            "maximum_active_arm_velocity_rad_s": active_velocity_sample["maximum_velocity_rad_s"],
            "maximum_active_velocity_joint": active_velocity_sample["maximum_velocity_joint"],
            "maximum_release_arm_velocity_rad_s": release_velocity_sample["maximum_velocity_rad_s"],
            "maximum_release_velocity_joint": release_velocity_sample["maximum_velocity_joint"],
            "maximum_active_arm_torque_nm": active_torque_sample["maximum_torque_nm"],
            "maximum_active_torque_joint": active_torque_sample["maximum_torque_joint"],
            "raised_reference_event": raised_reference_event,
            "raised_right_hand_m": raised_xyz,
            "raised_right_hand_target_residual_m": raised_target_residual,
            "raised_right_hand_target_error_norm_m": (
                vector_norm(raised_target_residual)
                if raised_target_residual is not None else None
            ),
            "return_settled_right_hand_residual_m": return_settled_sample["delta"],
            "return_settled_maximum_joint_residual_rad": return_settled_joint_residual,
            "post_native_release_right_hand_residual_m": final_sample["delta"],
            "post_native_release_maximum_joint_residual_rad": post_release_joint_residual,
            "oscillation": oscillation_motion,
        },
        "imu": {
            "maximum_gyroscope_norm_rad_s": maximum_gyro,
            "maximum_acceleration_change_from_first_sample_m_s2": maximum_accel_delta,
        },
        "fsm_states": fsm_states,
        "event_frames": event_frames,
    }
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
