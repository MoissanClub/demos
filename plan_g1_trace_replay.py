#!/usr/bin/env python3
"""Offline-plan a hash-bound Cartesian trace replay request."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.cartesian_command import CartesianWorkspace
from handshake.standalone_arm import ARM_JOINT_INDICES


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("request", type=Path)
    parser.add_argument("--expect-request-sha256", required=True)
    parser.add_argument("--g1-urdf", type=Path, default=Path(
        "/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"
    ))
    args = parser.parse_args(argv)
    raw = args.request.read_bytes()
    request_sha256 = hashlib.sha256(raw).hexdigest()
    if request_sha256 != args.expect_request_sha256.lower():
        parser.error("request SHA-256 mismatch")
    request = json.loads(raw)
    if request.get("schema_version") != "1.0":
        parser.error("unsupported trace replay schema")
    source = Path(request["source"]["path"])
    if hashlib.sha256(source.read_bytes()).hexdigest() != request["source"]["sha256"]:
        parser.error("immutable source trace hash mismatch")

    planner = G1CartesianArmIK(args.g1_urdf)
    arm = request["arm"]
    initial = {int(index): float(value) for index, value in arm["initial_reference_positions_rad"].items()}
    left, right = planner.forward_kinematics(initial)
    left_workspace = CartesianWorkspace(
        request["workspace_m"]["left"]["minimum"], request["workspace_m"]["left"]["maximum"]
    )
    right_workspace = CartesianWorkspace(
        request["workspace_m"]["right"]["minimum"], request["workspace_m"]["right"]["maximum"]
    )
    left_workspace.require_contains(left[:3, 3], "initial left hand")
    right_workspace.require_contains(right[:3, 3], "initial right hand")
    right_workspace.require_contains(arm["right_center_m"], "trace raised center")
    for offset in arm["right_offsets_m"]:
        right_workspace.require_contains(
            [center + delta for center, delta in zip(arm["right_center_m"], offset)],
            "trace waypoint",
        )
    right[:3, 3] = arm["right_center_m"]
    right[:3, :3] = arm["right_orientation"]
    raise_plan = planner.plan_minimum_peak_speed_trajectory(
        left, right, initial, arm["raise_duration_seconds"], arm["sample_rate_hz"],
        max_joint_step_rad=arm["maximum_joint_offset_rad"],
        max_joint_velocity_rad_s=arm["maximum_raise_velocity_rad_s"],
        max_candidates=5,
    )
    trace_plan = planner.plan_cartesian_offset_trace(
        left, right, raise_plan["endpoint"]["positions_rad"],
        waypoint_times_seconds=arm["waypoint_times_seconds"],
        right_offsets_m=arm["right_offsets_m"],
        sample_rate_hz=arm["sample_rate_hz"],
        max_joint_velocity_rad_s=arm["maximum_trace_velocity_rad_s"],
        max_joint_acceleration_rad_s2=arm["maximum_trace_acceleration_rad_s2"],
    )
    print(json.dumps({
        "request_sha256": request_sha256,
        "source_sha256": request["source"]["sha256"],
        "raise": {key: value for key, value in raise_plan.items() if key != "samples"},
        "trace": {key: value for key, value in trace_plan.items() if key not in ("samples", "waypoint_errors")},
        "trace_maximum_waypoint_translation_error_m": max(
            max(item["translation_error_m"].values()) for item in trace_plan["waypoint_errors"]
        ),
        "trace_maximum_waypoint_rotation_error_rad": max(
            max(item["rotation_error_rad"].values()) for item in trace_plan["waypoint_errors"]
        ),
        "closed_endpoint_maximum_joint_error_rad": max(
            abs(trace_plan["samples"][-1]["positions_rad"][index] - raise_plan["endpoint"]["positions_rad"][index])
            for index in ARM_JOINT_INDICES
        ),
    }, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
