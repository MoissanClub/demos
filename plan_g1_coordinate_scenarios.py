#!/usr/bin/env python3
"""Plan a coordinate scenario suite offline; never imports Unitree DDS."""
import argparse
import json
from pathlib import Path

from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.cartesian_command import (
    CartesianWorkspace, CoordinateMoveSafety, G1CoordinateMover,
)
from handshake.standalone_arm import ARM_JOINT_INDICES


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path, required=True)
    parser.add_argument("--initial-arm-q", nargs=14, type=float, required=True)
    parser.add_argument("--g1-urdf", type=Path, default=Path(
        "/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"
    ))
    args = parser.parse_args(argv)
    suite = json.loads(args.suite.read_text(encoding="utf-8"))
    initial = dict(zip(ARM_JOINT_INDICES, args.initial_arm_q))
    limits = suite["shared_limits"]
    workspace = suite["workspace_m"]
    safety = CoordinateMoveSafety(
        left_workspace=CartesianWorkspace(
            workspace["left"]["minimum"], workspace["left"]["maximum"],
        ),
        right_workspace=CartesianWorkspace(
            workspace["right"]["minimum"], workspace["right"]["maximum"],
        ),
        maximum_displacement_m=limits["maximum_displacement_m"],
        maximum_joint_offset_rad=limits["maximum_joint_offset_rad"],
        maximum_joint_velocity_rad_s=limits["maximum_joint_velocity_rad_s"],
        sample_rate_hz=limits["sample_rate_hz"],
        max_ik_candidates=limits["max_ik_candidates"],
    )
    planner = G1CartesianArmIK(args.g1_urdf)
    reports = []
    for scenario in suite["scenarios"]:
        executed = []
        mover = G1CoordinateMover(planner, lambda: initial, executed.append, safety)
        try:
            plan = mover.move(
                scenario["target_m"], scenario["maximum_time_seconds"],
            )
            reports.append({
                "id": scenario["id"],
                "valid": True,
                "target_m": scenario["target_m"],
                "maximum_time_seconds": scenario["maximum_time_seconds"],
                "maximum_joint_velocity_rad_s": plan["maximum_joint_velocity_rad_s"],
                "maximum_joint_step_rad": plan["endpoint"]["maximum_joint_step_rad"],
                "translation_error_m": plan["endpoint"]["translation_error_m"],
                "rotation_error_rad": plan["endpoint"]["rotation_error_rad"],
                "ik_selection": plan["endpoint"]["ik_selection"],
            })
        except (RuntimeError, ValueError) as exc:
            reports.append({
                "id": scenario["id"], "valid": False,
                "reason": f"{type(exc).__name__}: {exc}",
            })
    print(json.dumps({"publishes_commands": False, "scenarios": reports}, indent=2))
    return 0 if all(report["valid"] for report in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())
