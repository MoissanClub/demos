#!/usr/bin/env python3
"""Generate one offline G1 Cartesian IK step without importing Unitree DDS."""
import argparse
import json
from pathlib import Path

from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.cartesian_command import (
    CartesianDeltaCommand,
    CartesianWorkspace,
    G1CartesianCommandInterface,
)
from handshake.standalone_arm import ARM_JOINT_INDICES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-arm-q", nargs=14, type=float, required=True)
    parser.add_argument("--right-delta-m", nargs=3, type=float, required=True, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--left-delta-m", nargs=3, type=float, default=(0.0, 0.0, 0.0), metavar=("DX", "DY", "DZ"))
    parser.add_argument("--left-workspace-min-m", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--left-workspace-max-m", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--right-workspace-min-m", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--right-workspace-max-m", nargs=3, type=float, required=True, metavar=("X", "Y", "Z"))
    parser.add_argument("--duration-seconds", type=float, default=2.0)
    parser.add_argument("--sample-rate-hz", type=float, default=250.0)
    parser.add_argument("--maximum-displacement-m", type=float, default=0.02)
    parser.add_argument("--maximum-joint-offset-rad", type=float, default=0.40)
    parser.add_argument("--maximum-joint-velocity-rad-s", type=float, default=0.075)
    parser.add_argument("--summary-only", action="store_true")
    parser.add_argument("--g1-urdf", type=Path, default=Path(
        "/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"
    ))
    args = parser.parse_args()
    previous = dict(zip(ARM_JOINT_INDICES, args.initial_arm_q))
    planner = G1CartesianArmIK(args.g1_urdf)
    command = CartesianDeltaCommand(
        right_delta_m=args.right_delta_m,
        left_delta_m=args.left_delta_m,
        duration_seconds=args.duration_seconds,
        sample_rate_hz=args.sample_rate_hz,
        maximum_displacement_m=args.maximum_displacement_m,
        maximum_joint_offset_rad=args.maximum_joint_offset_rad,
        maximum_joint_velocity_rad_s=args.maximum_joint_velocity_rad_s,
    )
    result = G1CartesianCommandInterface(planner).plan(
        command,
        previous,
        CartesianWorkspace(args.left_workspace_min_m, args.left_workspace_max_m),
        CartesianWorkspace(args.right_workspace_min_m, args.right_workspace_max_m),
    )
    if args.summary_only:
        result = {key: value for key, value in result.items() if key != "samples"}
    print(json.dumps({"publishes_commands": False,
                      "model": planner.feedforward.configuration(), "trajectory": result}, indent=2))


if __name__ == "__main__":
    main()
