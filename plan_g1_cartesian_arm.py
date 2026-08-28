#!/usr/bin/env python3
"""Generate one offline G1 Cartesian IK step without importing Unitree DDS."""
import argparse
import json
from pathlib import Path

from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.standalone_arm import ARM_JOINT_INDICES


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-arm-q", nargs=14, type=float, required=True)
    parser.add_argument("--right-delta-m", nargs=3, type=float, required=True, metavar=("DX", "DY", "DZ"))
    parser.add_argument("--duration-seconds", type=float, default=2.0)
    parser.add_argument("--sample-rate-hz", type=float, default=250.0)
    parser.add_argument("--summary-only", action="store_true")
    parser.add_argument("--g1-urdf", type=Path, default=Path(
        "/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"
    ))
    args = parser.parse_args()
    if max(abs(v) for v in args.right_delta_m) > 0.02:
        parser.error("each Cartesian test delta is limited to 0.02 m")
    previous = dict(zip(ARM_JOINT_INDICES, args.initial_arm_q))
    planner = G1CartesianArmIK(args.g1_urdf)
    left, right = planner.forward_kinematics(previous)
    right[:3, 3] += args.right_delta_m
    result = planner.plan_trajectory(
        left, right, previous, args.duration_seconds, args.sample_rate_hz
    )
    if args.summary_only:
        result = {key: value for key, value in result.items() if key != "samples"}
    print(json.dumps({"publishes_commands": False, "right_delta_m": args.right_delta_m,
                      "model": planner.feedforward.configuration(), "trajectory": result}, indent=2))


if __name__ == "__main__":
    main()
