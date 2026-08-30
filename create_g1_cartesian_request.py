#!/usr/bin/env python3
"""Create one immutable, hash-addressed G1 Cartesian move request."""
import argparse
from pathlib import Path

from handshake.cartesian_command import CartesianPositionCommand, CartesianWorkspace
from handshake.cartesian_request import CartesianMoveRequest


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--attempt-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--right-target-m", nargs=3, type=float, required=True,
                        metavar=("X", "Y", "Z"))
    parser.add_argument("--left-workspace-min-m", nargs=3, type=float, required=True)
    parser.add_argument("--left-workspace-max-m", nargs=3, type=float, required=True)
    parser.add_argument("--right-workspace-min-m", nargs=3, type=float, required=True)
    parser.add_argument("--right-workspace-max-m", nargs=3, type=float, required=True)
    parser.add_argument("--duration-seconds", type=float, default=20.0)
    parser.add_argument("--sample-rate-hz", type=float, default=250.0)
    parser.add_argument("--maximum-displacement-m", type=float, default=0.01)
    parser.add_argument("--maximum-joint-offset-rad", type=float, default=0.05)
    parser.add_argument("--maximum-joint-velocity-rad-s", type=float, default=0.02)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    request = CartesianMoveRequest(
        attempt_id=args.attempt_id,
        command=CartesianPositionCommand(
            right_target_m=args.right_target_m,
            duration_seconds=args.duration_seconds,
            sample_rate_hz=args.sample_rate_hz,
            maximum_displacement_m=args.maximum_displacement_m,
            maximum_joint_offset_rad=args.maximum_joint_offset_rad,
            maximum_joint_velocity_rad_s=args.maximum_joint_velocity_rad_s,
        ),
        left_workspace=CartesianWorkspace(
            args.left_workspace_min_m, args.left_workspace_max_m,
        ),
        right_workspace=CartesianWorkspace(
            args.right_workspace_min_m, args.right_workspace_max_m,
        ),
    )
    request.write_new(args.output)
    print(f"request={args.output}; sha256={request.sha256}; publishes_commands=false")


if __name__ == "__main__":
    main()
