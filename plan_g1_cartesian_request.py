#!/usr/bin/env python3
"""Solve one hash-addressed Cartesian request offline; never imports Unitree DDS."""
import argparse
import json
from pathlib import Path

from handshake.cartesian_arm_ik import G1CartesianArmIK
from handshake.cartesian_command import G1CartesianCommandInterface
from handshake.cartesian_request import CartesianMoveRequest
from handshake.standalone_arm import ARM_JOINT_INDICES


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--request", type=Path, required=True)
    parser.add_argument("--expect-request-sha256", required=True)
    parser.add_argument("--initial-arm-q", nargs=14, type=float, required=True)
    parser.add_argument("--include-samples", action="store_true")
    parser.add_argument("--g1-urdf", type=Path, default=Path(
        "/home/dwei/xr_teleoperate/assets/g1/g1_body29_hand14.urdf"
    ))
    args = parser.parse_args(argv)
    request = CartesianMoveRequest.load(args.request)
    if request.sha256 != args.expect_request_sha256.lower():
        parser.error(
            f"request hash mismatch: expected {args.expect_request_sha256.lower()}, "
            f"calculated {request.sha256}"
        )
    initial = dict(zip(ARM_JOINT_INDICES, args.initial_arm_q))
    planner = G1CartesianArmIK(args.g1_urdf)
    result = G1CartesianCommandInterface(planner).plan_position(
        request.command, initial, request.left_workspace, request.right_workspace,
        minimum_peak_speed=True, max_ik_candidates=5,
    )
    if not args.include_samples:
        result = {key: value for key, value in result.items() if key != "samples"}
        if "oscillation" in result:
            result["oscillation"] = {
                key: value for key, value in result["oscillation"].items()
                if key != "samples"
            }
    print(json.dumps({
        "publishes_commands": False,
        "attempt_id": request.attempt_id,
        "request_sha256": request.sha256,
        "model": planner.feedforward.configuration(),
        "trajectory": result,
    }, indent=2))


if __name__ == "__main__":
    main()
