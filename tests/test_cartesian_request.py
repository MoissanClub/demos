import json
import tempfile
import unittest
from pathlib import Path

from handshake.cartesian_command import CartesianPositionCommand, CartesianWorkspace
from handshake.cartesian_request import CartesianMoveRequest


class CartesianMoveRequestTests(unittest.TestCase):
    def request(self):
        return CartesianMoveRequest(
            attempt_id="right-world-target-001",
            command=CartesianPositionCommand(
                right_target_m=(0.02, -0.23, -0.10),
                duration_seconds=8.0,
                maximum_displacement_m=0.01,
                maximum_joint_offset_rad=0.05,
                maximum_joint_velocity_rad_s=0.02,
            ),
            left_workspace=CartesianWorkspace(
                (-0.01, 0.21, -0.13), (0.04, 0.26, -0.08),
            ),
            right_workspace=CartesianWorkspace(
                (-0.01, -0.26, -0.13), (0.05, -0.21, -0.08),
            ),
        )

    def test_round_trip_has_stable_canonical_hash(self):
        request = self.request()
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "request.json"
            request.write_new(path)
            loaded = CartesianMoveRequest.load(path)
        self.assertEqual(loaded, request)
        self.assertEqual(loaded.sha256, request.sha256)
        self.assertEqual(len(request.sha256), 64)

    def test_unknown_fields_fail_closed(self):
        value = self.request().as_dict()
        value["command"]["unreviewed_override"] = True
        with self.assertRaisesRegex(ValueError, "missing or unknown fields"):
            CartesianMoveRequest.from_mapping(value)

    def test_existing_request_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "request.json"
            self.request().write_new(path)
            original = json.loads(path.read_text())
            with self.assertRaises(FileExistsError):
                self.request().write_new(path)
            self.assertEqual(json.loads(path.read_text()), original)

    def test_nonfinite_limit_fails_closed(self):
        with self.assertRaisesRegex(ValueError, "must be finite"):
            CartesianPositionCommand(
                right_target_m=(0.02, -0.23, -0.10),
                duration_seconds=float("nan"),
            )

    def test_twenty_second_coordinate_trajectory_is_allowed(self):
        command = CartesianPositionCommand(
            right_target_m=(0.04, -0.20, -0.10),
            duration_seconds=20.0,
        )
        self.assertEqual(command.duration_seconds, 20.0)

    def test_coordinate_trajectory_over_thirty_seconds_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "between 1 and 30"):
            CartesianPositionCommand(
                right_target_m=(0.04, -0.20, -0.10),
                duration_seconds=30.1,
            )


if __name__ == "__main__":
    unittest.main()
