import hashlib
import json
import tempfile
import unittest
import contextlib
import io
from datetime import datetime, timedelta, timezone
from pathlib import Path

from robot_dev_harness.run_artifacts import RunArtifacts
from robot_dev_harness.evidence import load_frame_timestamps, nearest_frame
from record_robot_dev_run import parse_args


class FakeClock:
    def __init__(self):
        self.utc = datetime(2026, 8, 30, 4, 5, 6, 123456, tzinfo=timezone.utc)
        self.ns = 10_000_000_000

    def utc_now(self):
        value = self.utc
        self.utc += timedelta(milliseconds=10)
        return value

    def monotonic_ns(self):
        value = self.ns
        self.ns += 10_000_000
        return value


class RunArtifactsTests(unittest.TestCase):
    def create_run(self, root):
        clock = FakeClock()
        return RunArtifacts.create(
            root=Path(root), slug="read-only-check", project="generic-robot",
            purpose="test", operator_safety_confirmation={"area_clear": True},
            worktree=Path(__file__).parents[1], utc_now=clock.utc_now,
            monotonic_ns=clock.monotonic_ns,
        )

    def test_run_is_self_contained_timestamped_and_checksummed(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = self.create_run(temporary)
            self.assertEqual(
                run.directory.name,
                "20260830T040506.123456Z_read-only-check",
            )
            self.assertTrue(run.record("imu", "robot.imu", {"x": 1.0}))
            self.assertTrue(run.record("imu", "robot.imu", {"x": 2.0}))
            run.finalize("complete", "test_complete", "# Verification\n\nPass.\n")

            manifest = json.loads(run.path("manifest.json").read_text())
            self.assertEqual(manifest["status"], "complete")
            self.assertEqual(manifest["streams"]["imu"]["record_count"], 2)
            rows = [json.loads(row) for row in run.path("telemetry/imu.jsonl").read_text().splitlines()]
            self.assertEqual([row["sequence"] for row in rows], [0, 1])
            self.assertTrue(all(row["timestamp_utc"].endswith("Z") for row in rows))
            checksum_rows = run.path("checksums.sha256").read_text().splitlines()
            telemetry_row = next(row for row in checksum_rows if row.endswith("telemetry/imu.jsonl"))
            expected = hashlib.sha256(run.path("telemetry/imu.jsonl").read_bytes()).hexdigest()
            self.assertEqual(telemetry_row.split()[0], expected)

    def test_existing_run_directory_is_never_reused(self):
        with tempfile.TemporaryDirectory() as temporary:
            first = self.create_run(temporary)
            with self.assertRaises(FileExistsError):
                self.create_run(temporary)
            first.finalize("aborted", "test_cleanup", "# Verification\n")

    def test_artifact_path_cannot_escape_run(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = self.create_run(temporary)
            with self.assertRaisesRegex(ValueError, "inside"):
                run.path("../outside")
            run.finalize("aborted", "test_cleanup", "# Verification\n")

    def test_non_serializable_record_fails_before_enqueue(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = self.create_run(temporary)
            with self.assertRaises(TypeError):
                run.record("imu", "robot.imu", {"bad": object()})
            self.assertTrue(run.record("imu", "robot.imu", {"good": True}))
            run.finalize("aborted", "test_cleanup", "# Verification\n")
            row = json.loads(run.path("telemetry/imu.jsonl").read_text())
            self.assertEqual(row["sequence"], 0)


class ReadOnlyCliTests(unittest.TestCase):
    def test_area_confirmation_is_required(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args(["--slug", "test", "--purpose", "test"])

    def test_capture_is_bounded_and_defaults_to_video6(self):
        args = parse_args([
            "--slug", "test", "--purpose", "test", "--confirm-area-clear",
        ])
        self.assertEqual(args.camera_device, "/dev/video6")
        self.assertEqual(args.duration_seconds, 5.0)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            parse_args([
                "--slug", "test", "--purpose", "test", "--confirm-area-clear",
                "--duration-seconds", "601",
            ])


class EvidenceTests(unittest.TestCase):
    def test_nearest_frame_uses_shared_monotonic_clock(self):
        rows = [
            {"frame_index": 0, "monotonic_ns": 100},
            {"frame_index": 1, "monotonic_ns": 200},
            {"frame_index": 2, "monotonic_ns": 300},
        ]
        self.assertEqual(nearest_frame(rows, 240)["frame_index"], 1)
        self.assertEqual(nearest_frame(rows, 280)["frame_index"], 2)

    def test_frame_index_loader_rejects_noncontiguous_rows(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "frames.jsonl"
            path.write_text(
                json.dumps({"frame_index": 0, "monotonic_ns": 100}) + "\n" +
                json.dumps({"frame_index": 2, "monotonic_ns": 200}) + "\n"
            )
            with self.assertRaisesRegex(ValueError, "contiguous"):
                load_frame_timestamps(path)


if __name__ == "__main__":
    unittest.main()
