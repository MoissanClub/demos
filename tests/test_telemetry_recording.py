import json
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

from handshake.recording import TelemetryRecorder, TrajectoryRecorder, upload_trajectories


class TelemetryRecorderTests(unittest.TestCase):
    def test_background_writer_drains_before_close(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "recording.jsonl"
            recorder = TelemetryRecorder(path, queue_size=4)
            recorder.start()
            self.assertTrue(recorder.record("brainco.touch", {"value": 12}, timestamp_ns=99))
            recorder.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(rows[0]["timestamp_monotonic_ns"], 99)
        self.assertEqual(rows[0]["data"], {"value": 12})
        self.assertEqual(rows[-1]["stream"], "recording.summary")

    def test_full_queue_drops_without_blocking(self):
        recorder = TelemetryRecorder(Path("unused"), queue_size=1)
        self.assertTrue(recorder.record("one", 1))
        self.assertFalse(recorder.record("two", 2))
        self.assertEqual(recorder.dropped_samples, 1)

    def test_each_trajectory_is_a_separate_finalized_file(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TrajectoryRecorder(Path(directory), queue_size=16)
            recorder.start()
            first = recorder.start_trajectory({"number": 1}, timestamp_ns=10)
            recorder.record("brainco.touch", {"value": 1}, timestamp_ns=11)
            recorder.finish_trajectory("success", "hand_open", timestamp_ns=12)
            second = recorder.start_trajectory({"number": 2}, timestamp_ns=20)
            recorder.record("brainco.touch", {"value": 2}, timestamp_ns=21)
            recorder.finish_trajectory("success", "hand_open", timestamp_ns=22)
            recorder.close()

            self.assertEqual(recorder.finalized_paths, [first, second])
            self.assertNotEqual(first, second)
            for path, value in ((first, 1), (second, 2)):
                rows = [json.loads(line) for line in path.read_text().splitlines()]
                self.assertEqual(rows[0]["stream"], "trajectory.metadata")
                self.assertEqual(rows[1]["data"], {"value": value})
                self.assertEqual(rows[-1]["stream"], "trajectory.summary")
                self.assertFalse(Path(str(path) + ".tmp").exists())

    def test_exit_aborts_an_active_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder = TrajectoryRecorder(Path(directory), queue_size=8)
            recorder.start()
            path = recorder.start_trajectory({})
            recorder.close()
            rows = [json.loads(line) for line in path.read_text().splitlines()]
        self.assertEqual(rows[-1]["data"]["result"], "aborted")
        self.assertEqual(rows[-1]["data"]["reason"], "controller_exit")

    def test_upload_uses_one_remote_file_per_trajectory(self):
        api = MagicMock()
        paths = [Path("one.jsonl"), Path("two.jsonl")]
        fake_hub = types.SimpleNamespace(HfApi=MagicMock(return_value=api))
        with patch.dict(sys.modules, {"huggingface_hub": fake_hub}):
            uploaded = upload_trajectories(paths, "owner/dataset", "run-1")
        api.create_repo.assert_called_once_with(
            repo_id="owner/dataset", repo_type="dataset", private=True, exist_ok=True
        )
        self.assertEqual(
            uploaded,
            ["trajectories/run-1/one.jsonl", "trajectories/run-1/two.jsonl"],
        )
        self.assertEqual(api.upload_file.call_count, 2)


if __name__ == "__main__":
    unittest.main()
