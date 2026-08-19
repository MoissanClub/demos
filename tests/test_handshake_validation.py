import json
import tempfile
import unittest
from pathlib import Path

from handshake.validation import StreamStats, collection_summary, discover_trajectories, validate_trajectory


TRAJECTORY_ID = "11111111-2222-3333-4444-555555555555"


def record(timestamp, stream, data):
    return {"timestamp_monotonic_ns": timestamp, "stream": stream, "data": data}


def write_trajectory(directory, rows, suffix=".jsonl"):
    path = Path(directory) / f"trajectory_20260816T014600.000000Z_{TRAJECTORY_ID}{suffix}"
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
    return path


def valid_rows(result="success"):
    return [
        record(10, "trajectory.metadata", {"trajectory_id": TRAJECTORY_ID}),
        {
            **record(11, "brainco.touch", [{"normal_force1": 1}]),
            "controller_state_before": "open_wait",
        },
        record(12, "controller.decision", {"state": "closing", "event": "contact_started"}),
        record(13, "controller.decision", {"state": "hold", "event": "max_close_reached"}),
        record(14, "controller.decision", {"state": "releasing", "event": "hold_timeout"}),
        record(15, "controller.decision", {"state": "open_wait", "event": "hand_open_confirmed"}),
        record(
            16,
            "trajectory.summary",
            {
                "trajectory_id": TRAJECTORY_ID,
                "result": result,
                "reason": "hand_open_confirmed",
                "dropped_samples_total": 0,
            },
        ),
    ]


class TrajectoryValidationTests(unittest.TestCase):
    def test_valid_success_reports_stream_metrics(self):
        with tempfile.TemporaryDirectory() as directory:
            result = validate_trajectory(write_trajectory(directory, valid_rows()))
        self.assertTrue(result.valid)
        self.assertEqual(result.classification, "success")
        self.assertEqual(result.controller_states, ["closing", "hold", "releasing", "open_wait"])
        self.assertEqual(
            result.state_cycle, ["open_wait", "closing", "hold", "releasing", "open_wait"]
        )
        self.assertTrue(result.full_state_transition)
        self.assertFalse(result.vision_signal_present)
        self.assertEqual(result.streams["controller.decision"]["count"], 4)
        self.assertEqual(result.dropped_samples_total, 0)

    def test_missing_summary_is_incomplete(self):
        with tempfile.TemporaryDirectory() as directory:
            result = validate_trajectory(write_trajectory(directory, valid_rows()[:-1], ".jsonl.tmp"))
        self.assertFalse(result.valid)
        self.assertEqual(result.classification, "incomplete")
        self.assertIn("missing trajectory.summary", result.errors)

    def test_success_requires_safe_state_progression(self):
        rows = valid_rows()
        rows = [
            row
            for row in rows
            if not (isinstance(row.get("data"), dict) and row["data"].get("state") == "releasing")
        ]
        with tempfile.TemporaryDirectory() as directory:
            result = validate_trajectory(write_trajectory(directory, rows))
        self.assertEqual(result.classification, "rejected")
        self.assertTrue(any("missing states: releasing" in error for error in result.errors))

    def test_uuid_mismatch_is_rejected(self):
        rows = valid_rows()
        rows[-1]["data"]["trajectory_id"] = "aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee"
        with tempfile.TemporaryDirectory() as directory:
            result = validate_trajectory(write_trajectory(directory, rows))
        self.assertEqual(result.classification, "rejected")
        self.assertTrue(any("UUIDs do not match" in error for error in result.errors))

    def test_non_monotonic_samples_are_reported_per_stream(self):
        rows = valid_rows()
        rows[4]["timestamp_monotonic_ns"] = 11
        with tempfile.TemporaryDirectory() as directory:
            result = validate_trajectory(write_trajectory(directory, rows))
        self.assertEqual(result.streams["controller.decision"]["non_monotonic_samples"], 1)

    def test_frequency_and_tolerance_scan_report_missing_sample(self):
        stats = StreamStats()
        for timestamp in (0, 100_000_000, 300_000_000, 400_000_000):
            stats.observe(timestamp)
        report = stats.to_dict()
        self.assertAlmostEqual(report["sample_frequency_hz"], 10.0)
        self.assertEqual(report["expected_sample_count"], 5)
        self.assertEqual(report["missing_sample_count"], 1)
        self.assertTrue(report["has_missing_samples"])
        self.assertAlmostEqual(report["coverage_tolerance_ms"], 10.0)

    def test_vision_signal_is_reported(self):
        rows = valid_rows()
        rows[2]["data"]["vision_state"] = "hand_present"
        rows[2]["data"]["vision_score"] = 0.25
        with tempfile.TemporaryDirectory() as directory:
            result = validate_trajectory(write_trajectory(directory, rows))
        self.assertTrue(result.vision_signal_present)
        self.assertEqual(result.vision_signal_sample_count, 1)

    def test_success_without_initial_open_wait_is_rejected(self):
        rows = valid_rows()
        del rows[1]["controller_state_before"]
        with tempfile.TemporaryDirectory() as directory:
            result = validate_trajectory(write_trajectory(directory, rows))
        self.assertFalse(result.full_state_transition)
        self.assertEqual(result.classification, "rejected")

    def test_malformed_json_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_trajectory(directory, valid_rows())
            with path.open("a", encoding="utf-8") as output:
                output.write("{broken\n")
            result = validate_trajectory(path)
        self.assertEqual(result.classification, "rejected")
        self.assertTrue(any("malformed JSON" in error for error in result.errors))

    def test_discovery_and_collection_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            path = write_trajectory(directory, valid_rows())
            paths = discover_trajectories([Path(directory)])
            results = [validate_trajectory(item) for item in paths]
        self.assertEqual(paths, [path])
        self.assertEqual(collection_summary(results)["classifications"], {"success": 1})
        self.assertEqual(collection_summary(results)["full_state_transition_count"], 1)


if __name__ == "__main__":
    unittest.main()
