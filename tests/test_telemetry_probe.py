import argparse
import json
import tempfile
import unittest
from pathlib import Path

from telemetry_probe import JsonlWriter, StreamStats, json_value, parse_args, unitree_lowstate_record


class Object:
    pass


class TelemetryProbeTests(unittest.TestCase):
    def test_json_value_preserves_public_sdk_fields(self):
        value = Object()
        value.positions = (1, 2, 3)
        value.status = 7
        self.assertEqual(json_value(value), {"positions": [1, 2, 3], "status": 7})

    def test_json_value_stops_recursive_sdk_fields(self):
        value = Object()
        value.name = "device"
        value.owner = value
        self.assertEqual(
            json_value(value),
            {"name": "device", "owner": "<recursive-reference>"},
        )

    def test_json_value_compacts_rust_enum(self):
        value = Object()
        value.int_value = 2
        value.Idle = value
        value.Running = Object()
        self.assertEqual(json_value(value), {"name": str(value), "int_value": 2})

    def test_writer_adds_monotonic_timestamp(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "probe.jsonl"
            writer = JsonlWriter(path)
            writer.write("test", {"raw": 3})
            writer.close()
            record = json.loads(path.read_text())
        self.assertEqual(record["stream"], "test")
        self.assertEqual(record["data"], {"raw": 3})
        self.assertGreater(record["timestamp_monotonic_ns"], 0)

    def test_stats_reports_rate_latency_and_errors(self):
        stats = StreamStats()
        stats.success(1_000_000_000, 2.0)
        stats.success(2_000_000_000, 4.0)
        stats.errors = 1
        summary = stats.summary()
        self.assertEqual(summary["observed_rate_hz"], 1.0)
        self.assertEqual(summary["latency_ms"]["median"], 3.0)
        self.assertEqual(summary["errors"], 1)

    def test_defaults_to_right_hand(self):
        args = parse_args(["--brainco"])
        self.assertTrue(args.right)
        self.assertEqual(args.slave_id, 0x7F)

    def test_unitree_record_labels_joint_and_preserves_raw_fields(self):
        motor = Object()
        motor.mode, motor.q, motor.dq, motor.ddq, motor.tau_est = 1, 2.0, 3.0, 4.0, 5.0
        motor.temperature, motor.vol, motor.motorstate = [30, 31], 48.0, 9
        msg = Object()
        msg.motor_state = [motor]
        msg.version, msg.mode_pr, msg.mode_machine, msg.tick = [1, 2], 0, 3, 4
        msg.imu_state = {"rpy": [0.0, 0.0, 0.0]}
        record = unitree_lowstate_record(msg)
        self.assertEqual(record["motors"][0]["joint_name"], "left_hip_pitch")
        self.assertEqual(record["motors"][0]["estimated_torque_nm"], 5.0)


if __name__ == "__main__":
    unittest.main()
