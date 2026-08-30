import hashlib
import json
import tempfile
import unittest
import contextlib
import io
from datetime import datetime, timedelta, timezone
from pathlib import Path

from robot_dev_harness.run_artifacts import RunArtifacts
from robot_dev_harness.evidence import (
    format_visual_review, load_frame_timestamps, nearest_frame,
)
from robot_dev_harness.session import EvidenceSession
from robot_dev_harness.commands import EvidenceBackedCommandTransport
from record_robot_dev_run import parse_args
from g1_recording_announcer import (
    START_PHRASE, STOP_PHRASE, UnitreeRecordingAnnouncer,
)


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

    def test_visual_review_ties_each_finding_to_exact_frames(self):
        markdown = format_visual_review([{
            "finding": "The outbound endpoint was clear of obstacles.",
            "frames": [
                {"event": "outbound start", "frame_index": 82,
                 "frame_timestamp_utc": "2026-08-30T04:17:45.555105Z"},
                {"event": "outbound finish", "frame_index": 322,
                 "frame_timestamp_utc": "2026-08-30T04:17:53.560065Z"},
            ],
        }])
        self.assertIn("frame `82` at `2026-08-30T04:17:45.555105Z`", markdown)
        self.assertIn("frame `322` at `2026-08-30T04:17:53.560065Z`", markdown)
        self.assertIn("clear of obstacles", markdown)

    def test_visual_review_rejects_an_untraceable_finding(self):
        with self.assertRaisesRegex(ValueError, "no frame references"):
            format_visual_review([{"finding": "Motion looked smooth.", "frames": []}])


class FakeSource:
    def __init__(self):
        self.started = False
        self.closed = False

    def start(self):
        self.started = True

    def close(self):
        self.closed = True


class FakeCamera:
    def __init__(self, fail=False):
        self.active = False
        self.error = None
        self.fail = fail

    def start(self):
        if self.fail:
            raise RuntimeError("camera failed")
        self.active = True

    def stop(self):
        self.active = False
        return {}


class FakeAnnouncer:
    def __init__(self, fail_start_announcement=False):
        self.events = []
        self.fail_start_announcement = fail_start_announcement

    def start(self):
        self.events.append("ready")

    def recording_started(self):
        self.events.append("recording_started")
        if self.fail_start_announcement:
            raise RuntimeError("announcement failed")

    def recording_stopped(self):
        self.events.append("recording_stopped")

    def close(self):
        self.events.append("closed")


class EvidenceSessionTests(unittest.TestCase):
    def test_session_requires_camera_before_command_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = RunArtifactsTests().create_run(temporary)
            source, camera = FakeSource(), FakeCamera()
            session = EvidenceSession(run, camera, [source])
            with self.assertRaisesRegex(RuntimeError, "not ready"):
                session.command("controller", {"q": [0.0]})
            session.start()
            self.assertTrue(source.started)
            self.assertTrue(session.ready)
            session.command("controller", {"q": [0.0]})
            session.finalize("complete", "test_complete", "# Verification\n")
            self.assertTrue(source.closed)
            command = json.loads(run.path("telemetry/commands.jsonl").read_text())
            self.assertEqual(command["data"]["q"], [0.0])

    def test_camera_start_failure_closes_started_sources(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = RunArtifactsTests().create_run(temporary)
            source = FakeSource()
            session = EvidenceSession(run, FakeCamera(fail=True), [source])
            with self.assertRaisesRegex(RuntimeError, "camera failed"):
                session.start()
            self.assertTrue(source.closed)
            session.finalize("incomplete", "camera_failed", "# Verification\n")

    def test_command_transport_records_before_one_physical_send(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = RunArtifactsTests().create_run(temporary)
            session = EvidenceSession(run, FakeCamera())
            observed = []

            def transport(command):
                # The command must already have a reserved sequence before the
                # separately reviewed transport is entered.
                observed.append((dict(command), run._sequences.get("commands")))
                return "sent"

            sender = EvidenceBackedCommandTransport(session, "test-controller", transport)
            with self.assertRaisesRegex(RuntimeError, "not ready"):
                sender.send({"position": 1.0})
            self.assertEqual(observed, [])
            session.start()
            self.assertEqual(sender.send({"position": 1.0}), "sent")
            self.assertEqual(observed, [({"position": 1.0}, 1)])
            session.finalize("complete", "test_complete", "# Verification\n")

    def test_command_transport_failure_is_recorded_and_not_retried(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = RunArtifactsTests().create_run(temporary)
            session = EvidenceSession(run, FakeCamera())
            calls = []

            def transport(command):
                calls.append(command)
                raise RuntimeError("transport down")

            session.start()
            sender = EvidenceBackedCommandTransport(session, "test-controller", transport)
            with self.assertRaisesRegex(RuntimeError, "transport down"):
                sender.send({"position": 1.0})
            self.assertEqual(len(calls), 1)
            session.finalize("aborted", "transport_failed", "# Verification\n")
            events = [
                json.loads(line)["data"]["event"]
                for line in run.path("telemetry/events.jsonl").read_text().splitlines()
            ]
            self.assertIn("command_transport_failed", events)

    def test_announcer_follows_video_lifecycle_once(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = RunArtifactsTests().create_run(temporary)
            announcer = FakeAnnouncer()
            session = EvidenceSession(run, FakeCamera(), announcer=announcer)
            session.start()
            session.finalize("complete", "done", "# Verification\n")
            session.finalize("complete", "done", "# Verification\n")
            self.assertEqual(
                announcer.events,
                ["ready", "recording_started", "recording_stopped", "closed"],
            )

    def test_start_announcement_failure_stops_video_and_prevents_readiness(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = RunArtifactsTests().create_run(temporary)
            camera = FakeCamera()
            announcer = FakeAnnouncer(fail_start_announcement=True)
            session = EvidenceSession(run, camera, announcer=announcer)
            with self.assertRaisesRegex(RuntimeError, "announcement failed"):
                session.start()
            self.assertFalse(camera.active)
            self.assertFalse(session.ready)
            self.assertEqual(
                announcer.events,
                ["ready", "recording_started", "recording_stopped", "closed"],
            )
            session.finalize("incomplete", "announcement_failed", "# Verification\n")


class FakeTtsClient:
    def __init__(self, result=0):
        self.result = result
        self.calls = []

    def TtsMaker(self, phrase, speaker_id):
        self.calls.append((phrase, speaker_id))
        return self.result


class UnitreeAnnouncerTests(unittest.TestCase):
    def test_exact_chinese_phrases_are_sent_once(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = RunArtifactsTests().create_run(temporary)
            client = FakeTtsClient()
            announcer = UnitreeRecordingAnnouncer(run, speaker_id=3)
            announcer.client = client
            announcer.recording_started()
            announcer.recording_started()
            announcer.recording_stopped()
            announcer.recording_stopped()
            self.assertEqual(client.calls, [(START_PHRASE, 3), (STOP_PHRASE, 3)])
            run.finalize("complete", "test_complete", "# Verification\n")

    def test_tts_failure_is_fail_closed_and_recorded(self):
        with tempfile.TemporaryDirectory() as temporary:
            run = RunArtifactsTests().create_run(temporary)
            announcer = UnitreeRecordingAnnouncer(run)
            announcer.client = FakeTtsClient(result=7)
            with self.assertRaisesRegex(RuntimeError, "return value 7"):
                announcer.recording_started()
            run.finalize("incomplete", "tts_failed", "# Verification\n")
            events = [
                json.loads(line) for line in
                run.path("telemetry/events.jsonl").read_text().splitlines()
            ]
            self.assertEqual(events[-1]["validity"], "error")
            self.assertEqual(events[-1]["data"]["phrase"], START_PHRASE)


if __name__ == "__main__":
    unittest.main()
