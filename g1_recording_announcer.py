"""Unitree G1 Chinese TTS backend for recording lifecycle announcements."""
from __future__ import annotations

from typing import Any

from handshake.unitree_cleanup import close_rpc_client
from robot_dev_harness.run_artifacts import RunArtifacts


START_PHRASE = "机器人开始移动"
STOP_PHRASE = "机器人停止移动"


class UnitreeRecordingAnnouncer:
    def __init__(self, run: RunArtifacts, speaker_id: int = 0) -> None:
        self.run = run
        self.speaker_id = speaker_id
        self.client: Any = None
        self.started_announced = False
        self.stopped_announced = False

    def start(self) -> None:
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

        self.client = AudioClient()
        self.client.SetTimeout(10.0)
        self.client.Init()
        self.run.record("events", "unitree-audio", {
            "event": "recording_announcer_ready", "speaker_id": self.speaker_id,
        })

    def recording_started(self) -> None:
        if self.started_announced:
            return
        self._say("recording_start_announcement", START_PHRASE)
        self.started_announced = True

    def recording_stopped(self) -> None:
        if self.stopped_announced:
            return
        self._say("recording_stop_announcement", STOP_PHRASE)
        self.stopped_announced = True

    def _say(self, event: str, phrase: str) -> None:
        if self.client is None:
            raise RuntimeError("recording announcer is not initialized")
        if not self.run.record("events", "unitree-audio", {
            "event": f"{event}_requested", "phrase": phrase,
        }):
            raise RuntimeError("could not record announcement request")
        result = self.client.TtsMaker(phrase, self.speaker_id)
        if result != 0:
            self.run.record("events", "unitree-audio", {
                "event": f"{event}_failed", "phrase": phrase, "return_value": result,
            }, validity="error")
            raise RuntimeError(f"Unitree TTS failed with return value {result}")
        if not self.run.record("events", "unitree-audio", {
            "event": f"{event}_accepted", "phrase": phrase, "return_value": result,
        }):
            raise RuntimeError("could not record announcement result")

    def close(self) -> None:
        close_rpc_client(self.client)
        self.client = None
