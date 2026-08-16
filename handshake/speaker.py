"""Configuration and nonblocking Unitree speaker support for the demo."""

import json
import sys
import threading
from typing import Any, Optional, Tuple

from .unitree_cleanup import close_rpc_client


def load_demo_config(path: str) -> Tuple[str, str, int]:
    """Load and validate settings that are intended to change without code edits."""
    try:
        with open(path, "r", encoding="utf-8") as config_file:
            config = json.load(config_file)
    except (OSError, json.JSONDecodeError) as exc:
        raise ValueError(f"cannot load config {path!r}: {exc}") from exc

    phrase = config.get("greeting_phrase")
    invitation_phrase = config.get("invitation_phrase", "你好")
    speaker_id = config.get("speaker_id", 0)
    if not isinstance(phrase, str) or not phrase.strip():
        raise ValueError("config greeting_phrase must be a nonempty string")
    if not isinstance(invitation_phrase, str) or not invitation_phrase.strip():
        raise ValueError("config invitation_phrase must be a nonempty string")
    if not isinstance(speaker_id, int) or isinstance(speaker_id, bool) or speaker_id < 0:
        raise ValueError("config speaker_id must be a nonnegative integer")
    return phrase.strip(), invitation_phrase.strip(), speaker_id


class SpeakerRunner:
    """Nonblocking, best-effort wrapper around the Unitree G1 TTS service."""

    def __init__(
        self,
        phrase: str,
        speaker_id: int,
        dry_run: bool,
        network_interface: Optional[str],
    ) -> None:
        self.phrase = phrase
        self.speaker_id = speaker_id
        self.dry_run = dry_run
        self.network_interface = network_interface
        self.client: Any = None
        self.enabled = True
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None

    def init(self, channel_initialized: bool) -> None:
        if self.dry_run:
            print(f"speaker: dry-run; would say {self.phrase!r} on hold.")
            return

        try:
            if not channel_initialized:
                from unitree_sdk2py.core.channel import ChannelFactoryInitialize

                if self.network_interface:
                    ChannelFactoryInitialize(0, self.network_interface)
                else:
                    ChannelFactoryInitialize(0)

            from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient

            self.client = AudioClient()
            self.client.SetTimeout(10.0)
            self.client.Init()
            print(f"speaker: enabled; greeting={self.phrase!r}, speaker_id={self.speaker_id}")
        except Exception as exc:
            self.enabled = False
            print(f"WARNING: speaker unavailable; greeting disabled: {exc}", file=sys.stderr)

    def greet(self) -> None:
        self.say(self.phrase)

    def say(self, phrase: str) -> None:
        """Speak one phrase without blocking the control loop."""
        if not self.enabled:
            return
        if self.dry_run:
            print(f"speaker: would say {phrase!r}")
            return

        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._thread = threading.Thread(
                target=self._speak, args=(phrase,), name="speaker_tts", daemon=True
            )
            self._thread.start()

    def _speak(self, phrase: str) -> None:
        try:
            ret = self.client.TtsMaker(phrase, self.speaker_id)
            print(f"speaker: said {phrase!r}, ret={ret}")
        except Exception as exc:
            print(f"WARNING: greeting failed: {exc}", file=sys.stderr)

    def close(self) -> None:
        if self._thread is not None:
            self._thread.join(timeout=10.0)
        close_rpc_client(self.client)
        self.client = None
