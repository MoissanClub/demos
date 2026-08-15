"""Non-blocking telemetry recording support for the handshake controller."""

import json
import os
import queue
import sys
import threading
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from telemetry_probe import json_value, unitree_lowstate_record


class TelemetryRecorder:
    """Serialize samples before enqueueing; perform all file I/O off-loop."""

    def __init__(self, path: Path, queue_size: int = 4096) -> None:
        self.path = path
        self.queue = queue.Queue(maxsize=queue_size)
        self.dropped_samples = 0
        self.write_error: Optional[str] = None
        self._thread: Optional[threading.Thread] = None
        self._file: Any = None
        self._closed = False

    def start(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("w", encoding="utf-8")
        self._thread = threading.Thread(target=self._write_loop, name="telemetry_writer", daemon=True)
        self._thread.start()

    def record(self, stream: str, data: Any, timestamp_ns: Optional[int] = None, **metadata: Any) -> bool:
        if self._closed or self.write_error:
            return False
        record = {
            "timestamp_monotonic_ns": timestamp_ns if timestamp_ns is not None else time.monotonic_ns(),
            "stream": stream,
            "data": json_value(data),
            **json_value(metadata),
        }
        try:
            self.queue.put_nowait(record)
            return True
        except queue.Full:
            self.dropped_samples += 1
            return False

    def close(self) -> None:
        if self._closed:
            return
        self.record(
            "recording.summary",
            {"dropped_samples": self.dropped_samples, "write_error": self.write_error},
            finished_at_utc=datetime.now(timezone.utc).isoformat(),
        )
        self._closed = True
        self.queue.put(None)
        if self._thread is not None:
            self._thread.join(timeout=5.0)
        if self._file is not None:
            self._file.close()

    def _write_loop(self) -> None:
        try:
            last_flush = time.monotonic()
            while True:
                record = self.queue.get()
                if record is None:
                    self._file.flush()
                    return
                self._file.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
                now = time.monotonic()
                if now - last_flush >= 0.25:
                    self._file.flush()
                    last_flush = now
        except Exception as exc:
            self.write_error = f"{type(exc).__name__}: {exc}"


class TrajectoryRecorder:
    """Write independently finalized trajectory files on one background thread."""

    def __init__(self, directory: Path, queue_size: int = 4096) -> None:
        self.directory = directory
        self.queue = queue.Queue(maxsize=queue_size)
        self.dropped_samples = 0
        self.write_error: Optional[str] = None
        self.finalized_paths: List[Path] = []
        self._lock = threading.Lock()
        self._active_id: Optional[str] = None
        self._active_path: Optional[Path] = None
        self._thread: Optional[threading.Thread] = None
        self._closed = False

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active_id is not None

    def start(self) -> None:
        self.directory.mkdir(parents=True, exist_ok=True)
        self._thread = threading.Thread(target=self._write_loop, name="trajectory_writer", daemon=True)
        self._thread.start()

    def start_trajectory(self, metadata: Dict[str, Any], timestamp_ns: Optional[int] = None) -> Path:
        trajectory_id = str(uuid.uuid4())
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
        path = self.directory / f"trajectory_{stamp}_{trajectory_id}.jsonl"
        with self._lock:
            if self._active_id is not None:
                raise RuntimeError("a trajectory is already active")
            self._active_id = trajectory_id
            self._active_path = path
            self._put(("open", trajectory_id, path))
        self.record(
            "trajectory.metadata",
            {"trajectory_id": trajectory_id, "started_at_utc": datetime.now(timezone.utc).isoformat(), **metadata},
            timestamp_ns=timestamp_ns,
        )
        return path

    def record(self, stream: str, data: Any, timestamp_ns: Optional[int] = None, **metadata: Any) -> bool:
        with self._lock:
            trajectory_id = self._active_id
            if trajectory_id is None or self._closed or self.write_error:
                return False
            record = {
                "timestamp_monotonic_ns": timestamp_ns if timestamp_ns is not None else time.monotonic_ns(),
                "stream": stream,
                "data": json_value(data),
                **json_value(metadata),
            }
            return self._put(("record", trajectory_id, record))

    def finish_trajectory(self, result: str, reason: str, timestamp_ns: Optional[int] = None) -> Optional[Path]:
        with self._lock:
            trajectory_id = self._active_id
            path = self._active_path
        if trajectory_id is None or path is None:
            return None
        self.record(
            "trajectory.summary",
            {
                "trajectory_id": trajectory_id,
                "result": result,
                "reason": reason,
                "dropped_samples_total": self.dropped_samples,
                "finished_at_utc": datetime.now(timezone.utc).isoformat(),
            },
            timestamp_ns=timestamp_ns,
        )
        with self._lock:
            self._put(("close", trajectory_id, path))
            self._active_id = None
            self._active_path = None
        return path

    def close(self) -> None:
        if self.active:
            self.finish_trajectory("aborted", "controller_exit")
        self._closed = True
        self.queue.put(("stop", None, None))
        if self._thread is not None:
            self._thread.join(timeout=10.0)

    def _put(self, item: Any) -> bool:
        # Keep two queue slots reserved for close/stop lifecycle commands.
        if item[0] == "record" and self.queue.qsize() >= max(0, self.queue.maxsize - 2):
            self.dropped_samples += 1
            return False
        try:
            self.queue.put_nowait(item)
            return True
        except queue.Full:
            self.dropped_samples += 1
            return False

    def _write_loop(self) -> None:
        current_id = None
        current_path = None
        current_file = None
        try:
            while True:
                action, trajectory_id, payload = self.queue.get()
                if action == "stop":
                    if current_file is not None:
                        current_file.close()
                    return
                if action == "open":
                    current_id = trajectory_id
                    current_path = payload
                    current_file = Path(str(current_path) + ".tmp").open("w", encoding="utf-8")
                elif action == "record" and trajectory_id == current_id and current_file is not None:
                    current_file.write(json.dumps(payload, separators=(",", ":"), ensure_ascii=False) + "\n")
                elif action == "close" and trajectory_id == current_id and current_file is not None:
                    current_file.flush()
                    current_file.close()
                    temporary = Path(str(current_path) + ".tmp")
                    temporary.replace(current_path)
                    self.finalized_paths.append(current_path)
                    current_id = current_path = current_file = None
        except Exception as exc:
            self.write_error = f"{type(exc).__name__}: {exc}"
            if current_file is not None:
                current_file.close()


def upload_trajectories(paths: List[Path], repo_id: str, run_id: str) -> List[str]:
    """Upload finalized files after robot cleanup; local files remain authoritative."""
    if not paths:
        return []
    from huggingface_hub import HfApi

    api = HfApi()
    api.create_repo(repo_id=repo_id, repo_type="dataset", private=True, exist_ok=True)
    uploaded = []
    for path in paths:
        remote_path = f"trajectories/{run_id}/{path.name}"
        api.upload_file(
            path_or_fileobj=str(path),
            path_in_repo=remote_path,
            repo_id=repo_id,
            repo_type="dataset",
            commit_message=f"Upload handshake trajectory {path.stem}",
        )
        uploaded.append(remote_path)
    return uploaded


class UnitreeStateRecorder:
    """Read-only G1 LowState subscriber that forwards samples to a recorder."""

    def __init__(
        self,
        recorder: Any,
        network_interface: Optional[str],
        topic: str,
        active_event: Optional[threading.Event] = None,
    ) -> None:
        self.recorder = recorder
        self.network_interface = network_interface
        self.topic = topic
        self.active_event = active_event
        self.subscriber: Any = None

    def start(self, channel_initialized: bool) -> None:
        sdk_path = os.environ.get("UNITREE_SDK2_PYTHON", os.path.expanduser("~/unitree_sdk2_python"))
        if os.path.isdir(sdk_path) and sdk_path not in sys.path:
            sys.path.insert(0, sdk_path)
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        if not channel_initialized:
            if self.network_interface:
                ChannelFactoryInitialize(0, self.network_interface)
            else:
                ChannelFactoryInitialize(0)
        self.subscriber = ChannelSubscriber(self.topic, LowState_)
        self.subscriber.Init(self._receive, 10)

    def _receive(self, message: Any) -> None:
        if self.active_event is not None and not self.active_event.is_set():
            return
        timestamp_ns = time.monotonic_ns()
        self.recorder.record(
            "unitree.lowstate",
            unitree_lowstate_record(message),
            timestamp_ns=timestamp_ns,
            topic=self.topic,
        )
