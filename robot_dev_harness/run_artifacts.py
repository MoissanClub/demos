"""Self-contained, timestamped run artifacts for physical robot development."""
from __future__ import annotations

import hashlib
import json
import os
import queue
import re
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Dict, Mapping, Optional


SCHEMA_VERSION = "1.0"
_SAFE_NAME = re.compile(r"^[a-z0-9][a-z0-9_.-]*$")


def utc_text(value: datetime) -> str:
    return value.astimezone(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%fZ")


def utc_path_text(value: datetime) -> str:
    return value.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")


def safe_name(value: str, label: str) -> str:
    normalized = value.strip().lower().replace(" ", "-")
    if not _SAFE_NAME.fullmatch(normalized) or ".." in normalized:
        raise ValueError(f"{label} must use lowercase letters, digits, '.', '_' or '-'")
    return normalized


def git_state(worktree: Path) -> Dict[str, Any]:
    def run(*args: str) -> str:
        return subprocess.run(
            ("git", "-C", str(worktree), *args), check=True,
            text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
        ).stdout.strip()

    try:
        return {
            "commit": run("rev-parse", "HEAD"),
            "branch": run("branch", "--show-current"),
            "dirty": bool(run("status", "--porcelain")),
        }
    except (OSError, subprocess.CalledProcessError) as exc:
        return {"unavailable": f"{type(exc).__name__}: {exc}"}


class RunArtifacts:
    """Write all evidence for one run through one background I/O queue.

    This class records evidence only. It intentionally knows nothing about
    robot commands, DDS, specific telemetry messages, or camera drivers.
    """

    def __init__(
        self,
        directory: Path,
        run_id: str,
        started_utc: datetime,
        started_monotonic_ns: int,
        queue_size: int = 16384,
        utc_now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> None:
        self.directory = directory
        self.run_id = run_id
        self.started_utc = started_utc
        self.started_monotonic_ns = started_monotonic_ns
        self.utc_now = utc_now
        self.monotonic_ns = monotonic_ns
        self.queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self.dropped_records = 0
        self.write_error: Optional[str] = None
        self._sequences: Dict[str, int] = {}
        self._sequence_lock = threading.Lock()
        self._files: Dict[str, Any] = {}
        self._thread = threading.Thread(
            target=self._write_loop, name=f"run_writer_{run_id}", daemon=True,
        )
        self._closed = False
        self._manifest: Dict[str, Any] = {}

    @classmethod
    def create(
        cls,
        root: Path,
        slug: str,
        project: str,
        purpose: str,
        operator_safety_confirmation: Mapping[str, Any],
        worktree: Path = Path("."),
        metadata: Optional[Mapping[str, Any]] = None,
        queue_size: int = 16384,
        utc_now: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
    ) -> "RunArtifacts":
        slug = safe_name(slug, "run slug")
        project = safe_name(project, "project")
        started_utc = utc_now()
        started_ns = monotonic_ns()
        run_id = f"{utc_path_text(started_utc)}_{slug}"
        directory = root / run_id
        directory.mkdir(parents=True, exist_ok=False)
        for child in ("telemetry", "video", "evidence"):
            (directory / child).mkdir()
        instance = cls(
            directory, run_id, started_utc, started_ns, queue_size,
            utc_now=utc_now, monotonic_ns=monotonic_ns,
        )
        instance._manifest = {
            "schema_version": SCHEMA_VERSION,
            "run_id": run_id,
            "project": project,
            "purpose": purpose,
            "status": "recording",
            "started_at_utc": utc_text(started_utc),
            "started_monotonic_ns": started_ns,
            "operator_safety_confirmation": dict(operator_safety_confirmation),
            "git": git_state(worktree.resolve()),
            "streams": {},
            "devices": {},
            "metadata": dict(metadata or {}),
        }
        instance._write_json_atomic("manifest.json", instance._manifest)
        instance._thread.start()
        return instance

    def record(
        self,
        stream: str,
        source: str,
        data: Any,
        timestamp_utc: Optional[datetime] = None,
        monotonic_ns: Optional[int] = None,
        device_timestamp: Optional[Mapping[str, Any]] = None,
        validity: str = "valid",
    ) -> bool:
        if self._closed or self.write_error:
            return False
        stream = safe_name(stream, "stream")
        source = safe_name(source, "source")
        with self._sequence_lock:
            sequence = self._sequences.get(stream, 0)
            record = {
                "schema_version": SCHEMA_VERSION,
                "run_id": self.run_id,
                "timestamp_utc": utc_text(timestamp_utc or self.utc_now()),
                "monotonic_ns": monotonic_ns if monotonic_ns is not None else self.monotonic_ns(),
                "source": source,
                "sequence": sequence,
                "validity": validity,
                "data": data,
            }
            if device_timestamp is not None:
                record["device_timestamp"] = dict(device_timestamp)
            payload = json.dumps(record, separators=(",", ":"), ensure_ascii=False)
            try:
                self.queue.put_nowait((stream, payload))
            except queue.Full:
                self.dropped_records += 1
                return False
            self._sequences[stream] = sequence + 1
            return True

    def register_device(self, name: str, details: Mapping[str, Any]) -> None:
        if self._closed:
            raise RuntimeError("run is already finalized")
        self._manifest["devices"][safe_name(name, "device name")] = dict(details)
        self._write_json_atomic("manifest.json", self._manifest)

    def path(self, relative: str) -> Path:
        candidate = (self.directory / relative).resolve()
        directory = self.directory.resolve()
        if directory not in candidate.parents:
            raise ValueError("artifact path must remain inside the run directory")
        return candidate

    def finalize(
        self,
        status: str,
        reason: str,
        verification_markdown: str,
        metadata: Optional[Mapping[str, Any]] = None,
    ) -> None:
        if self._closed:
            return
        if status not in {"complete", "aborted", "incomplete"}:
            raise ValueError("status must be complete, aborted, or incomplete")
        self._closed = True
        if self._thread.is_alive():
            try:
                self.queue.put((None, None), timeout=5.0)
            except queue.Full:
                self.write_error = self.write_error or "writer queue did not accept stop marker"
        self._thread.join(timeout=10.0)
        if self._thread.is_alive():
            self.write_error = self.write_error or "writer did not stop within 10 seconds"
        ended_utc = self.utc_now()
        ended_ns = self.monotonic_ns()
        effective_status = "incomplete" if self.write_error or self.dropped_records else status
        self._manifest.update({
            "status": effective_status,
            "reason": reason,
            "ended_at_utc": utc_text(ended_utc),
            "ended_monotonic_ns": ended_ns,
            "duration_seconds": (ended_ns - self.started_monotonic_ns) / 1e9,
            "dropped_records": self.dropped_records,
            "write_error": self.write_error,
        })
        if metadata:
            self._manifest["result_metadata"] = dict(metadata)
        self._manifest["streams"] = {
            stream: {"path": f"telemetry/{stream}.jsonl", "record_count": count}
            for stream, count in sorted(self._sequences.items())
        }
        self.path("verification.md").write_text(verification_markdown, encoding="utf-8")
        self._write_json_atomic("manifest.json", self._manifest)
        self._write_checksums()

    def _write_loop(self) -> None:
        try:
            while True:
                stream, payload = self.queue.get()
                if stream is None:
                    break
                file = self._files.get(stream)
                if file is None:
                    file = self.path(f"telemetry/{stream}.jsonl").open("x", encoding="utf-8")
                    self._files[stream] = file
                file.write(payload + "\n")
            for file in self._files.values():
                file.flush()
                os.fsync(file.fileno())
        except Exception as exc:
            self.write_error = f"{type(exc).__name__}: {exc}"
        finally:
            for file in self._files.values():
                file.close()

    def _write_json_atomic(self, relative: str, value: Mapping[str, Any]) -> None:
        target = self.path(relative)
        temporary = target.with_suffix(target.suffix + ".tmp")
        temporary.write_text(
            json.dumps(value, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        temporary.replace(target)

    def _write_checksums(self) -> None:
        rows = []
        for path in sorted(self.directory.rglob("*")):
            if not path.is_file() or path.name == "checksums.sha256":
                continue
            digest_state = hashlib.sha256()
            with path.open("rb") as source:
                for chunk in iter(lambda: source.read(1024 * 1024), b""):
                    digest_state.update(chunk)
            digest = digest_state.hexdigest()
            rows.append(f"{digest}  {path.relative_to(self.directory)}")
        self.path("checksums.sha256").write_text("\n".join(rows) + "\n", encoding="ascii")
