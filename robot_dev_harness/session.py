"""Lifecycle coordination for evidence-backed robot development sessions."""
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, Mapping, Optional, Protocol

from robot_dev_harness.evidence import extract_nearest_frame, load_frame_timestamps
from robot_dev_harness.run_artifacts import RunArtifacts


class TelemetrySource(Protocol):
    def start(self) -> None: ...
    def close(self) -> None: ...


class EvidenceCamera(Protocol):
    @property
    def active(self) -> bool: ...
    @property
    def error(self) -> Optional[str]: ...
    def start(self) -> None: ...
    def stop(self) -> Dict[str, Any]: ...


class EvidenceSession:
    """Coordinate recording readiness without knowing how a robot is controlled.

    The session deliberately provides no method that sends a robot command.
    Controllers record the command they are about to send with ``command()``;
    their separately reviewed actuator layer remains responsible for sending it.
    """

    def __init__(
        self,
        run: RunArtifacts,
        camera: EvidenceCamera,
        telemetry_sources: Iterable[TelemetrySource] = (),
    ) -> None:
        self.run = run
        self.camera = camera
        self.telemetry_sources = list(telemetry_sources)
        self._started_sources = []
        self._active = False
        self._finalized = False
        self.camera_summary: Optional[Dict[str, Any]] = None
        self.final_status: Optional[str] = None
        self.final_reason: Optional[str] = None

    @property
    def ready(self) -> bool:
        return self._active and self.camera.active and not self.camera.error

    def start(self) -> None:
        if self._active or self._finalized:
            raise RuntimeError("evidence session cannot be started in its current state")
        try:
            for source in self.telemetry_sources:
                source.start()
                self._started_sources.append(source)
            self.camera.start()
            if not self.camera.active or self.camera.error:
                raise RuntimeError(self.camera.error or "camera is not active after startup")
            self._active = True
            self.event("evidence_session_ready", {"commands_recorded": 0})
        except BaseException:
            self._close_sources()
            raise

    def require_ready(self) -> None:
        if not self.ready:
            raise RuntimeError(self.camera.error or "evidence session is not ready")

    def command(self, source: str, command: Mapping[str, Any]) -> None:
        """Record command intent before a separate actuator layer publishes it."""
        self.require_ready()
        if not self.run.record("commands", source, dict(command)):
            raise RuntimeError("command evidence could not be queued")

    def event(self, event: str, details: Optional[Mapping[str, Any]] = None) -> None:
        if not self.run.record(
            "events", "evidence-session", {"event": event, **dict(details or {})}
        ):
            raise RuntimeError("event evidence could not be queued")

    def finalize(
        self,
        status: str,
        reason: str,
        verification_markdown: str,
        result_metadata: Optional[Mapping[str, Any]] = None,
    ) -> None:
        if self._finalized:
            return
        effective_status, effective_reason = status, reason
        try:
            if self.camera.active:
                self.camera_summary = self.camera.stop()
            if self.camera_summary:
                self._extract_boundary_evidence(self.camera_summary)
            if self._active:
                self.event("evidence_session_stopped", {"camera": self.camera_summary})
        except BaseException as exc:
            effective_status = "incomplete"
            effective_reason += f"; evidence finalization failed: {type(exc).__name__}: {exc}"
        finally:
            self._active = False
            self._close_sources()
            metadata = dict(result_metadata or {})
            if self.camera_summary:
                metadata["camera"] = self.camera_summary
            if effective_status != status or effective_reason != reason:
                verification_markdown += (
                    "\n## Evidence finalization override\n\n"
                    f"- Effective result: **{effective_status}**\n"
                    f"- Effective reason: `{effective_reason}`\n"
                )
            self.run.finalize(
                effective_status, effective_reason, verification_markdown,
                metadata=metadata or None,
            )
            self.final_status = effective_status
            self.final_reason = effective_reason
            self._finalized = True

    def _extract_boundary_evidence(self, camera_summary: Mapping[str, Any]) -> None:
        index_path = self.run.path("video/frame_timestamps.jsonl")
        rows = load_frame_timestamps(index_path)
        evidence = []
        for event, row in (("recording-baseline", rows[0]), ("recording-final", rows[-1])):
            item = extract_nearest_frame(
                self.run.path(str(camera_summary["path"])), index_path,
                row["monotonic_ns"], event, self.run.path("evidence"),
            )
            item["evidence_path"] = str(
                Path(item["evidence_path"]).relative_to(self.run.directory.resolve())
            )
            evidence.append(item)
        self.event("boundary_evidence_extracted", {"evidence": evidence})

    def _close_sources(self) -> None:
        for source in reversed(self._started_sources):
            try:
                source.close()
            except BaseException as exc:
                self.run.record(
                    "events", "evidence-session",
                    {"event": "telemetry_source_close_failed",
                     "error": f"{type(exc).__name__}: {exc}"},
                    validity="error",
                )
        self._started_sources.clear()
