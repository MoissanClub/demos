"""Small adapters from existing telemetry callbacks to the run harness."""
from __future__ import annotations

import re
from typing import Any

from robot_dev_harness.run_artifacts import RunArtifacts


class LegacyTelemetryAdapter:
    """Accept the repository's existing ``record(stream, data, ...)`` calls."""

    def __init__(self, run: RunArtifacts):
        self.run = run

    def record(
        self, stream: str, data: Any, timestamp_ns: int = None, **metadata: Any
    ) -> bool:
        filename = re.sub(r"[^a-z0-9_.-]+", "-", stream.lower()).strip("-")
        payload = {"value": data}
        if metadata:
            payload["metadata"] = metadata
        return self.run.record(
            stream=filename,
            source=filename,
            data=payload,
            monotonic_ns=timestamp_ns,
        )
