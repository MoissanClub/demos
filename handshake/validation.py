"""Streaming validation for handshake trajectory JSONL files."""

from __future__ import annotations

import json
import math
import re
from bisect import bisect_left
from collections import Counter
from dataclasses import asdict, dataclass, field
from pathlib import Path
from statistics import median
from typing import Any, Dict, Iterable, List, Optional


TRAJECTORY_ID_RE = re.compile(
    r"^trajectory_\d{8}T\d{6}(?:\.\d+)?Z_"
    r"([0-9a-fA-F]{8}-[0-9a-fA-F]{4}-[0-9a-fA-F]{4}-"
    r"[0-9a-fA-F]{4}-[0-9a-fA-F]{12})\.jsonl(?:\.tmp)?$"
)
REQUIRED_SUCCESS_STATES = {"closing", "releasing", "open_wait"}


@dataclass
class StreamStats:
    count: int = 0
    first_timestamp_ns: Optional[int] = None
    last_timestamp_ns: Optional[int] = None
    maximum_gap_ms: Optional[float] = None
    non_monotonic_samples: int = 0
    timestamps_ns: List[int] = field(default_factory=list, repr=False)

    def observe(self, timestamp_ns: int) -> None:
        self.timestamps_ns.append(timestamp_ns)
        if self.first_timestamp_ns is None:
            self.first_timestamp_ns = timestamp_ns
        if self.last_timestamp_ns is not None:
            delta = timestamp_ns - self.last_timestamp_ns
            if delta < 0:
                self.non_monotonic_samples += 1
            else:
                gap_ms = delta / 1_000_000
                if self.maximum_gap_ms is None or gap_ms > self.maximum_gap_ms:
                    self.maximum_gap_ms = gap_ms
        self.last_timestamp_ns = timestamp_ns
        self.count += 1

    def to_dict(self) -> Dict[str, Any]:
        duration_s = None
        rate_hz = None
        if self.first_timestamp_ns is not None and self.last_timestamp_ns is not None:
            duration_s = max(0.0, (self.last_timestamp_ns - self.first_timestamp_ns) / 1e9)
            if self.count > 1 and duration_s > 0:
                rate_hz = (self.count - 1) / duration_s
        sample_frequency_hz = None
        expected_period_ms = None
        expected_sample_count = None
        missing_sample_count = None
        missing_sample_fraction = None
        coverage_tolerance_ms = None
        ordered = sorted(self.timestamps_ns)
        positive_deltas = [
            current - previous
            for previous, current in zip(ordered, ordered[1:])
            if current > previous
        ]
        if positive_deltas:
            period_ns = median(positive_deltas)
            sample_frequency_hz = 1e9 / period_ns
            expected_period_ms = period_ns / 1e6
            coverage_tolerance_ns = period_ns * 0.1
            coverage_tolerance_ms = coverage_tolerance_ns / 1e6
            span_ns = ordered[-1] - ordered[0]
            expected_sample_count = int(math.floor(span_ns / period_ns)) + 1
            missing_sample_count = 0
            for index in range(expected_sample_count):
                expected = ordered[0] + index * period_ns
                position = bisect_left(ordered, expected)
                nearest = []
                if position < len(ordered):
                    nearest.append(abs(ordered[position] - expected))
                if position:
                    nearest.append(abs(ordered[position - 1] - expected))
                if not nearest or min(nearest) > coverage_tolerance_ns:
                    missing_sample_count += 1
            missing_sample_fraction = missing_sample_count / expected_sample_count
        return {
            "count": self.count,
            "duration_s": duration_s,
            "effective_rate_hz": rate_hz,
            "sample_frequency_hz": sample_frequency_hz,
            "expected_period_ms": expected_period_ms,
            "coverage_tolerance_ms": coverage_tolerance_ms,
            "expected_sample_count": expected_sample_count,
            "missing_sample_count": missing_sample_count,
            "missing_sample_fraction": missing_sample_fraction,
            "has_missing_samples": missing_sample_count is not None and missing_sample_count > 0,
            "maximum_gap_ms": self.maximum_gap_ms,
            "non_monotonic_samples": self.non_monotonic_samples,
        }


@dataclass
class ValidationResult:
    path: str
    classification: str = "rejected"
    trajectory_id: Optional[str] = None
    result: Optional[str] = None
    reason: Optional[str] = None
    duration_s: Optional[float] = None
    dropped_samples_total: Optional[int] = None
    line_count: int = 0
    streams: Dict[str, Dict[str, Any]] = field(default_factory=dict)
    controller_states: List[str] = field(default_factory=list)
    controller_events: List[str] = field(default_factory=list)
    state_cycle: List[str] = field(default_factory=list)
    full_state_transition: bool = False
    vision_signal_present: bool = False
    vision_signal_sample_count: int = 0
    streams_with_missing_samples: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        return not self.errors and self.classification not in {"rejected", "incomplete"}

    def to_dict(self) -> Dict[str, Any]:
        value = asdict(self)
        value["valid"] = self.valid
        return value


def discover_trajectories(paths: Iterable[Path]) -> List[Path]:
    """Resolve files and recursively discover finalized and temporary episodes."""
    discovered = set()
    for path in paths:
        path = Path(path)
        if path.is_dir():
            discovered.update(path.rglob("trajectory_*.jsonl"))
            discovered.update(path.rglob("trajectory_*.jsonl.tmp"))
        elif path.is_file():
            discovered.add(path)
    return sorted(discovered)


def _finite_numbers(value: Any) -> bool:
    if isinstance(value, bool) or value is None or isinstance(value, str):
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(value)
    if isinstance(value, list):
        return all(_finite_numbers(item) for item in value)
    if isinstance(value, dict):
        return all(_finite_numbers(item) for item in value.values())
    return True


def _append_state(states: List[str], state: Any) -> None:
    if isinstance(state, str) and (not states or states[-1] != state):
        states.append(state)


def _has_full_state_transition(states: List[str]) -> bool:
    """Check open_wait -> closing -> [hold] -> releasing -> open_wait in order."""
    if not states or states[0] != "open_wait" or states[-1] != "open_wait":
        return False
    position = 0
    for required in ("closing", "releasing", "open_wait"):
        try:
            position = states.index(required, position + 1)
        except ValueError:
            return False
    return position == len(states) - 1


def validate_trajectory(path: Path) -> ValidationResult:
    """Validate one trajectory without retaining large telemetry rows in memory."""
    path = Path(path)
    report = ValidationResult(path=str(path))
    stats: Dict[str, StreamStats] = {}
    stream_counts: Counter[str] = Counter()
    metadata_rows: List[Dict[str, Any]] = []
    summary_rows: List[Dict[str, Any]] = []
    first_stream = None
    last_stream = None
    first_timestamp = None
    last_timestamp = None
    states = []
    state_cycle = []
    events = []
    vision_sample_count = 0

    filename_match = TRAJECTORY_ID_RE.match(path.name)
    filename_id = filename_match.group(1) if filename_match else None
    if filename_id is None:
        report.warnings.append("filename does not contain a recognized trajectory UUID")

    try:
        source = path.open("r", encoding="utf-8")
    except OSError as exc:
        report.errors.append(f"cannot open file: {exc}")
        return report

    with source:
        for line_number, line in enumerate(source, 1):
            if not line.strip():
                report.warnings.append(f"line {line_number}: blank line")
                continue
            try:
                row = json.loads(line)
            except (json.JSONDecodeError, UnicodeDecodeError) as exc:
                report.errors.append(f"line {line_number}: malformed JSON: {exc}")
                continue
            report.line_count += 1
            if not isinstance(row, dict):
                report.errors.append(f"line {line_number}: record is not an object")
                continue
            stream = row.get("stream")
            timestamp = row.get("timestamp_monotonic_ns")
            if not isinstance(stream, str) or not stream:
                report.errors.append(f"line {line_number}: missing or invalid stream")
                continue
            if not isinstance(timestamp, int) or isinstance(timestamp, bool):
                report.errors.append(f"line {line_number}: missing or invalid timestamp_monotonic_ns")
                continue
            if "data" not in row:
                report.errors.append(f"line {line_number}: missing data")
                continue
            if not _finite_numbers(row["data"]):
                report.errors.append(f"line {line_number}: data contains NaN or infinity")

            first_stream = stream if first_stream is None else first_stream
            last_stream = stream
            first_timestamp = timestamp if first_timestamp is None else min(first_timestamp, timestamp)
            last_timestamp = timestamp if last_timestamp is None else max(last_timestamp, timestamp)
            stream_counts[stream] += 1
            stats.setdefault(stream, StreamStats()).observe(timestamp)

            if stream == "trajectory.metadata" and isinstance(row["data"], dict):
                metadata_rows.append(row["data"])
            elif stream == "trajectory.summary" and isinstance(row["data"], dict):
                summary_rows.append(row["data"])
            elif stream == "controller.decision" and isinstance(row["data"], dict):
                state = row["data"].get("state")
                event = row["data"].get("event")
                _append_state(states, state)
                _append_state(state_cycle, state)
                if isinstance(event, str):
                    events.append(event)
                if row["data"].get("vision_state") is not None or row["data"].get("vision_score") is not None:
                    vision_sample_count += 1
            elif stream == "brainco.touch":
                _append_state(state_cycle, row.get("controller_state_before"))

    report.streams = {name: stats[name].to_dict() for name in sorted(stats)}
    report.streams_with_missing_samples = [
        name for name, stream in report.streams.items() if stream["has_missing_samples"]
    ]
    report.controller_states = states
    report.controller_events = events
    report.state_cycle = state_cycle
    report.full_state_transition = _has_full_state_transition(state_cycle)
    report.vision_signal_sample_count = vision_sample_count
    report.vision_signal_present = vision_sample_count > 0
    if first_timestamp is not None and last_timestamp is not None:
        report.duration_s = max(0.0, (last_timestamp - first_timestamp) / 1e9)

    if report.line_count == 0:
        report.errors.append("file contains no records")
    if first_stream != "trajectory.metadata":
        report.errors.append("first record is not trajectory.metadata")
    if len(metadata_rows) != 1:
        report.errors.append(f"expected one trajectory.metadata record, found {len(metadata_rows)}")
    if path.name.endswith(".tmp") or not summary_rows:
        report.classification = "incomplete"
        if path.name.endswith(".tmp"):
            report.errors.append("temporary trajectory was not atomically finalized")
        if not summary_rows:
            report.errors.append("missing trajectory.summary")
    elif len(summary_rows) != 1:
        report.errors.append(f"expected one trajectory.summary record, found {len(summary_rows)}")
    if summary_rows and last_stream != "trajectory.summary":
        report.errors.append("last record is not trajectory.summary")
    if not stream_counts["brainco.touch"]:
        report.errors.append("missing required brainco.touch stream")
    if not stream_counts["controller.decision"]:
        report.errors.append("missing required controller.decision stream")
    metadata = metadata_rows[0] if metadata_rows else {}
    summary = summary_rows[0] if summary_rows else {}
    metadata_id = metadata.get("trajectory_id")
    summary_id = summary.get("trajectory_id")
    report.trajectory_id = metadata_id if isinstance(metadata_id, str) else filename_id
    ids = [value for value in (filename_id, metadata_id, summary_id) if value is not None]
    if ids and any(value != ids[0] for value in ids[1:]):
        report.errors.append("filename, metadata, and summary trajectory UUIDs do not match")

    result = summary.get("result")
    reason = summary.get("reason")
    dropped = summary.get("dropped_samples_total")
    report.result = result if isinstance(result, str) else None
    report.reason = reason if isinstance(reason, str) else None
    if summary_rows:
        if report.result not in {"success", "aborted", "rejected"}:
            report.errors.append("summary result is missing or invalid")
        if not report.reason:
            report.errors.append("summary reason is missing or invalid")
        if not isinstance(dropped, int) or isinstance(dropped, bool) or dropped < 0:
            report.errors.append("summary dropped_samples_total is missing or invalid")
        else:
            report.dropped_samples_total = dropped

    if report.result == "success":
        missing_states = sorted(REQUIRED_SUCCESS_STATES.difference(states))
        if missing_states:
            report.errors.append("successful trajectory missing states: " + ", ".join(missing_states))
        if states and states[-1] != "open_wait":
            report.errors.append("successful trajectory does not end in open_wait")
        if not report.full_state_transition:
            report.errors.append("successful trajectory does not contain a full open_wait state cycle")

    if report.classification != "incomplete":
        if report.errors:
            report.classification = "rejected"
        elif report.result in {"success", "aborted", "rejected"}:
            report.classification = report.result
        else:
            report.classification = "rejected"
    return report


def collection_summary(results: Iterable[ValidationResult]) -> Dict[str, Any]:
    results = list(results)
    classifications = Counter(result.classification for result in results)
    return {
        "trajectory_count": len(results),
        "valid_count": sum(result.valid for result in results),
        "error_count": sum(len(result.errors) for result in results),
        "warning_count": sum(len(result.warnings) for result in results),
        "vision_signal_trajectory_count": sum(result.vision_signal_present for result in results),
        "full_state_transition_count": sum(result.full_state_transition for result in results),
        "trajectories_with_missing_samples": sum(
            bool(result.streams_with_missing_samples) for result in results
        ),
        "classifications": dict(sorted(classifications.items())),
    }
