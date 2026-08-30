"""Correlate monotonic telemetry events with recorded video frames."""
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping


def load_frame_timestamps(path: Path) -> List[Dict[str, Any]]:
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    if not rows:
        raise ValueError("frame timestamp index is empty")
    for expected, row in enumerate(rows):
        if row.get("frame_index") != expected:
            raise ValueError("frame timestamp indices are not contiguous")
        if not isinstance(row.get("monotonic_ns"), int):
            raise ValueError(f"frame {expected} has no valid monotonic timestamp")
    if any(
        current["monotonic_ns"] <= previous["monotonic_ns"]
        for previous, current in zip(rows, rows[1:])
    ):
        raise ValueError("frame monotonic timestamps are not strictly increasing")
    return rows


def nearest_frame(rows: List[Dict[str, Any]], event_monotonic_ns: int) -> Dict[str, Any]:
    if not isinstance(event_monotonic_ns, int):
        raise ValueError("event monotonic timestamp must be an integer")
    return min(rows, key=lambda row: abs(row["monotonic_ns"] - event_monotonic_ns))


def format_visual_review(
    observations: Iterable[Mapping[str, Any]],
    heading: str = "Visual review",
) -> str:
    """Format visual findings with mandatory, traceable source-frame references."""
    rows = list(observations)
    if not rows:
        raise ValueError("visual review must contain at least one observation")
    rendered = [f"## {heading}", ""]
    for observation_number, observation in enumerate(rows, start=1):
        finding = str(observation.get("finding", "")).strip()
        frames = observation.get("frames")
        if not finding:
            raise ValueError(f"visual observation {observation_number} has no finding")
        if not isinstance(frames, (list, tuple)) or not frames:
            raise ValueError(
                f"visual observation {observation_number} has no frame references"
            )
        references = []
        for frame_number, frame in enumerate(frames, start=1):
            if not isinstance(frame, Mapping):
                raise ValueError(
                    f"visual observation {observation_number} frame "
                    f"{frame_number} is invalid"
                )
            index = frame.get("frame_index")
            timestamp = frame.get("frame_timestamp_utc")
            if (
                not isinstance(index, int) or index < 0
                or not isinstance(timestamp, str) or not timestamp.strip()
            ):
                raise ValueError(
                    f"visual observation {observation_number} frame "
                    f"{frame_number} lacks an index or UTC timestamp"
                )
            event = str(frame.get("event", "")).strip()
            label = f"{event}: " if event else ""
            references.append(f"{label}frame `{index}` at `{timestamp}`")
        rendered.append(f"- {'; '.join(references)} — {finding}")
    return "\n".join(rendered) + "\n"


def extract_nearest_frame(
    video_path: Path,
    frame_timestamps_path: Path,
    event_monotonic_ns: int,
    event: str,
    evidence_directory: Path,
) -> Dict[str, Any]:
    """Extract and describe the frame closest to a telemetry event timestamp."""
    import cv2

    event_name = re.sub(r"[^a-z0-9_-]+", "-", event.lower()).strip("-")
    if not event_name:
        raise ValueError("event name is empty")
    rows = load_frame_timestamps(frame_timestamps_path)
    frame = nearest_frame(rows, event_monotonic_ns)
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open recorded video {video_path}")
    capture.set(cv2.CAP_PROP_POS_FRAMES, frame["frame_index"])
    ok, image = capture.read()
    capture.release()
    if not ok:
        raise RuntimeError(f"cannot decode video frame {frame['frame_index']}")
    timestamp_path = frame["timestamp_utc"].replace("-", "").replace(":", "")
    target = evidence_directory / (
        f"{event_name}_{timestamp_path}_{frame['frame_index']:06d}.jpg"
    )
    if target.exists():
        raise FileExistsError(target)
    if not cv2.imwrite(str(target), image):
        raise RuntimeError(f"cannot write evidence frame {target}")
    return {
        "event": event_name,
        "event_monotonic_ns": event_monotonic_ns,
        "frame_index": frame["frame_index"],
        "frame_timestamp_utc": frame["timestamp_utc"],
        "frame_monotonic_ns": frame["monotonic_ns"],
        "offset_ns": frame["monotonic_ns"] - event_monotonic_ns,
        "evidence_path": str(target),
    }
