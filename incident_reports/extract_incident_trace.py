#!/usr/bin/env python3
"""Extract a compact, read-only event trace from an incident JSONL file."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any, Dict


def compact(data: Dict[str, Any]) -> Dict[str, Any]:
    return {
        key: value
        for key, value in data.items()
        if key not in {"positions", "velocities", "centers_rad", "gains"}
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl", type=Path)
    args = parser.parse_args()

    origin_ns = None
    with args.jsonl.open("r", encoding="utf-8") as source:
        for line_number, line in enumerate(source, 1):
            row = json.loads(line)
            stream = row.get("stream")
            data = row.get("data", {})
            if stream not in {
                "controller.event",
                "standalone_arm.event",
                "standalone_arm.summary",
                "trajectory.summary",
                "recording.summary",
            }:
                continue
            event = data.get("event", stream)
            if event == "arm_sdk_command":
                continue
            timestamp_ns = int(row["timestamp_monotonic_ns"])
            if origin_ns is None:
                origin_ns = timestamp_ns
            relative_seconds = (timestamp_ns - origin_ns) / 1e9
            print(
                f"{relative_seconds:.6f}\t{stream}\t{event}\t"
                f"{json.dumps(compact(data), sort_keys=True)}"
            )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
