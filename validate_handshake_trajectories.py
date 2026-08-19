#!/usr/bin/env python3
"""Validate finalized and incomplete handshake trajectory recordings."""

import argparse
import json
import sys
from pathlib import Path

from handshake.validation import collection_summary, discover_trajectories, validate_trajectory


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "paths",
        nargs="*",
        type=Path,
        default=[Path("telemetry/trajectories")],
        help="trajectory file or directory (default: telemetry/trajectories)",
    )
    parser.add_argument("--json", action="store_true", help="write the full report as JSON")
    parser.add_argument("--output", type=Path, help="write JSON to this path instead of stdout")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    paths = discover_trajectories(args.paths)
    if not paths:
        print("No trajectory JSONL files found.", file=sys.stderr)
        return 2

    results = [validate_trajectory(path) for path in paths]
    payload = {
        "summary": collection_summary(results),
        "trajectories": [result.to_dict() for result in results],
    }
    if args.json or args.output:
        rendered = json.dumps(payload, indent=2, sort_keys=True) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(rendered, encoding="utf-8")
        else:
            sys.stdout.write(rendered)
    else:
        for result in results:
            marker = "OK" if result.valid else "FAIL"
            duration = "?" if result.duration_s is None else f"{result.duration_s:.2f}s"
            print(f"{marker:4} {result.classification:10} {duration:>9} {result.path}")
            print(
                f"       vision: {'yes' if result.vision_signal_present else 'no'}; "
                f"full state transition: {'yes' if result.full_state_transition else 'no'}; "
                f"missing samples: {'yes' if result.streams_with_missing_samples else 'no'}; "
                f"states: {' -> '.join(result.state_cycle) or 'none'}"
            )
            for stream, stats in result.streams.items():
                frequency = stats["sample_frequency_hz"]
                if frequency is None:
                    frequency_text = "n/a"
                else:
                    frequency_text = f"{frequency:.2f} Hz"
                missing = stats["missing_sample_count"]
                expected = stats["expected_sample_count"]
                coverage_text = "n/a" if missing is None else f"{missing}/{expected} missing"
                print(f"       {stream}: {frequency_text}, {coverage_text}")
            for message in result.errors:
                print(f"       error: {message}")
            for message in result.warnings:
                print(f"       warning: {message}")
        summary = payload["summary"]
        print(
            f"\n{summary['trajectory_count']} trajectories: "
            f"{summary['valid_count']} valid, {summary['error_count']} errors, "
            f"{summary['warning_count']} warnings; {summary['classifications']}"
        )
    return 1 if any(result.errors for result in results) else 0


if __name__ == "__main__":
    raise SystemExit(main())
