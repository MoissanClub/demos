#!/usr/bin/env python3
"""Upload missing completed trajectories and remove abandoned temporary files."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

from huggingface_hub import HfApi


DEFAULT_REPO = "davidwei79/g1-handshake-data"
DEFAULT_ROOT = Path(__file__).resolve().parent / "telemetry" / "trajectories"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--local-root", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--repo-id", default=DEFAULT_REPO)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Show uploads and removals without changing the Hub or local files.",
    )
    return parser.parse_args()


def validate_completed(path: Path) -> tuple[bool, str]:
    """Require valid JSONL with matching metadata and summary trajectory IDs."""
    first: dict[str, Any] | None = None
    last: dict[str, Any] | None = None
    line_count = 0

    try:
        with path.open("r", encoding="utf-8") as handle:
            for line_count, line in enumerate(handle, start=1):
                if not line.strip():
                    return False, f"blank line at {line_count}"
                row = json.loads(line)
                if not isinstance(row, dict):
                    return False, f"line {line_count} is not a JSON object"
                if first is None:
                    first = row
                last = row
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        return False, f"line {line_count}: {exc}"

    if first is None or last is None:
        return False, "empty file"
    if first.get("stream") != "trajectory.metadata":
        return False, "first row is not trajectory.metadata"
    if last.get("stream") != "trajectory.summary":
        return False, "last row is not trajectory.summary"

    first_data = first.get("data")
    last_data = last.get("data")
    if not isinstance(first_data, dict) or not isinstance(last_data, dict):
        return False, "metadata or summary data is not an object"
    trajectory_id = first_data.get("trajectory_id")
    if not trajectory_id or trajectory_id != last_data.get("trajectory_id"):
        return False, "metadata and summary trajectory IDs do not match"
    if str(trajectory_id) not in path.name:
        return False, "trajectory ID is not present in filename"
    if last_data.get("result") not in {"success", "aborted"}:
        return False, "summary result is neither success nor aborted"
    return True, f"{line_count} rows"


def remote_path(local_root: Path, path: Path) -> str:
    relative = path.relative_to(local_root)
    if len(relative.parts) < 2:
        raise ValueError(f"trajectory is not inside a run directory: {path}")
    return (Path("trajectories") / relative).as_posix()


def main() -> int:
    args = parse_args()
    root = args.local_root.expanduser().resolve()
    if not root.is_dir():
        print(f"ERROR: local trajectory directory does not exist: {root}", file=sys.stderr)
        return 2

    api = HfApi()
    try:
        remote_files = set(
            api.list_repo_files(repo_id=args.repo_id, repo_type="dataset")
        )
    except Exception as exc:
        print(
            f"ERROR: cannot read Hugging Face dataset {args.repo_id}: {exc}\n"
            "Authenticate first with `hf auth login`.",
            file=sys.stderr,
        )
        return 2

    completed: list[tuple[Path, str]] = []
    invalid: list[tuple[Path, str]] = []
    for path in sorted(root.glob("*/trajectory_*.jsonl")):
        valid, detail = validate_completed(path)
        if valid:
            completed.append((path, remote_path(root, path)))
        else:
            invalid.append((path, detail))

    missing = [(path, target) for path, target in completed if target not in remote_files]
    present = len(completed) - len(missing)
    print(
        f"Hub comparison: {len(completed)} completed local, "
        f"{present} already uploaded, {len(missing)} missing"
    )

    upload_failures = 0
    for path, target in missing:
        if args.dry_run:
            print(f"WOULD UPLOAD {path} -> {target}")
            continue
        print(f"UPLOAD {path} -> {target}")
        try:
            api.upload_file(
                path_or_fileobj=path,
                path_in_repo=target,
                repo_id=args.repo_id,
                repo_type="dataset",
                commit_message=f"Upload recovered handshake trajectory {path.stem}",
            )
        except Exception as exc:
            upload_failures += 1
            print(f"ERROR: upload failed for {path}: {exc}", file=sys.stderr)

    temporary_files = sorted(root.glob("*/trajectory_*.jsonl.tmp"))
    removed = 0
    removal_failures = 0
    for path in temporary_files:
        if args.dry_run:
            print(f"WOULD REMOVE {path}")
            continue
        try:
            path.unlink()
            removed += 1
            print(f"REMOVED {path}")
        except OSError as exc:
            print(f"ERROR: could not remove {path}: {exc}", file=sys.stderr)
            removal_failures += 1

    for path, reason in invalid:
        print(f"WARNING: skipped incomplete/invalid file {path}: {reason}", file=sys.stderr)

    removal_count = len(temporary_files) if args.dry_run else removed
    action = "would remove" if args.dry_run else "removed"
    print(
        f"Done: uploaded {0 if args.dry_run else len(missing) - upload_failures} "
        f"of {len(missing)} missing trajectories; {action} {removal_count} temporary files."
    )
    return 1 if upload_failures or removal_failures or invalid else 0


if __name__ == "__main__":
    raise SystemExit(main())
