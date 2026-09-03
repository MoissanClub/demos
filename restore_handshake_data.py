#!/usr/bin/env python3
"""Restore G1 handshake recordings archived in the Hugging Face dataset."""

from __future__ import annotations

import argparse
import hashlib
import os
import shutil
import sys
from pathlib import Path, PurePosixPath

from huggingface_hub import HfApi, hf_hub_download


DEFAULT_REPO = "davidwei79/g1-handshake-data"
ARCHIVE_PREFIXES = ("artifacts/", "logs/", "telemetry/")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-id", default=DEFAULT_REPO)
    parser.add_argument(
        "--destination", type=Path, default=Path(__file__).resolve().parent,
        help="Restored project root (default: directory containing this script).",
    )
    parser.add_argument("--revision", default="main")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--overwrite", action="store_true",
        help="Replace an existing local file when its content differs.",
    )
    parser.add_argument(
        "--cache-dir", type=Path,
        help="Optional Hugging Face download cache (use a disk with enough space).",
    )
    return parser.parse_args(argv)


def local_relative_path(remote_path: str) -> Path | None:
    """Map dataset archive paths back to their original project locations."""
    pure = PurePosixPath(remote_path)
    if pure.is_absolute() or ".." in pure.parts:
        raise ValueError(f"unsafe repository path: {remote_path}")
    if remote_path.startswith("trajectories/"):
        return Path("telemetry") / Path(*pure.parts)
    if remote_path.startswith("raw_invalid_trajectories/"):
        return Path("telemetry/trajectories") / Path(*pure.parts[1:])
    if remote_path.startswith(ARCHIVE_PREFIXES):
        return Path(*pure.parts)
    return None


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main(argv=None) -> int:
    args = parse_args(argv)
    destination = args.destination.expanduser().resolve()
    api = HfApi()
    try:
        remote_files = api.list_repo_files(
            repo_id=args.repo_id, repo_type="dataset", revision=args.revision,
        )
    except Exception as exc:
        print(
            f"ERROR: cannot read dataset {args.repo_id}: {exc}\n"
            "Install `huggingface_hub` and authenticate with `hf auth login`.",
            file=sys.stderr,
        )
        return 2

    restore = []
    for remote_path in remote_files:
        relative = local_relative_path(remote_path)
        if relative is not None:
            restore.append((remote_path, destination / relative))

    print(f"Restore plan: {len(restore)} files from {args.repo_id} to {destination}")
    if args.dry_run:
        for remote_path, target in restore:
            print(f"WOULD RESTORE {remote_path} -> {target}")
        return 0

    restored = identical = conflicts = failures = 0
    for index, (remote_path, target) in enumerate(restore, start=1):
        try:
            cached = Path(hf_hub_download(
                repo_id=args.repo_id,
                filename=remote_path,
                repo_type="dataset",
                revision=args.revision,
                cache_dir=str(args.cache_dir.expanduser()) if args.cache_dir else None,
            ))
            if target.exists():
                if target.is_file() and sha256(target) == sha256(cached):
                    identical += 1
                    print(f"[{index}/{len(restore)}] PRESENT {target}")
                    continue
                if not args.overwrite:
                    conflicts += 1
                    print(f"CONFLICT: {target} differs; use --overwrite", file=sys.stderr)
                    continue
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(f".{target.name}.restore.tmp")
            shutil.copy2(cached, temporary)
            os.replace(temporary, target)
            restored += 1
            print(f"[{index}/{len(restore)}] RESTORED {target}")
        except Exception as exc:
            failures += 1
            print(f"ERROR: {remote_path}: {exc}", file=sys.stderr)

    print(
        f"Done: restored {restored}, already identical {identical}, "
        f"conflicts {conflicts}, failures {failures}."
    )
    return 1 if conflicts or failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
