# Recovery After Reimaging

## Preservation status

The repository and its local runtime data were audited on 2026-09-04 before
reimaging.

- GitHub repository: `git@github.com:MoissanClub/demos.git`
- GitHub branch: `main`
- Last preservation commit before this guide: `7cae8eb`
- Hugging Face dataset: <https://huggingface.co/datasets/davidwei79/g1-handshake-data>
- Final Hugging Face reconciliation: 912 expected local artifacts, 0 missing

The Hugging Face archive includes:

- 78 validated handshake trajectories under `trajectories/`
- 3 non-canonical or incomplete captures under `raw_invalid_trajectories/`
- robot-run videos and evidence under `artifacts/`
- standalone telemetry and diagnostics under `telemetry/`
- runtime logs under `logs/`

The following reproducible caches and dependencies were intentionally not
archived:

- `.deps/`
- `.cache/`
- `.pytest_cache/`
- `__pycache__/` directories

## Restore procedure

Clone the GitHub repository:

```bash
git clone git@github.com:MoissanClub/demos.git
cd demos
```

Install the Hugging Face client and authenticate with an account that can read
the dataset:

```bash
python -m pip install huggingface_hub
hf auth login
```

Preview the restoration without writing files:

```bash
python restore_handshake_data.py --dry-run
```

Restore the data into the cloned repository:

```bash
python restore_handshake_data.py
```

The script restores canonical and raw trajectories to
`telemetry/trajectories/` and preserves the original layout of `artifacts/`,
`telemetry/`, and `logs/`. It skips files with identical content and reports a
conflict if a local file differs. Use `--overwrite` only after reviewing such
conflicts.

For a machine with limited system-disk space, place the download cache on a
larger disk:

```bash
python restore_handshake_data.py --cache-dir /path/to/large/disk/hf-cache
```

## Verify the restored checkout

```bash
git status --short --branch
python -m unittest tests.test_brainco_replay
python -m py_compile \
  handshake/brainco_replay.py \
  run_g1_reviewed_trace_replay.py \
  run_brainco_hand_cycle_test.py \
  restore_handshake_data.py
```

The data directories are intentionally not committed to GitHub. Seeing them as
untracked or ignored in `git status` after restoration is expected.
