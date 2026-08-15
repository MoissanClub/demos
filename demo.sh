#!/usr/bin/env bash
set -euo pipefail

demo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
python_bin="/home/dwei/miniconda3/envs/g1brainco/bin/python"

export CYCLONEDDS_HOME="${demo_dir}/.deps/cyclonedds-install"

exec "${python_bin}" "${demo_dir}/g1_brainco_handshake_demo.py" \
  --right \
  --enable-arm \
  --record-telemetry \
  --arm-network-interface eth0 \
  "$@"
