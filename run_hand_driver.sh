#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-/mnt/newssd/conda_envs/inspire_clean/bin/python}"

if [[ -f "${REPO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/.env"
  set +a
fi

export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-visionpro-g1}"

exec "${PYTHON}" "${REPO_ROOT}/scripts/Headless_driver_r.py" "$@"
