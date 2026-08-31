#!/usr/bin/env bash
# Shared helpers for repository launch scripts.

if [[ -z "${G1_TELEOP_ROOT:-}" ]]; then
  G1_TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
fi

g1_teleop_load_env() {
  if [[ -f "${G1_TELEOP_ROOT}/.env" ]]; then
    set -a
    # shellcheck disable=SC1091
    source "${G1_TELEOP_ROOT}/.env"
    set +a
  fi
}

g1_teleop_prepare_pythonpath() {
  export PYTHONPATH="${G1_TELEOP_ROOT}${PYTHONPATH:+:${PYTHONPATH}}"
}

g1_teleop_python_works() {
  local py="$1"
  [[ -n "${py}" ]] || return 1
  if [[ "${py}" == */* ]]; then
    [[ -x "${py}" ]] || return 1
  else
    command -v "${py}" >/dev/null 2>&1 || return 1
  fi
  "${py}" -c "import zmq" >/dev/null 2>&1
}

g1_teleop_python() {
  local candidate

  if [[ -n "${SONIC_PYTHON:-}" ]]; then
    printf '%s\n' "${SONIC_PYTHON}"
    return 0
  fi

  if [[ -n "${PYTHON:-}" ]] && g1_teleop_python_works "${PYTHON}"; then
    printf '%s\n' "${PYTHON}"
    return 0
  fi

  for candidate in \
    "${HOME}/.config/visionpro-g1-inspire-teleop/venv_sonic_sim/bin/python" \
    "/mnt/newssd/conda_envs/inspire_clean/bin/python" \
    "$(command -v python3 2>/dev/null || true)" \
    "$(command -v python 2>/dev/null || true)"
  do
    if g1_teleop_python_works "${candidate}"; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done

  printf '%s\n' "${PYTHON:-python3}"
}

g1_teleop_require_python() {
  local py="$1"
  if g1_teleop_python_works "${py}"; then
    return 0
  fi

  cat <<EOF >&2
ERROR: Python at '${py}' cannot import pyzmq (required for SONIC bridge).

Fix one of:
  1) Add to .env:
       SONIC_PYTHON=/path/to/your/conda/env/bin/python
     Example:
       SONIC_PYTHON=/mnt/newssd/conda_envs/inspire_clean/bin/python

  2) Activate conda, then rerun:
       conda activate inspire_clean
       pip install -r requirements.txt
       ./run_sonic_avp_teleop.sh <VP_IP>

  3) Install deps into current python:
       pip install pyzmq msgpack numpy scipy
EOF
  exit 1
}
