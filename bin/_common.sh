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

g1_teleop_python() {
  printf '%s\n' "${SONIC_PYTHON:-${PYTHON:-python3}}"
}
