#!/usr/bin/env bash
set -euo pipefail

source "$(dirname "$0")/_common.sh"
g1_teleop_load_env
g1_teleop_prepare_pythonpath

PYTHON="$(g1_teleop_python)"
g1_teleop_require_python "${PYTHON}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-visionpro-g1}"

exec "${PYTHON}" "${G1_TELEOP_ROOT}/scripts/visionpro_g1_right_arm_hand.py" "$@"
