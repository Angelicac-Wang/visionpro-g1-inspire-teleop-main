#!/usr/bin/env bash
set -euo pipefail

# Stream a full G1 humanoid mesh into Apple Vision Pro (AR view).
# This is a passive Isaac Lab demo: the robot is visible in VP, but it does
# not follow your hand yet. For hand-driven sim teleop, use run_sim_xr_teleop.sh.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-/mnt/newssd/conda_envs/unitree_sim_env/bin/python}"
VISIONPRO_TELEOP_ROOT="${VISIONPRO_TELEOP_ROOT:-/mnt/newssd/unitree_sim_isaaclab/inspire_hand_ws/VisionProTeleop}"
SCRIPT_DIR="${REPO_ROOT}/scripts"

if [[ -f "${REPO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/.env"
  set +a
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/isaac_env.sh"
isaac_prepare_env "${PYTHON}"

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <vision_pro_ip_or_room_code> [--device cpu|cuda:0] [--headless]"
  echo
  echo "Examples:"
  echo "  $0 MLBS-4109 --device cpu              # local window + Vision Pro"
  echo "  $0 MLBS-4109 --device cpu --headless   # Vision Pro only, no local window"
  echo
  echo "For local window only, use ./run_g1_local_view.sh"
  exit 1
fi

AVP_ENDPOINT="$1"
shift

exec "${PYTHON}" "${VISIONPRO_TELEOP_ROOT}/examples/13_g1_freefall.py" \
  --ip "${AVP_ENDPOINT}" \
  --num-envs 1 \
  --drop-height 1.2 \
  --kit_args "${ISAAC_KIT_ARGS}" \
  "$@"
