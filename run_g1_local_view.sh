#!/usr/bin/env bash
set -euo pipefail

# Open a local Isaac Lab window showing the full G1 robot.
# Optionally also stream the same scene to Vision Pro.

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

AVP_ENDPOINT=""
DEVICE="cpu"
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --avp-endpoint)
      AVP_ENDPOINT="$2"
      shift 2
      ;;
    --device)
      DEVICE="$2"
      shift 2
      ;;
    --headless)
      EXTRA_ARGS+=("--headless")
      shift
      ;;
    *)
      echo "Unknown argument: $1"
      echo
      echo "Usage: $0 [--avp-endpoint MLBS-xxxx] [--device cpu|cuda:0] [--headless]"
      echo
      echo "Examples:"
      echo "  $0                              # local window only"
      echo "  $0 --avp-endpoint MLBS-4109     # local window + Vision Pro stream"
      exit 1
      ;;
  esac
done

CMD=(
  "${PYTHON}"
  "${VISIONPRO_TELEOP_ROOT}/examples/13_g1_freefall.py"
  --num-envs 1
  --drop-height 1.2
  --device "${DEVICE}"
  --kit_args "${ISAAC_KIT_ARGS}"
)

if [[ -n "${AVP_ENDPOINT}" ]]; then
  CMD+=(--ip "${AVP_ENDPOINT}")
fi

if ((${#EXTRA_ARGS[@]})); then
  CMD+=("${EXTRA_ARGS[@]}")
fi

echo "Starting local G1 viewer (Isaac Lab window should open on this machine)."
if [[ -n "${AVP_ENDPOINT}" ]]; then
  echo "Also streaming to Vision Pro: ${AVP_ENDPOINT}"
else
  echo "Vision Pro streaming disabled. Add --avp-endpoint to enable."
fi
echo

exec "${CMD[@]}"
