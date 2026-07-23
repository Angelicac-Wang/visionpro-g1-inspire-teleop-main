#!/usr/bin/env bash
set -euo pipefail

# Terminal 3 for pick-up teleop: AVP upper body + Inspire finger sim + head walk.
#
# Requires:
#   Terminal 1: ./run_sonic_sim_loop_pnp.sh
#   Terminal 2: ./run_sonic_deploy.sh

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${SONIC_PYTHON:-/mnt/newssd/conda_envs/inspire_clean/bin/python}"
AVP_ENDPOINT="${1:-}"

if [[ -z "${AVP_ENDPOINT}" ]]; then
  echo "Usage: $0 <Vision_Pro_IP_or_room_code> [extra avp_to_sonic_zmq args...]"
  exit 1
fi
shift

if [[ ! -x "${PYTHON}" ]]; then
  echo "Python not found: ${PYTHON}"
  exit 1
fi

if [[ -f "${REPO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/.env"
  set +a
fi

cat <<EOF
SONIC Terminal 3 — AVP pick-up teleop
  Endpoint: ${AVP_ENDPOINT}
  Arm/wrist: same defaults as ./run_sonic_avp_teleop.sh (official-calib, calibrated wrist)
  Inspire hand sim: ON (pinch fingers to close gripper in MuJoCo)
  Locomotion: head walk after T

Pick-up flow:
  F -> ] -> S -> T  (same staged calib as walk teleop)
  Walk to cube, reach, pinch to grasp
  P pause | H head zero | S re-sync anytime | c alias for F
EOF

cd "${REPO_ROOT}"
exec "${PYTHON}" scripts/g1_avp_sonic_teleop_pick.py \
  --avp-endpoint "${AVP_ENDPOINT}" \
  --print-debug \
  "$@"
