#!/usr/bin/env bash
set -euo pipefail

# Terminal 3 for SONIC + Apple Vision Pro:
#   AVP tracking -> ZMQ planner (vr 3-point upper body + head locomotion)
#
# Requires:
#   Terminal 1: ./run_sonic_sim_loop.sh
#   Terminal 2: ./run_sonic_deploy.sh
#
# Usage:
#   ./run_sonic_avp_teleop.sh 192.168.2.14
#   ./run_sonic_avp_teleop.sh MLBS-4109   # cross-network room code

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
  echo "Set SONIC_PYTHON or install inspire_clean conda env."
  exit 1
fi

if [[ -f "${REPO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/.env"
  set +a
fi

cat <<EOF
SONIC Terminal 3 — AVP bridge
  Endpoint: ${AVP_ENDPOINT}
  PUB bind: tcp://*:5556  (SONIC deploy must use --zmq-host localhost)
  Mapping: official-calib (head + 2 hands, Pico-style yaw-compensated wrists)
  Locomotion: head walk after T
  MuJoCo FPV: robot head_camera -> Vision Pro (default ON; Terminal 1 publishes :5555)

Vision Pro:
  Open Tracking Streamer app -> Start (same Wi-Fi as this PC).
  You should see the sim robot view inside the app while teleoperating.

MuJoCo (Terminal 1): press 9 once after ] if robot hangs in the air.

Operator:
  1) Wait for "AVP tracking locked"
  2) F — CALIB_FULL: forearms-forward, hold ~2s (c works too)
  3) ] — ENGAGE policy; wait for balance (MuJoCo key 9 if needed)
  4) S — CALIB_SYNC: match robot arms, hold ~2s (uses deploy feedback :5557)
  5) T — TELEOP: head walk + squat + hands
  P pause | H head zero | S re-sync if arms drift
  6) Press o here / O in deploy to stop
EOF

cd "${REPO_ROOT}"
exec "${PYTHON}" scripts/g1_avp_sonic_teleop.py \
  --avp-endpoint "${AVP_ENDPOINT}" \
  --print-debug \
  "$@"
