#!/usr/bin/env bash
set -euo pipefail

# Terminal 1 — MuJoCo REMS Task A corridor scene (3.0 m x 2.2 m, 5 waypoints).
# Pair with: bin/sonic-deploy.sh + bin/sonic-teleop.sh

source "$(dirname "$0")/_common.sh"
g1_teleop_load_env

export GR00T_ROOT="${GR00T_ROOT:-/mnt/newssd/GR00T-WholeBodyControl}"
SCENE_XML="$("${G1_TELEOP_ROOT}/tools/generate_corridor_scene.py")"
export SONIC_ROBOT_SCENE="${SCENE_XML}"

cat <<EOF
SONIC Terminal 1 — REMS Task A corridor scene
  Scene: ${SCENE_XML}
  Next: bin/sonic-deploy.sh  |  bin/sonic-teleop.sh <VP_IP>
  Eval: add --eval-log runs/task_a_imu_on.csv to Terminal 3
EOF

exec "$(dirname "$0")/sonic-sim.sh" "$@"
