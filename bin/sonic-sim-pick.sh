#!/usr/bin/env bash
set -euo pipefail

# Terminal 1 — MuJoCo pick-up scene (table + cube).
# Pair with: bin/sonic-deploy.sh + bin/sonic-teleop-pick.sh

source "$(dirname "$0")/_common.sh"
g1_teleop_load_env

export GR00T_ROOT="${GR00T_ROOT:-/mnt/newssd/GR00T-WholeBodyControl}"
SCENE_XML="$("${G1_TELEOP_ROOT}/tools/generate_pnp_scene.py")"
export SONIC_ROBOT_SCENE="${SCENE_XML}"

cat <<EOF
SONIC Terminal 1 — pick-up scene
  Scene: ${SCENE_XML}
  Next: bin/sonic-deploy.sh  |  bin/sonic-teleop-pick.sh <VP_IP>
EOF

exec "$(dirname "$0")/sonic-sim.sh" "$@"
