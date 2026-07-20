#!/usr/bin/env bash
set -euo pipefail

# Terminal 1 for pick-up task: MuJoCo sim with table + cube in front of G1.
# Pair with run_sonic_deploy.sh + run_sonic_avp_teleop_pick.sh

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GR00T_ROOT="${GR00T_ROOT:-/mnt/newssd/GR00T-WholeBodyControl}"
export GR00T_ROOT

SCENE_XML="$("${REPO_ROOT}/scripts/generate_pnp_scene.py")"
export SONIC_ROBOT_SCENE="${SCENE_XML}"

cat <<EOF
SONIC Terminal 1 — MuJoCo pick-up scene
  Scene: ${SCENE_XML}
  Object: 7 cm cube on table (~0.5 m in front of robot)
  Next: Terminal 2 -> ./run_sonic_deploy.sh
        Terminal 3 -> ./run_sonic_avp_teleop_pick.sh <VP_IP>
  MuJoCo tip: press 9 to toggle elastic band after ]
EOF

exec "${REPO_ROOT}/run_sonic_sim_loop.sh"
