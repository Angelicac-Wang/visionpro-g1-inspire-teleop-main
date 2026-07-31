#!/usr/bin/env bash
set -euo pipefail

# Terminal 1 for WHOLE-BODY (arms + legs/walking) G1 simulation.
# Unlike the PickPlace tasks (legs locked), the "Wholebody" task runs a real
# RL walking policy (assets/model/policy.onnx) in Isaac Sim. The robot stands
# and walks according to a velocity command published on the DDS topic
#   rt/run_command/cmd  ->  string "[vx, vy, vyaw, height]"  (default [0,0,0,0.8])
#
# sim_main.py auto-detects "Wholebody" in the task name and switches the action
# source to dds_wholebody (enable_wholebody_dds), so no extra flag is needed.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_PYTHON="${SIM_PYTHON:-/mnt/newssd/conda_envs/unitree_sim_env/bin/python}"
UNITREE_SIM_ROOT="${UNITREE_SIM_ROOT:-/mnt/newssd/unitree_sim_isaaclab}"
SCRIPT_DIR="${REPO_ROOT}/scripts"

# Whole-body tasks (pick the EE you want the hands to use):
#   Isaac-Move-Cylinder-G129-Inspire-Wholebody   (Inspire dexterous hands)
#   Isaac-Move-Cylinder-G129-Dex3-Wholebody      (Dex3 three-finger hands)
#   Isaac-Move-Cylinder-G129-Dex1-Wholebody      (Dex1 two-finger grippers)
TASK="${TASK:-Isaac-Move-Cylinder-G129-Inspire-Wholebody}"

if [[ -z "${DEVICE:-}" ]]; then
  if nvidia-smi >/dev/null 2>&1; then
    DEVICE="cuda:0"
  else
    DEVICE="cpu"
  fi
fi

if [[ -f "${REPO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/.env"
  set +a
fi

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/isaac_env.sh"
isaac_prepare_env "${SIM_PYTHON}"

cat <<EOF
Starting G1 WHOLE-BODY simulation (arms + legs).
  task=${TASK}
  device=${DEVICE}

The legs are driven by an RL walking policy inside the sim. It walks when a
velocity command arrives on DDS topic rt/run_command/cmd.

How to drive the legs after this window is ready:
  A) Keyboard test (no Vision Pro):
       ${SCRIPT_DIR}/../scripts/wholebody_walk_test.py
     or:  ./scripts/wholebody_walk_test.py
  B) Vision Pro head tracking + hands:
       XR_HEAD_LOCO=1 ./run_xr_teleop.sh

IMPORTANT:
  - First startup can take 2-5 minutes. Do NOT Ctrl+C while loading.
  - Wait until this window prints ALL of these:
      ========= create dds success =========
      ========= start controller success =========
      ***  Please left-click on the Sim window to activate rendering. ***
  - Then click the Isaac window once. The robot should stand up and balance.
    (If it just stands still, that is correct: default command = stand.)
EOF

cd "${UNITREE_SIM_ROOT}"
exec "${SIM_PYTHON}" "${SCRIPT_DIR}/launch_sim_main.py" \
  --device "${DEVICE}" \
  --enable_cameras \
  --task "${TASK}" \
  --enable_inspire_dds \
  --robot_type g129 \
  --kit_args "${ISAAC_KIT_ARGS}"
