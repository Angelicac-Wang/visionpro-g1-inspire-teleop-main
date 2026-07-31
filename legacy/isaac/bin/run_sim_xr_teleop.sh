#!/usr/bin/env bash
set -euo pipefail

# Hand-driven G1 simulation in Apple Vision Pro via xr_teleoperate (TeleVuer).
# You control the sim robot with your hands. Vision Pro shows the robot POV
# camera plus hand markers, not a full third-person robot mesh.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_PYTHON="${SIM_PYTHON:-/mnt/newssd/conda_envs/unitree_sim_env/bin/python}"
TV_PYTHON="${TV_PYTHON:-/mnt/newssd/conda_envs/tv/bin/python}"
UNITREE_SIM_ROOT="${UNITREE_SIM_ROOT:-/mnt/newssd/unitree_sim_isaaclab}"
XR_TELEOP_ROOT="${XR_TELEOP_ROOT:-${UNITREE_SIM_ROOT}/xr_teleoperate}"
SCRIPT_DIR="${REPO_ROOT}/scripts"

HOST_IP="${HOST_IP:-$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')}"
TASK="${TASK:-Isaac-PickPlace-Cylinder-G129-Inspire-Joint}"
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
Starting G1 simulation XR teleop.

Terminal 1 (this script launches sim in background):
  sim_main.py task=${TASK}

Terminal 2 (run manually after sim is ready):
  cd ${REPO_ROOT}
  ./run_xr_teleop.sh

Or without conda:
  ${TV_PYTHON} ${XR_TELEOP_ROOT}/teleop/teleop_hand_and_arm.py \\
    --arm=G1_29 \\
    --ee=inspire_dfx \\
    --sim \\
    --display-mode=immersive \\
    --img-server-ip ${HOST_IP}

Vision Pro browser:
  https://${HOST_IP}:8012/?ws=wss://${HOST_IP}:8012
  -> click Virtual Reality
  -> in terminal press r to start teleop

IMPORTANT:
  - First startup can take 2-5 minutes. Do NOT Ctrl+C while loading.
  - Wait until Terminal 1 prints ALL of these:
      ========= create dds success =========
      ========= start controller success =========
      ***  Please left-click on the Sim window to activate rendering. ***
  - Only then click the Isaac window once, and start Terminal 2.
  - img-server-ip must be this PC IP (${HOST_IP}), NOT a Vision Pro room code

Note: this mode shows robot first-person view, not a full-body mesh in AR.
For full-body G1 in Vision Pro, use ./run_g1_vr_view.sh instead.
EOF

cd "${UNITREE_SIM_ROOT}"
exec "${SIM_PYTHON}" "${SCRIPT_DIR}/launch_sim_main.py" \
  --device "${DEVICE}" \
  --enable_cameras \
  --task "${TASK}" \
  --enable_inspire_dds \
  --robot_type g129 \
  --kit_args "${ISAAC_KIT_ARGS}"
