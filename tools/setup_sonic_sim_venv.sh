#!/usr/bin/env bash
set -euo pipefail

# User-local SONIC MuJoCo sim environment.
#
# Shared GR00T .venv_sim on /mnt/newssd is often broken (another user's dead uv symlink)
# or not writable. This installs deps into:
#   ~/.config/visionpro-g1-inspire-teleop/venv_sonic_sim
# and runs gear_sonic via PYTHONPATH (no editable install into GR00T).
#
# Usage: ./tools/setup_sonic_sim_venv.sh

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GR00T_ROOT="${GR00T_ROOT:-/mnt/newssd/GR00T-WholeBodyControl}"
VENV_DIR="${SONIC_VENV_SIM:-${HOME}/.config/visionpro-g1-inspire-teleop/venv_sonic_sim}"
BASE_PYTHON="${SONIC_BASE_PYTHON:-/mnt/newssd/conda_envs/inspire_clean/bin/python}"
SDK2_PATH="${GR00T_ROOT}/external_dependencies/unitree_sdk2_python"

if [[ ! -d "${GR00T_ROOT}/gear_sonic" ]]; then
  echo "GR00T repo not found: ${GR00T_ROOT}"
  exit 1
fi

if [[ ! -x "${BASE_PYTHON}" ]]; then
  echo "Base Python not found: ${BASE_PYTHON}"
  exit 1
fi

mkdir -p "$(dirname "${VENV_DIR}")"

if [[ ! -x "${VENV_DIR}/bin/python" ]]; then
  echo "[setup_sonic_sim] Creating venv at ${VENV_DIR}"
  "${BASE_PYTHON}" -m venv "${VENV_DIR}"
fi

# shellcheck disable=SC1091
source "${VENV_DIR}/bin/activate"
python -m pip install -U pip wheel setuptools

echo "[setup_sonic_sim] Installing sim runtime deps..."
pip install \
  "numpy==1.26.4" "scipy==1.15.3" torch joblib tqdm easydict loguru \
  tyro mujoco pin pyyaml pyzmq msgpack msgpack-numpy opencv-python \
  "cyclonedds==0.10.2"

export PYTHONPATH="${GR00T_ROOT}:${SDK2_PATH}"
python -c "
import tyro, mujoco, gear_sonic
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
print('OK gear_sonic=', gear_sonic.__file__)
"

cat <<EOF

SONIC sim venv ready:
  ${VENV_DIR}

Run:
  cd ${REPO_ROOT}
  ./bin/sonic-sim.sh

EOF
