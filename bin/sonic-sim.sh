#!/usr/bin/env bash
set -euo pipefail

# Terminal 1 — SONIC MuJoCo sim loop (sim2sim).
# Pair with: bin/sonic-deploy.sh + bin/sonic-teleop.sh

source "$(dirname "$0")/_common.sh"
g1_teleop_load_env

GR00T_ROOT="${GR00T_ROOT:-/mnt/newssd/GR00T-WholeBodyControl}"
USER_VENV="${HOME}/.config/visionpro-g1-inspire-teleop/venv_sonic_sim"
SDK2_PATH="${GR00T_ROOT}/external_dependencies/unitree_sdk2_python"
VENV_SIM="${SONIC_VENV_SIM:-${USER_VENV}}"

_works() {
  local py="$1"
  [[ -x "${py}" ]] && PYTHONPATH="${GR00T_ROOT}:${SDK2_PATH}" "${py}" -c "import tyro, gear_sonic" >/dev/null 2>&1
}

if ! _works "${VENV_SIM}/bin/python"; then
  if _works "${GR00T_ROOT}/.venv_sim/bin/python"; then
    VENV_SIM="${GR00T_ROOT}/.venv_sim"
  else
    cat <<EOF
No working SONIC sim Python found.

Fix once:
  cd ${G1_TELEOP_ROOT}
  ./tools/setup_sonic_sim_venv.sh
  ./bin/sonic-sim.sh
EOF
    exit 1
  fi
fi

if [[ ! -d "${GR00T_ROOT}" ]]; then
  echo "GR00T-WholeBodyControl not found at ${GR00T_ROOT}"
  echo "Set GR00T_ROOT in .env (see .env.example)."
  exit 1
fi

export PYTHONPATH="${GR00T_ROOT}:${SDK2_PATH}${PYTHONPATH:+:${PYTHONPATH}}"

UNITREE_DDS_LIB_DIR="${GR00T_ROOT}/gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/$(uname -m)"
if [[ -d "${UNITREE_DDS_LIB_DIR}" ]]; then
  _ld="${LD_LIBRARY_PATH:-}"
  _ld="$(echo "${_ld}" | tr ':' '\n' | grep -vF "${UNITREE_DDS_LIB_DIR}" | paste -sd: - || true)"
  export LD_LIBRARY_PATH="${UNITREE_DDS_LIB_DIR}${_ld:+:${_ld}}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE

cat <<EOF
SONIC Terminal 1 — MuJoCo sim
  GR00T_ROOT: ${GR00T_ROOT}
  Python: ${VENV_SIM}/bin/python
  Next: bin/sonic-deploy.sh  |  bin/sonic-teleop.sh <VP_IP>
  MuJoCo: press 9 to toggle elastic band
EOF

FPV_ARGS=()
if [[ "${SONIC_ENABLE_FPV:-1}" != "0" ]]; then
  FPV_ARGS=(--enable-offscreen --enable-image-publish)
  echo "  Camera publish: ON (ZMQ :5555 ego_view)"
fi

g1_teleop_prepare_pythonpath
cd "${GR00T_ROOT}"
exec "${VENV_SIM}/bin/python" "${G1_TELEOP_ROOT}/tools/sonic_run_sim_loop.py" "${FPV_ARGS[@]}" "$@"
