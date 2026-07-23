#!/usr/bin/env bash
set -euo pipefail

# Terminal 1 for SONIC sim2sim: MuJoCo virtual G1 (NOT Isaac Sim).
# Pair with run_sonic_deploy.sh + run_sonic_avp_teleop.sh

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GR00T_ROOT="${GR00T_ROOT:-/mnt/newssd/GR00T-WholeBodyControl}"
USER_VENV="${HOME}/.config/visionpro-g1-inspire-teleop/venv_sonic_sim"
SDK2_PATH="${GR00T_ROOT}/external_dependencies/unitree_sdk2_python"

if [[ -f "${REPO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/.env"
  set +a
fi

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

Shared GR00T venv is often broken here:
  ${GR00T_ROOT}/.venv_sim/bin/python  (dead symlink to another user's uv Python)

Fix once:
  cd ${REPO_ROOT}
  ./scripts/setup_sonic_sim_venv.sh
  ./run_sonic_sim_loop.sh

If admin can fix the shared venv:
  bash ${GR00T_ROOT}/install_scripts/install_mujoco_sim.sh
EOF
    exit 1
  fi
fi

if [[ ! -d "${GR00T_ROOT}" ]]; then
  echo "GR00T-WholeBodyControl not found at ${GR00T_ROOT}"
  exit 1
fi

export PYTHONPATH="${GR00T_ROOT}:${SDK2_PATH}${PYTHONPATH:+:${PYTHONPATH}}"

# Match deploy DDS setup: sim Python must use the same Unitree CycloneDDS libs on lo.
UNITREE_DDS_LIB_DIR="${GR00T_ROOT}/gear_sonic_deploy/thirdparty/unitree_sdk2/thirdparty/lib/$(uname -m)"
if [[ -d "${UNITREE_DDS_LIB_DIR}" ]]; then
  _ld="${LD_LIBRARY_PATH:-}"
  _ld="$(echo "${_ld}" | tr ':' '\n' | grep -vF "${UNITREE_DDS_LIB_DIR}" | paste -sd: - || true)"
  export LD_LIBRARY_PATH="${UNITREE_DDS_LIB_DIR}${_ld:+:${_ld}}"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE

cat <<EOF
SONIC Terminal 1 — MuJoCo sim loop
  Repo: ${GR00T_ROOT}
  Python: ${VENV_SIM}/bin/python
  Next: Terminal 2 -> ./run_sonic_deploy.sh
        Terminal 3 -> ./run_sonic_avp_teleop.sh <VP_IP>
  MuJoCo tip: press 9 to toggle elastic band (helps robot settle on floor)
  FPV: head_camera streams to Vision Pro when Terminal 3 uses --enable-mujoco-fpv (default)
EOF

FPV_ARGS=()
if [[ "${SONIC_ENABLE_FPV:-1}" != "0" ]]; then
  FPV_ARGS=(--enable-offscreen --enable-image-publish)
  echo "  Camera publish: ON (ZMQ :5555 ego_view / head_camera)"
fi

cd "${GR00T_ROOT}"
exec "${VENV_SIM}/bin/python" "${REPO_ROOT}/scripts/sonic_run_sim_loop.py" "${FPV_ARGS[@]}" "$@"
