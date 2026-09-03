#!/usr/bin/env bash
set -euo pipefail

# Terminal 2 — SONIC policy deploy (ZMQ subscriber on :5556).
# Usage: bin/sonic-deploy.sh [sim|real]

source "$(dirname "$0")/_common.sh"
g1_teleop_load_env

GR00T_ROOT="${GR00T_ROOT:-/mnt/newssd/GR00T-WholeBodyControl}"
DEPLOY_DIR="${GR00T_ROOT}/gear_sonic_deploy"
BINARY="${DEPLOY_DIR}/target/release/g1_deploy_onnx_ref"
UNITREE_DDS_LIB_DIR="${DEPLOY_DIR}/thirdparty/unitree_sdk2/thirdparty/lib/$(uname -m)"
TENSORRT_CANDIDATES=(
  "${TensorRT_ROOT:-}"
  "/mnt/newssd/TensorRT/TensorRT-10.13.0.35"
  "/opt/TensorRT"
  "${HOME}/TensorRT"
)
TARGET_MODE="${1:-sim}"

if [[ "${TARGET_MODE}" != "sim" && "${TARGET_MODE}" != "real" ]]; then
  echo "Usage: $0 [sim|real]"
  exit 1
fi

if [[ ! -d "${DEPLOY_DIR}" ]]; then
  echo "gear_sonic_deploy not found: ${DEPLOY_DIR}"
  exit 1
fi

cd "${DEPLOY_DIR}"
set +eu
# shellcheck disable=SC1091
source scripts/setup_env.sh
set -eu

g1_teleop_prepend_ld_path() {
  local dir="$1"
  [[ -d "${dir}" ]] || return 0
  local rest
  rest="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -vF "${dir}" | paste -sd: - || true)"
  export LD_LIBRARY_PATH="${dir}${rest:+:${rest}}"
}

g1_teleop_strip_ld_paths() {
  local entry
  for entry in "$@"; do
    LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -vF "${entry}" | paste -sd: - || true)"
    export LD_LIBRARY_PATH
  done
}

if [[ -z "${TensorRT_ROOT:-}" ]]; then
  for candidate in "${TENSORRT_CANDIDATES[@]}"; do
    [[ -n "${candidate}" && -d "${candidate}/lib" ]] || continue
    export TensorRT_ROOT="${candidate}"
    break
  done
fi

# Drop ROS CycloneDDS from the path; deploy uses Unitree's paired libddsc/libddscxx.
g1_teleop_strip_ld_paths "/opt/ros/humble/lib"

if [[ -n "${TensorRT_ROOT:-}" ]]; then
  g1_teleop_prepend_ld_path "${TensorRT_ROOT}/lib"
else
  echo "WARNING: TensorRT_ROOT is not set and no install was auto-detected."
  echo "  Add to .env: TensorRT_ROOT=/mnt/newssd/TensorRT/TensorRT-10.13.0.35"
fi

# Unitree DDS must stay first so libddsc and libddscxx come from the same build.
if [[ -d "${UNITREE_DDS_LIB_DIR}" ]]; then
  g1_teleop_prepend_ld_path "${UNITREE_DDS_LIB_DIR}"
fi

# Preload the NVIDIA driver only. Prepending all of /usr/lib/x86_64-linux-gnu breaks DDS pairing.
NVIDIA_LIBCUDA="/usr/lib/x86_64-linux-gnu/libcuda.so.1"
if [[ -f "${NVIDIA_LIBCUDA}" ]]; then
  export LD_PRELOAD="${NVIDIA_LIBCUDA}${LD_PRELOAD:+:${LD_PRELOAD}}"
fi

if [[ "${TARGET_MODE}" == "sim" ]]; then
  unset RMW_IMPLEMENTATION ROS_LOCALHOST_ONLY FASTRTPS_DEFAULT_PROFILES_FILE
fi

CHECKPOINT_DECODER="policy/release/model_decoder.onnx"
CHECKPOINT_ENCODER="policy/release/model_encoder.onnx"
MOTION_DATA="reference/example/"
OBS_CONFIG="policy/release/observation_config.yaml"
PLANNER="planner/target_vel/V2/planner_sonic.onnx"
INPUT_TYPE="zmq_manager"
OUTPUT_TYPE="all"
ZMQ_HOST="localhost"

if [[ "${TARGET_MODE}" == "sim" ]]; then
  NET_IF="lo"
  EXTRA=(--disable-crc-check)
else
  NET_IF="${SONIC_NET_IF:-}"
  if [[ -z "${NET_IF}" ]]; then
    NET_IF="$(ip -4 route get 192.168.123.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="dev") {print $(i+1); exit}}')"
  fi
  if [[ -z "${NET_IF}" ]]; then
    echo "Set SONIC_NET_IF for real robot, e.g. SONIC_NET_IF=enp3s0 $0 real"
    exit 1
  fi
  EXTRA=()
fi

INIT_ARM_POSE="${SONIC_INIT_ARM_POSE:-forearms-forward}"
POLICY_START_RAMP_SEC="${SONIC_POLICY_START_RAMP_SEC:-0.8}"

if [[ ! -x "${BINARY}" ]]; then
  echo "Prebuilt binary missing: ${BINARY}"
  echo "Build once in ${DEPLOY_DIR} with deploy.sh --input-type zmq_manager --zmq-host localhost ${TARGET_MODE}"
  exit 1
fi

cat <<EOF
SONIC Terminal 2 — deploy (${TARGET_MODE})
  ZMQ: tcp://${ZMQ_HOST}:5556
  Emergency stop: press O in this terminal
EOF

if [[ "${TARGET_MODE}" == "real" ]]; then
  cat <<'EOF'

Real robot preflight (required before deploy can proceed):
  1. Robot hoisted; wait for zero-torque mode (joints move freely by hand).
  2. On Unitree remote: press L2+R2 until debug mode (yellow LED, damping).
  3. PC NIC on 192.168.123.x (SONIC_NET_IF in .env, e.g. enp3s0).
  If you see "Failed to switch to Release Mode", the sport controller is still
  active — repeat step 2 or reboot the robot and re-enter debug mode.

EOF
fi

exec "${BINARY}" "${NET_IF}" "${CHECKPOINT_DECODER}" "${MOTION_DATA}" \
  --obs-config "${OBS_CONFIG}" \
  --encoder-file "${CHECKPOINT_ENCODER}" \
  --planner-file "${PLANNER}" \
  --input-type "${INPUT_TYPE}" \
  --output-type "${OUTPUT_TYPE}" \
  --zmq-host "${ZMQ_HOST}" \
  --init-arm-pose "${INIT_ARM_POSE}" \
  --policy-start-ramp-sec "${POLICY_START_RAMP_SEC}" \
  "${EXTRA[@]}"
