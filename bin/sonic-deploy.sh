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
TENSORRT_ROOT_DEFAULT="${TENSORRT_ROOT:-/opt/TensorRT}"
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

if [[ -d "${UNITREE_DDS_LIB_DIR}" ]]; then
  _ld="${LD_LIBRARY_PATH:-}"
  _ld="$(echo "${_ld}" | tr ':' '\n' | grep -vF "${UNITREE_DDS_LIB_DIR}" | paste -sd: - || true)"
  export LD_LIBRARY_PATH="${UNITREE_DDS_LIB_DIR}${_ld:+:${_ld}}"
fi

if [[ -z "${TensorRT_ROOT:-}" && -d "${TENSORRT_ROOT_DEFAULT}" ]]; then
  export TensorRT_ROOT="${TENSORRT_ROOT_DEFAULT}"
  export LD_LIBRARY_PATH="${TensorRT_ROOT}/lib:${LD_LIBRARY_PATH:-}"
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
