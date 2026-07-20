#!/usr/bin/env bash
set -euo pipefail

# Terminal 2 for SONIC sim2sim: C++ policy + ZMQ manager subscriber (port 5556).
#
# Shared GR00T tree under /mnt/newssd is often read-only for other users
# (build/ owned by another account). When target/release/g1_deploy_onnx_ref
# already exists, we run it directly and skip deploy.sh -> just build.
#
# Usage:
#   ./run_sonic_deploy.sh          # sim (MuJoCo, interface lo)
#   ./run_sonic_deploy.sh real     # real robot (clear safety zone first)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GR00T_ROOT="${GR00T_ROOT:-/mnt/newssd/GR00T-WholeBodyControl}"
DEPLOY_DIR="${GR00T_ROOT}/gear_sonic_deploy"
BINARY="${DEPLOY_DIR}/target/release/g1_deploy_onnx_ref"
UNITREE_DDS_LIB_DIR="${DEPLOY_DIR}/thirdparty/unitree_sdk2/thirdparty/lib/$(uname -m)"
TENSORRT_ROOT_DEFAULT="/mnt/newssd/TensorRT/TensorRT-10.13.0.35"
TARGET_MODE="${1:-sim}"

if [[ "${TARGET_MODE}" != "sim" && "${TARGET_MODE}" != "real" ]]; then
  echo "Usage: $0 [sim|real]"
  exit 1
fi

if [[ -f "${REPO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/.env"
  set +a
fi

if [[ ! -d "${DEPLOY_DIR}" ]]; then
  echo "gear_sonic_deploy not found: ${DEPLOY_DIR}"
  exit 1
fi

cd "${DEPLOY_DIR}"
# setup_env.sh has pipelines (grep | head) that exit 1 when TensorRT is unset;
# with set -e that aborts this wrapper right after "FastRTPS ... configured".
set +eu
# shellcheck disable=SC1091
source scripts/setup_env.sh
set -eu

# setup_sonic_runtime + setup_env can leave ROS libs ahead of Unitree CycloneDDS:
# setup_env's prepend_ld_library_path skips dirs already in LD_LIBRARY_PATH, so
# libddsc/libddscxx can come from different trees and crash during DDS init.
if [[ -d "${UNITREE_DDS_LIB_DIR}" ]]; then
  _ld="${LD_LIBRARY_PATH:-}"
  _ld="$(echo "${_ld}" | tr ':' '\n' | grep -vF "${UNITREE_DDS_LIB_DIR}" | paste -sd: - || true)"
  export LD_LIBRARY_PATH="${UNITREE_DDS_LIB_DIR}${_ld:+:${_ld}}"
fi

# Binary links TensorRT from the shared install; default when ~/.bashrc has no export.
if [[ -z "${TensorRT_ROOT:-}" && -d "${TENSORRT_ROOT_DEFAULT}" ]]; then
  export TensorRT_ROOT="${TENSORRT_ROOT_DEFAULT}"
  export LD_LIBRARY_PATH="${TensorRT_ROOT}/lib:${LD_LIBRARY_PATH}"
fi

# zmq_manager sim does not need ROS2; avoid FastRTPS/CycloneDDS cross-talk.
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
    echo "Could not detect robot network interface. Set SONIC_NET_IF, e.g.:"
    echo "  SONIC_NET_IF=enp3s0 ./run_sonic_deploy.sh real"
    exit 1
  fi
  EXTRA=()
fi

cat <<EOF
SONIC Terminal 2 — deploy (${TARGET_MODE})
  ZMQ subscriber: tcp://${ZMQ_HOST}:5556
  Emergency stop in THIS terminal: press O
  Wait for "Init done" before starting Terminal 3.
EOF

if [[ ! -x "${BINARY}" ]]; then
  echo "Prebuilt binary missing: ${BINARY}"
  echo "Ask admin to build once in ${DEPLOY_DIR}, or run (needs write access):"
  echo "  cd ${DEPLOY_DIR} && ./deploy.sh --input-type zmq_manager --zmq-host localhost ${TARGET_MODE}"
  exit 1
fi

echo "Using prebuilt deploy binary (skip just build): ${BINARY}"
echo "Network interface: ${NET_IF}"
echo ""

exec "${BINARY}" "${NET_IF}" "${CHECKPOINT_DECODER}" "${MOTION_DATA}" \
  --obs-config "${OBS_CONFIG}" \
  --encoder-file "${CHECKPOINT_ENCODER}" \
  --planner-file "${PLANNER}" \
  --input-type "${INPUT_TYPE}" \
  --output-type "${OUTPUT_TYPE}" \
  --zmq-host "${ZMQ_HOST}" \
  "${EXTRA[@]}"
