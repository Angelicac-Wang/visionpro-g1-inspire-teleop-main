#!/usr/bin/env bash
set -euo pipefail

# Start left (topic l) and/or right (topic r) Inspire hand drivers as separate processes.
# Official G1 dual-hand IPs (see inspire_hand_sdk/example/Vision_driver_double.py):
#   left  hand: 192.168.123.210  -> rt/inspire_hand/ctrl/l
#   right hand: 192.168.123.211  -> rt/inspire_hand/ctrl/r

source "$(dirname "$0")/_common.sh"
g1_teleop_load_env
g1_teleop_prepare_pythonpath

PYTHON="$(g1_teleop_python)"
g1_teleop_require_python "${PYTHON}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/matplotlib-visionpro-g1}"

LEFT_IP="${INSPIRE_HAND_LEFT_IP:-192.168.123.210}"
RIGHT_IP="${INSPIRE_HAND_RIGHT_IP:-192.168.123.211}"
DDS_NETWORK="${INSPIRE_HAND_DDS_NETWORK:-${SONIC_NET_IF:-}}"
SIDES="both"
LEFT_DEVICE_ID=1
RIGHT_DEVICE_ID=1

usage() {
  cat <<EOF
Usage: $0 [options]

Options:
  --left-ip IP           Left hand Modbus IP (topic l). Default: ${LEFT_IP}
  --right-ip IP          Right hand Modbus IP (topic r). Default: ${RIGHT_IP}
  --dds-network IFACE    DDS NIC for both drivers, e.g. enp3s0
  --left-device-id ID    Left Modbus device ID. Default: 1
  --right-device-id ID   Right Modbus device ID. Default: 1
  --sides both|l|r       Which drivers to start. Default: both

Teleop pairing (SONIC):
  AVP left hand  -> rt/inspire_hand/ctrl/l
  AVP right hand -> rt/inspire_hand/ctrl/r
  Use: --enable-inspire-hand-dds --hand-dds-sides both --hand-dds-network enp3s0

Note: driver Hz logs mean Modbus TCP is up, not that DDS commands are arriving.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --left-ip)
      LEFT_IP="$2"
      shift 2
      ;;
    --right-ip)
      RIGHT_IP="$2"
      shift 2
      ;;
    --dds-network)
      DDS_NETWORK="$2"
      shift 2
      ;;
    --left-device-id)
      LEFT_DEVICE_ID="$2"
      shift 2
      ;;
    --right-device-id)
      RIGHT_DEVICE_ID="$2"
      shift 2
      ;;
    --sides)
      SIDES="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

DRIVER="${G1_TELEOP_ROOT}/scripts/Headless_driver_r.py"
PIDS=()
DDS_ARGS=()
if [[ -n "${DDS_NETWORK}" ]]; then
  DDS_ARGS=(--dds-network "${DDS_NETWORK}")
fi

cleanup() {
  for pid in "${PIDS[@]:-}"; do
    kill "${pid}" 2>/dev/null || true
  done
}
trap cleanup INT TERM EXIT

start_side() {
  local side="$1"
  local ip="$2"
  local device_id="$3"
  local label="left"
  if [[ "${side}" == "r" ]]; then
    label="right"
  fi
  echo "Starting ${label} hand driver: ip=${ip} topic=rt/inspire_hand/ctrl/${side} device_id=${device_id}"
  "${PYTHON}" "${DRIVER}" --lr "${side}" --ip "${ip}" --device-id "${device_id}" "${DDS_ARGS[@]}" &
  PIDS+=("$!")
}

if [[ "${SIDES}" == "both" || "${SIDES}" == "l" ]]; then
  start_side l "${LEFT_IP}" "${LEFT_DEVICE_ID}"
fi
if [[ "${SIDES}" == "both" || "${SIDES}" == "r" ]]; then
  start_side r "${RIGHT_IP}" "${RIGHT_DEVICE_ID}"
fi

echo "Inspire hand drivers running (PIDs: ${PIDS[*]}). Press Ctrl+C to stop."

wait
