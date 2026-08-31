#!/usr/bin/env bash
set -euo pipefail

# Capture AVP left-hand open/close calibration for Inspire finger mapping.

source "$(dirname "$0")/_common.sh"
g1_teleop_load_env
g1_teleop_prepare_pythonpath

PYTHON="$(g1_teleop_python)"
g1_teleop_require_python "${PYTHON}"
AVP_ENDPOINT="${1:-}"

if [[ -z "${AVP_ENDPOINT}" ]]; then
  echo "Usage: $0 <vision_pro_ip_or_room_code> [--output PATH] [--sample-seconds SEC]"
  exit 1
fi
shift

OUTPUT="${G1_TELEOP_ROOT}/scripts/visionpro_left_hand_calibration.json"

cat <<EOF
Left-hand calibration
  Endpoint: ${AVP_ENDPOINT}
  Output:   ${OUTPUT}

Steps:
  1. Open left hand naturally -> Enter
  2. Close left hand / fist   -> Enter
EOF

cd "${G1_TELEOP_ROOT}"
exec "${PYTHON}" scripts/calibrate_avp_hand.py \
  --avp-endpoint "${AVP_ENDPOINT}" \
  --side left \
  --output "${OUTPUT}" \
  "$@"
