#!/usr/bin/env bash
set -euo pipefail

# Terminal 3 — AVP → SONIC bridge (whole-body teleop + head locomotion).

source "$(dirname "$0")/_common.sh"
g1_teleop_load_env
g1_teleop_prepare_pythonpath

PYTHON="$(g1_teleop_python)"
g1_teleop_require_python "${PYTHON}"
AVP_ENDPOINT="${1:-}"

if [[ -z "${AVP_ENDPOINT}" ]]; then
  echo "Usage: $0 <vision_pro_ip_or_room_code> [extra bridge args...]"
  exit 1
fi
shift

cat <<EOF
SONIC Terminal 3 — AVP bridge
  Endpoint: ${AVP_ENDPOINT}
  Operator: F -> ] -> S -> T  (see docs/OPERATIONS.md)
  Emergency stop: o here, O in deploy terminal
EOF

cd "${G1_TELEOP_ROOT}"
exec "${PYTHON}" scripts/g1_avp_sonic_teleop.py \
  --avp-endpoint "${AVP_ENDPOINT}" \
  --print-debug \
  "$@"
