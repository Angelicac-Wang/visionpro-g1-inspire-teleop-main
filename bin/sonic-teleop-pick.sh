#!/usr/bin/env bash
set -euo pipefail

# Terminal 3 — pick-up teleop (Inspire finger sim + head walk).

source "$(dirname "$0")/_common.sh"
g1_teleop_load_env
g1_teleop_prepare_pythonpath

PYTHON="$(g1_teleop_python)"
AVP_ENDPOINT="${1:-}"

if [[ -z "${AVP_ENDPOINT}" ]]; then
  echo "Usage: $0 <vision_pro_ip_or_room_code> [extra bridge args...]"
  exit 1
fi
shift

cat <<EOF
SONIC Terminal 3 — pick-up teleop
  Endpoint: ${AVP_ENDPOINT}
  Flow: F -> ] -> S -> T  (pinch to grasp in MuJoCo)
EOF

cd "${G1_TELEOP_ROOT}"
exec "${PYTHON}" scripts/g1_avp_sonic_teleop_pick.py \
  --avp-endpoint "${AVP_ENDPOINT}" \
  --print-debug \
  "$@"
