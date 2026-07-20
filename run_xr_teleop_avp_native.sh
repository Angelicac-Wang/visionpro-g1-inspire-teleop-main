#!/usr/bin/env bash
# XR teleop using the native Vision Pro "Tracking Streamer" app (VisionProTeleop),
# NOT Safari at :8012. Sends real head + hand tracking to the PC.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
AVP_ENDPOINT="${1:-${AVP_ENDPOINT:-${AVP_IP:-}}}"

if [[ -z "${AVP_ENDPOINT}" ]]; then
  cat <<EOF
Usage: $0 <vision_pro_ip_or_room_code> [extra run_xr_teleop args...]

Examples:
  $0 192.168.2.50              # same Wi‑Fi: Vision Pro IP (gRPC tracking)
  $0 MLBS-4109                 # cross-network room code from Tracking Streamer app

Vision Pro setup (Tracking Streamer app — App Store):
  1) Install/open "Tracking Streamer" on Vision Pro
  2) Same Wi‑Fi as PC:
       - App shows your VP IP OR a room code
       - Use the IP:  $0 192.168.x.x
     OR cross-network:
       - Enable Cross-Network in the app
       - Enter the room code shown on VP into this script
  3) Tap Start / connect in the app BEFORE pressing [r] on the PC

Terminal 1 must already be running:
  ./run_sim_wholebody.sh   (walking + arms)
  or ./run_sim_xr_teleop.sh

Optional env:
  XR_HEAD_LOCO=1             head lean/turn -> walk (needs wholebody sim)
  AVP_VIDEO_SIZE=1280x720    sim picture size sent to VP
  AVP_HT_BACKEND=grpc        same-LAN only (default); webrtc for room code

EOF
  exit 1
fi

shift || true

export XR_CLIENT=native
export AVP_ENDPOINT

cat <<EOF
=== Native Tracking Streamer mode ===
  Endpoint: ${AVP_ENDPOINT}
  Safari 8012 is NOT used — no certificate setup needed for teleop.

  After teleop starts, press [r] in THIS terminal when VP app is connected.
  You should see: [avp_native] Tracking live (head det=...)

EOF

exec env XR_CLIENT=native AVP_ENDPOINT="${AVP_ENDPOINT}" \
  "${REPO_ROOT}/run_xr_teleop.sh" "$@"
