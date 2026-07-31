#!/usr/bin/env bash
set -euo pipefail

# Terminal 2 helper for xr_teleoperate simulation mode.
# No conda needed: this script uses the tv env python directly.

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TV_PYTHON="${TV_PYTHON:-/mnt/newssd/conda_envs/tv/bin/python}"
XR_TELEOP_ROOT="${XR_TELEOP_ROOT:-/mnt/newssd/unitree_sim_isaaclab/xr_teleoperate}"
TELEOP_DIR="${XR_TELEOP_ROOT}/teleop"
SCRIPT_DIR="${REPO_ROOT}/scripts"
USER_CERT_DIR="${HOME}/.config/xr_teleoperate"
SOURCE_CERT_DIR="${TELEOP_DIR}/televuer"

# webrtc = 60001 stream (confirmed working on your AVP). We now place the video
#          plane on BOTH Vision Pro eye layers (1=left, 2=right) so both eyes show it.
# zmq = JPEG over 8012 as a duplicated stereo image (fallback).
XR_VIDEO_MODE="${XR_VIDEO_MODE:-webrtc}"
DISPLAY_MODE="${DISPLAY_MODE:-immersive}"
# Head-tracking -> whole-body walking. Set XR_HEAD_LOCO=1 to let AVP head lean/turn
# drive the legs. Only meaningful with a Wholebody sim task (run_sim_wholebody.sh).
XR_HEAD_LOCO="${XR_HEAD_LOCO:-0}"
export XR_HEAD_LOCO

HOST_IP="${HOST_IP:-$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1;i<=NF;i++) if ($i=="src") {print $(i+1); exit}}')}"

if [[ -f "${REPO_ROOT}/.env" ]]; then
  set -a
  # shellcheck disable=SC1091
  source "${REPO_ROOT}/.env"
  set +a
fi

mkdir -p "${USER_CERT_DIR}"
if [[ ! -r "${USER_CERT_DIR}/key.pem" ]]; then
  if [[ -r "${SOURCE_CERT_DIR}/key.pem" ]]; then
    cp "${SOURCE_CERT_DIR}/cert.pem" "${SOURCE_CERT_DIR}/key.pem" "${USER_CERT_DIR}/"
    chmod 600 "${USER_CERT_DIR}/key.pem"
  else
    echo "SSL key not readable at ${SOURCE_CERT_DIR}/key.pem"
    echo "Run once with sudo:"
    echo "  sudo cp ${SOURCE_CERT_DIR}/cert.pem ${SOURCE_CERT_DIR}/key.pem ${USER_CERT_DIR}/"
    echo "  sudo chown $(id -un):$(id -gn) ${USER_CERT_DIR}/cert.pem ${USER_CERT_DIR}/key.pem"
    exit 1
  fi
fi

export XR_TELEOP_CERT="${USER_CERT_DIR}/cert.pem"
export XR_TELEOP_KEY="${USER_CERT_DIR}/key.pem"

if [[ "${XR_VIDEO_MODE}" == "zmq" ]]; then
  export XR_FORCE_ZMQ=1
else
  unset XR_FORCE_ZMQ
fi

if [[ -z "${HOST_IP}" ]]; then
  echo "Could not detect host IP. Set HOST_IP manually, e.g.:"
  echo "  HOST_IP=192.168.2.32 $0"
  exit 1
fi

cat <<EOF
Starting XR teleop (sim mode).
Video mode: ${XR_VIDEO_MODE}  display mode: ${DISPLAY_MODE}  head-loco: ${XR_HEAD_LOCO}

$( [[ "${XR_HEAD_LOCO}" == "1" ]] && cat <<'HL'
HEAD-LOCO IS ON (whole-body walking via head tracking):
  - Terminal 1 MUST be a Wholebody task (./run_sim_wholebody.sh).
  - Default mode=velocity (work-master): move head to walk, stop when head stops.
  - Head turn drives robot rotation; uses robot-frame motion derivative.
  - Press [b] in this terminal to reset sim if the robot falls.
  - Tune speed: HEAD_LOCO_V_LIN=0.35 HEAD_LOCO_SMOOTH=0.12 XR_HEAD_LOCO=1 ...
  - Wrong direction: HEAD_LOCO_SIGN_X=-1 HEAD_LOCO_SIGN_Y=-1 XR_HEAD_LOCO=1 ...
  - Legacy P-control: HEAD_LOCO_MODE=displacement XR_HEAD_LOCO=1 ...
HL
)

IMPORTANT:
  1) Terminal 1 must already show:
       ========= create dds success =========
       ========= start controller success =========
  2) You must have clicked inside the Isaac window once
  3) img-server-ip is THIS PC's IP: ${HOST_IP}
     Do NOT use Vision Pro room codes like MLBS-xxxx here

Vision Pro certificate (8012 uses SAME cert as 60001 — no separate 8012 cert):
  Install rootCA once: ./scripts/setup_vp_certs.sh -> AirDrop rootCA.pem to VP
  Settings -> General -> About -> Certificate Trust Settings -> trust xr-teleoperate-local
  If hand tracking lines appear in VR, 8012 cert/WSS is already OK.

Vision Pro setup (do this BEFORE pressing r):

  Step A - warm up + trust the WebRTC stream (the sim video source):
    https://${HOST_IP}:60001
    -> Advanced -> Proceed -> Start -> confirm you SEE the sim camera
    -> then CLOSE this 60001 tab

  Step B - open XR teleop:
    https://${HOST_IP}:8012/?ws=wss://${HOST_IP}:8012
    -> Advanced -> Proceed -> Virtual Reality

  Step C - back on this PC terminal, press r to start teleop

Video mode notes:
  - webrtc (default): sim video plane placed on BOTH eye layers (1 and 2).
      Step A is required so the 60001 cert/stream is ready.
  - zmq fallback: XR_VIDEO_MODE=zmq ./run_xr_teleop.sh  (stereo JPEG over 8012)
  - ego window instead of full screen:  DISPLAY_MODE=ego ./run_xr_teleop.sh

What you should see:
  - grid + colored hand lines = WebXR + hand tracking OK
  - sim video in BOTH eyes = eye-layer fix working
  - Isaac window on PC: robot moves when you press r
EOF

exec "${TV_PYTHON}" "${SCRIPT_DIR}/launch_teleop_hand_and_arm.py" \
  --arm=G1_29 \
  --ee=inspire_dfx \
  --sim \
  --display-mode="${DISPLAY_MODE}" \
  --img-server-ip "${HOST_IP}"
