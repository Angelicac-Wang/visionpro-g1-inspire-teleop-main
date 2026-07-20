#!/usr/bin/env bash
# Step 2: AVP controls arms, keyboard controls legs (recommended on Vision Pro Safari).
# Run this AFTER ./run_sim_wholebody.sh is up in another terminal.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cat <<EOF
=== Whole-body Step 2 (recommended for Vision Pro Safari) ===

Terminal 1 (already running):
  ./run_sim_wholebody.sh

Terminal 2 (this script — AVP arm teleop):
  ./run_xr_teleop.sh
  AVP Safari -> https://YOUR_PC_IP:8012/?ws=wss://YOUR_PC_IP:8012 -> Virtual Reality
  Press r in Terminal 2 when ready.

Terminal 3 (keyboard leg control — open a NEW terminal):
  cd ${REPO_ROOT}
  ./scripts/wholebody_walk_test.py
  Click Terminal 3, HOLD w/a/z/x to walk. Watch Isaac window.

Why not head-loco on AVP Safari?
  Safari WebXR often does NOT stream head/hand pose back to the PC (det=0).
  VR hand lines can be local-only. Keyboard legs always work (Step 1 proved it).

EOF
exec "${REPO_ROOT}/run_xr_teleop.sh"
