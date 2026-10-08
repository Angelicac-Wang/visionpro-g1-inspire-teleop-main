#!/usr/bin/env bash
# One-person teleop start on the Jetson: launches the AVP bridge in tmux and
# runs F -> ] -> S -> T automatically, retrying F/S if calibration fails.
#
# Usage (on the Jetson, with sim/deploy already running):
#   ./solo_teleop.sh 192.168.2.5 --no-mujoco-fpv
# This disables headset video only; simulated fingers remain enabled.
# Add --no-inspire-hand-sim to disable simulated finger commands (e.g. real robot).
# Then put on the Vision Pro within WAIT_PUTON seconds and stand still for F.
# After ] the script waits: press 9 in the MuJoCo window, then Enter here.
# Then stand still again in the calibration pose for S; T follows automatically.
#
# Watch / take over:   tmux attach -t tele   (detach: Ctrl-b then d)
# Full bridge log:     /tmp/solo_bridge.log  (image-staleness warnings filtered out)
# Pause / stop:        tmux send-keys -t tele p    |    tmux send-keys -t tele o
#
# Tunables (env vars): WAIT_PUTON=6  BALANCE_SEC=6  REPO=~/projects/visionpro-g1-inspire-teleop-main

set -u

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <VISION_PRO_IP> [extra bridge args, e.g. --no-mujoco-fpv]"
  exit 1
fi

AVP_IP="$1"
shift
SESSION=tele
REPO="${REPO:-$HOME/projects/visionpro-g1-inspire-teleop-main}"
WAIT_PUTON="${WAIT_PUTON:-6}"
BALANCE_SEC="${BALANCE_SEC:-6}"
TRIES=3
LOG="${LOG:-/tmp/solo_bridge.log}"

log() { echo "[solo $(date +%T)] $*"; }

# Number of times a line matching $1 appears in the bridge output so far.
# Read from a log file: the FPV image warnings flood the tmux scrollback.
count() { grep -c -- "$1" "$LOG" 2>/dev/null || true; }

# Wait up to $2 seconds for a new line matching $1 (beyond count $3).
wait_new() {
  local pattern="$1" timeout="$2" before="$3" i
  for ((i = 0; i < timeout * 2; i++)); do
    (( $(count "$pattern") > before )) && return 0
    sleep 0.5
  done
  return 1
}

# Press a calibration key and retry while the bridge reports FAILED.
calibrate() {
  local key="$1" done_pattern="$2" attempt done_before fail_before
  for ((attempt = 1; attempt <= TRIES; attempt++)); do
    done_before=$(count "$done_pattern")
    fail_before=$(count "Calibration FAILED")
    log "pressing $key (attempt $attempt/$TRIES) - hold still"
    tmux send-keys -t "$SESSION" "$key"
    for ((i = 0; i < 30; i++)); do
      if (( $(count "$done_pattern") > done_before )); then
        log "$key PASS"
        return 0
      fi
      if (( $(count "Calibration FAILED") > fail_before )); then
        log "$key FAILED (moved too much), retrying in 3 s"
        sleep 3
        continue 2
      fi
      sleep 0.5
    done
    log "$key: no answer from bridge within 15 s"
  done
  return 1
}

command -v tmux >/dev/null || { echo "tmux not installed: sudo apt install tmux"; exit 1; }
[[ -d "$REPO" ]] || { echo "Repo not found at $REPO (set REPO=...)"; exit 1; }

tmux kill-session -t "$SESSION" 2>/dev/null
: > "$LOG"
tmux new-session -d -s "$SESSION" -x 220 -y 50 \
  "cd '$REPO' && PYTHONUNBUFFERED=1 ./run_sonic_avp_teleop.sh '$AVP_IP' $* 2>&1 | grep --line-buffered -v 'No new image message' | tee -a '$LOG'; echo; echo 'bridge exited'; bash"
log "bridge started in tmux session '$SESSION', waiting for Vision Pro tracking..."
# Keys pressed before "AVP tracking locked" are ignored by the bridge, so wait for it.
wait_new "Press F (or c) for CALIB_FULL" 90 0 || { log "no Vision Pro tracking after 90 s - is Tracking Streamer open on $AVP_IP? tmux attach -t $SESSION"; exit 1; }
log "tracking locked"

for ((s = WAIT_PUTON; s > 0; s--)); do
  printf '\r[solo] put on the headset and stand still: %2d s ' "$s"
  sleep 1
done
echo

calibrate f "CALIB_FULL done" || { log "F failed $TRIES times - check tracking, then: tmux attach -t $SESSION"; exit 1; }

engage_before=$(count "ENGAGE:")
log "pressing ] (start balance policy)"
tmux send-keys -t "$SESSION" "]"
wait_new "ENGAGE:" 10 "$engage_before" || { log "] not confirmed - check deploy terminal, then: tmux attach -t $SESSION"; exit 1; }
printf '\a'
log "MuJoCo sim: press 9 in the MuJoCo sim window to drop the robot, then come back and press Enter here"
log "(real robot: just press Enter)"
read -r -p "[solo] press Enter after the robot is on the ground... "
for ((s = BALANCE_SEC; s > 0; s--)); do
  printf '\r[solo] waiting for the robot to balance, then back to the calibration pose: %2d s ' "$s"
  sleep 1
done
echo

calibrate s "CALIB_SYNC done" || { log "S failed $TRIES times - tmux attach -t $SESSION"; exit 1; }

log "pressing T - teleop is live"
tmux send-keys -t "$SESSION" "t"
log "done. Pause: tmux send-keys -t $SESSION p   Stop: tmux send-keys -t $SESSION o"
