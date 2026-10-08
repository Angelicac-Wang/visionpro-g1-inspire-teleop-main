#!/usr/bin/env bash
# Copy the NVIDIA runtime libraries the SONIC deploy binary needs (CUDA runtime,
# TensorRT, ...) from the old Jetson into ~/projects/sonic_libs/lib on this one,
# and point .env at them. Nothing outside ~/projects is written; no system
# packages are installed.
#
# Usage (on the new Jetson): bash copy_sonic_libs.sh [epcap@192.168.2.68] [--build]
# If runtime libraries are already copied: bash build_sonic_deploy.sh

set -euo pipefail

OLD="epcap@192.168.2.68"
BUILD_AFTER_COPY=false
for arg in "$@"; do
  case "$arg" in
    --build) BUILD_AFTER_COPY=true ;;
    -*) echo "Unknown option: $arg" >&2; exit 1 ;;
    *) OLD="$arg" ;;
  esac
done
PROJ="$HOME/projects"
BIN="$PROJ/GR00T-WholeBodyControl/gear_sonic_deploy/target/release/g1_deploy_onnx_ref"
ROOT="$PROJ/sonic_libs"
DEST="$ROOT/lib"
ENV_FILE="$PROJ/visionpro-g1-inspire-teleop-main/.env"
CM="/tmp/sonic_cm_$$"
SSH_OPTS="-o ControlMaster=auto -o ControlPath=$CM -o ControlPersist=300"

[[ -x "$BIN" ]] || { echo "deploy binary not found: $BIN"; exit 1; }
mkdir -p "$DEST"

missing_here() { LD_LIBRARY_PATH="$DEST" ldd "$BIN" | awk '/not found/{print $1}'; }

echo "Connecting to $OLD (enter the old Jetson's password once)..."
ssh $SSH_OPTS "$OLD" true

# name -> path of every library the binary loads on the old Jetson.
old_map="$(ssh $SSH_OPTS "$OLD" "ldd '$BIN'" | awk '$2=="=>" && $3 ~ /^\// {print $1, $3}')"

fetch_one() {  # one library; follow the symlink so DEST holds the real file
  rsync -aL -e "ssh $SSH_OPTS" "$OLD:$1" "$DEST/" 2>/dev/null || true
}
fetch_family() {  # a glob of siblings; keep their relative symlinks (saves GBs)
  rsync -a -e "ssh $SSH_OPTS" "$OLD:$1" "$DEST/" 2>/dev/null || true
}

# TensorRT also dlopens sibling libraries (builder resources, plugins), so take
# the whole libnvinfer / libnvonnxparser family from wherever it lives.
for name in libnvinfer.so.10 libnvonnxparser.so.10; do
  path="$(awk -v n="$name" '$1==n{print $2}' <<<"$old_map")"
  if [[ -n "$path" ]]; then fetch_family "$(dirname "$path")/${name%%.so*}*.so*"; fi
done

# Then copy whatever ldd still cannot resolve here, until nothing is missing.
for round in 1 2 3 4 5; do
  mapfile -t names < <(missing_here)
  if (( ${#names[@]} == 0 )); then break; fi
  echo "round $round, still missing: ${names[*]}"
  for name in "${names[@]}"; do
    path="$(awk -v n="$name" '$1==n{print $2}' <<<"$old_map")"
    if [[ -z "$path" ]]; then
      echo "  $name is not used on the old Jetson either - cannot copy it"
      continue
    fi
    fetch_one "$path"
  done
done
ssh $SSH_OPTS -O exit "$OLD" 2>/dev/null || true

# Let bin/sonic-deploy.sh find them (it prepends $TensorRT_ROOT/lib to LD_LIBRARY_PATH).
if grep -q '^TensorRT_ROOT=' "$ENV_FILE" 2>/dev/null; then
  sed -i "s|^TensorRT_ROOT=.*|TensorRT_ROOT=$ROOT|" "$ENV_FILE"
else
  echo "TensorRT_ROOT=$ROOT" >> "$ENV_FILE"
fi

du -sh "$DEST"
if [[ -z "$(missing_here)" ]]; then
  echo "ALL FOUND - runtime libraries are ready."
  if "$BUILD_AFTER_COPY"; then
    exec bash "$PROJ/build_sonic_deploy.sh"
  fi
  echo "To rebuild after source changes: bash $PROJ/build_sonic_deploy.sh"
else
  echo "STILL MISSING: $(missing_here | tr '\n' ' ')"
  exit 1
fi
