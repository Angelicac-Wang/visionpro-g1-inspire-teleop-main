#!/usr/bin/env bash
# Build SONIC on Ubuntu 22.04 / Jetson arm64 using project-local dependencies.
# Runtime libraries must already exist in sonic_libs/lib (copy_sonic_libs.sh).
# Usage: bash build_sonic_deploy.sh
# Optional: JOBS=4 (default). No sudo, SSH, or robot startup is performed.
set -euo pipefail

PROJ="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$PROJ/sonic_libs"
SYSROOT="$ROOT/sysroot"
DEPLOY="$PROJ/GR00T-WholeBodyControl/gear_sonic_deploy"
BUILD="$DEPLOY/build-agx-orin"
ORT="$PROJ/jetson-teleop/onnxruntime-linux-aarch64-1.16.3"
BIN="$DEPLOY/target/release/g1_deploy_onnx_ref"
JOBS="${JOBS:-4}"
TRT_VERSION=10.7.0.23

die() { echo "ERROR: $*" >&2; exit 1; }
[[ "$(uname -m)" == aarch64 ]] || die "This script requires Jetson arm64."
[[ "$JOBS" =~ ^[1-9][0-9]*$ ]] || die "JOBS must be a positive integer."
for tool in apt-get dpkg-deb curl g++ make python3; do
  command -v "$tool" >/dev/null || die "Required tool not found: $tool"
done
for path in "$DEPLOY/CMakeLists.txt" "$ORT/include/onnxruntime_cxx_api.h" \
  "$ROOT/lib/libnvinfer.so.10.7.0" "$ROOT/lib/libnvonnxparser.so.10.7.0" \
  "$ROOT/lib/libcudart.so.12" "$ROOT/lib/libcudla.so.1"; do
  [[ -f "$path" ]] || die "Missing $path; run copy_sonic_libs.sh for runtime libraries first."
done
mkdir -p "$ROOT/debs" "$SYSROOT" "$ROOT/backups"

echo "Downloading development packages into $ROOT/debs ..."
(
  cd "$ROOT/debs"
  apt-get download cmake cmake-data librhash0 libeigen3-dev libzmq3-dev \
    libmsgpack-dev nlohmann-json3-dev libgtest-dev googletest zlib1g-dev \
    cuda-cudart-dev-12-6 cuda-cccl-12-6 cuda-driver-dev-12-6 cuda-crt-12-6
  # Match the existing 10.7 runtime, not JetPack's default 10.3 headers.
  for package in libnvinfer-headers-dev libnvonnxparsers-dev; do
    archive="${package}_${TRT_VERSION}-1+cuda12.6_arm64.deb"
    if [[ ! -f "$archive" ]]; then
      curl -fSL --retry 3 --connect-timeout 15 --max-time 180 \
        "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/$archive" \
        -o "$archive.part"
      mv -- "$archive.part" "$archive"
    fi
  done
  for archive in ./*.deb; do dpkg-deb -x "$archive" "$SYSROOT"; done
)

# Complete development symlinks using the already-working runtime libraries.
ln -sfnT sysroot/usr/include/aarch64-linux-gnu "$ROOT/include"
ln -sfn libcudart.so.12 "$ROOT/lib/libcudart.so"
ln -sfn libcudla.so.1 "$ROOT/lib/libcudla.so"
ln -sfn /usr/lib/aarch64-linux-gnu/libzmq.so.5 "$SYSROOT/usr/lib/aarch64-linux-gnu/libzmq.so"
CUDA_ROOT="$SYSROOT/usr/local/cuda-12.6"
CUDA_LIB="$CUDA_ROOT/targets/aarch64-linux/lib"
ln -sfn "$ROOT/lib/libcudart.so.12" "$CUDA_LIB/libcudart.so.12"
ln -sfn "$ROOT/lib/libcudla.so.1" "$CUDA_LIB/libcudla.so"

export TensorRT_ROOT="$ROOT" CUDAToolkit_ROOT="$CUDA_ROOT" onnxruntime_ROOT="$ORT" HAS_ROS2=0
DDS_LIB="$DEPLOY/thirdparty/unitree_sdk2/thirdparty/lib/aarch64"
export LD_LIBRARY_PATH="$DDS_LIB:$ROOT/lib:$ORT/lib:$SYSROOT/usr/lib/aarch64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
CMAKE="$SYSROOT/usr/bin/cmake"

# Upstream hard-codes target/release. Stage build outputs so a failed link or
# check cannot replace the user's existing executable.
cat > "$ROOT/stage-output.cmake" <<'CMAKE'
function(sonic_stage_outputs)
  foreach(target g1_deploy_onnx_ref run_tests)
    if(TARGET ${target})
      set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
    endif()
  endforeach()
endfunction()
cmake_language(DEFER CALL sonic_stage_outputs)
CMAKE

"$CMAKE" -S "$DEPLOY" -B "$BUILD" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$SYSROOT/usr" \
  -DCMAKE_CXX_FLAGS="-I$SYSROOT/usr/include -I$ROOT/include" \
  -DCMAKE_EXE_LINKER_FLAGS="-Wl,-rpath-link,$ROOT/lib -Wl,-rpath-link,/usr/lib/aarch64-linux-gnu/nvidia" \
  -DCMAKE_PROJECT_g1_deploy_INCLUDE="$ROOT/stage-output.cmake" \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$SYSROOT/usr/src/googletest" \
  2>&1 | tee "$ROOT/configure.log"
"$CMAKE" --build "$BUILD" --target g1_deploy_onnx_ref run_tests --parallel "$JOBS" \
  2>&1 | tee "$ROOT/build.log"

ldd "$BUILD/bin/g1_deploy_onnx_ref" > "$ROOT/ldd.log"
if grep -q 'not found' "$ROOT/ldd.log"; then
  cat "$ROOT/ldd.log"
  die "New binary has unresolved libraries; previous executable kept."
fi
# No positional arguments: prints usage and returns before DDS/robot setup.
smoke_status=0
"$BUILD/bin/g1_deploy_onnx_ref" > "$ROOT/smoke.log" 2>&1 || smoke_status=$?
[[ "$smoke_status" == 0 ]] && grep -q 'Usage:' "$ROOT/smoke.log" \
  || die "Usage smoke check failed; see $ROOT/smoke.log."

if [[ -d "$DEPLOY/reference/bones_072925_test" ]]; then
  (cd "$DEPLOY" && "$BUILD/bin/run_tests") 2>&1 | tee "$ROOT/test.log"
else
  echo "SKIP FK test: upstream reference/bones_072925_test fixture is absent." | tee "$ROOT/test.log"
fi

if [[ -f "$BIN" ]]; then
  backup="$(mktemp "$ROOT/backups/g1_deploy_onnx_ref.XXXXXXXX")"
  cp -p -- "$BIN" "$backup"
  echo "Previous binary backed up to $backup"
fi
install -m 755 "$BUILD/bin/g1_deploy_onnx_ref" "$BIN.new"
mv -f -- "$BIN.new" "$BIN"
echo "BUILD OK: $BIN"
echo "Restart deploy to load the new binary, then repeat F -> ] -> S -> T."
