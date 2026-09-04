#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${G1_BRIDGE_BUILD_DIR:-${ROOT_DIR}/build}"
BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

cmake_args=(
  -S "${ROOT_DIR}"
  -B "${BUILD_DIR}"
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"
)
if [[ -n "${ONNXRUNTIME_ROOT:-}" ]]; then
  cmake_args+=( -DONNXRUNTIME_ROOT="${ONNXRUNTIME_ROOT}" )
fi
cmake "${cmake_args[@]}" "$@"

if ! cmake --build "${BUILD_DIR}" --target help | grep '^... g1_npz_policy$' >/dev/null; then
  echo "g1_npz_policy is unavailable: install ONNX Runtime C++ and set ONNXRUNTIME_ROOT." >&2
  exit 1
fi

cmake --build "${BUILD_DIR}" --target g1_udp_bridge g1_npz_policy -j"$(nproc)"
