#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
REPO_ROOT="$(cd "${G1_ROOT}/.." && pwd)"
BUILD_DIR="${G1_BRIDGE_BUILD_DIR:-${G1_ROOT}/build}"
BRIDGE_BIN="${BUILD_DIR}/g1_udp_bridge"
POLICY_BIN="${BUILD_DIR}/g1_npz_policy"
BRIDGE_RUNNER="${G1_ROOT}/scripts/run_bridge.sh"
BRIDGE_CONFIG="${G1_BRIDGE_CONFIG:-${G1_ROOT}/config/g1_bridge.yaml}"
CONTROLLER_CONFIG="${G1_CONTROLLER_CONFIG:-${REPO_ROOT}/sim2real/config/g1/controller.yaml}"
TRACKING_CONFIG="${G1_TRACKING_CONFIG:-${REPO_ROOT}/sim2real/config/g1/tracking.yaml}"
POLICY_PATH="${GRIT_POLICY_PATH:-${REPO_ROOT}/sim2real/checkpoints/policy.onnx}"
LOCOMOTION_POLICY_PATH="${LOCOMOTION_POLICY_PATH:-${REPO_ROOT}/sim2real/checkpoints/Unitree-G1-AMP-Flat_model_30000.onnx}"
NETWORK_INTERFACE="${G1_NET:-}"
INFERENCE_THREADS="${GRIT_INFERENCE_THREADS:-4}"
BRIDGE_CPUS="${G1_BRIDGE_CPUS:-}"
POLICY_CPUS="${GRIT_POLICY_CPUS:-}"
VOICE_ANNOUNCEMENTS="${GRIT_VOICE_ANNOUNCEMENTS:-true}"
MOTION_FILE=""

usage() {
  cat <<'EOF'
Usage: run_npz_onboard.sh --net IFACE --motion-file FILE_OR_DIR [options]

Options:
  --net IFACE              Network interface connected to the G1
  --motion-file PATH       50 Hz GRIT NPZ file or a directory of NPZ files
  --motion-path PATH       Alias for --motion-file
  --policy-path PATH       Whole-body GRIT ONNX override
  --locomotion-policy PATH Locomotion ONNX override
  --inference-threads N    ONNX Runtime intra-op threads (default: 4)
  --bridge-cpus LIST       taskset CPU list for the native bridge
  --policy-cpus LIST       taskset CPU list for C++ inference
  --voice-announcements B  Enable Unitree speaker announcements: true/false (default: true)
  --bridge-config PATH     Native bridge YAML override
  --controller-config PATH Controller YAML override
  --tracking-config PATH   Tracking YAML override
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --net|--network-interface) NETWORK_INTERFACE="${2:?missing value for $1}"; shift 2 ;;
    --motion-file|--motion-path) MOTION_FILE="${2:?missing value for $1}"; shift 2 ;;
    --policy-path) POLICY_PATH="${2:?missing value for $1}"; shift 2 ;;
    --locomotion-policy) LOCOMOTION_POLICY_PATH="${2:?missing value for $1}"; shift 2 ;;
    --inference-threads) INFERENCE_THREADS="${2:?missing value for $1}"; shift 2 ;;
    --bridge-cpus) BRIDGE_CPUS="${2:?missing value for $1}"; shift 2 ;;
    --policy-cpus) POLICY_CPUS="${2:?missing value for $1}"; shift 2 ;;
    --voice-announcements) VOICE_ANNOUNCEMENTS="${2:?missing value for $1}"; shift 2 ;;
    --bridge-config) BRIDGE_CONFIG="${2:?missing value for $1}"; shift 2 ;;
    --controller-config) CONTROLLER_CONFIG="${2:?missing value for $1}"; shift 2 ;;
    --tracking-config) TRACKING_CONFIG="${2:?missing value for $1}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

if [[ -z "${NETWORK_INTERFACE}" || -z "${MOTION_FILE}" ]]; then
  usage >&2
  exit 2
fi
if [[ ! -d "/sys/class/net/${NETWORK_INTERFACE}" ]]; then
  echo "Network interface does not exist: ${NETWORK_INTERFACE}" >&2
  exit 1
fi
for required in "${BRIDGE_BIN}" "${POLICY_BIN}" "${BRIDGE_RUNNER}" "${BRIDGE_CONFIG}" \
  "${CONTROLLER_CONFIG}" "${TRACKING_CONFIG}" "${POLICY_PATH}" "${MOTION_FILE}"; do
  if [[ ! -e "${required}" ]]; then
    echo "Required path does not exist: ${required}" >&2
    exit 1
  fi
done
if [[ ! -e "${LOCOMOTION_POLICY_PATH}" ]]; then
  echo "Required path does not exist: ${LOCOMOTION_POLICY_PATH}" >&2
  exit 1
fi
if ! [[ "${INFERENCE_THREADS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--inference-threads must be a positive integer" >&2
  exit 2
fi
case "${VOICE_ANNOUNCEMENTS,,}" in
  true|1|yes|on) VOICE_ANNOUNCEMENTS="true" ;;
  false|0|no|off) VOICE_ANNOUNCEMENTS="false" ;;
  *) echo "--voice-announcements must be true or false" >&2; exit 2 ;;
esac

bridge_pid=""
policy_pid=""
cleanup() {
  trap - EXIT INT TERM HUP
  if [[ -n "${policy_pid}" ]] && kill -0 "${policy_pid}" 2>/dev/null; then
    kill -TERM "${policy_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bridge_pid}" ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
    kill -TERM "${bridge_pid}" 2>/dev/null || true
  fi
  [[ -z "${policy_pid}" ]] || wait "${policy_pid}" 2>/dev/null || true
  [[ -z "${bridge_pid}" ]] || wait "${bridge_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM HUP

onnx_path="$(ldd "${POLICY_BIN}" | awk '$1 ~ /^libonnxruntime\.so/ && $2 == "=>" && $3 ~ /^\// {print $3; exit}')"
if [[ -z "${onnx_path}" ]]; then
  echo "ONNX Runtime shared library cannot be resolved for: ${POLICY_BIN}" >&2
  exit 1
fi
onnx_lib="$(dirname "${onnx_path}")"

echo "[run_npz_onboard] net=${NETWORK_INTERFACE}"
echo "[run_npz_onboard] motion=${MOTION_FILE}"
echo "[run_npz_onboard] policy=${POLICY_PATH}"
echo "[run_npz_onboard] locomotion_policy=${LOCOMOTION_POLICY_PATH}"
echo "[run_npz_onboard] voice_announcements=${VOICE_ANNOUNCEMENTS}"

bridge_command=(bash "${BRIDGE_RUNNER}" --voice-announcements "${VOICE_ANNOUNCEMENTS}")
policy_command=(
  "${POLICY_BIN}"
  --controller-config "${CONTROLLER_CONFIG}"
  --tracking-config "${TRACKING_CONFIG}"
  --motion-file "${MOTION_FILE}"
  --policy-path "${POLICY_PATH}"
  --locomotion-policy "${LOCOMOTION_POLICY_PATH}"
  --inference-threads "${INFERENCE_THREADS}"
)
if [[ -n "${BRIDGE_CPUS}" ]]; then
  bridge_command=(taskset -c "${BRIDGE_CPUS}" "${bridge_command[@]}")
fi
if [[ -n "${POLICY_CPUS}" ]]; then
  policy_command=(taskset -c "${POLICY_CPUS}" "${policy_command[@]}")
fi

env \
  G1_NET="${NETWORK_INTERFACE}" \
  G1_BRIDGE_BUILD_DIR="${BUILD_DIR}" \
  G1_BRIDGE_CONFIG="${BRIDGE_CONFIG}" \
  "${bridge_command[@]}" </dev/null &
bridge_pid=$!

env LD_LIBRARY_PATH="${onnx_lib}:${LD_LIBRARY_PATH:-}" "${policy_command[@]}" &
policy_pid=$!

set +e
wait -n "${bridge_pid}" "${policy_pid}"
status=$?
set -e
if ! kill -0 "${bridge_pid}" 2>/dev/null && kill -0 "${policy_pid}" 2>/dev/null; then
  echo "Native bridge exited before the policy process." >&2
  [[ ${status} -ne 0 ]] || status=1
fi
exit "${status}"
