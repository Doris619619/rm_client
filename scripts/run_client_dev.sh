#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${PROJECT_ROOT}/../.." && pwd)"
LOG_DIR="${PROJECT_ROOT}/logs"
PID_DIR="${LOG_DIR}/pids"
PARAM_FILE="${PROJECT_ROOT}/config/judge_params.yaml"
FRONTEND_DIR="${PROJECT_ROOT}/frontend"
FRONTEND_PORT="${FRONTEND_PORT:-8081}"

RECEIVER_LOG="${LOG_DIR}/judge_receiver.dev.log"
BRIDGE_LOG="${LOG_DIR}/state_bridge.dev.log"
FRONTEND_LOG="${LOG_DIR}/frontend.dev.log"

RECEIVER_PID_FILE="${PID_DIR}/judge_receiver.pid"
BRIDGE_PID_FILE="${PID_DIR}/state_bridge.pid"
FRONTEND_PID_FILE="${PID_DIR}/frontend_server.pid"

mkdir -p "${LOG_DIR}" "${PID_DIR}"

source_ros2() {
  # Temporarily disable nounset while sourcing external setup scripts
  set +u
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
    return
  fi

  local distro
  for distro in humble iron jazzy rolling; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      # shellcheck disable=SC1090
      source "/opt/ros/${distro}/setup.bash"
      set -u
      return
    fi
  done

  set -u
  echo "[ERROR] 未找到 ROS2 setup.bash，请先安装 ROS2 或设置 ROS_DISTRO。" >&2
  exit 1
}

start_and_record() {
  local name="$1"
  local pid_file="$2"
  local log_file="$3"
  shift 3

  nohup "$@" >"${log_file}" 2>&1 &
  local pid=$!
  echo "${pid}" >"${pid_file}"

  sleep 1
  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[ERROR] ${name} 启动失败，请检查日志: ${log_file}" >&2
    exit 1
  fi
}

source_ros2
# source workspace setup, also allow undefined vars during sourcing
set +u
# shellcheck disable=SC1090
source "${WORKSPACE_ROOT}/install/setup.bash"
set -u

if [[ ! -f "${PARAM_FILE}" ]]; then
  echo "[ERROR] 参数文件不存在: ${PARAM_FILE}" >&2
  exit 1
fi

if [[ ! -d "${FRONTEND_DIR}" ]]; then
  echo "[ERROR] 前端目录不存在: ${FRONTEND_DIR}" >&2
  exit 1
fi

start_and_record "judge_receiver" "${RECEIVER_PID_FILE}" "${RECEIVER_LOG}" \
  ros2 run rm_client judge_receiver --ros-args --params-file "${PARAM_FILE}"

start_and_record "state_bridge" "${BRIDGE_PID_FILE}" "${BRIDGE_LOG}" \
  python3 "${PROJECT_ROOT}/scripts/state_bridge.py"

start_and_record "frontend_server" "${FRONTEND_PID_FILE}" "${FRONTEND_LOG}" \
  bash -lc "cd \"${FRONTEND_DIR}\" && exec python3 -m http.server ${FRONTEND_PORT}"

# 可选：如需联调 mock，可在此处追加启动命令（默认不依赖 mock）
# 例如：start_and_record "mock" "${PID_DIR}/mock.pid" "${LOG_DIR}/mock.dev.log" ./RMMock

echo "receiver log: ${RECEIVER_LOG}"
echo "bridge log: ${BRIDGE_LOG}"
echo "frontend: http://127.0.0.1:${FRONTEND_PORT}"
