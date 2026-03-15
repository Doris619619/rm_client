#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LOG_DIR="${PROJECT_ROOT}/logs"
PID_DIR="${LOG_DIR}/pids"

RECEIVER_PID_FILE="${PID_DIR}/judge_receiver.pid"
BRIDGE_PID_FILE="${PID_DIR}/state_bridge.pid"
FRONTEND_PID_FILE="${PID_DIR}/frontend_server.pid"

safe_stop_by_pidfile() {
  local name="$1"
  local pid_file="$2"
  local expect_pattern="$3"

  if [[ ! -f "${pid_file}" ]]; then
    echo "[INFO] ${name}: 未找到 pid 文件，跳过"
    return
  fi

  local pid
  pid="$(cat "${pid_file}")"

  if [[ -z "${pid}" || ! "${pid}" =~ ^[0-9]+$ ]]; then
    echo "[WARN] ${name}: pid 文件格式异常，跳过 ${pid_file}"
    rm -f "${pid_file}"
    return
  fi

  if ! kill -0 "${pid}" 2>/dev/null; then
    echo "[INFO] ${name}: 进程已不在运行"
    rm -f "${pid_file}"
    return
  fi

  local cmdline
  cmdline="$(tr '\0' ' ' < "/proc/${pid}/cmdline" 2>/dev/null || true)"

  if [[ "${cmdline}" != *"${expect_pattern}"* ]]; then
    echo "[WARN] ${name}: pid=${pid} 命令行不匹配，避免误杀"
    echo "[WARN] 实际 cmdline: ${cmdline}"
    return
  fi

  kill "${pid}" 2>/dev/null || true
  sleep 1

  if kill -0 "${pid}" 2>/dev/null; then
    kill -9 "${pid}" 2>/dev/null || true
  fi

  rm -f "${pid_file}"
  echo "[OK] ${name}: 已停止"
}

mkdir -p "${PID_DIR}"

safe_stop_by_pidfile "judge_receiver" "${RECEIVER_PID_FILE}" "ros2 run rm_client judge_receiver"
safe_stop_by_pidfile "state_bridge" "${BRIDGE_PID_FILE}" "${PROJECT_ROOT}/scripts/state_bridge.py"
safe_stop_by_pidfile "frontend_server" "${FRONTEND_PID_FILE}" "python3 -m http.server"
