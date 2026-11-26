#!/usr/bin/env bash
# ==============================================================================
# Jetson Orin Nano 8GB 一键低抖动启动脚本（tools/run_orin_nano.sh）
# ==============================================================================
# 自动完成 4 项 6 核 / 8GB 实时保护，再启动主程序：
#   1. 剥离 /opt/ros/* 环境变量，防止与系统 ROS 2 冲突；
#   2. 限制 OMP/OpenBLAS/MKL 线程池为 1~2 核，防止视觉/矩阵线程抢占 200Hz 达妙 MIT 控制线程；
#   3. 自动选用项目内 .venv/bin/python；
#   4. 默认加载 config/v14_orin_nano.yaml + --headless（SSH 终端按键 + 1Hz 状态打印）。
#
# 用法：
#   ./tools/run_orin_nano.sh                         # Orin Nano 8GB 30fps 黄金稳态真机模式
#   ./tools/run_orin_nano.sh --super60               # Orin Nano Super 25W 高频模式（60fps + 隔帧光流 stride=2）
#   ./tools/run_orin_nano.sh --no-arm --mjpeg-port 8081  # 纯视觉 + 浏览器远程调试画面
#   ./tools/run_orin_nano.sh --mujoco-host <PC_IP>   # 无头真机 + 推 100Hz 遥测到 PC 端 MuJoCo 3D
#   ./tools/run_orin_nano.sh --teleop --state-host <PC_IP> --cmd-bind 0.0.0.0 --allow-cmd-host <PC_IP>
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

# 1. 剥离 ROS 路径
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$(echo "${PYTHONPATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi

# 2. 限制后台线程数，为 200Hz MIT 与 100Hz PBVS 预留独占 CPU 核
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export MKL_NUM_THREADS="${MKL_NUM_THREADS:-1}"
export NUMEXPR_NUM_THREADS="${NUMEXPR_NUM_THREADS:-1}"
export PYTHONUNBUFFERED=1

# 3. 自动选择项目内主控 Python
if [[ -n "${CONTROL_PYTHON:-}" ]]; then
  PY_BIN="${CONTROL_PYTHON}"
elif [[ -x "${PROJECT_ROOT}/.venv/bin/python" ]]; then
  PY_BIN="${PROJECT_ROOT}/.venv/bin/python"
else
  PY_BIN="$(command -v python3)"
fi

MODE="v14"
SUPER60=0
GUI_MODE=0
PASS_ARGS=()

for arg in "$@"; do
  case "${arg}" in
    --teleop) MODE="v15" ;;
    --super60) SUPER60=1 ;;
    --gui) GUI_MODE=1 ;;
    -h|--help)
      sed -n '2,18p' "$0"
      exit 0
      ;;
    *) PASS_ARGS+=("${arg}") ;;
  esac
done

if [[ "${MODE}" == "v15" ]]; then
  echo "[Orin Nano 8GB] 启动 v15 双向遥操作桥接 (OMP=${OMP_NUM_THREADS}, BLAS=${OPENBLAS_NUM_THREADS})"
  exec "${PY_BIN}" "${PROJECT_ROOT}/mujoco_teleop_v15.py" "${PASS_ARGS[@]}"
fi

CMD=("${PY_BIN}" "${PROJECT_ROOT}/palm_follow_v14.py" "--v14-cfg" "${PROJECT_ROOT}/config/v14_orin_nano.yaml")
if [[ "${GUI_MODE}" -eq 0 ]]; then
  CMD+=("--headless")
fi
if [[ "${SUPER60}" -eq 1 ]]; then
  CMD+=("--fps" "60" "--detect-stride" "2")
fi
CMD+=("${PASS_ARGS[@]}")

echo "[Orin Nano 8GB] 执行: ${CMD[*]}"
exec "${CMD[@]}"
