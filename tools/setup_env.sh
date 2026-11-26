#!/usr/bin/env bash
# ==============================================================================
# Seeed reBot Arm B601-DM + Orbbec Gemini 305 跨平台一键环境搭建脚本
# （自动适配 Jetson Orin Nano 8GB / Orin NX 16GB / x86_64 桌面主机）
# ==============================================================================
# 会在项目根目录内创建完全独立的本地虚拟环境（不污染系统或外部路径）：
#   1. .venv/        : 主控制环境（优先从 third_party/wheels/<arch>/ 安装内置 SDK）
#   2. .venv_mujoco/ : MuJoCo Studio 4.0+ 3D 数字孪生环境（Python 3.12，桌面默认安装；
#                      Jetson 默认跳过，推荐在 PC 端跨机运行 3D 窗口）
#
# Jetson Orin Nano 8GB 专项保护（自动触发，亦可传 --orin-nano 显式启用）：
#   - 使用 requirements-orin-nano.txt（跳过 pin 的 C++ 编译，防 8GB RAM OOM，
#     运行时自动启用内置精确纯 NumPy DM 运动学/重力引擎）；
#   - 单独以 --no-deps 安装内置 pyorbbecsdk2 ARM64 Wheel，绕过臃肿/缺轮子的 open3d/pygame/av；
#   - 检查 Swap 容量，不足 4GB 时给出扩容提示。
#
# 用法：
#   ./tools/setup_env.sh                 # 自动按平台安装
#   ./tools/setup_env.sh --orin-nano     # 强制按 Orin Nano 8GB 轻量防 OOM 策略安装
#   ./tools/setup_env.sh --with-pin      # 在 Orin Nano 上仍尝试安装 pin (需 >=4GB Swap)
#   ./tools/setup_env.sh --with-mujoco   # 强制同时安装 .venv_mujoco
#   ./tools/setup_env.sh --control-only  # 仅创建主控环境 .venv
#   ./tools/setup_env.sh --mujoco-only   # 仅创建数字孪生环境 .venv_mujoco
#   ./tools/setup_env.sh --offline       # 严格离线模式（仅从 third_party/wheels* 安装）
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ARCH="$(uname -m)"
IS_JETSON=0
IS_ORIN_NANO=0
MODEL_TXT=""
if [[ -r /proc/device-tree/model ]]; then
  MODEL_TXT="$(tr -d '\0' </proc/device-tree/model 2>/dev/null || true)"
fi
if [[ -f /etc/nv_tegra_release || "${ARCH}" == "aarch64" ]]; then
  IS_JETSON=1
fi
if echo "${MODEL_TXT}" | grep -qi "orin nano"; then
  IS_ORIN_NANO=1
elif [[ "${IS_JETSON}" -eq 1 && -r /proc/meminfo ]]; then
  MEM_KB="$(awk '/^MemTotal:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
  if [[ "${MEM_KB}" -gt 0 && "${MEM_KB}" -le 9800000 ]]; then
    IS_ORIN_NANO=1
  fi
fi

DO_CONTROL=1
DO_MUJOCO=1
if [[ "${IS_JETSON}" -eq 1 ]]; then
  DO_MUJOCO=0
fi
FORCE_WITH_PIN=0
OFFLINE=0
CONTROL_BASE_PY=""
MUJOCO_BASE_PY=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --orin-nano) IS_JETSON=1; IS_ORIN_NANO=1; DO_MUJOCO=0; shift ;;
    --with-pin) FORCE_WITH_PIN=1; shift ;;
    --control-only) DO_MUJOCO=0; shift ;;
    --mujoco-only) DO_CONTROL=0; DO_MUJOCO=1; shift ;;
    --with-mujoco) DO_MUJOCO=1; shift ;;
    --offline) OFFLINE=1; shift ;;
    --python) CONTROL_BASE_PY="$2"; shift 2 ;;
    --mujoco-python) MUJOCO_BASE_PY="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,28p' "$0"
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: $1" >&2
      exit 1
      ;;
  esac
done

# 剔除 /opt/ros/*，防止与 Pinocchio / OpenCV 发生 ABI 冲突
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$(echo "${PYTHONPATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi

export CMEEL_SKIP_TESTS=1

WHEELS_ARCH_DIR="${PROJECT_ROOT}/third_party/wheels/${ARCH}"
WHEELS_ROOT_DIR="${PROJECT_ROOT}/third_party/wheels"
WHEELS_MJ_DIR="${PROJECT_ROOT}/third_party/wheels_mujoco"

echo "=================================================================="
echo "  平台架构     : ${ARCH} ${MODEL_TXT:+(${MODEL_TXT})}"
echo "  Jetson 模式  : $([[ "${IS_JETSON}" -eq 1 ]] && echo 'ON' || echo 'OFF') | Orin Nano 8GB 轻量防 OOM: $([[ "${IS_ORIN_NANO}" -eq 1 ]] && echo 'ON' || echo 'OFF')"
echo "  主控环境     : $([[ "${DO_CONTROL}" -eq 1 ]] && echo 'INSTALL (.venv)' || echo 'SKIP')"
echo "  MuJoCo 3D    : $([[ "${DO_MUJOCO}" -eq 1 ]] && echo 'INSTALL (.venv_mujoco)' || echo 'SKIP (推荐 PC 跨机跑 3D，或加 --with-mujoco)')"
echo "=================================================================="

_pip_install() {
  local py_bin="$1"
  shift
  if command -v uv >/dev/null 2>&1; then
    uv pip install --python "${py_bin}" "$@"
  else
    "${py_bin}" -m pip install "$@"
  fi
}

# ------------------------------------------------------------------------------
# 1. 主控制环境 (.venv)
# ------------------------------------------------------------------------------
if [[ "${DO_CONTROL}" -eq 1 ]]; then
  echo ""
  echo "[1/2] 配置项目内主控制环境 -> ${PROJECT_ROOT}/.venv"

  if [[ "${IS_ORIN_NANO}" -eq 1 && -r /proc/meminfo ]]; then
    SWAP_KB="$(awk '/^SwapTotal:/ {print $2}' /proc/meminfo 2>/dev/null || echo 0)"
    if [[ "${SWAP_KB}" -lt 3500000 ]]; then
      echo "[提示] 检测到当前 Swap 仅 $((SWAP_KB / 1024)) MB；Orin Nano 8GB 建议先运行："
      echo "       sudo ./tools/jetson_perf.sh --setup-swap 4G"
    fi
  fi

  if [[ -z "${CONTROL_BASE_PY}" ]]; then
    if command -v python3.10 >/dev/null 2>&1; then
      CONTROL_BASE_PY="$(command -v python3.10)"
    elif command -v python3 >/dev/null 2>&1; then
      CONTROL_BASE_PY="$(command -v python3)"
    else
      echo "[错误] 未找到 python3，请先安装 python3 (>=3.10) 与 python3-venv。" >&2
      exit 1
    fi
  fi

  PY_VER="$("${CONTROL_BASE_PY}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  echo "  使用基础解释器: ${CONTROL_BASE_PY} (Python ${PY_VER})"
  if [[ "${PY_VER}" == "3.8" ]]; then
    echo "[警告] 检测到 Python 3.8（JetPack 5）；Orin Nano 强烈建议使用 JetPack 6 (Ubuntu 22.04, Python 3.10)，" >&2
    echo "       或使用 uv/Miniforge 创建 Python 3.10 环境以直接命中内置 ARM64 cp310 Wheel！" >&2
  fi

  if [[ ! -x "${PROJECT_ROOT}/.venv/bin/python" ]]; then
    if command -v uv >/dev/null 2>&1; then
      uv venv "${PROJECT_ROOT}/.venv" --python "${CONTROL_BASE_PY}" --seed
    else
      "${CONTROL_BASE_PY}" -m venv "${PROJECT_ROOT}/.venv"
    fi
  fi

  PIP_ARGS=()
  if [[ -d "${WHEELS_ARCH_DIR}" ]]; then
    PIP_ARGS+=(--find-links "${WHEELS_ARCH_DIR}")
  fi
  if [[ -d "${WHEELS_ROOT_DIR}" ]]; then
    PIP_ARGS+=(--find-links "${WHEELS_ROOT_DIR}")
  fi
  if [[ "${OFFLINE}" -eq 1 ]]; then
    PIP_ARGS+=(--no-index)
  fi

  if ! command -v uv >/dev/null 2>&1; then
    "${PROJECT_ROOT}/.venv/bin/pip" install --upgrade pip setuptools wheel || true
  fi

  if [[ "${IS_ORIN_NANO}" -eq 1 && "${FORCE_WITH_PIN}" -eq 0 ]]; then
    echo "  [Orin Nano 8GB 策略] 安装精简核心依赖 requirements-orin-nano.txt（跳过 pin 编译防 OOM）..."
    _pip_install "${PROJECT_ROOT}/.venv/bin/python" "${PIP_ARGS[@]}" -r "${PROJECT_ROOT}/requirements-orin-nano.txt"
    echo "  [Orin Nano 8GB 策略] 以 --no-deps 安装 pyorbbecsdk2（跳过无用且易报错的 open3d/pygame/av）..."
    _pip_install "${PROJECT_ROOT}/.venv/bin/python" "${PIP_ARGS[@]}" --no-deps "pyorbbecsdk2>=2.0"
  else
    if ! _pip_install "${PROJECT_ROOT}/.venv/bin/python" "${PIP_ARGS[@]}" -r "${PROJECT_ROOT}/requirements.txt"; then
      if [[ "${IS_JETSON}" -eq 1 ]]; then
        echo ""
        echo "[警告] ARM64 完整安装失败（通常为 pin/cmeel 或 pyorbbecsdk2 的 open3d 传递依赖失败）。"
        echo "       自动切换至 Orin 精简防 OOM 安装方案（rebot_adapter.py 将自动启用内置精确纯 NumPy DM 引擎）..."
        _pip_install "${PROJECT_ROOT}/.venv/bin/python" "${PIP_ARGS[@]}" -r "${PROJECT_ROOT}/requirements-orin-nano.txt"
        _pip_install "${PROJECT_ROOT}/.venv/bin/python" "${PIP_ARGS[@]}" --no-deps "pyorbbecsdk2>=2.0"
      else
        exit 1
      fi
    fi
  fi
  echo "[完成] 主控制环境已就绪: ${PROJECT_ROOT}/.venv/bin/python"
fi

# ------------------------------------------------------------------------------
# 2. MuJoCo Studio 4.0+ 数字孪生独立环境 (.venv_mujoco)
# ------------------------------------------------------------------------------
if [[ "${DO_MUJOCO}" -eq 1 ]]; then
  echo ""
  echo "[2/2] 配置项目内 MuJoCo Studio 4.0+ 环境 -> ${PROJECT_ROOT}/.venv_mujoco"

  if [[ ! -x "${PROJECT_ROOT}/.venv_mujoco/bin/python" ]]; then
    if [[ -n "${MUJOCO_BASE_PY}" ]]; then
      "${MUJOCO_BASE_PY}" -m venv "${PROJECT_ROOT}/.venv_mujoco"
    elif command -v uv >/dev/null 2>&1; then
      uv venv "${PROJECT_ROOT}/.venv_mujoco" --python 3.12 --seed
    elif command -v python3.12 >/dev/null 2>&1; then
      python3.12 -m venv "${PROJECT_ROOT}/.venv_mujoco"
    elif command -v python3.11 >/dev/null 2>&1; then
      python3.11 -m venv "${PROJECT_ROOT}/.venv_mujoco"
    else
      echo "[错误] MuJoCo Studio 4.0+ 需要 Python >= 3.11（推荐 3.12）。" >&2
      echo "       请安装 uv (curl -LsSf https://astral.sh/uv/install.sh | sh) 或 python3.12 后重试。" >&2
      exit 1
    fi
  fi

  MJ_PIP_ARGS=()
  if [[ -d "${WHEELS_MJ_DIR}" ]]; then
    MJ_PIP_ARGS+=(--find-links "${WHEELS_MJ_DIR}")
  fi
  if [[ "${OFFLINE}" -eq 1 ]]; then
    MJ_PIP_ARGS+=(--no-index)
  fi

  if ! command -v uv >/dev/null 2>&1; then
    "${PROJECT_ROOT}/.venv_mujoco/bin/pip" install --upgrade pip || true
  fi
  _pip_install "${PROJECT_ROOT}/.venv_mujoco/bin/python" "${MJ_PIP_ARGS[@]}" -r "${PROJECT_ROOT}/requirements-mujoco.txt"
  echo "[完成] MuJoCo Studio 环境已就绪: ${PROJECT_ROOT}/.venv_mujoco/bin/python"
fi

echo ""
echo "=================================================================="
echo "  环境配置完成！下一步自检与测试："
echo "    ./.venv/bin/python tools/check_env.py"
echo "    ./tools/run_all_tests.sh"
if [[ "${IS_ORIN_NANO}" -eq 1 ]]; then
  echo "  Orin Nano 8GB 实机启动（一键带线程隔离与 30fps 优化配置）："
  echo "    sudo ./tools/jetson_perf.sh"
  echo "    ./tools/run_orin_nano.sh --no-arm            # 纯视觉测帧率"
  echo "    ./tools/run_orin_nano.sh --mjpeg-port 8081   # 无头真机 + 浏览器远程调试画面"
fi
echo "=================================================================="
