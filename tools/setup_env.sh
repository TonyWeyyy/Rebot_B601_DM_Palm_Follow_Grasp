#!/usr/bin/env bash
# ==============================================================================
# Seeed reBot Arm B601-DM + Orbbec Gemini 305 项目内独立环境一键搭建脚本
# （自动兼容 x86_64 桌面主机 与 NVIDIA Jetson Orin NX/Nano aarch64）
# ==============================================================================
# 会在项目根目录内创建完全独立的本地虚拟环境（不污染系统或外部路径）：
#   1. .venv/        : 主控制环境（Python 3.10+，优先从 third_party/wheels/<arch>/
#                      安装 motorbridge 与 pyorbbecsdk2，并安装 pin/mediapipe/opencv 等）
#   2. .venv_mujoco/ : MuJoCo Studio 4.0+ 3D 数字孪生环境（Python 3.12）
#                      - x86_64 桌面默认同时创建 .venv 与 .venv_mujoco
#                      - Jetson ARM64 默认仅创建 .venv（推荐 PC 跨机跑 MuJoCo 3D，
#                        如 Orin NX 16GB 需本机跑 MuJoCo 请附加 --with-mujoco）
#
# 用法：
#   ./tools/setup_env.sh                 # x86_64 装全套；Jetson 默认仅装主控 .venv
#   ./tools/setup_env.sh --with-mujoco   # 强制同时安装 .venv_mujoco（如 Orin NX 16GB 本机 3D）
#   ./tools/setup_env.sh --control-only  # 仅创建主控环境 .venv
#   ./tools/setup_env.sh --mujoco-only   # 仅创建数字孪生环境 .venv_mujoco
#   ./tools/setup_env.sh --offline       # 严格离线模式（仅从 third_party/wheels* 安装）
#   ./tools/setup_env.sh --python /path/to/python3.10
# ==============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ARCH="$(uname -m)"
IS_JETSON=0
if [[ -f /etc/nv_tegra_release || "${ARCH}" == "aarch64" ]]; then
  IS_JETSON=1
fi

DO_CONTROL=1
# Jetson 默认优先只装主控环境（节省内存/磁盘，推荐 PC 跨机跑 MuJoCo），可加 --with-mujoco 开启
DO_MUJOCO=1
if [[ "${IS_JETSON}" -eq 1 ]]; then
  DO_MUJOCO=0
fi
OFFLINE=0
CONTROL_BASE_PY=""
MUJOCO_BASE_PY=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --control-only) DO_MUJOCO=0; shift ;;
    --mujoco-only) DO_CONTROL=0; DO_MUJOCO=1; shift ;;
    --with-mujoco) DO_MUJOCO=1; shift ;;
    --offline) OFFLINE=1; shift ;;
    --python) CONTROL_BASE_PY="$2"; shift 2 ;;
    --mujoco-python) MUJOCO_BASE_PY="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,23p' "$0"
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: $1" >&2
      exit 1
      ;;
  esac
done

# 剔除 /opt/ros/*，防止与 Pinocchio 发生 ABI 冲突
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$(echo "${PYTHONPATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi

# ARM64 上构建 cmeel/pin 时跳过耗时的 C++ 内部测试
export CMEEL_SKIP_TESTS=1

WHEELS_ARCH_DIR="${PROJECT_ROOT}/third_party/wheels/${ARCH}"
WHEELS_ROOT_DIR="${PROJECT_ROOT}/third_party/wheels"
WHEELS_MJ_DIR="${PROJECT_ROOT}/third_party/wheels_mujoco"

echo "=================================================================="
echo "  平台架构: ${ARCH} | Jetson/ARM64 模式: $([[ "${IS_JETSON}" -eq 1 ]] && echo 'ON' || echo 'OFF')"
echo "  主控环境: $([[ "${DO_CONTROL}" -eq 1 ]] && echo 'INSTALL (.venv)' || echo 'SKIP')"
echo "  MuJoCo  : $([[ "${DO_MUJOCO}" -eq 1 ]] && echo 'INSTALL (.venv_mujoco)' || echo 'SKIP (需要时加 --with-mujoco)')"
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
    echo "[警告] 检测到 Python 3.8（常见于 JetPack 5）；建议安装 uv 或 Miniforge 使用 Python 3.10，" >&2
    echo "       以直接命中内置 third_party/wheels/aarch64/ 的 cp310 Wheel！" >&2
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

  if ! _pip_install "${PROJECT_ROOT}/.venv/bin/python" "${PIP_ARGS[@]}" -r "${PROJECT_ROOT}/requirements.txt"; then
    if [[ "${IS_JETSON}" -eq 1 ]]; then
      echo ""
      echo "[警告] ARM64 完整安装失败（通常为 pin/cmeel 无预编译轮子或缺少 cmake/python3-dev）。"
      echo "       正在自动降级为【不含 pin 的核心依赖】安装（rebot_adapter.py 将自动启用内置精确纯 NumPy DM 引擎）..."
      TMP_REQ="$(mktemp)"
      grep -Ev '^\s*pin([<>=!~].*)?$' "${PROJECT_ROOT}/requirements.txt" > "${TMP_REQ}"
      _pip_install "${PROJECT_ROOT}/.venv/bin/python" "${PIP_ARGS[@]}" -r "${TMP_REQ}"
      rm -f "${TMP_REQ}"
      echo "[提示] 核心依赖安装完成！如需额外安装 Pinocchio，可用 Miniforge/conda-forge: conda install -c conda-forge pinocchio"
    else
      exit 1
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
      echo "       请安装 uv (curl -LsSf https://astral.sh/uv/install.sh | sh) 或 python3.12 后重试，" >&2
      echo "       或仅运行 ./tools/setup_env.sh --control-only。" >&2
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
echo "  环境配置完成！请运行以下自检命令确认："
echo "    ./.venv/bin/python tools/check_env.py"
echo "    ./tools/run_all_tests.sh"
if [[ "${IS_JETSON}" -eq 1 ]]; then
  echo "  Jetson 上电性能一键锁定："
  echo "    sudo ./tools/jetson_perf.sh"
fi
echo "=================================================================="
