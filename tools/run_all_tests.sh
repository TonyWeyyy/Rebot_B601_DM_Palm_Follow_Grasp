#!/usr/bin/env bash
# 一键运行全部 5 套离线验证单元测试（无真机 / 无相机 / 无 X11 即可执行）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${PROJECT_ROOT}"

# 剔除 /opt/ros/* 避免与 Pinocchio 冲突
if [[ -n "${PYTHONPATH:-}" ]]; then
  export PYTHONPATH="$(echo "${PYTHONPATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
  export LD_LIBRARY_PATH="$(echo "${LD_LIBRARY_PATH}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd: || true)"
fi

# 选择主控 Python（优先使用项目内 .venv）
if [[ -n "${CONTROL_PYTHON:-}" ]]; then
  CTRL_PY="${CONTROL_PYTHON}"
elif [[ -x "${PROJECT_ROOT}/.venv/bin/python" ]]; then
  CTRL_PY="${PROJECT_ROOT}/.venv/bin/python"
else
  CTRL_PY="$(command -v python3)"
fi

echo "=================================================================="
echo "  使用主控 Python: ${CTRL_PY} ($(uname -m))"
echo "=================================================================="

echo ""
echo "[1/5] 达妙 DM 底层控制 + P0~P4 + 运动学/重力模型一致性测试"
"${CTRL_PY}" tools/test_dm_controller.py

echo ""
echo "[2/5] v14 四层架构 + Gemini 305 深度保护链 + UDP 桥测试"
"${CTRL_PY}" tools/test_v14_architecture.py

echo ""
echo "[3/5] v15 双向遥操作安全隔离墙测试"
"${CTRL_PY}" tools/test_v15_teleop.py

echo ""
echo "[4/5] Jetson ARM64 / 无头终端按键 / 跨机白名单 / 纯 NumPy 回退测试"
"${CTRL_PY}" tools/test_jetson_readiness.py

# 选择 MuJoCo Studio Python
MJ_PY=""
if [[ -n "${MUJOCO_STUDIO_PYTHON:-}" && -x "${MUJOCO_STUDIO_PYTHON}" ]]; then
  MJ_PY="${MUJOCO_STUDIO_PYTHON}"
else
  for cand in \
    "${PROJECT_ROOT}/.venv_mujoco/bin/python" \
    "${PROJECT_ROOT}/.venv/bin/python" \
    "${CTRL_PY}" \
    "${HOME}/.venvs/mujoco-studio/bin/python"
  do
    if [[ -n "${cand}" && -x "${cand}" ]] && "${cand}" -c "import mujoco" >/dev/null 2>&1; then
      MJ_PY="${cand}"
      break
    fi
  done
fi

echo ""
if [[ -n "${MJ_PY}" ]]; then
  echo "[5/5] MuJoCo B601-DM + 支架 STL + Gemini 305 场景编译测试 (${MJ_PY})"
  "${MJ_PY}" tools/test_mujoco_twin_dm.py
else
  echo "[5/5] 跳过 MuJoCo 场景测试（未检测到 mujoco>=4.0.1；桌面或 Orin NX 需本机 3D 可运行 ./tools/setup_env.sh --mujoco-only）"
fi

echo ""
echo "=================================================================="
echo "  离线测试套件执行完毕！"
echo "=================================================================="
