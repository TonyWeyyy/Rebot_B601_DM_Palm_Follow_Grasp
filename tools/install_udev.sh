#!/usr/bin/env bash
# 一键安装本项目的系统设备权限：
#   1. 将内置 third_party/udev/99-obsensor-libusb.rules 安装到 /etc/udev/rules.d/
#   2. 将当前用户加入 dialout（达妙串口 /dev/ttyACM0）与 video（相机）用户组
#   3. 重新加载 udev 规则
#   4. 可选：--can0 启动 SocketCAN can0 (1Mbps)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
RULES_SRC="${PROJECT_ROOT}/third_party/udev/99-obsensor-libusb.rules"
RULES_DST="/etc/udev/rules.d/99-obsensor-libusb.rules"

SETUP_CAN0=0
for arg in "$@"; do
  case "${arg}" in
    --can0) SETUP_CAN0=1 ;;
    -h|--help)
      echo "用法: $0 [--can0]"
      echo "  安装 Orbbec Gemini 305 udev 规则并配置 dialout/video 权限。"
      echo "  --can0  额外尝试启用 SocketCAN can0 (1Mbps)"
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: ${arg}" >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "${RULES_SRC}" ]]; then
  echo "[错误] 未找到内置 udev 规则文件: ${RULES_SRC}" >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-$USER}"
SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  SUDO="sudo"
fi

echo "[1/3] 安装 Orbbec Gemini 305 USB udev 规则 -> ${RULES_DST}"
${SUDO} install -m 0644 "${RULES_SRC}" "${RULES_DST}"
${SUDO} udevadm control --reload-rules
${SUDO} udevadm trigger

echo "[2/3] 将用户 '${TARGET_USER}' 加入 dialout 与 video 用户组"
${SUDO} usermod -aG dialout,video "${TARGET_USER}"

if [[ "${SETUP_CAN0}" -eq 1 ]]; then
  echo "[3/3] 配置 SocketCAN can0 (1Mbps)..."
  ${SUDO} ip link set can0 down 2>/dev/null || true
  ${SUDO} ip link set can0 type can bitrate 1000000
  ${SUDO} ip link set can0 up
  ip -details link show can0
else
  echo "[3/3] 跳过 SocketCAN 配置（默认使用 /dev/ttyACM0 串口；如需 can0 请附加 --can0）"
fi

echo ""
echo "=================================================================="
echo "  硬件权限安装完成！"
echo "  1. 请【重新插拔】Orbbec Gemini 305 USB 线；"
echo "  2. 如首次加入 dialout/video 组，请【注销重新登录】或执行 'newgrp dialout' 生效。"
echo "=================================================================="
