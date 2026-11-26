#!/usr/bin/env bash
# 一键安装本项目的系统设备权限与 Jetson USB/CAN 优化：
#   1. 安装内置 Orbbec Gemini 305 udev 规则与达妙串口桥防 ModemManager 抢占规则
#   2. 将当前用户加入 dialout（串口）与 video（相机）用户组
#   3. 重新加载 udev 规则
#   4. 若在 NVIDIA Jetson 上运行：自动调高 usbfs_memory_mb=256 并关闭 USB 自动休眠
#   5. 可选：--can0 启动通用 USB-CAN (1Mbps)；--jetson-can 启动 Jetson Orin 板载原生 mttcan (1Mbps)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ORBBEC_RULES_SRC="${PROJECT_ROOT}/third_party/udev/99-obsensor-libusb.rules"
ORBBEC_RULES_DST="/etc/udev/rules.d/99-obsensor-libusb.rules"
DM_RULES_SRC="${PROJECT_ROOT}/third_party/udev/99-rebot-dm-serial.rules"
DM_RULES_DST="/etc/udev/rules.d/99-rebot-dm-serial.rules"

SETUP_CAN0=0
SETUP_JETSON_CAN=0
for arg in "$@"; do
  case "${arg}" in
    --can0) SETUP_CAN0=1 ;;
    --jetson-can) SETUP_JETSON_CAN=1 ;;
    -h|--help)
      echo "用法: $0 [--can0 | --jetson-can]"
      echo "  安装 Orbbec Gemini 305 与达妙串口 udev 规则，配置 dialout/video 权限，并在 Jetson 上优化 USB 缓冲。"
      echo "  --can0        额外启用通用 USB-CAN SocketCAN can0 (1Mbps)"
      echo "  --jetson-can  额外加载 Jetson Orin 板载原生 mttcan 并启用 can0 (1Mbps)"
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: ${arg}" >&2
      exit 1
      ;;
  esac
done

if [[ ! -f "${ORBBEC_RULES_SRC}" ]]; then
  echo "[错误] 未找到内置 Orbbec udev 规则文件: ${ORBBEC_RULES_SRC}" >&2
  exit 1
fi

TARGET_USER="${SUDO_USER:-$USER}"
SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  SUDO="sudo"
fi

IS_JETSON=0
if [[ -f /etc/nv_tegra_release || "$(uname -m)" == "aarch64" ]]; then
  IS_JETSON=1
fi

echo "[1/4] 安装 Orbbec Gemini 305 与达妙串口桥 udev 规则"
${SUDO} install -m 0644 "${ORBBEC_RULES_SRC}" "${ORBBEC_RULES_DST}"
if [[ -f "${DM_RULES_SRC}" ]]; then
  ${SUDO} install -m 0644 "${DM_RULES_SRC}" "${DM_RULES_DST}"
fi
${SUDO} udevadm control --reload-rules
${SUDO} udevadm trigger

echo "[2/4] 将用户 '${TARGET_USER}' 加入 dialout 与 video 用户组"
${SUDO} usermod -aG dialout,video "${TARGET_USER}"

if [[ "${IS_JETSON}" -eq 1 ]]; then
  echo "[3/4] 检测到 ARM64/Jetson：调高 USB 缓冲区 (usbfs_memory_mb=256) 并禁用自动休眠"
  if [[ -w /sys/module/usbcore/parameters/usbfs_memory_mb ]] || [[ -n "${SUDO}" ]]; then
    echo 256 | ${SUDO} tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null 2>&1 || true
    echo -1 | ${SUDO} tee /sys/module/usbcore/parameters/autosuspend >/dev/null 2>&1 || true
  fi
  ${SUDO} tee /etc/modprobe.d/99-rebot-orbbec-usb.conf >/dev/null <<'EOF'
# Orbbec Gemini 305 60fps RGB-D on Jetson: 增大 usbfs 缓冲并禁止 USB 休眠
options usbcore usbfs_memory_mb=256 autosuspend=-1
EOF
  echo "  提示：部分 JetPack 内核将 usbcore 编译为内建模块，重启后建议运行 'sudo ./tools/jetson_perf.sh'"
  echo "        或在 /boot/extlinux/extlinux.conf 的 APPEND 行末尾加入: usbcore.usbfs_memory_mb=256 usbcore.autosuspend=-1"
else
  echo "[3/4] 非 Jetson 平台，跳过 Jetson 专属 USB 参数调整"
fi

if [[ "${SETUP_JETSON_CAN}" -eq 1 ]]; then
  echo "[4/4] 加载 Jetson Orin 板载原生 mttcan 并启动 can0 (1Mbps)..."
  ${SUDO} modprobe can || true
  ${SUDO} modprobe can_raw || true
  ${SUDO} modprobe mttcan
  ${SUDO} ip link set can0 down 2>/dev/null || true
  ${SUDO} ip link set can0 type can bitrate 1000000
  ${SUDO} ip link set can0 up
  ip -details link show can0
elif [[ "${SETUP_CAN0}" -eq 1 ]]; then
  echo "[4/4] 配置 SocketCAN can0 (1Mbps)..."
  ${SUDO} ip link set can0 down 2>/dev/null || true
  ${SUDO} ip link set can0 type can bitrate 1000000
  ${SUDO} ip link set can0 up
  ip -details link show can0
else
  echo "[4/4] 跳过 SocketCAN 配置（默认使用 /dev/ttyACM0 串口；如需 CAN 请加 --can0 或 --jetson-can）"
fi

echo ""
echo "=================================================================="
echo "  硬件权限安装完成！"
echo "  1. 请【重新插拔】Orbbec Gemini 305 USB 3.0 线与达妙串口线；"
echo "  2. 如首次加入 dialout/video 组，请【注销重新登录】或执行 'newgrp dialout' 生效。"
echo "=================================================================="
