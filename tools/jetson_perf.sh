#!/usr/bin/env bash
# ==============================================================================
# NVIDIA Jetson Orin NX / Orin Nano 实时性能与 USB 3.0 状态配置脚本
# ==============================================================================
# 用于保障 B601-DM 200Hz MIT 控制线程 + Gemini 305 60fps RGB-D 的低抖动运行：
#   1. 切换 nvpmodel 至高性能档（默认 mode 0，Orin NX 16GB MAXN / Super 模式）；
#   2. 执行 jetson_clocks 锁定 CPU/EMC 频率，消除动态调频引起的控制周期抖动；
#   3. 设置 usbfs_memory_mb=256 与 usbcore.autosuspend=-1，防止 Gemini 305 掉帧。
#
# 用法：
#   ./tools/jetson_perf.sh --status      # 只读查看当前 Jetson 功耗/USB/CAN 状态
#   sudo ./tools/jetson_perf.sh          # 一键应用高性能配置（默认 nvpmodel -m 0）
#   sudo ./tools/jetson_perf.sh --mode 2 # 指定 nvpmodel 档位（如 25W 档）
# ==============================================================================
set -euo pipefail

STATUS_ONLY=0
POWER_MODE="0"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --status) STATUS_ONLY=1; shift ;;
    --mode) POWER_MODE="$2"; shift 2 ;;
    -h|--help)
      sed -n '2,18p' "$0"
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: $1" >&2
      exit 1
      ;;
  esac
done

_show_status() {
  echo "=================================================================="
  echo "  Jetson 实时控制状态检查 ($(uname -m))"
  echo "=================================================================="
  if [[ -f /proc/device-tree/model ]]; then
    echo "  设备型号 : $(tr -d '\0' </proc/device-tree/model)"
  fi
  if [[ -f /etc/nv_tegra_release ]]; then
    echo "  L4T 版本 : $(head -n 1 /etc/nv_tegra_release)"
  fi
  if command -v nvpmodel >/dev/null 2>&1; then
    echo "  功耗模式 : $(nvpmodel -q 2>/dev/null | tr '\n' ' ' || echo '需 sudo 查看')"
  else
    echo "  功耗模式 : 未检测到 nvpmodel（非 Jetson 系统）"
  fi
  if [[ -r /sys/module/usbcore/parameters/usbfs_memory_mb ]]; then
    echo "  usbfs_mb : $(cat /sys/module/usbcore/parameters/usbfs_memory_mb) MB (推荐 >= 128)"
  fi
  if [[ -r /sys/module/usbcore/parameters/autosuspend ]]; then
    echo "  USB休眠  : $(cat /sys/module/usbcore/parameters/autosuspend) (推荐 -1 禁用)"
  fi
  echo "  串口设备 : $(ls /dev/ttyACM* 2>/dev/null | tr '\n' ' ' || echo '无')"
  if ip link show can0 >/dev/null 2>&1; then
    echo "  CAN0 状态: $(ip -br link show can0)"
  fi
  echo "=================================================================="
}

if [[ "${STATUS_ONLY}" -eq 1 ]]; then
  _show_status
  exit 0
fi

SUDO=""
if [[ "${EUID}" -ne 0 ]]; then
  SUDO="sudo"
fi

if [[ ! -f /etc/nv_tegra_release ]]; then
  echo "[提示] 未检测到 /etc/nv_tegra_release，当前主机似乎不是 Jetson。"
  _show_status
  exit 0
fi

echo "[1/3] 设置 Jetson 功耗模式: nvpmodel -m ${POWER_MODE}"
if command -v nvpmodel >/dev/null 2>&1; then
  ${SUDO} nvpmodel -m "${POWER_MODE}" || echo "[警告] nvpmodel 切换失败，请用 'sudo nvpmodel -q --verbose' 查看可用档位"
fi

echo "[2/3] 锁定 CPU/内存时钟: jetson_clocks"
if command -v jetson_clocks >/dev/null 2>&1; then
  ${SUDO} jetson_clocks || echo "[警告] jetson_clocks 执行失败"
fi

echo "[3/3] 设置 Gemini 305 USB 3.0 缓冲 (usbfs_memory_mb=256, autosuspend=-1)"
echo 256 | ${SUDO} tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null 2>&1 || true
echo -1 | ${SUDO} tee /sys/module/usbcore/parameters/autosuspend >/dev/null 2>&1 || true

_show_status
