#!/usr/bin/env bash
# ==============================================================================
# NVIDIA Jetson Orin Nano 8GB / Orin NX 实时性能、内存与 USB 3.0 优化脚本
# ==============================================================================
# 核心功能：
#   1. 智能识别最高功耗档（自动兼容 Orin Nano JetPack 6.1 15W 与 JetPack 6.2 Super 25W/MAXN_SUPER）；
#   2. 执行 jetson_clocks 锁定 6/8 核 Cortex-A78AE 频率，消除 200Hz MIT 周期抖动；
#   3. 设置 usbfs_memory_mb=256 与 usbcore.autosuspend=-1，防止 Gemini 305 掉帧/断连；
#   4. 针对 Orin Nano 8GB 提供一键 4GB Swap 创建（--setup-swap 4G）与无头启动切换
#      （--headless-boot 关闭 GNOME 桌面立省 ~1.8GB 内存，随时 --desktop-boot 恢复）。
#
# 用法：
#   ./tools/jetson_perf.sh --status          # 只读查看当前型号/功耗/内存/Swap/USB/启动模式
#   sudo ./tools/jetson_perf.sh              # 一键切换至最高性能档 + 锁频 + USB 256MB 缓冲
#   sudo ./tools/jetson_perf.sh --mode 2     # 手动指定 nvpmodel 档位 ID
#   sudo ./tools/jetson_perf.sh --setup-swap 4G  # 在 NVMe 上创建 4GB /swapfile（防 8GB OOM）
#   sudo ./tools/jetson_perf.sh --headless-boot  # 设为纯命令行启动（重启生效，立省 ~1.8GB RAM）
#   sudo ./tools/jetson_perf.sh --desktop-boot   # 恢复 Ubuntu 图形桌面启动（重启生效）
# ==============================================================================
set -euo pipefail

STATUS_ONLY=0
POWER_MODE="auto"
SETUP_SWAP_SIZE=""
BOOT_TARGET=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --status) STATUS_ONLY=1; shift ;;
    --mode) POWER_MODE="$2"; shift 2 ;;
    --setup-swap) SETUP_SWAP_SIZE="$2"; shift 2 ;;
    --headless-boot) BOOT_TARGET="multi-user.target"; shift ;;
    --desktop-boot) BOOT_TARGET="graphical.target"; shift ;;
    -h|--help)
      sed -n '2,22p' "$0"
      exit 0
      ;;
    *)
      echo "[错误] 未知参数: $1" >&2
      exit 1
      ;;
  esac
done

_detect_best_nvpmodel_id() {
  local conf="/etc/nvpmodel.conf"
  if [[ ! -r "${conf}" ]]; then
    echo "0"
    return
  fi
  # 按优先级匹配 Orin Nano Super / Orin NX 最高性能档位名称：MAXN_SUPER > MAXN > 25W > 15W > 0
  for kw in "MAXN_SUPER" "MAXN" "25W" "15W"; do
    local id
    id="$(awk -v k="${kw}" '
      /POWER_MODEL/ && /ID=/ && toupper($0) ~ k {
        for (i=1; i<=NF; i++) {
          if ($i ~ /^ID=/) {
            gsub(/^ID=|>/, "", $i)
            print $i
            exit
          }
        }
      }' "${conf}" 2>/dev/null || true)"
    if [[ -n "${id}" ]]; then
      echo "${id}"
      return
    fi
  done
  echo "0"
}

_show_status() {
  echo "=================================================================="
  echo "  Jetson 实时控制与内存状态检查 ($(uname -m))"
  echo "=================================================================="
  if [[ -r /proc/device-tree/model ]]; then
    echo "  设备型号 : $(tr -d '\0' </proc/device-tree/model)"
  fi
  if [[ -r /etc/nv_tegra_release ]]; then
    echo "  L4T 版本 : $(head -n 1 /etc/nv_tegra_release)"
  fi
  if [[ -r /proc/meminfo ]]; then
    awk '
      /^MemTotal:/     { mt=$2/1024 }
      /^MemAvailable:/ { ma=$2/1024 }
      /^SwapTotal:/    { st=$2/1024 }
      /^SwapFree:/     { sf=$2/1024 }
      END {
        printf "  物理内存 : 可用 %.0f MB / 总计 %.0f MB\n", ma, mt
        printf "  Swap 空间: 可用 %.0f MB / 总计 %.0f MB (Orin Nano 8GB 推荐 >= 4096 MB)\n", sf, st
      }
    ' /proc/meminfo
  fi
  if command -v systemctl >/dev/null 2>&1; then
    echo "  默认启动 : $(systemctl get-default 2>/dev/null || echo '未知')"
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

if [[ -n "${SETUP_SWAP_SIZE}" ]]; then
  echo "[Swap] 配置 ${SETUP_SWAP_SIZE} 持久化交换文件 -> /swapfile"
  if swapon --show=NAME 2>/dev/null | grep -qx "/swapfile"; then
    echo "  /swapfile 已处于启用状态，跳过重复创建。"
  else
    ${SUDO} fallocate -l "${SETUP_SWAP_SIZE}" /swapfile
    ${SUDO} chmod 600 /swapfile
    ${SUDO} mkswap /swapfile
    ${SUDO} swapon /swapfile
    if ! grep -q '^/swapfile ' /etc/fstab; then
      echo '/swapfile none swap sw 0 0' | ${SUDO} tee -a /etc/fstab >/dev/null
    fi
    echo "  [完成] /swapfile (${SETUP_SWAP_SIZE}) 已启用并写入 /etc/fstab。"
  fi
fi

if [[ -n "${BOOT_TARGET}" ]]; then
  echo "[启动模式] 切换 systemd 默认目标 -> ${BOOT_TARGET}"
  ${SUDO} systemctl set-default "${BOOT_TARGET}"
  if [[ "${BOOT_TARGET}" == "multi-user.target" ]]; then
    echo "  重启后将进入纯命令行无头模式（释放约 1.8GB 内存）；随时可运行 --desktop-boot 恢复桌面。"
  else
    echo "  重启后将恢复图形桌面模式。"
  fi
fi

if [[ ! -f /etc/nv_tegra_release ]]; then
  echo "[提示] 未检测到 /etc/nv_tegra_release，跳过 Jetson 专属 nvpmodel/jetson_clocks 步骤。"
  _show_status
  exit 0
fi

if [[ "${POWER_MODE}" == "auto" ]]; then
  POWER_MODE="$(_detect_best_nvpmodel_id)"
  echo "[自动识别] 选用最高可用功耗档 ID=${POWER_MODE}"
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
