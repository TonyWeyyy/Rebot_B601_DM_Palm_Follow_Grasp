#!/usr/bin/env python3
"""
Seeed reBot Arm B601-DM + Orbbec Gemini 305 跨平台环境与静态资产自检工具
========================================================================
支持 x86_64 桌面与 NVIDIA Jetson Orin (aarch64)，可在任意拷贝后的项目目录内运行：
  1. 平台架构 / Jetson 识别与 ROS 环境变量冲突检查；
  2. 全部主控 Python 依赖（支持 Jetson 无 Pinocchio 时校验内置纯 NumPy 精确引擎）；
  3. 项目内置全部静态资产（DM URDF、支架 STL/3MF、MediaPipe 模型、配置文件、
     x86_64 & aarch64 双架构 SDK Wheel 与 Orbbec/串口 udev 规则）；
  4. MuJoCo Studio 4.0+ 数字孪生 Python 环境检查；
  5. Linux / Jetson 硬件权限、USB 缓冲 (`usbfs_memory_mb`)、功耗档与外设就绪度。
"""

from __future__ import annotations

import grp
import importlib
import importlib.metadata
import importlib.util
import os
import platform
import pwd
import shutil
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


def _ok(msg: str) -> None:
    print(f"  [OK]   {msg}")


def _warn(msg: str) -> None:
    print(f"  [WARN] {msg}")


def _fail(msg: str) -> None:
    print(f"  [FAIL] {msg}")


def _detect_jetson() -> tuple[bool, str]:
    tegra = Path("/etc/nv_tegra_release")
    model_file = Path("/proc/device-tree/model")
    is_jetson = tegra.is_file()
    info_parts: list[str] = []
    if model_file.is_file():
        try:
            txt = model_file.read_bytes().replace(b"\x00", b"").decode("utf-8", errors="ignore").strip()
            if txt:
                info_parts.append(txt)
                if "jetson" in txt.lower() or "orin" in txt.lower():
                    is_jetson = True
        except Exception:
            pass
    if tegra.is_file():
        try:
            line = tegra.read_text(encoding="utf-8", errors="ignore").splitlines()[0].strip()
            if line:
                info_parts.append(line)
        except Exception:
            pass
    return is_jetson, " | ".join(info_parts)


def _find_mujoco_python() -> Path | None:
    candidates: list[Path] = []
    env_py = os.environ.get("MUJOCO_STUDIO_PYTHON", "").strip()
    if env_py:
        candidates.append(Path(env_py))
    candidates.extend(
        [
            PROJECT_ROOT / ".venv_mujoco" / "bin" / "python",
            PROJECT_ROOT / ".venv" / "bin" / "python",
            Path(sys.executable),
            Path.home() / ".venvs" / "mujoco-studio" / "bin" / "python",
        ]
    )
    seen: set[str] = set()
    for p in candidates:
        sp = str(p)
        if sp in seen or not p.is_file() or not os.access(p, os.X_OK):
            continue
        seen.add(sp)
        try:
            r = subprocess.run(
                [sp, "-c", "import mujoco; from mujoco import studio; print(mujoco.__version__)"],
                capture_output=True,
                text=True,
                timeout=10,
                check=False,
            )
            if r.returncode == 0:
                return p
        except Exception:
            continue
    return None


def main() -> int:
    failures = 0
    arch = platform.machine()
    is_jetson, jetson_info = _detect_jetson()

    print("=" * 72)
    print("  reBot Arm B601-DM + Gemini 305 独立项目环境自检")
    print(f"  项目路径: {PROJECT_ROOT}")
    print(f"  平台架构: {arch}" + (f" (Jetson: {jetson_info})" if is_jetson else ""))
    print(f"  Python  : {sys.executable} ({sys.version.split()[0]})")
    print("=" * 72)

    # 1. ROS 环境变量冲突检查
    print("\n[1] 环境变量与 ROS 冲突检查")
    ros_conflict = False
    for var in ("PYTHONPATH", "LD_LIBRARY_PATH"):
        val = os.environ.get(var, "")
        if "/opt/ros" in val:
            ros_conflict = True
            _warn(
                f"{var} 包含 /opt/ros 路径，可能导致 Pinocchio 段错误。"
                "建议运行前剔除 /opt/ros。"
            )
    if not ros_conflict:
        _ok("PYTHONPATH / LD_LIBRARY_PATH 无 /opt/ros 冲突")

    # 2. 主控 Python 依赖检查
    print("\n[2] 主控核心 Python 依赖检查")
    pkgs = [
        ("numpy", "numpy"),
        ("yaml", "PyYAML"),
        ("cv2", "opencv-contrib-python"),
        ("scipy", "scipy"),
        ("pynput", "pynput"),
        ("mediapipe", "mediapipe"),
        ("pinocchio", "pin (PyPI) / pinocchio (conda-forge)"),
        ("motorbridge", "motorbridge"),
        ("pyorbbecsdk", "pyorbbecsdk2"),
    ]
    for mod_name, dist_hint in pkgs:
        if mod_name == "pynput" and not os.environ.get("DISPLAY"):
            if importlib.util.find_spec("pynput") is not None:
                try:
                    ver = importlib.metadata.version("pynput")
                except Exception:
                    ver = "installed"
                _ok(f"{mod_name:14s} -> {ver} (无 DISPLAY/SSH 模式将自动回退 POSIX 终端按键)")
            else:
                _warn(f"{mod_name:14s} 未安装（无头 SSH 模式下可使用内置 POSIX 终端按键）")
            continue
        try:
            mod = importlib.import_module(mod_name)
            ver = getattr(mod, "__version__", "installed")
            if mod_name == "cv2" and not hasattr(mod, "aruco"):
                _fail("cv2 已安装但缺少 cv2.aruco，请安装 opencv-contrib-python")
                failures += 1
            else:
                _ok(f"{mod_name:14s} -> {ver}")
        except Exception as exc:
            if mod_name == "pinocchio":
                try:
                    from servo_v14.robot_backend import PureNumpyDMKinematics
                    import numpy as np

                    kin = PureNumpyDMKinematics()
                    p, _, J = kin.fk_and_jacobian(np.array([0.0, -0.5, -0.5, 0.0, 0.0, 0.0]))
                    if p.shape == (3,) and J.shape == (6, 6):
                        _warn(
                            "pinocchio 未安装，但内置精确纯 NumPy B601-DM 运动学/重力引擎可用 "
                            "（v14/v15 真机与仿真均正常运行；手眼联合标定脚本仍建议安装 pinocchio）"
                        )
                        continue
                except Exception:
                    pass
            _fail(f"{mod_name:14s} 导入失败 ({exc}) | 安装包: {dist_hint}")
            failures += 1

    # 3. 项目内置静态资产与 SDK 包完整性
    print("\n[3] 项目内置资产与多架构 SDK 完整性检查")
    required_files = [
        "urdf/Rebot_B601_DM/urdf/ReBot_Arm_DM.urdf",
        "urdf/Rebot_B601_DM/meshes/visual/gemini305_mount_v3.stl",
        "相机壳3.0.3MF",
        "config/hand_landmarker.task",
        "config/dm_mit.yaml",
        "config/v14.yaml",
        "config/v14_jetson.yaml",
        "config/grasp.yaml",
        "config/home_pose.json",
        "config/hand_eye_calibration.json",
        "config/joint_offset.json",
        "third_party/udev/99-obsensor-libusb.rules",
        "third_party/udev/99-rebot-dm-serial.rules",
    ]
    for rel in required_files:
        p = PROJECT_ROOT / rel
        if p.is_file():
            size_kb = p.stat().st_size / 1024.0
            _ok(f"{rel} ({size_kb:.1f} KB)")
        else:
            _fail(f"缺失必要内置文件: {rel}")
            failures += 1

    for wheel_arch in ("x86_64", "aarch64"):
        wdir = PROJECT_ROOT / "third_party" / "wheels" / wheel_arch
        wheels = sorted(wdir.glob("*.whl")) if wdir.is_dir() else []
        if wheels:
            tag = "当前架构" if wheel_arch == arch else "跨架构预置"
            _ok(f"内置 {wheel_arch} SDK Wheel ({len(wheels)} 个, {tag}): {', '.join(w.name for w in wheels)}")
        else:
            _warn(f"third_party/wheels/{wheel_arch}/ 暂无本地 Wheel（联机可通过 pip 自动下载）")

    # 4. MuJoCo Studio 4.0+ 数字孪生环境检查
    print("\n[4] MuJoCo Studio 4.0+ 数字孪生环境检查 (v15)")
    mj_py = _find_mujoco_python()
    if mj_py is not None:
        r = subprocess.run(
            [str(mj_py), "-c", "import mujoco; print(mujoco.__version__)"],
            capture_output=True,
            text=True,
            check=False,
        )
        _ok(f"MuJoCo Studio 可用: {mj_py} (mujoco {r.stdout.strip()})")
    else:
        hint = (
            "Jetson 推荐 PC 端跨机运行 MuJoCo（或加 ./tools/setup_env.sh --with-mujoco 本机安装）"
            if is_jetson
            else "运行 ./tools/setup_env.sh --mujoco-only 可自动创建项目内 .venv_mujoco"
        )
        _warn(f"未检测到本地 MuJoCo Studio 4.0+ 环境（不影响 v14 视觉伺服；{hint}）")

    # 5. Linux / Jetson 硬件权限与外设状态（仅提示，不计入失败）
    print("\n[5] Linux / Jetson 硬件权限、USB 缓冲与外设状态（仅提示）")
    for rules_name in ("99-obsensor-libusb.rules", "99-rebot-dm-serial.rules"):
        udev_dst = Path("/etc/udev/rules.d") / rules_name
        if udev_dst.is_file():
            _ok(f"系统已安装 udev 规则: {udev_dst}")
        else:
            _warn(f"尚未安装 {rules_name}，接入真机前请运行: sudo ./tools/install_udev.sh")

    usbfs_path = Path("/sys/module/usbcore/parameters/usbfs_memory_mb")
    if usbfs_path.is_file():
        try:
            mb = int(usbfs_path.read_text(encoding="utf-8").strip())
            if mb >= 128:
                _ok(f"USB 缓冲区 usbfs_memory_mb = {mb} MB")
            elif is_jetson:
                _warn(f"Jetson 当前 usbfs_memory_mb = {mb} MB (<128)，跑 Gemini 305 60fps 建议运行 sudo ./tools/jetson_perf.sh")
            else:
                _ok(f"USB 缓冲区 usbfs_memory_mb = {mb} MB（桌面通常够用；如 60fps 丢帧可调至 128+）")
        except Exception:
            pass

    if is_jetson:
        auto_path = Path("/sys/module/usbcore/parameters/autosuspend")
        if auto_path.is_file():
            try:
                val = auto_path.read_text(encoding="utf-8").strip()
                if val == "-1":
                    _ok("Jetson USB autosuspend = -1（已禁用自动休眠）")
                else:
                    _warn(f"Jetson USB autosuspend = {val}，建议运行 sudo ./tools/jetson_perf.sh 设为 -1")
            except Exception:
                pass
        if shutil.which("nvpmodel"):
            r = subprocess.run(["nvpmodel", "-q"], capture_output=True, text=True, check=False)
            summary = " ".join(r.stdout.strip().splitlines()) if r.stdout else "需 sudo 查询"
            _ok(f"Jetson nvpmodel 状态: {summary}")

    try:
        user = pwd.getpwuid(os.getuid()).pw_name
        user_groups = {g.gr_name for g in grp.getgrall() if user in g.gr_mem}
        user_groups.add(grp.getgrgid(os.getgid()).gr_name)
        for gname in ("dialout", "video"):
            if gname in user_groups:
                _ok(f"当前用户 '{user}' 已在 '{gname}' 组")
            else:
                _warn(f"当前用户 '{user}' 不在 '{gname}' 组（请运行 sudo ./tools/install_udev.sh 并重新登录）")
    except Exception:
        pass

    tty_acms = sorted(Path("/dev").glob("ttyACM*"))
    if tty_acms:
        _ok(f"检测到串口设备: {', '.join(str(p) for p in tty_acms)}")
    else:
        _warn("当前未检测到 /dev/ttyACM*（若尚未连接 B601-DM 或改用 can0 可忽略）")

    if shutil.which("ip"):
        r = subprocess.run(["ip", "-br", "link", "show", "can0"], capture_output=True, text=True, check=False)
        if r.returncode == 0 and r.stdout.strip():
            _ok(f"检测到 SocketCAN 接口: {r.stdout.strip()}")

    if shutil.which("lsusb"):
        r = subprocess.run(["lsusb"], capture_output=True, text=True, check=False)
        orbbec_lines = [ln for ln in r.stdout.splitlines() if "2bc5:" in ln.lower()]
        if orbbec_lines:
            _ok(f"检测到 Orbbec USB 设备: {orbbec_lines[0].strip()}")
        else:
            _warn("当前未检测到 Orbbec USB 相机 (2bc5:*)（若尚未插相机可忽略）")

    print("\n" + "=" * 72)
    if failures == 0:
        print("  自检通过！主控依赖与项目内资产全部完整。")
        print("=" * 72)
        return 0
    print(f"  自检发现 {failures} 项失败，请按上述提示修复后重试。")
    print("=" * 72)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
