# AGENTS.md

## Overview
Fully self-contained, cross-platform (`x86_64` + **NVIDIA Jetson Orin Nano 8GB / Orin NX 16GB `aarch64`, JetPack 6**) eye-in-hand visual servoing + pinch grasping + bidirectional MuJoCo Studio 3D digital twin for the **Seeed reBot Arm B601-DM** (Damiao `4340P` × 3 + `4310` × 4 on `/dev/ttyACM0 @ 921600` `dm-serial`, or SocketCAN `can0` / Jetson onboard `mttcan`) and **Orbbec Gemini 305** RGB-D wrist camera (mounted in 3D-printed bracket `相机壳3.0.3MF`, 0° tilt).

This repository must remain **100% standalone and relocatable** with zero hardcoded host paths or external repository dependencies:
- Bundled multi-arch hardware SDK wheels & udev rules: `third_party/wheels/x86_64/` + `third_party/wheels/aarch64/` (`motorbridge`, `pyorbbecsdk2`), `third_party/udev/99-obsensor-libusb.rules`, and `third_party/udev/99-rebot-dm-serial.rules`.
- Bundled URDF/STL & vision model: `urdf/Rebot_B601_DM/` (including `gemini305_mount_v3.stl`) and `config/hand_landmarker.task`.
- Standalone setup/check/calibration scripts: `tools/install_udev.sh`, `tools/jetson_perf.sh` (auto-detects highest `nvpmodel` including Orin Nano Super 25W/MAXN_SUPER, optional `--setup-swap 4G` & `--headless-boot`), `tools/setup_env.sh` (auto-detects Orin Nano 8GB using `requirements-orin-nano.txt` + `pyorbbecsdk2 --no-deps` to avoid 8GB OOM and `open3d` bloat), `tools/run_orin_nano.sh`, `tools/check_env.py`, `tools/dm_zero_calibrate.py`, `tools/run_all_tests.sh`.
- Main entrypoints (clean v14 + v15 architecture):
  - `palm_follow_v14.py`: 4-layer decoupled visual servoing + pinch grasping (`servo_v14/` — `estimator.py` → `state_machine.py` → `controller_pbvs.py` → `safety_shield.py` + `depth_guard.py` + `robot_backend.py` + `terminal_keys.py` + `mjpeg_preview.py` + non-blocking UDP `mujoco_bridge.py`). Auto-selects `config/v14_orin_nano.yaml` on Orin Nano 8GB (`640x480@30fps`, `cv_num_threads=2`, `detector_backend=cpu`; optional `--fps 60 --detect-stride 2` for Super 25W hybrid optical-flow mode; optional `--mjpeg-port 8081` headless browser preview), `config/v14_jetson.yaml` on Orin NX, or `config/v14.yaml` on desktop, with `SIGTERM` drop-proof safe shutdown.
  - `mujoco_teleop_v15.py` + `servo_v14/teleop_bridge.py`: bidirectional MuJoCo ↔ B601-DM safe teleoperation over dual UDP `:19870`/`:19871`, supporting both localhost and Jetson ↔ PC LAN split deployment (`--state-host`, `--cmd-bind`, `--allow-cmd-host` source-IP whitelist, 8° anti-jump gate, 250ms watchdog, 0.42m soft wall, Z≥0.05m table guard, M6 interlock, `SIGTERM` drop-proof safe shutdown).
  - `mujoco_twin/` (`run_twin.py`, `scene_builder.py`, `sim_plugin.py`, `viewer_plugin.py`, `messages.py`) + `tools/launch_mujoco_twin.sh`: MuJoCo Studio 4.0+ 3D digital twin (`--native` / `--web`, LAN `--udp-bind` / `--cmd-host`) loading local `urdf/Rebot_B601_DM/urdf/ReBot_Arm_DM.urdf` + `gemini305_mount_v3.stl` + Gemini 305 visual body + 0.35m optical aim ray.
- Low-level DM controller:
  - `dm_controller.py` + `config/dm_mit.yaml`: standalone 200Hz Damiao `motorbridge` controller (primary `mit` mode with P0~P4 features; optional `posvel` fallback for arm joints with J7 always in MIT).
  - `rebot_adapter.py`: radian/meter/Nm adapter with isolated Pinocchio gravity model (`0.095 kg` Gemini 305 + bracket payload) and automatic exact `PureNumpyDMKinematics` fallback when Pinocchio is unavailable on ARM64.
- Camera:
  - `gemini305_camera.py`: Orbbec SDK v2 (`pyorbbecsdk2`) Pipeline with HW D2C alignment, native 60fps profile selection, dynamic `depth_scale`, and `latest_only=True` queue drain.

## Verification & Testing (Run Before Hardware)
```bash
# 0. Environment & bundled-asset self-check (supports x86_64 & Jetson aarch64)
python3 tools/check_env.py

# 1. Run all 5 offline verification suites (auto-selects .venv / .venv_mujoco)
./tools/run_all_tests.sh
```

## Critical B601-DM Hardware Facts & Safety Red Lines
1. **DM Joint Direction & Negative Limits**: Unlike B601-RS, B601-DM `joint2` and `joint3` operate in **`[-3.14, 0.0]` rad** (all 6 joint axes are opposite to RS). Never copy RS joint angles, `home_pose.json`, or `hand_eye_calibration.json` into this project.
2. **Damiao Motor Torque Ceilings**: `4340P` (J1~J3) peak = **27 Nm** (`tau_limit_nm` = `[15, 21, 21] Nm`); `4310` (J4~J6, J7) peak = **7 Nm** (`tau_limit_nm` = `[5.2, 5.2, 5.2] Nm`).
3. **Safe Shutdown Sequence**: Never disable motors in a non-zero / elevated pose. Every exit path (`q`, `Esc`, `Ctrl+C`, AND `SIGTERM`) executes: stop 100Hz servo → `backend.hold()` → `backend.stop()` (minimum-jerk smooth return to `[0, 0, 0, 0, 0, 0]` with gravity fade → settle → `disable_all()` → `shutdown()` → `close()`).
4. **Single Bus Master**: All 7 motors (6 arm + J7 gripper) share one `ReBotDMMITController` instance on `/dev/ttyACM0` (or `can0`). Never open a second `motorbridge` handle.
5. **4-Layer Depth Protection Chain (Gemini 305)**: `min_valid_m = 0.07m`, `max_valid_m = 1.20m`, default target `distance = 0.35m`. Do not bypass `DepthProtectionChain`.
6. **Workspace & Speed Limits**: Horizontal radial soft wall `<= 0.42m` (`+5cm` hard breach → return HOME), Cartesian `max_vel <= 0.85 m/s`, `HARD_MAX_JOINT_SPEED = [2.5, 2.0, 2.0, 3.0, 3.0, 2.5] rad/s` (J2/J3 `<= 2.0 rad/s`).
7. **LAN Teleop Safety**: When binding `mujoco_teleop_v15.py` to `--cmd-bind 0.0.0.0` on Jetson, enforce `--allow-cmd-host <PC_IP>` source-IP filtering.
8. **ROS Conflict**: Strip `/opt/ros/*` from `PYTHONPATH` / `LD_LIBRARY_PATH` if Pinocchio segfaults.
