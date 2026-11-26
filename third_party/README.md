# 内置第三方硬件 SDK 与系统规则 (`third_party/`)

本目录随项目一起打包，使本项目在拷贝到任意 **x86_64 桌面主机** 或 **NVIDIA Jetson Orin (`aarch64`, JetPack 6 / Python 3.10)** 后，**无需依赖外部仓库**即可完成硬件驱动与 SDK 安装：

## 目录内容

1. **`wheels/x86_64/` — Linux x86_64 (Python 3.10) 预打包硬件 SDK Wheel**
   - `motorbridge-0.5.5-cp310-cp310-manylinux_2_17_x86_64.whl`：达妙（Damiao）电机控制 SDK（内含 `libmotor_abi.so` 与 `ws_gateway`）。
   - `pyorbbecsdk2-2.1.2-cp310-cp310-manylinux_2_27_x86_64.whl`：奥比中光 Orbbec SDK v2 Python 封装（内含 `libOrbbecSDK.so.2.9.3`，支持 Gemini 305 60fps 与硬件 D2C 对齐）。

2. **`wheels/aarch64/` — NVIDIA Jetson Orin ARM64 (JetPack 6, Python 3.10) 预打包硬件 SDK Wheel**
   - `motorbridge-0.5.5-cp310-cp310-manylinux_2_17_aarch64.manylinux2014_aarch64.whl`
   - `pyorbbecsdk2-2.1.2-cp310-cp310-manylinux_2_27_aarch64.whl`
   - 运行 `./tools/setup_env.sh` 时会自动按 `uname -m` 优先使用对应架构子目录中的本地 Wheel；若目标主机为其他 Python 小版本，pip 会自动回退从 PyPI 拉取对应版本。

3. **`udev/` — 系统硬件权限规则**
   - `99-obsensor-libusb.rules`：奥比中光全系列（含 Gemini 305 PID `0840/0841/0842`）官方 USB 权限规则。
   - `99-rebot-dm-serial.rules`：达妙 USB 串口桥 `/dev/ttyACM*` 非 root 权限 + `ID_MM_DEVICE_IGNORE=1`（防止 Ubuntu/Jetson 的 ModemManager 抢占串口）。
   - 通过 `sudo ./tools/install_udev.sh`（或 Jetson 板载 CAN 加 `--jetson-can`）一键安装。

## 其他内置静态资产

- `urdf/Rebot_B601_DM/`：完整的 Seeed reBot Arm B601-DM 官方 URDF、视觉/碰撞 STL 网格，以及由 `相机壳3.0.3MF` 转换的 `gemini305_mount_v3.stl`。
- `config/hand_landmarker.task`：MediaPipe 手部 21 关键点离线推理模型（7.5 MB，运行期零联网下载）。

## 如需打包 100% 断网全量 Wheel 仓库（可选）

在有网机器上可按目标架构提前下载全部 Python 依赖：

```bash
# x86_64 主机
pip download -d third_party/wheels/x86_64 -r requirements.txt
pip download -d third_party/wheels_mujoco --python-version 3.12 --only-binary=:all: -r requirements-mujoco.txt

# Jetson Orin (aarch64, JetPack 6, Python 3.10)
pip download -d third_party/wheels/aarch64 -r requirements.txt
```
