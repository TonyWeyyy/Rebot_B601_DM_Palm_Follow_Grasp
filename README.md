# reBot Arm B601-DM + Orbbec Gemini 305 视觉伺服抓取与双向数字孪生系统（v14 + v15）

本工程为 **100% 独立自包含、支持 x86_64 桌面与 NVIDIA Jetson Orin (`aarch64`, JetPack 6) 双平台** 的完整项目：内置 B601-DM 完整 URDF/STL 模型、3D 打印支架源文件与网格、MediaPipe 离线手掌模型、**x86_64 与 ARM64 双架构** 达妙 `motorbridge` 与奥比中光 `pyorbbecsdk2` 硬件 SDK Wheel、Orbbec 与串口防抢占 udev 规则、达妙机械零位标定工具以及一键环境搭建与自检脚本。**将本文件夹整体打包拷贝到任意机器（PC 或 Jetson Orin）、任意路径下，严格按本 README 操作即可独立完成全部安装、标定与运行，无需关联任何外部仓库。**

**一台 Gemini 305 相机、一次 MediaPipe 21 点检测、一条达妙总线句柄**，即可同时实现：
- **v14 视觉伺服 + 捏合抓取（`palm_follow_v14.py`）**：手掌 3D 移动 → 机械臂保持固定姿态与期望距离（默认 `0.35 m`）实时跟随；拇指-食指捏合 → J7 刚性平行夹爪同步开合；
- **v15 MuJoCo 双向数字孪生遥操作（`mujoco_teleop_v15.py` + `mujoco_twin/`）**：在本地原生 3D 窗口拖动关节滑块或 3D 目标球（实时 DLS IK）平滑安全驱动外部真机，同时 100Hz 实时镜像真机关节角、速度、力矩、温度与 3D 掌心，并支持一键回放历史高频 CSV 日志。

---

## 目录

1. [硬件清单与关键差异须知](#一硬件清单与关键差异须知)
2. [项目目录结构（全部内置依赖一览）](#二项目目录结构全部内置依赖一览)
3. [从零开始的环境配置与安装教程](#三从零开始的环境配置与安装教程)
4. [离线全量验证（无需硬件）](#四离线全量验证无需硬件)
5. [首次真机上电与标定 SOP（必做）](#五首次真机上电与标定-sop必做)
6. [日常运行指南（v14 视觉伺服 & v15 双向遥操作）](#六日常运行指南v14-视觉伺服--v15-双向遥操作)
7. [配置文件说明与按症状调参速查](#七配置文件说明与按症状调参速查)
8. [常见故障排查 FAQ](#八常见故障排查-faq)
9. [每次运行前安全 Checklist](#九每次运行前安全-checklist)
10. [NVIDIA Jetson Orin NX 16GB（JetPack 6）部署与跨机联动指南](#十nvidia-jetson-orin-nx-16gbjetpack-6部署与跨机联动指南)

---

## 一、硬件清单与关键差异须知

### 1.1 硬件组成
| 组件 | 型号 / 规格 | 说明 |
|---|---|---|
| **机械臂** | Seeed reBot Arm B601-DM | 6 轴 + J7 夹爪：J1~J3 为达妙 `4340P`（峰值 `27 Nm`，ID `0x01~0x03`，反馈 `0x11~0x13`）；J4~J6 与 J7 夹爪为达妙 `4310`（峰值 `7 Nm`，ID `0x04~0x07`，反馈 `0x14~0x17`） |
| **末端夹爪** | 官方刚性平行夹爪 | 达妙 `4310` 驱动，URDF 对应对称平动副 `finger_left` (`0 ~ +28.5 mm`) 与 `finger_right` (`-28.5 ~ 0 mm`) |
| **通信接口** | 达妙 USB 串口桥（默认）或 SocketCAN | 默认 `/dev/ttyACM0 @ 921600`（`dm-serial`）；若使用 USB-CAN 可在 `config/dm_mit.yaml` 中将 `channel` 改为 `can0` |
| **RGB-D 相机** | Orbbec Gemini 305（USB 版） | 超紧凑全局快门双目相机（`42×42×23 mm`，`~68 g`，基线 `18 mm`），理想工作距离 **`0.07 ~ 0.50 m`**（有效 `0.04 ~ 1.0 m+`），支持 `60fps` 彩色/深度与硬件 D2C 对齐 |
| **相机支架** | [相机壳3.0.3MF](相机壳3.0.3MF) | 3D 打印包裹外壳（外廓 `44×58×26 mm`，内腔 `42×42×26 mm`，两侧预留 `18×10 mm` 走线口，**0° 倾角前视**），建议用 PLA / PETG、0.2mm 层高、≥25% 填充打印；整套腕部相机载荷仅 `~0.095 kg` |
| **推荐主机** | Ubuntu 22.04 x86_64 或 **NVIDIA Jetson Orin NX 16GB**（JetPack 6 / Ubuntu 22.04，亦兼容 Orin Nano Super 8GB / AGX Orin） | 需至少 1 个 **USB 3.0/3.2 接口**（接 Gemini 305 跑 60fps）与 1 个 USB 接口（接 DM 串口桥）或 Jetson 板载 CAN (`mttcan`) |

### 1.2 从 B601-RS 迁移到 B601-DM 的重要安全须知
> **严禁直接复制 B601-RS 的关节角、标定文件或底层控制参数到本项目！**
> 1. **关节方向相反、J2/J3 为负区间**：B601-DM 的 `joint2` 和 `joint3` 工作范围是 **`[-3.14, 0.0]` rad**（零位支架在 `0`，抬臂为负角），全部 6 个关节轴向均与 RS 相反。
> 2. **4310 腕部力矩与刚度更柔和**：DM 手腕电机 `4310` 峰值力矩为 `7 Nm`（RS00 为 `14 Nm`），底层安全监督默认按 `75%` 封顶在 `[15.0, 21.0, 21.0, 5.2, 5.2, 5.2] Nm`，腕部 MIT 刚度 `kp=18`，因此**必须运行 `p3_gravity_verify.py` 校准重力前馈**以消除静态下垂。
> 3. **严禁非零位失能**：任何退出路径都必须走完「停止伺服 → 原地保持 → minimum-jerk 缓慢回零到位 → 失能」。
> 4. **与 RS 的复用边界**：**可复用**的仅是 Gemini 305 相机本体、[相机壳3.0.3MF](相机壳3.0.3MF) 打印件、`0.095 kg` 相机负载重量与桌面 ArUco 标定板；**必须在本机重标**的有 `config/home_pose.json`、`config/grasp.yaml`、`config/dm_mit.yaml`（`gravity_ff_scale`）、`config/hand_eye_calibration.json` 与 `config/joint_offset.json`（因 DM 末端法兰几何、电机力矩特性与个体装配零偏均不同于 RS）。

---

## 二、项目目录结构（全部内置依赖一览）

```text
<项目根目录>/
├── palm_follow_v14.py             ★ v14.0 视觉伺服 + 捏合抓取主程序（支持真机 / --sim / --no-arm）
├── mujoco_teleop_v15.py           ★ v15.0 MuJoCo <-> B601-DM 双向遥操作桥接主程序
├── dm_controller.py               ★ B601-DM 独立 200Hz 底层控制器（MIT P0~P4 主模式 + POS_VEL 可选回退）
├── rebot_adapter.py               ★ DM 机械臂适配层（Pinocchio 双模型隔离 + J7 单总线夹爪通道）
├── gemini305_camera.py            ★ Orbbec Gemini 305 SDK v2 采集层（60fps + HW D2C + 队列排空）
├── palm_detection.py              MediaPipe Tasks 21点检测 + 迟滞 ROI + LK 光流桥接 + 捏合提取
├── kalman_target.py               3D 恒速卡尔曼滤波（分轴 Q + 深度冻结期 XY 子空间更新）
├── servo_timing.py                时间校准 / 静止保持 / 伸展调度 / 软墙 / 振荡阻尼等纯算法
├── servo_diagnostics.py           非阻塞高频 CSV 诊断记录器
├── requirements.txt               主控制 Python 环境依赖清单（含 PyPI `pin`、SDK、OpenCV 等）
├── requirements-mujoco.txt        MuJoCo Studio 4.0+ 数字孪生环境依赖清单
├── 相机壳3.0.3MF                   Gemini 305 腕部外壳 3D 打印源文件
│
├── third_party/                   ★ 内置第三方硬件 SDK 与系统规则（零外部仓库依赖）
│   ├── wheels/
│   │   ├── x86_64/                Linux x86_64 cp310 预打包 SDK Wheel（motorbridge + pyorbbecsdk2）
│   │   └── aarch64/               Jetson Orin ARM64 cp310 预打包 SDK Wheel（motorbridge + pyorbbecsdk2）
│   ├── udev/
│   │   ├── 99-obsensor-libusb.rules  奥比中光全系（含 Gemini 305）官方 USB udev 权限规则
│   │   └── 99-rebot-dm-serial.rules  达妙串口权限 + 禁止 ModemManager 抢占规则
│   └── README.md                  双架构 SDK 版本与全离线 wheel 打包说明
│
├── servo_v14/                     ★ v14/v15 四层解耦核心包
│   ├── config.py / types.py       不可变 dataclass 配置与强类型数据契约
│   ├── depth_guard.py             4 层深度保护链（适配 Gemini 305 0.07 ~ 1.20 m）
│   ├── estimator.py               层 1：统一世界系目标估计器（基于 200Hz 实测 q_act）
│   ├── state_machine.py           层 2：8 态显式有限状态机
│   ├── controller_pbvs.py         层 3：固定参考光轴世界系 PBVS 控制器
│   ├── safety_shield.py           层 4：不可绕过安全护盾（软/硬墙、奇异、限位缓冲、硬速红线）
│   ├── robot_backend.py           可插拔后端（HardwareRobotBackend + DM DigitalTwinBackend + 纯 NumPy 回退）
│   ├── pipeline.py                四层流水线编排器
│   ├── terminal_keys.py           桌面 pynput + 无头 SSH POSIX 终端统一按键监听
│   ├── mujoco_bridge.py           双通道非阻塞 UDP 协议（支持本机与 Jetson↔PC 跨机来源白名单）
│   └── teleop_bridge.py           v15.0 遥操作安全控制器（5 道安全防线）
│
├── mujoco_twin/                   ★ MuJoCo Studio 4.0+ 3D 实时数字孪生工作区
│   ├── run_twin.py                独立启动器（--native 本地原生窗口 / --web 浏览器）
│   ├── scene_builder.py           B601-DM MjSpec 装配（DM STL + 相机壳3.0 STL + Gemini 305 + 0.35m光轴）
│   ├── sim_plugin.py              仿真插件（镜像 / 日志回放 / 关节滑块 / 3D 目标球 DLS IK）
│   ├── viewer_plugin.py           Dear ImGui 实时遥测与遥操作面板
│   └── messages.py                Sim <-> Viewer 强类型消息定义
│
├── urdf/Rebot_B601_DM/            内置完整 B601-DM URDF 与视觉/碰撞 STL（含 gemini305_mount_v3.stl）
├── config/                        全部配置、标定产物与内置离线视觉模型
│   ├── dm_mit.yaml                ★ DM 底层串口/电机/MIT P0~P4/POS_VEL 切换配置
│   ├── v14.yaml                   ★ v14 桌面默认视觉伺服、深度保护与数字孪生参数
│   ├── v14_jetson.yaml            ★ Jetson Orin NX 16GB 专用低延迟配置预设（640x480@60fps, CPU XNNPACK）
│   ├── grasp.yaml                 ★ J7 夹爪捏合映射与开合角度标定
│   ├── home_pose.json             示教 HOME 位姿（首次真机前请用 teach_home.py 重录）
│   ├── hand_eye_calibration.json  手眼外参 T_tcp_camera（首次真机前请用 handeye_joint_calib.py 重标）
│   ├── joint_offset.json          6 轴关节零偏（随联合标定自动生成）
│   └── hand_landmarker.task       MediaPipe 手掌关键点离线推理模型（7.5 MB，已内置）
│
└── tools/                         安装、自检、标定、系统辨识与离线测试工具集
    ├── install_udev.sh            ★ 一键安装 Orbbec/串口 udev 规则、Jetson USB 缓冲与可选 CAN0/mttcan
    ├── jetson_perf.sh             ★ Jetson Orin 性能档锁定（nvpmodel + jetson_clocks + usbfs=256MB）
    ├── setup_env.sh               ★ 按架构自动在项目内创建 .venv (+ 可选 .venv_mujoco) 独立环境
    ├── check_env.py               ★ 跨平台一键自检依赖、内置资产、Jetson USB/功耗与硬件权限
    ├── run_all_tests.sh           ★ 一键运行全部 5 套离线验证单元测试
    ├── launch_mujoco_twin.sh      ★ 一键启动 MuJoCo Studio 3D 窗口（支持本机与跨机联动参数）
    ├── dm_zero_calibrate.py       ★ 独立达妙电机机械零位标定 / 清除故障 / 随动读角工具
    ├── systemd/                   Jetson 无头开机自启 systemd 模板（含 SIGTERM 安全回零）
    ├── test_dm_controller.py      DM 控制器与运动学/重力一致单测
    ├── test_v14_architecture.py   v14 四层架构、深度保护与闭环单测
    ├── test_v15_teleop.py         v15 双向遥操作安全防线单测
    ├── test_jetson_readiness.py   Jetson ARM64 / 无头按键 / 跨机白名单 / 纯 NumPy 回退单测
    ├── test_mujoco_twin_dm.py     MuJoCo DM 场景与 STL 编译自检
    ├── enumerate_camera_modes.py  枚举 Gemini 305 支持的彩色/深度流档位
    ├── camera_intrinsic_check.py  校验 Gemini 305 内参、畸变与深度精度
    ├── teach_home.py              示教 B601-DM 工作 HOME 位姿
    ├── gripper_j7_calibrate.py    标定 J7 刚性平行夹爪闭合/全开角度
    ├── mit_feedback_probe.py      P0：核对 200Hz 反馈精度与重力符号
    ├── p3_gravity_verify.py       P3：分级静态重力前馈下垂校验
    ├── hand_eye_calibration_auto.py  示教手眼标定基准点 viewpoint.json / 自动标定
    ├── handeye_joint_calib.py     ★ 推荐：手眼外参 + 6 关节零偏联合标定
    ├── joint_sysid.py             单轴正弦扫频系统辨识（滞后/带宽/共振峰）
    ├── analyze_servo_log.py       高频 CSV 伺服日志定量分析
    └── replay_digital_twin.py     离线数字孪生闭环回放与参数扫描
```

---

## 三、从零开始的环境配置与安装教程

将本项目解压或拷贝到**任意路径**后，先在终端进入该项目根目录：

```bash
cd /path/to/rebot_B601_dm_palm_follow_grasp
```

本系统采用 **项目内双虚拟环境解耦设计**（全部创建在项目目录内的 `.venv/` 与 `.venv_mujoco/`，不污染系统环境）：
- **`.venv/`（主控环境，Python 3.10+）**：运行视觉伺服 `palm_follow_v14.py`、遥操作桥接 `mujoco_teleop_v15.py` 及全部标定工具；优先直接安装内置 `third_party/wheels/` 中的 `motorbridge` 与 `pyorbbecsdk2`；
- **`.venv_mujoco/`（MuJoCo Studio 环境，Python 3.12）**：仅用于运行 3D 数字孪生可视化窗口（`./tools/launch_mujoco_twin.sh`），两进程通过本机 UDP 零阻塞通信。

### 3.1 安装系统基础依赖（Ubuntu 20.04 / 22.04）

```bash
sudo apt-get update
sudo apt-get install -y \
    python3 python3-venv python3-pip \
    git curl build-essential pkg-config \
    libgl1 libglib2.0-0 libusb-1.0-0-dev \
    libxkbcommon-x11-0 libxcb-cursor0 \
    can-utils
```

> 建议同时安装 `uv`（可自动下载 Python 3.12 并秒建 `.venv_mujoco`）：
> ```bash
> curl -LsSf https://astral.sh/uv/install.sh | sh
> source "$HOME/.local/bin/env"   # 或重新打开终端
> ```

### 3.2 一键配置硬件设备权限（串口 / CAN / Orbbec Gemini 305）

项目已内置完整的官方 Orbbec USB 规则（`third_party/udev/99-obsensor-libusb.rules`，涵盖 Gemini 305 PID `0840/0841/0842`），运行内置脚本即可一步完成：

```bash
# 安装 Orbbec udev 规则 + 将当前用户加入 dialout（串口）与 video（相机）组
sudo ./tools/install_udev.sh

# 若改用 SocketCAN (can0) 而非默认串口 /dev/ttyACM0，可附加 --can0：
# sudo ./tools/install_udev.sh --can0
```

完成后执行两项确认：
1. **重新插拔** Gemini 305 的 USB-C 线（确保插在 **USB 3.0 蓝色接口**，`lsusb -t` 中速率为 `5000M` 或 `10000M`）；
2. 若当前用户是首次加入 `dialout`/`video` 组，请**注销重新登录**（或临时执行 `sudo chmod 666 /dev/ttyACM0`）；
3. 核对设备号：
   ```bash
   ls -l /dev/ttyACM*      # 若不是 ttyACM0（如 ttyACM1），请同步修改 config/dm_mit.yaml 中的 hardware.channel
   lsusb -d 2bc5:          # 应看到 Orbbec Gemini 305 (2bc5:0840 / 0841 / 0842)
   ```

> 若使用 SocketCAN (`can0`)，请把 [config/dm_mit.yaml](config/dm_mit.yaml) 中的 `hardware.channel` 改为 `"can0"`。

### 3.3 一键搭建项目内独立 Python 环境（推荐）

直接运行项目内置的一键安装脚本，会在当前项目目录内自动创建 `.venv/` 和 `.venv_mujoco/`，并优先从内置 `third_party/wheels/` 安装 `motorbridge` 与 `pyorbbecsdk2`：

```bash
./tools/setup_env.sh
```

常用可选参数：
- 仅搭建主控环境（不装 MuJoCo）：`./tools/setup_env.sh --control-only`
- 仅补装 MuJoCo Studio 3D 环境：`./tools/setup_env.sh --mujoco-only`
- 指定基础 Python：`./tools/setup_env.sh --python /usr/bin/python3.10`
- 严格断网离线安装（预先按 `third_party/README.md` 备齐全部 wheel 后）：`./tools/setup_env.sh --offline`

之后每次在终端操作主控程序前，激活项目内虚拟环境：
```bash
source .venv/bin/activate
```

<details>
<summary>备选方式：手动使用 Conda / Pip 安装（展开查看）</summary>

若你更习惯自己管理 Conda 环境而非 `.venv/`：

```bash
# 1. 创建 Python 3.10 Conda 环境
conda create -n rebot-dm python=3.10 -y
conda activate rebot-dm

# 2. 安装 Pinocchio（conda-forge 叫 pinocchio；纯 pip 安装时 PyPI 包名必须叫 `pin`！）
conda install -c conda-forge pinocchio -y

# 3. 优先使用项目内置硬件 SDK wheel 安装全部依赖（按当前架构目录匹配 x86_64 / aarch64）
pip install --find-links "third_party/wheels/$(uname -m)" --find-links third_party/wheels -r requirements.txt

# 4. 单独创建项目内 MuJoCo Studio 4.0+ 虚拟环境（Python 3.12）
./tools/setup_env.sh --mujoco-only
```
</details>

#### ⚠️ 重要踩坑：若主机装有 ROS / ROS2，必须剔除 `/opt/ros` 路径
如果终端中 `source` 过 ROS/ROS2 的 `setup.bash`，`/opt/ros/*` 会覆盖环境变量导致 `pinocchio` 导入时直接段错误（Segmentation fault）。运行前请执行（`tools/setup_env.sh` 与 `tools/run_all_tests.sh` 已自动处理）：

```bash
export PYTHONPATH=$(echo "${PYTHONPATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
export LD_LIBRARY_PATH=$(echo "${LD_LIBRARY_PATH:-}" | tr ':' '\n' | grep -v '/opt/ros' | paste -sd:)
```

### 3.4 一键自检环境与内置资产

```bash
source .venv/bin/activate
python3 tools/check_env.py
```

看到 `自检通过！主控依赖与项目内资产全部完整。` 即表明 Python 依赖、内置 URDF/STL、`hand_landmarker.task`、内置 SDK Wheel 及 MuJoCo 环境全部就绪。

---

## 四、离线全量验证（无需硬件）

环境装好后，**在连接任何真实硬件前**，运行项目内置一键测试脚本验证全部 **5 套自动化测试（23 项）**（覆盖代码、DM URDF、支架 STL、动力学、安全墙与 Jetson 无头/跨机适配）：

```bash
./tools/run_all_tests.sh
```

它会自动依次运行以下 5 项测试（也可分别单独执行）：
1. `python3 tools/test_dm_controller.py` — DM 底层控制（MIT & POS_VEL、P0~P4、Pinocchio vs NumPy 运动学/重力一致、标定/探针工具接口，8 项）；
2. `python3 tools/test_v14_architecture.py` — v14 四层架构、Gemini 305 深度保护链、闭环跟踪与 UDP 通道 A（5 项）；
3. `python3 tools/test_v15_teleop.py` — v15 双向遥操作 8° 对齐门控、DM 负区间限位、250ms 看门狗与 UDP 通道 B（4 项）；
4. `python3 tools/test_jetson_readiness.py` — Jetson ARM64 预设、无头终端按键、跨机来源白名单与纯 NumPy 回退一致性（5 项）；
5. `.venv_mujoco/bin/python tools/test_mujoco_twin_dm.py` — MuJoCo 4.0+ 编译 DM URDF + 相机壳 3.0 STL + Gemini 305 + 对称夹爪自检（1 项）。

全部 `OK` 后，可先启动一次**无真机离线双向仿真**熟悉界面：

```bash
# 终端 1：启动本地原生 MuJoCo 3D 窗口（自动使用项目内 .venv_mujoco）
./tools/launch_mujoco_twin.sh --native

# 终端 2：启动 v15 离线数字孪生桥接
source .venv/bin/activate
python3 mujoco_teleop_v15.py --sim
```
在 MuJoCo 右侧面板依次体验 `Joint Sliders` 和 `3D IK Sphere`（点击 `ARM Real Robot Follow` 后拖动滑块或双击橙色球 `Ctrl+右键` 平移），确认双向同步流畅后再接真机。

---

## 五、首次真机上电与标定 SOP（必做）

> 仓库内随附的 `home_pose.json` / `hand_eye_calibration.json` / `joint_offset.json` / `grasp.yaml` 仅为安全初值占位。**首次真机视觉跟随前，请严格按以下 7 步顺序完成实机标定**（全程清空机械臂周围 0.6m 半径工作空间，手边保留急停电源开关）。以下所有命令均在 `source .venv/bin/activate` 后于项目根目录执行。

### 第 1 步：机械装配与达妙电机机械零点确认
1. 用 3D 打印件 [相机壳3.0.3MF](相机壳3.0.3MF) 装入 Orbbec Gemini 305，紧固至 B601-DM 末端夹爪上方，确保相机光轴大致朝向夹爪前方（0° 倾角）且 USB-C 线缆留有足够随动弯曲余量；
2. 确认机械臂底座已牢固夹紧在桌面上；
3. 使用本项目内置的独立达妙标定工具核对或写入机械零点：
   ```bash
   # 1.1 先只读随动查看 7 轴角度与方向（手托机械臂，确认收拢贴合限位支架时接近 0°，抬起时 J2/J3 为负角）
   python3 tools/dm_zero_calibrate.py

   # 1.2 若机械零位未校准或有明显偏移，先将手臂完全收拢贴合限位支架、夹爪闭合，再写入机械零位（需输入 YES 二次确认）：
   python3 tools/dm_zero_calibrate.py --set-zero

   # 1.3 若某轴达妙电机曾触发过流/报错锁死，可用本工具一键清除错误：
   python3 tools/dm_zero_calibrate.py --clear-error
   ```

### 第 2 步：验证 Gemini 305 相机流、帧率与纯视觉检测（不连机械臂）
```bash
# 2.1 查看相机实际枚举到的彩色/深度流与 60fps 档位
python3 tools/enumerate_camera_modes.py

# 2.2 核对相机内参、畸变（默认 force_zero_distortion=true）与深度实测误差
#     需将 ArUco 标记（DICT_4X4_50, ID=0，黑边外边长 100mm）置于相机前 20~40cm
python3 tools/camera_intrinsic_check.py

# 2.3 纯视觉演练（不连接机械臂）：伸手到相机前 15~50cm，确认掌心绿点、深度读数和捏合流畅
python3 palm_follow_v14.py --no-arm
```
> **实机健康参考值（Gemini 305, USB 3.0）**：彩色 `848x480@60 MJPG` + 深度 `848x480@60 Y16`，内参约 `fx≈408.8, fy≈408.7, cx≈424, cy≈242`，`depth_scale = 1.0000 mm/unit`，`camera_intrinsic_check.py` 正常重投影误差 `< 1.0 px`（实测约 `0.46 px`）、PnP 距离与深度图偏差 `< 3%`（实测约 `1%`）。

### 第 3 步：验证 DM 200Hz 通信反馈与重力模型符号（P0）
```bash
# 3.1 零运动只读探针（原地不动，先核对 200Hz 通信与反馈缓存）
python3 tools/mit_feedback_probe.py

# 3.2 低速抬至默认 HOME，核对非零悬臂位姿下的 200Hz 反馈精度与 Pinocchio 重力符号（结束后自动安全回零）
python3 tools/mit_feedback_probe.py --move-home
```
- 检查报告中 6 轴反馈正常、`can_fail_total == 0`、`max_pos_diff_deg <= 0.30°`，且主要承载轴（J2/J3/J4）重力力矩 `sign_match == True`；
- 若连接报错，检查 [config/dm_mit.yaml](config/dm_mit.yaml) 中的 `hardware.channel` 是否与 `ls /dev/ttyACM*` 一致，以及当前用户是否已加入 `dialout` 组。

### 第 4 步：示教工作 HOME 位姿 + 标定 J7 夹爪行程
```bash
# 4.1 示教一个视野开阔、肘部舒展的工作 HOME 位姿（写入 config/home_pose.json）
python3 tools/teach_home.py

# 4.2 键盘微调 J7 夹爪，实测官方刚性平行夹爪的完全闭合角与全开角（自动写入 config/grasp.yaml）
python3 tools/gripper_j7_calibrate.py
```
> 提示：标定 J7 时注意不要顶死限位过久，测到刚好完全闭合不打滑的角度记为 `closed_deg`，完全张开留 2~3° 余量记为 `open_deg`。

### 第 5 步：分级校验 DM 静态重力前馈（P3）
因 DM 手腕 `4310` 的官方 MIT 刚度（`kp=18`）比 RS00 更柔和，校准重力前馈能显著消除静态下垂与提升稳态精度：

```bash
python3 tools/p3_gravity_verify.py --pose home
# 如需同时检验前伸悬臂态：
python3 tools/p3_gravity_verify.py --pose extended
```
- 脚本会依次测试 `scale = 0.0 → 0.35 → 0.60 → 0.80`，若发现方向反了（下垂反而放大 >0.45°）或 MOS 温度 ≥85°C 会自动中止防呆；
- 根据输出中下垂最小且不过补的比例，更新 [config/dm_mit.yaml](config/dm_mit.yaml) 中的 `mit_ext.gravity_ff_scale`（通常 J2/J3/J4 在 `0.60 ~ 0.85` 之间）。

### 第 6 步：新支架 + Gemini 305 手眼外参 & 关节零偏联合标定（关键！）
准备一张固定贴在桌面上的 **ArUco 标定板**（`DICT_4X4_50`，`ID=0`，黑边外边长 `100 mm`，平贴固定不动、光线均匀）：

```bash
# 6.1 仅示教一个让相机清晰看到 ArUco 板的基准观察点并保存至 config/viewpoint.json
#     （加 --dry-run 可在保存 viewpoint.json 并完成安全盒预检后直接退出，避免重复采样）
python3 tools/hand_eye_calibration_auto.py --dry-run

# 6.2 在基准点附近自动安全采样 16 个组合位姿（每 6 个位姿自动回零散热 40s），
#     联合求解 T_tcp_camera 与 6 轴零偏，自动覆盖 config/hand_eye_calibration.json 和 config/joint_offset.json
python3 tools/handeye_joint_calib.py
```
- **验收标准**：脚本末尾输出的不动点残差分散度 **`< 2.0 cm`**（实测典型值 `0.2 ~ 0.5 cm`）且各轴零偏 `|δ| < 12°` 即为优秀；
- 标定文件落盘后，`palm_follow_v14.py` 与 `mujoco_teleop_v15.py` 下次启动会**自动加载** `config/hand_eye_calibration.json` 与 `config/joint_offset.json`，无需手动改代码；
- 如需离线复算优化参数（不动臂、不开相机），可直接执行 `python3 tools/handeye_joint_calib.py --reuse`；若分散度偏大，可运行 `python3 tools/calib_diagnose.py` 定位病灶。

### 第 7 步：（可选）DM 单关节正弦扫频系统辨识
若希望进一步优化速度前馈比例、伸展阻尼和离线数字孪生拟合度：
```bash
python3 tools/joint_sysid.py --pose home --with-vel-ff
```

---

## 六、日常运行指南（v14 视觉伺服 & v15 双向遥操作）

### 6.1 v14 手掌跟随 + 捏合抓取（`palm_follow_v14.py`）

**首次实机验收务必按“从弱到强”四步推进**：
```bash
source .venv/bin/activate

# 第 0 步：纯视觉演练（不连机械臂）
python3 palm_follow_v14.py --no-arm

# 第 1 步：仅夹爪捏合（手臂不动，核对夹爪开合方向与流畅度）
python3 palm_follow_v14.py --no-follow

# 第 2 步：低速仅跟随（启动自动全开夹爪防遮挡，先低速测静止/慢移/急停/贴墙回撤）
python3 palm_follow_v14.py --no-gripper --gain 1.5 --max-vel 0.15

# 第 3 步：真机完整模式（默认 distance=0.35m，60fps；若同时开着 MuJoCo 窗口会自动 3D 镜像）
python3 palm_follow_v14.py
```

**常用 CLI 参数与运行快捷键**：
| 参数 / 快捷键 | 作用 |
|---|---|
| `--v14-cfg config/v14_jetson.yaml` | 切换配置文件（如 Jetson 专用 640×480@60fps 预设） |
| `--sim` | 使用真相机 + 离线 DM 仿真臂（不碰真机总线） |
| `--distance 0.30` | 临时覆盖期望跟随距离（范围 `0.10 ~ 0.60 m`，默认 `0.35 m`） |
| `--gain 2.0 --gain-x 1.6` | 临时覆盖竖直/深度增益与水平偏航增益 |
| `--max-vel 0.25` | 临时覆盖末端线速度上限（硬红线 `≤ 0.85 m/s`） |
| `--fps 30` / `--fps 60` | 覆盖相机帧率（默认 `60`） |
| `--detector-backend auto\|cpu\|gpu` | 覆盖 MediaPipe 推理后端（Jetson 无头推荐 `cpu`） |
| `--no-gripper` / `--no-follow` | 禁用夹爪（启动自动打开）/ 禁用手臂跟随 |
| `--headless` / `--auto-start` | 无 OpenCV 窗口模式（SSH 终端按键）/ 到 HOME 后自动启动跟随 |
| `--mujoco-host <IP>` / `--no-mujoco-sync` | 跨机推送 3D 状态至局域网 PC / 关闭 UDP 同步 |
| **`g` / `Space`** | 启动或暂停跟随（同时清除 M6 安全监督锁存） |
| **`q` / `Esc` / `Ctrl+C`** | 统一安全退出：**原地保持 → 缓慢回零到位 → 失能** |

### 6.2 v15 MuJoCo Studio 3D 双向数字孪生遥操作（`mujoco_teleop_v15.py`）

```bash
# 终端 1：启动本地原生 MuJoCo 3D 窗口（也可用 --web 打开 http://localhost:8080/）
./tools/launch_mujoco_twin.sh --native

# 终端 2：启动 B601-DM 双向安全遥操作桥接
source .venv/bin/activate
python3 mujoco_teleop_v15.py --sim                 # 离线仿真演练
python3 mujoco_teleop_v15.py --max-speed 20        # 真机低速安全双向联动（推荐首次验收用 20°/s）
```

**MuJoCo 面板操作流程**：
1. 顶部 `Link` 显示 `BIDIRECTIONAL DM HW TELEOP` 即已双向联通（默认处于只读 `Monitor Only` 安全态）；
2. 切换至 **`Joint Sliders`**（拖动 J1~J6 / J7 滑块）或 **`3D IK Sphere`**（在 3D 视口中**双击橙金色目标球 → `Ctrl+右键` 拖拽平移 / `Ctrl+左键` 旋转**，也可拖右侧 X/Y/Z 滑块）；
3. 点击 **`Sync to Real Pose`** 让虚拟目标贴合真机当前姿态（需满足 `Align ≤ 8.0°` 防跳变门限）；
4. 点击 **`ARM Real Robot Follow`** 解锁，拖动虚拟臂即可安全驱动真机低速跟随；
5. 随时点击 **`DISARM / HOLD NOW`** 或在终端按 **`Space` / `h`** 立即急停解除武装并原地保持；按 **`q` / `Esc` / `Ctrl+C`** 安全回零退出；
6. 在无实时连接时，展开底部 **`Hardware Log Replay`** 可选择 `logs/servo_*.csv` 一键 3D 回放实机历史轨迹。

---

## 七、配置文件说明与按症状调参速查

**参数优先级：命令行参数 > YAML 配置 > 代码默认值**。日常调参只需编辑以下 3 个文件：

| 配置文件 | 管理内容 | 关键切换项 |
|---|---|---|
| [config/dm_mit.yaml](config/dm_mit.yaml) | DM 串口/波特率、7 路电机 MIT/POS_VEL 参数、P0~P4 扩展开关、M6 力矩/过温保护 | `hardware.control_mode: "mit"`（默认，推荐）或 `"posvel"`（内置位置速度环回退，若 MIT 暂未调好重力前馈可先切此档排查）；`vel_ff_enable` / `gravity_ff_enable` / `gain_sched_enable` / `supervisor_enable` 均可独立开关 |
| [config/v14.yaml](config/v14.yaml) | 四层伺服参数（估计器/FSM/PBVS/安全护盾）、Gemini 305 深度保护、DM 仿真动力学 | `pbvs.distance: 0.35`、`depth.min_valid_m: 0.07`、`safety.workspace_max_radius: 0.42` |
| [config/grasp.yaml](config/grasp.yaml) | 捏合像素范围、One Euro 滤波、J7 开合角、`hold_while_moving` | `gripper.hold_while_moving: true`（运动时冻结夹爪防抖，停稳再捏合） |

### 常见视觉伺服症状调参速查（`config/v14.yaml`）
| 症状 | 优先检查 / 调整项 | 调整方向 |
|---|---|---|
| 停手后机械臂仍有微小晃动 | 先运行 `--no-gripper` 排除 J7 振动；检查 `p3_gravity_verify.py` 重力补偿 | `fsm.deadband_in_m`: `0.020 → 0.024`；`depth.oe_min_cutoff`: `0.85 → 0.65` |
| 到位过冲、来回摆几下才停稳 | 检查 `pbvs.world_ff_gain` 与近端渐弱半径 | `pbvs.world_ff_gain`: `0.62 → 0.48`；`pbvs.ff_taper_dist`: `0.08 → 0.11` |
| 左右快速挥手时底座 J1 发冲/振荡 | 降低水平偏航增益或单次限速 | `pbvs.gain_x`: `2.20 → 1.60` 或 `dm_mit.yaml` 调大 `gain_sched_kd_far[0]` |
| 手腕 J4 静态明显下垂 | DM 4310 腕部 `kp=18` 较柔，优先加大重力前馈 | 跑 `p3_gravity_verify.py`，将 `dm_mit.yaml` 中 `gravity_ff_scale[3]` 调至 `0.75~0.85` |
| 跟随整体发肉、滞后大 | 先确认相机运行在 60fps（USB 3.0）与 `dm_mit.yaml` 中 `vel_ff_enable: true` | `pbvs.gain`: `2.50 → 2.75`；补光并启用 `camera.color_exposure_ms: 6.0` |
| 快速挥手频繁丢手 | 室内运动模糊导致 MediaPipe 漏检 | 改善正面补光 + 确认 `camera.color_exposure_ms: 6.0` |

### 高频日志分析命令
每次运行 `palm_follow_v14.py` 会自动在 `logs/servo_dm_v14_*.csv` 记录 100Hz 伺服与 200Hz 底层执行器状态，可用以下命令定量评估：
```bash
python3 tools/analyze_servo_log.py logs/servo_dm_v14_<timestamp>.csv --json
python3 tools/replay_digital_twin.py logs/servo_dm_v14_<timestamp>.csv
```

---

## 八、常见故障排查 FAQ

| 故障现象 | 原因与解决方法 |
|---|---|
| `Permission denied: '/dev/ttyACM0'` | 运行 `sudo ./tools/install_udev.sh` 并注销重新登录（临时可用 `sudo chmod 666 /dev/ttyACM0`） |
| `达妙电机 xxx 无有效反馈` / 电机报错锁死 | 1) 检查电源与 `ls /dev/ttyACM*`；2) 关闭占用串口的其他进程（严禁多进程同开 `/dev/ttyACM0`）；3) 运行 `python3 tools/dm_zero_calibrate.py --clear-error` 清除达妙错误锁死或断电重启 |
| 导入 `pinocchio` 时 `Segmentation fault` | 环境变量中混入了 `/opt/ros/*`：执行 [3.3 节](#️-重要踩坑若主机装有-ros--ros2必须剔除-optros-路径) 中的 `PYTHONPATH` / `LD_LIBRARY_PATH` 剔除命令后再启动 |
| `pip install pinocchio` 装错包或导入失败 | PyPI 上的 Pinocchio 官方包名是 **`pin`**（已在 `requirements.txt` 中写好），conda-forge 上才叫 `pinocchio`；请始终运行 `./tools/setup_env.sh` 或 `pip install --find-links third_party/wheels -r requirements.txt` |
| Gemini 305 无法打开或帧率只有 30fps | 1) 运行 `sudo ./tools/install_udev.sh` 并重插相机；2) 检查是否插在 **USB 3.0** 接口（`lsusb -t` 应为 `5000M/10000M`）；3) 确认安装的是 `pyorbbecsdk2` 而非旧版 `pyorbbecsdk` |
| 机械臂朝反方向跟随 / 偏差固定 | 手眼标定未做或相机装偏：请先运行 `tools/handeye_joint_calib.py` 完成实机标定，勿使用默认占位外参跑真机 |
| 深度经常为 0 或跳变 | 手离相机 `< 4cm` 超出近距下限，或处于强反光/强逆光环境；建议工作距离保持在 `0.15 ~ 0.50 m` |
| 夹爪开合方向反了或不动 | 检查 `config/grasp.yaml` 中 `gripper.enabled: true`，重跑 `tools/gripper_j7_calibrate.py`（方向反了可直接对调 `closed_deg` 与 `open_deg`） |
| 运行中途触发 `M6_FAULT` 原地锁死 | 发生外力碰撞、限位顶死或力矩超 4340P/4310 阈值（安全软保持，不会掉臂）；移开障碍物确认安全后按 **`g`** 清除锁存恢复 |
| `./tools/launch_mujoco_twin.sh` 提示找不到 Python | 在项目根目录执行 `./tools/setup_env.sh --mujoco-only` 创建项目内 `.venv_mujoco/` 即可 |
| 远程 SSH 下无法打开本地 3D 窗口 | 改用 `./tools/launch_mujoco_twin.sh --web --http-port 8080`，在本地浏览器访问 `http://<主机IP>:8080/`；主程序加 `--headless` |

---

## 九、每次运行前安全 Checklist

- [ ] 机械臂底座已稳固夹紧，[相机壳3.0.3MF](相机壳3.0.3MF) 与 Gemini 305 固定牢靠、USB-C 线缆不干涉各关节运动
- [ ] 仅运行单个控制进程占用 `/dev/ttyACM0`（或 `can0`），无后台残留进程
- [ ] 首次上机或改动支架/零点后，已完成 `dm_zero_calibrate.py` → `teach_home.py` → `gripper_j7_calibrate.py` → `p3_gravity_verify.py` → `handeye_joint_calib.py`
- [ ] `config/v14.yaml` 中三轴工作空间与水平软墙 `workspace_max_radius ≤ 0.42 m` 符合当前桌面环境
- [ ] 改参后按 `--no-arm` → `--no-follow` → `--no-gripper --gain 1.5 --max-vel 0.15` → 完整模式逐级验收
- [ ] 工作半径内无人员与易碎物；测试期间**盯紧机械臂实体、手边随时可按 `q` / `Esc` 或硬件电源开关**
- [ ] 退出一律等待程序完成「保持 → 缓慢回零到位 → 失能」，**严禁在屈肘抬臂状态强制断控失能**

---

## 十、NVIDIA Jetson Orin NX 16GB（JetPack 6）部署与跨机联动指南

本项目针对推荐平台 **Jetson Orin NX 16GB（JetPack 6.x / Ubuntu 22.04 / Python 3.10 / `aarch64`，例如 Seeed reComputer Super J4012）** 完成了全套 ARM64 适配（同时兼容 Orin Nano Super 8GB 与 AGX Orin）。

### 10.1 一键完成 Jetson 端安装与硬件调优

将本项目拷贝或 `git clone` 到 Jetson 的 NVMe SSD 上后，在项目根目录依次执行：

```bash
# 1. 安装基础系统包（若需在 ARM64 本地编译 pin 建议保留 cmake 与 python3-dev）
sudo apt-get update
sudo apt-get install -y \
    python3 python3-venv python3-pip python3-dev cmake build-essential pkg-config \
    libgl1 libglib2.0-0 libusb-1.0-0-dev can-utils

# 2. 安装 Orbbec + 达妙串口防 ModemManager 抢占 udev 规则，并自动调高 usbfs_memory_mb=256
#    - 若使用达妙 USB 串口桥 (/dev/ttyACM0)：
sudo ./tools/install_udev.sh
#    - 若使用 Jetson Orin 载板原生 CAN (mttcan -> can0 @ 1Mbps)：
# sudo ./tools/install_udev.sh --jetson-can
# （改用 can0 后记得把 config/dm_mit.yaml 中 hardware.channel 改为 "can0"）

# 3. 创建项目内主控环境 .venv（自动命中 third_party/wheels/aarch64/ 内置 ARM64 SDK Wheel！）
#    默认在 Jetson 上仅装主控环境（节省资源，推荐 PC 跨机跑 MuJoCo 3D）；
#    如果 Orin NX 16GB 也要本机跑 MuJoCo Studio，请加 --with-mujoco
./tools/setup_env.sh

# 4. 每次重启后锁定 Orin 高性能档、CPU 时钟与 256MB USB 3.0 缓冲（保障 200Hz MIT 低抖动）
sudo ./tools/jetson_perf.sh

# 5. 运行跨平台自检与全部离线单元测试
source .venv/bin/activate
python3 tools/check_env.py
./tools/run_all_tests.sh
```

> **关于 ARM64 上的 Pinocchio**：`setup_env.sh` 会自动携带 `CMEEL_SKIP_TESTS=1` 安装 `pin`；若目标机器无预编译轮子或未装 conda-forge，脚本会自动降级安装其余全部依赖，`rebot_adapter.py` 与手眼标定工具（`hand_eye_calibration_auto.py` / `handeye_joint_calib.py`）运行时均会自动切换至**内置精确纯 NumPy B601-DM 运动学/雅可比/静重力引擎**（与 Pinocchio 误差 `< 0.01 mm` / `< 0.002 Nm`），从标定到 v14/v15 真机与仿真全链路均可零 C++ 编译依赖直接运行。

### 10.2 Jetson 无显示器（SSH）运行 v14 手掌跟随抓取

无头模式下程序会自动切换为 **POSIX 终端按键监听**（无需 X11 显示器，SSH 终端直接按 `g` / `Space` 启停、按 `q` / `Esc` / `Ctrl+C` 或收到 `SIGTERM` 均触发缓慢回零安全退出），并每秒打印一行实时状态：

```bash
source .venv/bin/activate

# 推荐：使用 Jetson Orin NX 专属配置（640x480@60fps + ROI 加速 + CPU XNNPACK 零 EGL 报错）
python3 palm_follow_v14.py --v14-cfg config/v14_jetson.yaml --headless

# 如需同时将 100Hz 状态实时镜像到局域网 PC 的 MuJoCo 3D 窗口：
python3 palm_follow_v14.py --v14-cfg config/v14_jetson.yaml --headless --mujoco-host <PC_IP>

# 若部署在 Jetson Orin Nano Super 8GB 且负载偏高，可临时降为 30fps：
python3 palm_follow_v14.py --v14-cfg config/v14_jetson.yaml --headless --fps 30
```

### 10.3 Jetson ↔ PC 局域网跨机 v15 双向数字孪生遥操作（强烈推荐）

让 **Jetson** 专心跑 200Hz 达妙底层控制与安全保护，**开发机 PC** 跑高帧率 MuJoCo 3D 原生窗口：

```bash
# 【在 PC 端】启动 MuJoCo Studio 3D 窗口，监听 0.0.0.0:19870 并把遥操作指令发往 Jetson
./tools/launch_mujoco_twin.sh --native --udp-bind 0.0.0.0 --cmd-host <JETSON_IP> --allow-state-host <JETSON_IP>

# 【在 Jetson 端】启动 v15 双向安全桥接（自动启用来源 IP 白名单，仅允许该 PC 遥控！）
source .venv/bin/activate
python3 mujoco_teleop_v15.py --state-host <PC_IP> --cmd-bind 0.0.0.0 --allow-cmd-host <PC_IP> --max-speed 20
```

> **局域网安全保障**：
> 1. `--allow-cmd-host <PC_IP>` 会丢弃非白名单 IP 发来的任何 UDP 遥操作包；
> 2. 仍完整保留 8° 虚实对齐门限、250ms 断网看门狗自动软保持、0.42m 软墙、Z ≥ 5cm 护桌及 SSH 终端 `Space`/`h` 本地急停解除武装。

### 10.4 可选：Jetson 无头 `systemd` 开机自启服务

仓库已在 [tools/systemd/rebot-palm-follow.service.example](tools/systemd/rebot-palm-follow.service.example) 提供服务模板（包含 `--auto-start`、`KillSignal=SIGTERM` 与 `TimeoutStopSec=35` 保障停服时完整执行缓慢回零后再失能）。**请务必先完成第 5 章全部真机标定与手动验证稳定后再启用自启服务。**

