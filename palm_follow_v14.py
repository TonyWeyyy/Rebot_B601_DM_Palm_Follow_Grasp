#!/usr/bin/env python3
"""
v14.0 手掌跟随 + 捏合抓取主程序（reBot Arm B601-DM + Orbbec Gemini 305）
=======================================================================
四层解耦视觉伺服架构 + 统一世界系 PBVS + 离线数字孪生 + MuJoCo 实时同步：
    1. 视觉/主线程（60fps）：Gemini305Camera + PalmDetector(ROI) + LKPalmBridge
       + DepthProtectionChain(4 层深度保护，适配 0.07m~1.20m) + PinchGripperMapper + OpenCV HUD
    2. 100Hz 伺服线程：ServoV14Pipeline
       Layer 1 (WorldTargetEstimator) -> Layer 2 (ServoFSM) -> Layer 3 (WorldPBVSController)
       -> Layer 4 (SafetyShield) -> RobotBackend (HardwareRobotBackend | DigitalTwinBackend)
    3. 键盘监听线程（pynput）：g/空格 启停跟随（同步清除 M6 锁存）、q/Esc 统一安全退出

运行模式：
    python3 palm_follow_v14.py                       # B601-DM 真机完整模式（跟随 + 夹爪）
    python3 palm_follow_v14.py --sim                 # 离线数字孪生模式（真相机 + DM 仿真臂）
    python3 palm_follow_v14.py --no-gripper          # 仅跟随（启动主动打开夹爪防遮挡）
    python3 palm_follow_v14.py --no-follow           # 仅捏合夹爪
    python3 palm_follow_v14.py --no-arm              # 纯视觉演练（不连硬件）
    python3 palm_follow_v14.py --headless            # 无窗口模式（SSH 场景）
"""

from __future__ import annotations

import argparse
import json
import os
import platform
import signal
import sys
import threading
import time
from dataclasses import replace
from datetime import datetime
from typing import Optional

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.abspath(__file__))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from gemini305_camera import Gemini305Camera
from palm_detection import (
    LKPalmBridge,
    OneEuroFilter,
    PalmDetector,
    PalmTracker,
    draw_landmarks,
    draw_palm,
    pinch_distance_px,
    pixel_to_3d,
)
from servo_diagnostics import DiagnosticRecorder
from servo_timing import CameraTimestampCalibrator
from servo_v14.config import (
    V14Config,
    load_v14_config,
    validate_v14_config,
)
from servo_v14.depth_guard import DepthProtectionChain
from servo_v14.mjpeg_preview import MjpegPreviewServer
from servo_v14.mujoco_bridge import DEFAULT_MUJOCO_HOST, DEFAULT_MUJOCO_PORT, MuJoCoTwinPublisher
from servo_v14.pipeline import ServoV14Pipeline
from servo_v14.robot_backend import (
    DEFAULT_DM_HOME_Q,
    DigitalTwinBackend,
    HardwareRobotBackend,
    RobotBackend,
)
from servo_v14.terminal_keys import UnifiedKeyListener
from servo_v14.types import FSMMode, Observation

DEFAULT_HOME_JOINTS = DEFAULT_DM_HOME_Q.copy()


class PinchGripperMapper:
    """2D 捏合像素距离 → DM J7 夹爪目标角（deg）映射器，支持手臂运动自动冻结。"""

    def __init__(self, grasp_cfg: dict) -> None:
        pinch = grasp_cfg.get("pinch") or {}
        grip = grasp_cfg.get("gripper") or {}
        self.enabled: bool = bool(grip.get("enabled", True))
        self.hold_while_moving: bool = bool(grip.get("hold_while_moving", True))
        self.move_speed_threshold: float = float(grip.get("move_speed_threshold", 0.05))
        self.closed_px: float = float(pinch.get("closed_px", 20.0))
        self.open_px: float = float(pinch.get("open_px", 200.0))
        self.closed_deg: float = float(grip.get("closed_deg", 0.0))
        self.open_deg: float = float(grip.get("open_deg", 180.0))
        self.deadband_deg: float = float(pinch.get("deadband_deg", 2.0))
        self.filter_type: str = str(pinch.get("filter_type", "one_euro"))
        self.smoothing: float = float(pinch.get("smoothing", 0.35))
        self._oe = OneEuroFilter(
            min_cutoff=float(pinch.get("min_cutoff_hz", 2.0)),
            beta=float(pinch.get("beta", 0.01)),
            d_cutoff=float(pinch.get("dcutoff_hz", 1.0)),
        )
        self._last_deg: Optional[float] = None
        self.frozen_by_motion: bool = False

    def seed(self, current_deg: float) -> None:
        self._last_deg = float(current_deg)
        self._oe.reset()

    def update(
        self,
        pinch_px: Optional[float],
        timestamp_s: float,
        arm_joint_speed: float,
    ) -> Optional[float]:
        if not self.enabled or pinch_px is None:
            return None
        if self.hold_while_moving and arm_joint_speed > self.move_speed_threshold:
            self.frozen_by_motion = True
            return None
        self.frozen_by_motion = False

        span_px = max(1.0, self.open_px - self.closed_px)
        ratio = float(np.clip((pinch_px - self.closed_px) / span_px, 0.0, 1.0))
        raw_deg = self.closed_deg + ratio * (self.open_deg - self.closed_deg)

        if self.filter_type == "one_euro":
            filt_deg = float(self._oe(raw_deg, timestamp_s))
        else:
            prev = raw_deg if self._last_deg is None else self._last_deg
            filt_deg = prev + self.smoothing * (raw_deg - prev)

        if self._last_deg is not None and abs(filt_deg - self._last_deg) < self.deadband_deg:
            return None
        self._last_deg = filt_deg
        return filt_deg


def load_home_joints() -> np.ndarray:
    home_file = os.path.join(ROOT, "config", "home_pose.json")
    if os.path.exists(home_file):
        with open(home_file, encoding="utf-8") as stream:
            data = json.load(stream)
        q = np.asarray(data["home_joint_pos"], dtype=np.float64)
        print(f"[HOME] 使用示教 HOME: {np.round(q, 3).tolist()}")
        return q
    return DEFAULT_HOME_JOINTS.copy()


def load_hand_eye_matrix() -> np.ndarray:
    path = os.path.join(ROOT, "config", "hand_eye_calibration.json")
    with open(path, encoding="utf-8") as stream:
        raw = json.load(stream)
    T = np.asarray(raw["T_tcp_camera"], dtype=np.float64)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError("hand_eye_calibration.json 中 T_tcp_camera 矩阵格式无效")
    if raw.get("calibrated") is False:
        print("[标定提示] 当前 hand_eye_calibration.json 为理论名义外参（0° 倾角），真机跟随前请运行 tools/handeye_joint_calib.py 完成实机标定！")
    return T


def load_grasp_cfg(path: str) -> dict:
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def detect_default_v14_cfg() -> str:
    """按运行平台自动选择默认 v14 配置文件：
    - Jetson Orin Nano 8GB -> config/v14_orin_nano.yaml
    - 其他 Jetson ARM64（如 Orin NX / AGX Orin）-> config/v14_jetson.yaml
    - x86_64 桌面主机 -> config/v14.yaml
    """
    nano_cfg = os.path.join(ROOT, "config", "v14_orin_nano.yaml")
    jetson_cfg = os.path.join(ROOT, "config", "v14_jetson.yaml")
    default_cfg = os.path.join(ROOT, "config", "v14.yaml")

    model_txt = ""
    try:
        with open("/proc/device-tree/model", "rb") as f:
            model_txt = f.read().replace(b"\x00", b"").decode("utf-8", errors="ignore").lower()
    except OSError:
        pass

    if "orin nano" in model_txt and os.path.isfile(nano_cfg):
        return nano_cfg

    is_arm_jetson = platform.machine() == "aarch64" and (
        os.path.isfile("/etc/nv_tegra_release") or "jetson" in model_txt or "orin" in model_txt
    )
    if is_arm_jetson:
        try:
            mem_kb = 0
            with open("/proc/meminfo", encoding="utf-8") as f:
                for line in f:
                    if line.startswith("MemTotal:"):
                        mem_kb = int(line.split()[1])
                        break
            if 0 < mem_kb <= 9_800_000 and os.path.isfile(nano_cfg):
                return nano_cfg
        except OSError:
            pass
        if os.path.isfile(jetson_cfg):
            return jetson_cfg

    return default_cfg


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="B601-DM + Gemini 305 v14.0 视觉伺服 + 捏合抓取主程序")
    p.add_argument(
        "--v14-cfg",
        default=None,
        help="v14 配置文件路径（省略时自动按平台选用 v14.yaml / v14_jetson.yaml / v14_orin_nano.yaml）",
    )
    p.add_argument("--grasp-cfg", default=os.path.join(ROOT, "config", "grasp.yaml"), help="夹爪捏合配置文件路径")
    p.add_argument("--no-arm", action="store_true", help="纯视觉演练（不连接机械臂）")
    p.add_argument("--sim", action="store_true", help="启用离线数字孪生机械臂后端（不碰真机总线）")
    p.add_argument("--no-follow", action="store_true", help="仅启用夹爪捏合，禁用手臂跟随")
    p.add_argument("--no-gripper", action="store_true", help="仅启用跟随（启动主动打开夹爪防遮挡）")
    p.add_argument("--headless", action="store_true", help="无 OpenCV 窗口模式（Jetson/SSH 推荐，自动启用终端按键）")
    p.add_argument("--auto-start", action="store_true", help="到达 HOME 后自动开启跟随（适合无头/服务化部署）")
    p.add_argument("--distance", type=float, default=None, help="覆盖期望跟随距离 (m，默认 0.35)")
    p.add_argument("--gain", type=float, default=None, help="覆盖竖直/深度增益")
    p.add_argument("--gain-x", type=float, default=None, help="覆盖水平偏航增益")
    p.add_argument("--max-vel", type=float, default=None, help="覆盖最大末端线速度 (m/s, ≤0.85)")
    p.add_argument("--fps", type=int, default=None, help="覆盖相机帧率 (15/30/60)")
    p.add_argument(
        "--detector-backend",
        choices=("auto", "cpu", "gpu"),
        default=None,
        help="覆盖 MediaPipe 推理后端（Jetson 无头推荐 cpu）",
    )
    p.add_argument(
        "--detect-stride",
        type=int,
        default=None,
        help="MediaPipe 跳帧步长 (1~4)；设为 2 时每 2 帧检测一次、间隔帧由 LK 光流桥接（Orin Nano 60fps 推荐）",
    )
    p.add_argument(
        "--mjpeg-port",
        type=int,
        default=0,
        help="可选：开启无头轻量 MJPEG 浏览器调试预览端口（如 8081 -> http://<IP>:8081/；默认 0 关闭零开销）",
    )
    p.add_argument("--no-mujoco-sync", action="store_true", help="关闭向 MuJoCo Studio 数字孪生的 UDP 同步")
    p.add_argument(
        "--mujoco-host",
        default=DEFAULT_MUJOCO_HOST,
        help=f"MuJoCo 状态推送目标主机（默认 {DEFAULT_MUJOCO_HOST}；跨机推送到 PC 请填 PC IP）",
    )
    p.add_argument("--mujoco-port", type=int, default=DEFAULT_MUJOCO_PORT, help=f"MuJoCo UDP 端口（默认 {DEFAULT_MUJOCO_PORT}）")
    return p.parse_args(argv)


def apply_cli_overrides(cfg: V14Config, args: argparse.Namespace) -> V14Config:
    pbvs = cfg.pbvs
    safety = cfg.safety
    cam = cfg.camera
    if args.distance is not None:
        pbvs = replace(pbvs, distance=args.distance)
    if args.gain is not None:
        pbvs = replace(pbvs, gain=args.gain)
    if args.gain_x is not None:
        pbvs = replace(pbvs, gain_x=args.gain_x)
    if args.max_vel is not None:
        safety = replace(safety, max_vel=args.max_vel)
    if args.fps is not None:
        cam = replace(cam, fps=args.fps)
    if getattr(args, "detector_backend", None) is not None:
        cam = replace(cam, detector_backend=str(args.detector_backend).lower())
    if getattr(args, "detect_stride", None) is not None:
        cam = replace(cam, detect_stride=int(args.detect_stride))
    return validate_v14_config(replace(cfg, pbvs=pbvs, safety=safety, camera=cam))


def main(argv=None) -> int:
    args = parse_args(argv)
    v14_cfg_path = args.v14_cfg or detect_default_v14_cfg()
    cfg = apply_cli_overrides(load_v14_config(v14_cfg_path), args)
    if cfg.camera.cv_num_threads > 0:
        cv2.setNumThreads(int(cfg.camera.cv_num_threads))
    grasp_cfg = load_grasp_cfg(args.grasp_cfg)
    gripper_mapper = PinchGripperMapper(grasp_cfg)
    enable_follow = (not args.no_arm) and (not args.no_follow)
    enable_gripper = (not args.no_arm) and (not args.no_gripper) and gripper_mapper.enabled

    print("=" * 68)
    print("reBot Arm B601-DM + Orbbec Gemini 305 手掌跟随与捏合抓取 [v14/v15]")
    print("=" * 68)
    mode_tag = "SIM数字孪生" if args.sim else ("纯视觉" if args.no_arm else "B601-DM真机")
    print(
        f"配置: {os.path.basename(v14_cfg_path)} | 后端: {mode_tag} | follow={'ON' if enable_follow else 'OFF'} "
        f"| gripper={'ON' if enable_gripper else 'OFF'} | {cfg.camera.width}x{cfg.camera.height}@{cfg.camera.fps}fps "
        f"| stride={cfg.camera.detect_stride}"
    )
    print(
        f"参数: distance={cfg.pbvs.distance:.2f}m gain={cfg.pbvs.gain:.2f}/{cfg.pbvs.gain_x:.2f} "
        f"max_vel={cfg.safety.max_vel:.2f}m/s max_r={cfg.safety.workspace_max_radius:.2f}m "
        f"depth=[{cfg.depth.min_valid_m:.2f},{cfg.depth.max_valid_m:.2f}]m"
    )
    print("操作: g/空格 启停跟随 | q/Esc/Ctrl+C 统一安全退出并回零")
    print("=" * 68)

    backend: Optional[RobotBackend] = None
    pipeline: Optional[ServoV14Pipeline] = None
    camera: Optional[Gemini305Camera] = None
    detector: Optional[PalmDetector] = None
    diag: Optional[DiagnosticRecorder] = None
    mj_pub: Optional[MuJoCoTwinPublisher] = None
    preview_srv: Optional[MjpegPreviewServer] = None
    kb_listener = None

    stop_event = threading.Event()
    prev_sigterm = None

    def _on_sigterm(_signum, _frame) -> None:
        print("\n[v14] 收到 SIGTERM，进入统一安全回零退出链...", flush=True)
        stop_event.set()

    try:
        prev_sigterm = signal.getsignal(signal.SIGTERM)
        signal.signal(signal.SIGTERM, _on_sigterm)
    except Exception:
        prev_sigterm = None

    state_lock = threading.Lock()
    shared = {
        "following": False,
        "obs": Observation(
            seq=0, meas_time_s=0.0, host_time_s=0.0,
            visible=False, frozen=False, p_cam=None,
        ),
        "joint_speed": 0.0,
        "mode": FSMMode.IDLE,
        "err_norm": 0.0,
        "last_toggle_s": 0.0,
    }
    home_joints = load_home_joints()
    T_tcp_cam = load_hand_eye_matrix() if enable_follow else np.eye(4, dtype=np.float64)

    def toggle_following() -> None:
        now = time.monotonic()
        with state_lock:
            if now - shared["last_toggle_s"] < 0.25:
                return
            shared["last_toggle_s"] = now
            new_state = not shared["following"]
            shared["following"] = new_state
        if new_state:
            if backend is not None:
                cleared = backend.clear_safety_fault()
                if cleared:
                    print("[v14] 已清除 M6 安全监督软停机锁存")
                backend.hold()
            if pipeline is not None:
                pipeline.reset(keep_pose_history=True)
                pipeline.lock_current_orientation()
            print("[v14] 跟随启动")
        else:
            if backend is not None:
                backend.hold()
            print("[v14] 跟随暂停（保持当前位姿）")

    try:
        if not args.no_arm:
            if args.sim:
                offset_path = os.path.join(ROOT, "config", "joint_offset.json")
                backend = DigitalTwinBackend(
                    sim_cfg=cfg.sim,
                    q_init=home_joints,
                    use_pinocchio=True,
                    joint_offset_path=offset_path,
                )
                print(f"[SIM] B601-DM 数字孪生就绪（Pinocchio={'ON' if backend.using_pinocchio else '解析回退'}）")
            else:
                backend = HardwareRobotBackend()
                print("[HW] 缓慢移动至示教 HOME 位姿...")
                backend.move_to_joints(home_joints, speed_deg_s=20.0, wait=True)
                backend.set_joint_speed_limits(np.asarray(cfg.safety.max_vel_joints, dtype=np.float64))

            if args.no_gripper:
                backend.set_gripper_deg(gripper_mapper.open_deg)
                print(f"[夹爪] --no-gripper 模式：已主动打开至 {gripper_mapper.open_deg:.1f}°")
            else:
                gripper_mapper.seed(backend.get_gripper_deg())

            if enable_follow:
                pipeline = ServoV14Pipeline(cfg=cfg, backend=backend, T_tcp_camera=T_tcp_cam)
                pipeline.lock_current_orientation(home_joints)

            if not args.no_mujoco_sync:
                mj_pub = MuJoCoTwinPublisher(
                    host=args.mujoco_host,
                    port=args.mujoco_port,
                    max_hz=100.0,
                    enabled=True,
                )
                print(f"[MuJoCo] 实时数字孪生同步已就绪 -> udp://{args.mujoco_host}:{args.mujoco_port}")

        camera = Gemini305Camera(
            width=cfg.camera.width,
            height=cfg.camera.height,
            fps=cfg.camera.fps,
            color_exposure_ms=cfg.camera.color_exposure_ms,
            force_zero_distortion=cfg.camera.force_zero_distortion,
        )
        detector = PalmDetector(
            backend=cfg.camera.detector_backend,
            roi_enable=cfg.camera.roi_crop_enable,
            roi_scale=cfg.camera.roi_crop_scale,
        )
        tracker = PalmTracker()
        lk_bridge = LKPalmBridge(
            enable=cfg.camera.hybrid_track_enable,
            max_frames=cfg.camera.hybrid_max_frames,
            box_half=cfg.camera.hybrid_box_half,
            max_step_px=cfg.camera.hybrid_max_step_px,
        )
        depth_guard = DepthProtectionChain(cfg.depth, target_distance_m=cfg.pbvs.distance)
        ts_cal = CameraTimestampCalibrator(max_transport_s=cfg.camera.hw_max_transport_s)

        if cfg.camera.diag_enable and enable_follow:
            log_dir = os.path.join(ROOT, "logs")
            os.makedirs(log_dir, exist_ok=True)
            log_path = os.path.join(
                log_dir, f"servo_dm_v14_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
            diag = DiagnosticRecorder(log_path, queue_size=cfg.camera.diag_queue_size)
            diag.start()
            print(f"[诊断] 高频日志: {log_path}")

        servo_thread: Optional[threading.Thread] = None
        if enable_follow and pipeline is not None and backend is not None:
            def servo_loop() -> None:
                dt_nom = cfg.ctrl_dt_s
                next_tick = time.perf_counter()
                last_t = time.monotonic()
                while not stop_event.is_set():
                    now = time.monotonic()
                    dt = max(1e-4, min(0.05, now - last_t))
                    last_t = now
                    with state_lock:
                        obs = shared["obs"]
                        following = shared["following"]

                    robot_state = backend.get_state(now)
                    shielded, telem = pipeline.step(
                        obs=obs,
                        robot_state=robot_state,
                        dt_s=dt,
                        following_enabled=following,
                        apply_to_backend=True,
                    )

                    if shielded.trigger_return_home:
                        print("[安全护盾] 越出硬工作空间 >5cm，缓慢自动回 HOME！")
                        with state_lock:
                            shared["following"] = False
                        backend.hold()
                        backend.move_to_joints(home_joints, speed_deg_s=15.0, wait=True)
                        pipeline.reset(keep_pose_history=False)
                        pipeline.lock_current_orientation(home_joints)

                    with state_lock:
                        shared["joint_speed"] = telem.joint_speed_max
                        shared["mode"] = telem.mode
                        shared["err_norm"] = telem.error_norm_m

                    if diag is not None:
                        diag.log(pipeline.telemetry_to_diag_row(telem, robot_state, following))

                    if mj_pub is not None:
                        mj_pub.publish(
                            q_act=telem.q_act,
                            q_cmd=telem.q_cmd,
                            dq_act=telem.dq_act,
                            tau_act=telem.tau_act,
                            gripper_deg=backend.get_gripper_deg(),
                            p_world=telem.p_world_est if (telem.visible and telem.kf_valid) else None,
                            p_cam=obs.p_cam if obs.visible else None,
                            visible=telem.visible,
                            following=following,
                            mode=telem.mode.value,
                            err_norm_m=telem.error_norm_m,
                            track_err_deg=telem.tracking_err_max_deg,
                            tmos_max=robot_state.t_mos_max_c,
                            m6_fault=robot_state.supervisor_latched,
                            source="sim" if args.sim else "hw",
                        )

                    next_tick += dt_nom
                    sleep_s = next_tick - time.perf_counter()
                    if sleep_s > 0:
                        time.sleep(sleep_s)
                    else:
                        next_tick = time.perf_counter()

            servo_thread = threading.Thread(target=servo_loop, name="dm-v14-servo-100hz", daemon=True)
            servo_thread.start()

        def _on_key(key_name: str) -> Optional[bool]:
            if key_name in ("esc", "q", "ctrl_c"):
                stop_event.set()
                return False
            if key_name in ("space", "g") and enable_follow:
                toggle_following()
            return None

        kb_listener = UnifiedKeyListener(_on_key, prefer_pynput=not args.headless)
        kb_backend = kb_listener.start()
        print(f"[按键] 键盘监听后端: {kb_backend}（g/空格 启停跟随 | q/Esc/Ctrl+C 安全回零退出）")

        if int(args.mjpeg_port) > 0:
            preview_srv = MjpegPreviewServer(port=int(args.mjpeg_port))
            preview_srv.start()
            print(f"[预览] 无头轻量 MJPEG 调试流已就绪 -> http://<本机IP>:{preview_srv.bound_port}/ （无浏览器连接时零开销）")

        if args.auto_start and enable_follow:
            print("[v14] --auto-start 已启用：到达 HOME 后自动开始手掌跟随")
            toggle_following()

        seq = 0
        frame_idx = 0
        lost_streak = 0
        prev_had_hand = False
        detect_stride = max(1, int(cfg.camera.detect_stride))
        fps_win_t = time.monotonic()
        fps_win_frames = 0
        vis_fps = 0.0
        last_headless_print = 0.0
        while not stop_event.is_set():
            t0 = time.perf_counter()
            color_img, depth_img, depth_scale = camera.get_frames(latest_only=True)
            host_t = time.monotonic()
            if color_img is None:
                time.sleep(0.002)
                continue

            frame_idx += 1
            hw_ts_raw, _ = camera.get_last_frame_timing()
            hw_ts = hw_ts_raw if cfg.camera.hw_timestamp_enable else None
            meas_t, hw_used = ts_cal.update(hw_ts, host_t)

            # 跳帧加速（detect_stride > 1）：已锁定手掌时，间隔帧跳过 MediaPipe 直接由 <0.5ms LK 光流桥接；
            # 一旦光流丢跟，同帧立即补跑全图/ROI MediaPipe 检测，零额外掉帧惩罚。
            skip_det = (
                detect_stride > 1
                and cfg.camera.hybrid_track_enable
                and prev_had_hand
                and (frame_idx % detect_stride) != 0
            )
            if skip_det:
                u_det, v_det, landmarks = None, None, None
                out_u, out_v, src = lk_bridge.update(color_img, None)
                if out_u is None:
                    u_det, v_det, landmarks = detector.detect(color_img)
                    det_uv = (u_det, v_det) if u_det is not None else None
                    out_u, out_v, src = lk_bridge.update(color_img, det_uv)
            else:
                u_det, v_det, landmarks = detector.detect(color_img)
                det_uv = (u_det, v_det) if u_det is not None else None
                out_u, out_v, src = lk_bridge.update(color_img, det_uv)
            proc_s = time.perf_counter() - t0
            prev_had_hand = out_u is not None
            track = tracker.update((out_u, out_v) if out_u is not None else None)

            if enable_gripper and backend is not None and landmarks is not None and src == "det":
                p_px, _, _ = pinch_distance_px(landmarks, color_img.shape)
                with state_lock:
                    cur_js = shared["joint_speed"]
                g_deg = gripper_mapper.update(p_px, host_t, cur_js)
                if g_deg is not None:
                    backend.set_gripper_deg(g_deg)

            if track is not None and track["lost"] == 0:
                lost_streak = 0
                u, v = int(track["u"]), int(track["v"])
                eff_scale = depth_scale if depth_scale is not None else (camera.depth_scale or 0.001)
                raw_z = depth_guard.extract_raw_depth(depth_img, u, v, eff_scale)
                d_res = depth_guard.update(raw_z, host_t)
                if d_res.valid:
                    p_cam = np.asarray(
                        pixel_to_3d(u, v, d_res.z_m, camera.fx, camera.fy, camera.cx, camera.cy),
                        dtype=np.float64,
                    )
                    track["x3d"], track["y3d"], track["z3d"] = float(p_cam[0]), float(p_cam[1]), float(p_cam[2])
                    seq += 1
                    obs_new = Observation(
                        seq=seq,
                        meas_time_s=meas_t,
                        host_time_s=host_t,
                        visible=True,
                        frozen=d_res.frozen,
                        p_cam=p_cam,
                        processing_s=proc_s,
                        hw_time_used=hw_used,
                    )
                else:
                    obs_new = Observation(
                        seq=seq, meas_time_s=meas_t, host_time_s=host_t,
                        visible=False, frozen=False, p_cam=None,
                        processing_s=proc_s, hw_time_used=hw_used,
                    )
            else:
                lost_streak += 1
                if lost_streak >= 30:
                    depth_guard.reset()
                obs_new = Observation(
                    seq=seq, meas_time_s=meas_t, host_time_s=host_t,
                    visible=False, frozen=False, p_cam=None,
                    processing_s=proc_s, hw_time_used=hw_used,
                )

            with state_lock:
                shared["obs"] = obs_new
                cur_mode = shared["mode"]
                cur_err = shared["err_norm"]

            if mj_pub is not None and servo_thread is None and backend is not None:
                st = backend.get_state(host_t)
                mj_pub.publish(
                    q_act=st.q_act,
                    q_cmd=st.q_cmd,
                    dq_act=st.dq_act,
                    tau_act=st.tau_act,
                    gripper_deg=backend.get_gripper_deg(),
                    p_cam=obs_new.p_cam if obs_new.visible else None,
                    visible=obs_new.visible,
                    following=False,
                    mode="GRIPPER_ONLY",
                    tmos_max=st.t_mos_max_c,
                    m6_fault=st.supervisor_latched,
                    source="sim" if args.sim else "hw",
                )

            fps_win_frames += 1
            now_fps = time.monotonic()
            if now_fps - fps_win_t >= 1.0:
                vis_fps = fps_win_frames / max(1e-3, now_fps - fps_win_t)
                fps_win_frames = 0
                fps_win_t = now_fps

            need_overlay = (not args.headless) or (preview_srv is not None and preview_srv.active_clients > 0)
            vis = None
            if need_overlay:
                vis = color_img.copy()
                if landmarks is not None:
                    draw_landmarks(vis, landmarks)
                if track is not None and track["lost"] == 0:
                    draw_palm(vis, track, int(camera.cx), int(camera.cy))
                hud = f"B601-DM v14 [{cur_mode.value}] err={cur_err * 100:.1f}cm {vis_fps:.0f}fps"
                cv2.putText(vis, hud, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                if preview_srv is not None and preview_srv.active_clients > 0:
                    preview_srv.push_frame(vis)

            if not args.headless and vis is not None:
                cv2.imshow("reBot B601-DM + Gemini 305 Palm Follow", vis)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    stop_event.set()
                elif key in (ord("g"), ord(" ")) and enable_follow:
                    toggle_following()
            elif args.headless and (now_fps - last_headless_print >= 1.0):
                last_headless_print = now_fps
                z_str = f"{obs_new.p_cam[2]:.2f}m" if (obs_new.visible and obs_new.p_cam is not None) else "---"
                print(
                    f"[v14-headless] mode={cur_mode.value:11s} vis={'Y' if obs_new.visible else 'N'} "
                    f"fps={vis_fps:4.1f} det={proc_s * 1000.0:4.1f}ms z={z_str} err={cur_err * 100:4.1f}cm",
                    flush=True,
                )

    except KeyboardInterrupt:
        print("\n[v14] 收到 Ctrl+C，进入统一安全退出链...")
    finally:
        stop_event.set()
        if prev_sigterm is not None:
            try:
                signal.signal(signal.SIGTERM, prev_sigterm)
            except Exception:
                pass
        if "servo_thread" in locals() and servo_thread is not None and servo_thread.is_alive():
            servo_thread.join(timeout=2.0)
        if kb_listener is not None:
            try:
                kb_listener.stop()
            except Exception:
                pass
        if detector is not None:
            try:
                detector.close()
            except Exception:
                pass
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
        if not args.headless:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        if diag is not None:
            diag.close()
        if preview_srv is not None:
            preview_srv.stop()
        if mj_pub is not None:
            mj_pub.close()
        if backend is not None:
            print("[v14] 机械臂保持并安全缓慢回零...")
            backend.stop()
        print("[v14] 安全退出完成。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
