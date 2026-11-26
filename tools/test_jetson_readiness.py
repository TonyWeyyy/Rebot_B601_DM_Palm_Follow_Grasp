#!/usr/bin/env python3
"""
Jetson Orin (aarch64) 适配与无头/跨机安全专项离线测试
====================================================
无需真实硬件或 X11 显示器，验证：
  1. RebotAdapter 在无 Pinocchio 时自动回退内置纯 NumPy DM 引擎，与 Pinocchio 的
     FK / 雅可比 / 静重力力矩（含/不含 0.095kg 相机负载）逐点一致；
  2. UnifiedKeyListener 无 X11 下的 POSIX 非阻塞按键解析与 ANSI 方向键过滤；
  3. MuJoCoTwinReceiver / MuJoCoTeleopCmdReceiver 局域网来源 IP 白名单防误控；
  4. config/v14_jetson.yaml 配置合法性与全部 DM 安全红线；
  5. v14 / v15 跨机与无头 CLI 参数解析，以及 x86_64 + aarch64 双架构内置 SDK Wheel 完整性。
"""

from __future__ import annotations

import os
import pathlib
import sys
import threading
import time
import unittest
import zipfile

import numpy as np

ROOT = pathlib.Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco_teleop_v15
import palm_follow_v14
import rebot_adapter
from servo_v14.config import load_v14_config, validate_v14_config
from servo_v14.mujoco_bridge import (
    MuJoCoTeleopCmdPublisher,
    MuJoCoTeleopCmdReceiver,
    MuJoCoTwinPublisher,
    MuJoCoTwinReceiver,
)
from servo_v14.terminal_keys import UnifiedKeyListener, normalize_posix_bytes


class _FakeArm:
    def __init__(self) -> None:
        self.gravity_cb = None
        self.speeds = None

    def set_gravity_torque_fn(self, fn) -> None:
        self.gravity_cb = fn

    def set_max_speeds(self, speeds) -> None:
        self.speeds = list(speeds)


class _FakeIO:
    def __init__(self) -> None:
        self.arm = _FakeArm()
        self.ext_cfg = {
            "gravity_payload_mass_kg": 0.095,
            "gravity_payload_com_link6_m": [0.055, 0.0, 0.075],
        }
        self.ext_info = {"active": True}
        self.gripper_deg = 180.0

    def read_joint_positions_deg(self):
        return [0.0, -28.65, -28.65, 0.0, 0.0, 0.0]


class JetsonReadinessTests(unittest.TestCase):
    def test_rebot_adapter_pure_numpy_fallback_parity(self) -> None:
        io_np = _FakeIO()
        ad_np = rebot_adapter.RebotAdapter(io=io_np, use_pinocchio=False)
        self.assertFalse(ad_np.using_pinocchio)
        self.assertIsNotNone(io_np.arm.gravity_cb)

        test_poses = [
            np.array([0.0, -0.50, -0.50, 0.0, 0.0, 0.0]),
            np.array([0.45, -1.10, -0.85, 0.35, -0.25, 0.60]),
            np.array([-0.70, -1.45, -1.20, -0.50, 0.40, -0.90]),
        ]

        if rebot_adapter.pin is not None:
            ad_pin = rebot_adapter.RebotAdapter(io=_FakeIO(), use_pinocchio=True)
            self.assertTrue(ad_pin.using_pinocchio)
            np.testing.assert_allclose(ad_np.q_min, ad_pin.q_min, atol=1e-4)
            np.testing.assert_allclose(ad_np.q_max, ad_pin.q_max, atol=1e-4)
            for q in test_poses:
                p_np, R_np, J_np = ad_np.fk_and_jacobian(q)
                p_pin, R_pin, J_pin = ad_pin.fk_and_jacobian(q)
                self.assertLess(float(np.linalg.norm(p_np - p_pin)), 1e-4)
                self.assertLess(float(np.max(np.abs(R_np - R_pin))), 1e-4)
                self.assertLess(float(np.max(np.abs(J_np - J_pin))), 1e-4)

                g_np_no = ad_np.gravity_torque(q, include_payload=False)
                g_pin_no = ad_pin.gravity_torque(q, include_payload=False)
                self.assertLess(float(np.max(np.abs(g_np_no - g_pin_no))), 2e-3)

                g_np_pl = ad_np.gravity_torque(q, include_payload=True)
                g_pin_pl = ad_pin.gravity_torque(q, include_payload=True)
                self.assertLess(float(np.max(np.abs(g_np_pl - g_pin_pl))), 2e-3)
        else:
            p, R, J = ad_np.fk_and_jacobian(test_poses[0])
            self.assertEqual(p.shape, (3,))
            self.assertEqual(R.shape, (3, 3))
            self.assertEqual(J.shape, (6, 6))
            self.assertTrue(np.all(np.isfinite(ad_np.gravity_torque(test_poses[0], include_payload=True))))

    def test_headless_posix_key_listener(self) -> None:
        # ANSI 方向键序列应被过滤，独立 Esc / Space / 字母应准确识别
        self.assertEqual(normalize_posix_bytes(b"\x1b[Ag H\x1b"), ["g", "space", "h", "esc"])

        r_fd, w_fd = os.pipe()
        received: list[str] = []
        done = threading.Event()

        def on_key(k: str):
            received.append(k)
            if k == "q":
                done.set()
                return False
            return True

        listener = UnifiedKeyListener(on_key, prefer_pynput=False, fd=r_fd)
        try:
            backend = listener.start()
            self.assertEqual(backend, "posix_tty")
            os.write(w_fd, b"G \x1b[Bhq")
            self.assertTrue(done.wait(timeout=1.5), f"未在超时内收到按键，已收到: {received}")
            self.assertEqual(received, ["g", "space", "h", "q"])
        finally:
            listener.stop()
            os.close(r_fd)
            os.close(w_fd)

    def test_udp_lan_sender_whitelist(self) -> None:
        # 1) 白名单匹配 127.0.0.1 -> 放行
        rx_ok = MuJoCoTeleopCmdReceiver(
            host="127.0.0.1", port=0, watchdog_s=0.5, allowed_sender_host="127.0.0.1"
        )
        port_ok = rx_ok._sock.getsockname()[1]
        pub_ok = MuJoCoTeleopCmdPublisher(host="127.0.0.1", port=port_ok, max_hz=100.0)

        # 2) 白名单为其他 IP -> 丢弃 127.0.0.1 发来的包
        rx_block = MuJoCoTeleopCmdReceiver(
            host="127.0.0.1", port=0, watchdog_s=0.5, allowed_sender_host="10.255.255.1"
        )
        port_block = rx_block._sock.getsockname()[1]
        pub_block = MuJoCoTeleopCmdPublisher(host="127.0.0.1", port=port_block, max_hz=100.0)

        # 3) 状态通道同样支持来源白名单
        st_rx_block = MuJoCoTwinReceiver(
            host="127.0.0.1", port=0, stale_timeout_s=0.5, allowed_sender_host="10.255.255.1"
        )
        st_port = st_rx_block._sock.getsockname()[1]
        st_pub = MuJoCoTwinPublisher(host="127.0.0.1", port=st_port, max_hz=100.0)

        try:
            q_des = [0.0, -0.5, -0.5, 0.0, 0.0, 0.0]
            pub_ok.publish(armed=True, mode="JOINT", q_des=q_des, force=True)
            pub_block.publish(armed=True, mode="JOINT", q_des=q_des, force=True)
            st_pub.publish(q_act=q_des, q_cmd=q_des, force=True)
            time.sleep(0.03)

            pkt_ok = rx_ok.poll_latest()
            self.assertIsNotNone(pkt_ok)
            self.assertEqual(rx_ok.last_sender_host, "127.0.0.1")
            self.assertEqual(rx_ok.dropped_untrusted_count, 0)

            pkt_blk = rx_block.poll_latest()
            self.assertIsNone(pkt_blk)
            self.assertGreaterEqual(rx_block.dropped_untrusted_count, 1)

            st_blk = st_rx_block.poll_latest()
            self.assertIsNone(st_blk)
            self.assertGreaterEqual(st_rx_block.dropped_untrusted_count, 1)
        finally:
            rx_ok.close()
            pub_ok.close()
            rx_block.close()
            pub_block.close()
            st_rx_block.close()
            st_pub.close()

    def test_jetson_config_and_cli_overrides(self) -> None:
        cfg_path = ROOT / "config" / "v14_jetson.yaml"
        self.assertTrue(cfg_path.is_file())
        cfg = validate_v14_config(load_v14_config(str(cfg_path)))
        self.assertEqual(cfg.camera.width, 640)
        self.assertEqual(cfg.camera.height, 480)
        self.assertEqual(cfg.camera.fps, 60)
        self.assertEqual(cfg.camera.detector_backend, "cpu")
        self.assertTrue(cfg.camera.roi_crop_enable)
        # 安全红线不可放宽
        self.assertAlmostEqual(cfg.pbvs.distance, 0.35, places=3)
        self.assertAlmostEqual(cfg.depth.min_valid_m, 0.07, places=3)
        self.assertLessEqual(cfg.safety.workspace_max_radius, 0.42)
        self.assertLessEqual(cfg.safety.max_vel, 0.85)
        self.assertLessEqual(cfg.safety.max_vel_joints[1], 2.0)
        self.assertLessEqual(cfg.safety.max_vel_joints[2], 2.0)

        v14_args = palm_follow_v14.parse_args(
            [
                "--v14-cfg",
                str(cfg_path),
                "--headless",
                "--auto-start",
                "--detector-backend",
                "cpu",
                "--mujoco-host",
                "192.168.1.50",
                "--fps",
                "30",
            ]
        )
        cfg2 = palm_follow_v14.apply_cli_overrides(cfg, v14_args)
        self.assertEqual(cfg2.camera.fps, 30)
        self.assertEqual(cfg2.camera.detector_backend, "cpu")
        self.assertEqual(v14_args.mujoco_host, "192.168.1.50")
        self.assertTrue(v14_args.auto_start)

        v15_args = mujoco_teleop_v15.parse_args(
            [
                "--sim",
                "--state-host",
                "192.168.1.50",
                "--cmd-bind",
                "0.0.0.0",
                "--allow-cmd-host",
                "192.168.1.50",
                "--max-speed",
                "20",
            ]
        )
        self.assertEqual(v15_args.state_host, "192.168.1.50")
        self.assertEqual(v15_args.cmd_bind, "0.0.0.0")
        self.assertEqual(v15_args.allow_cmd_host, "192.168.1.50")

    def test_multiarch_bundled_wheels_integrity(self) -> None:
        for arch in ("x86_64", "aarch64"):
            wdir = ROOT / "third_party" / "wheels" / arch
            self.assertTrue(wdir.is_dir(), f"缺少架构 Wheel 目录: {wdir}")
            whls = sorted(wdir.glob("*.whl"))
            names = [w.name for w in whls]
            self.assertTrue(
                any(n.startswith("motorbridge-") and arch in n for n in names),
                f"{arch} 缺少 motorbridge wheel: {names}",
            )
            self.assertTrue(
                any(n.startswith("pyorbbecsdk2-") and arch in n for n in names),
                f"{arch} 缺少 pyorbbecsdk2 wheel: {names}",
            )
            for whl in whls:
                with zipfile.ZipFile(whl, "r") as zf:
                    self.assertIsNone(zf.testzip(), f"Wheel 压缩包损坏: {whl}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
