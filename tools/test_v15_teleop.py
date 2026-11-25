#!/usr/bin/env python3
"""B601-DM v15.0 双向数字孪生遥操作安全桥接单元测试。"""

from __future__ import annotations

import os
import sys
import time
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from servo_v14.mujoco_bridge import MuJoCoTeleopCmdPublisher, MuJoCoTeleopCmdReceiver
from servo_v14.robot_backend import DEFAULT_DM_HOME_Q, DigitalTwinBackend
from servo_v14.teleop_bridge import (
    SafeTeleopConfig,
    SafeTeleopController,
    TeleopCommand,
    parse_teleop_packet,
)


class V15DMTeleopTests(unittest.TestCase):
    def _make_controller(self):
        backend = DigitalTwinBackend(q_init=DEFAULT_DM_HOME_Q, use_pinocchio=True, rng_seed=None)
        cfg = SafeTeleopConfig(align_tol_deg=8.0, gripper_min_deg=0.0, gripper_max_deg=180.0).validate()
        ctrl = SafeTeleopController(cfg=cfg, backend=backend, enable_gripper=True)
        return backend, ctrl

    def test_alignment_gate_blocks_large_jump(self) -> None:
        backend, ctrl = self._make_controller()
        st = backend.get_state(0.0)
        q_far = DEFAULT_DM_HOME_Q + np.array([0.4, 0.0, 0.0, 0.0, 0.0, 0.0])  # ~23° > 8°
        cmd = TeleopCommand(seq=1, armed=True, mode="JOINT", q_des=q_far)
        res = ctrl.step(cmd, st, dt_s=0.01)
        self.assertFalse(res.armed)
        self.assertIn("ALIGN_REQUIRED", res.reason)

    def test_armed_smooth_tracking_and_dm_negative_limit_clamp(self) -> None:
        backend, ctrl = self._make_controller()
        # 1) 对齐解锁
        res0 = ctrl.step(
            TeleopCommand(seq=1, armed=True, mode="JOINT", q_des=DEFAULT_DM_HOME_Q.copy()),
            backend.get_state(0.0),
            dt_s=0.01,
        )
        self.assertTrue(res0.armed)

        # 2) 发送合法 DM 目标（J1=+0.20, J2=-0.65, J3=-0.60）平滑收敛
        q_tgt = np.array([0.20, -0.65, -0.60, 0.10, 0.0, 0.0])
        for k in range(1, 200):
            res = ctrl.step(
                TeleopCommand(seq=k + 1, armed=True, mode="JOINT", q_des=q_tgt, max_speed_deg_s=30.0),
                backend.get_state(k * 0.01),
                dt_s=0.01,
            )
        self.assertLess(float(np.max(np.abs(backend.q_cmd - q_tgt))), 0.02)

        # 3) 发送非法正值 J2=+1.0（RS 残留坐标），必须被 DM 软限位强硬截断在负区间内
        res_bad = ctrl.step(
            TeleopCommand(seq=300, armed=True, mode="JOINT", q_des=np.array([0.0, 1.0, 1.0, 0.0, 0.0, 0.0])),
            backend.get_state(3.0),
            dt_s=0.01,
        )
        self.assertTrue(res_bad.limit_clamped)
        self.assertLess(backend.q_cmd[1], 0.0)
        self.assertLess(backend.q_cmd[2], 0.0)

    def test_watchdog_and_m6_fault_interlock(self) -> None:
        backend, ctrl = self._make_controller()
        ctrl.step(TeleopCommand(seq=1, armed=True, mode="JOINT", q_des=DEFAULT_DM_HOME_Q.copy()), backend.get_state(0.0), 0.01)
        self.assertTrue(ctrl.is_armed)

        # Watchdog None -> 立即解除武装
        res_wd = ctrl.step(None, backend.get_state(0.3), 0.01)
        self.assertFalse(res_wd.armed)
        self.assertEqual(res_wd.reason, "WATCHDOG_TIMEOUT")

        # M6 故障锁存 -> FAULT_HOLD
        backend.supervisor_latched = True
        backend.supervisor_reason = "track_err"
        res_m6 = ctrl.step(
            TeleopCommand(seq=2, armed=True, mode="JOINT", q_des=DEFAULT_DM_HOME_Q.copy()),
            backend.get_state(0.4),
            0.01,
        )
        self.assertFalse(res_m6.armed)
        self.assertEqual(res_m6.mode, "FAULT_HOLD")

    def test_udp_channel_b_roundtrip_and_watchdog(self) -> None:
        rx = MuJoCoTeleopCmdReceiver(port=29871, watchdog_s=0.08)
        pub = MuJoCoTeleopCmdPublisher(port=29871, max_hz=1000.0)
        try:
            self.assertIsNone(rx.bind_error)
            ok = pub.publish(armed=True, mode="IK_3D", q_des=DEFAULT_DM_HOME_Q, gripper_deg=120.0, force=True)
            self.assertTrue(ok)
            time.sleep(0.02)
            pkt = rx.poll_latest()
            cmd = parse_teleop_packet(pkt)
            self.assertIsNotNone(cmd)
            assert cmd is not None
            self.assertTrue(cmd.armed)
            self.assertEqual(cmd.mode, "IK_3D")
            self.assertAlmostEqual(cmd.gripper_deg, 120.0, places=1)
            # 超时看门狗触发返回 None
            time.sleep(0.10)
            self.assertIsNone(rx.poll_latest())
        finally:
            pub.close()
            rx.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
