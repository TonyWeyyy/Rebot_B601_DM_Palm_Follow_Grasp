#!/usr/bin/env python3
"""B601-DM + Gemini 305 v14 四层解耦架构与数字孪生闭环单元测试。"""

from __future__ import annotations

import os
import sys
import time
import unittest
from dataclasses import replace

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from palm_follow_v14 import PinchGripperMapper
from servo_v14.config import load_v14_config, validate_v14_config
from servo_v14.depth_guard import DepthProtectionChain
from servo_v14.mujoco_bridge import MuJoCoTwinPublisher, MuJoCoTwinReceiver
from servo_v14.pipeline import ServoV14Pipeline
from servo_v14.robot_backend import DEFAULT_DM_HOME_Q, DigitalTwinBackend
from servo_v14.types import FSMMode, Observation

T_TCP_CAM_NOMINAL = np.array(
    [
        [0.0, -1.0, 0.0, 0.055],
        [1.0, 0.0, 0.0, 0.000],
        [0.0, 0.0, 1.0, 0.075],
        [0.0, 0.0, 0.0, 1.000],
    ],
    dtype=np.float64,
)


class V14DMArchitectureTests(unittest.TestCase):
    def test_v14_config_defaults_and_safety_redlines(self) -> None:
        cfg = load_v14_config(os.path.join(ROOT, "config", "v14.yaml"))
        self.assertAlmostEqual(cfg.pbvs.distance, 0.35, places=4)
        self.assertAlmostEqual(cfg.depth.min_valid_m, 0.07, places=4)
        self.assertAlmostEqual(cfg.depth.max_valid_m, 1.20, places=4)
        self.assertLessEqual(cfg.safety.workspace_max_radius, 0.42)

        with self.assertRaises(ValueError):
            validate_v14_config(replace(cfg, safety=replace(cfg.safety, workspace_max_radius=0.50)))
        with self.assertRaises(ValueError):
            validate_v14_config(replace(cfg, safety=replace(cfg.safety, max_vel=1.20)))
        with self.assertRaises(ValueError):
            validate_v14_config(
                replace(cfg, safety=replace(cfg.safety, max_vel_joints=(2.0, 2.5, 2.0, 2.0, 2.0, 2.0)))
            )

    def test_depth_guard_gemini305_close_range_and_4_layers(self) -> None:
        cfg = load_v14_config(os.path.join(ROOT, "config", "v14.yaml"))
        dg = DepthProtectionChain(cfg.depth, target_distance_m=cfg.pbvs.distance)
        # 5 帧一致恢复测试（0.10m 在 Gemini 305 有效范围 [0.07, 1.20] 内！）
        res = None
        for k in range(5):
            res = dg.update(0.10, 1.0 + k * 0.016)
        assert res is not None
        self.assertTrue(res.valid)
        self.assertFalse(res.frozen)
        self.assertAlmostEqual(res.z_m, 0.10, delta=0.01)

        # 单帧跳变拒收 -> 进入 frozen 复用上一帧安全深度
        r_jump = dg.update(0.45, 1.10)
        self.assertTrue(r_jump.valid)
        self.assertTrue(r_jump.frozen)
        self.assertAlmostEqual(r_jump.z_m, res.z_m, delta=0.01)

        # 超出近距下限 0.03m 被拒收并触发冻结超时后清零
        r_to = dg.update(0.03, 1.50)
        self.assertFalse(r_to.valid)

    def test_closed_loop_palm_follow_and_m6_fault_hold(self) -> None:
        cfg = load_v14_config(os.path.join(ROOT, "config", "v14.yaml"))
        backend = DigitalTwinBackend(sim_cfg=cfg.sim, q_init=DEFAULT_DM_HOME_Q, use_pinocchio=True, rng_seed=7)
        pipe = ServoV14Pipeline(cfg=cfg, backend=backend, T_tcp_camera=T_TCP_CAM_NOMINAL)
        pipe.lock_current_orientation(DEFAULT_DM_HOME_Q)

        # 模拟真实手掌运动：从初始相机前方 0.35m 处以 ~0.20m/s 平滑向右上前移动 12cm 后悬停
        tcp0, rot0 = backend.fk_only(DEFAULT_DM_HOME_Q)
        cam_p0 = tcp0 + rot0 @ T_TCP_CAM_NOMINAL[:3, 3]
        cam_r0 = rot0 @ T_TCP_CAM_NOMINAL[:3, :3]
        p_start = cam_p0 + cam_r0 @ np.array([0.0, 0.0, 0.35])
        p_end = cam_p0 + cam_r0 @ np.array([0.08, -0.06, 0.42])

        modes_seen = set()
        err_end = None
        seq = 0
        for step in range(260):
            t = step * 0.01
            # 0.0~0.2s 静止建立初值，0.2~0.8s 平滑移动 12cm，0.8~2.6s 悬停收敛并进入 HOLDING
            if t < 0.2:
                frac = 0.0
            elif t < 0.8:
                frac = (t - 0.2) / 0.6
            else:
                frac = 1.0
            p_world_target = p_start + frac * (p_end - p_start)

            st = backend.get_state(t)
            tcp_a, rot_a = backend.fk_only(st.q_act)
            cam_pa = tcp_a + rot_a @ T_TCP_CAM_NOMINAL[:3, 3]
            cam_ra = rot_a @ T_TCP_CAM_NOMINAL[:3, :3]
            p_cam = cam_ra.T @ (p_world_target - cam_pa)
            if step % 2 == 0:
                seq += 1
            obs = Observation(seq=seq, meas_time_s=t, host_time_s=t, visible=True, frozen=False, p_cam=p_cam)
            _, telem = pipe.step(obs, st, dt_s=0.01, following_enabled=True, apply_to_backend=True)
            modes_seen.add(telem.mode)
            err_end = telem.error_norm_m

        assert err_end is not None
        self.assertTrue(bool(modes_seen & {FSMMode.TRACKING, FSMMode.DOCKING}))
        self.assertIn(FSMMode.HOLDING, modes_seen)
        # 迟滞退出死区 3.4cm + DM 保守初值重力前馈下的微小静态下垂，收敛误差应 < 4.5cm
        self.assertLess(err_end, 0.045)
        # 验证机械臂确实朝目标显著移动且全部关节留在 DM 负区间软限位内
        tcp_final, _ = backend.fk_only(backend.q_cmd)
        self.assertGreater(float(np.linalg.norm(tcp_final - tcp0)), 0.06)
        self.assertTrue(np.all(backend.q_cmd >= backend.q_min - 1e-6))
        self.assertTrue(np.all(backend.q_cmd <= backend.q_max + 1e-6))

        # M6 故障锁存应立即切入 FAULT_HOLD 且速度清零
        backend.supervisor_latched = True
        backend.supervisor_reason = "over_torque_J4"
        st_f = backend.get_state(3.0)
        sh_f, tel_f = pipe.step(obs, st_f, dt_s=0.01, following_enabled=True, apply_to_backend=True)
        self.assertEqual(tel_f.mode, FSMMode.FAULT_HOLD)
        self.assertTrue(np.allclose(sh_f.dq_cmd_safe, 0.0))

    def test_pinch_gripper_mapper_motion_freeze(self) -> None:
        mapper = PinchGripperMapper({"pinch": {"closed_px": 20.0, "open_px": 200.0}, "gripper": {"closed_deg": 0.0, "open_deg": 180.0}})
        mapper.seed(90.0)
        # 手臂运动超过阈值 -> 冻结
        self.assertIsNone(mapper.update(180.0, 1.0, arm_joint_speed=0.20))
        self.assertTrue(mapper.frozen_by_motion)
        # 手臂静止 -> 更新夹爪角
        deg = mapper.update(180.0, 1.05, arm_joint_speed=0.0)
        self.assertIsNotNone(deg)
        self.assertFalse(mapper.frozen_by_motion)

    def test_udp_channel_a_roundtrip(self) -> None:
        rx = MuJoCoTwinReceiver(port=29870)
        pub = MuJoCoTwinPublisher(port=29870, max_hz=1000.0)
        try:
            self.assertIsNone(rx.bind_error)
            ok = pub.publish(q_act=DEFAULT_DM_HOME_Q, q_cmd=DEFAULT_DM_HOME_Q, mode="TRACKING", force=True)
            self.assertTrue(ok)
            time.sleep(0.02)
            pkt = rx.poll_latest()
            self.assertIsNotNone(pkt)
            assert pkt is not None
            self.assertEqual(pkt["mode"], "TRACKING")
            self.assertAlmostEqual(pkt["q_act"][1], -0.5, places=3)
        finally:
            pub.close()
            rx.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
