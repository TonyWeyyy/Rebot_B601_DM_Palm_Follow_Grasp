#!/usr/bin/env python3
"""B601-DM 底层控制器（P0~P4 / MIT / POS_VEL）与 DM Pinocchio/NumPy 运动学一致性单元测试。"""

from __future__ import annotations

import math
import os
import sys
import time
import unittest
from dataclasses import dataclass

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT not in sys.path:
    sys.path.insert(0, ROOT)

from dm_controller import (
    CanFaultMonitor,
    DEFAULT_DM_MIT_CONFIG_PATH,
    ReBotDMMITController,
    SafetySupervisor,
    compute_p1_mit_step,
    compute_p3_gains,
    compute_p3_gravity_tau,
    cradle_zero_fade,
    load_dm_config,
    validate_dm_mit_ext_cfg,
)
from servo_v14.robot_backend import (
    URDF_DEFAULT_PATH,
    _PinocchioKinematicsHelper,
    _PureNumpyURDFKinematics,
)


@dataclass
class _MockMotorState:
    pos: float = 0.0
    vel: float = 0.0
    torq: float = 0.0
    t_mos: float = 35.0
    t_rotor: float = 36.0
    status_code: int = 0


class _MockDamiaoMotor:
    def __init__(self, motor_id: int, feedback_id: int, model: str, init_pos: float = 0.0) -> None:
        self.motor_id = motor_id
        self.feedback_id = feedback_id
        self.model = model
        self.state = _MockMotorState(pos=init_pos)
        self.mode = None
        self.mit_cmds: list[tuple[float, float, float, float, float]] = []
        self.pv_cmds: list[tuple[float, float]] = []
        self.registers: dict[int, float] = {}

    def ensure_mode(self, mode, timeout_ms: int = 1000) -> None:
        del timeout_ms
        self.mode = mode

    def write_register_f32(self, rid: int, val: float) -> None:
        self.registers[rid] = float(val)

    def send_mit(self, pos: float, vel: float, kp: float, kd: float, tau: float) -> None:
        self.mit_cmds.append((pos, vel, kp, kd, tau))
        # 一阶模拟电机平滑跟上目标位置
        self.state.pos += 0.35 * (pos - self.state.pos)
        self.state.vel = vel
        self.state.torq = tau

    def send_pos_vel(self, pos: float, vlim: float) -> None:
        self.pv_cmds.append((pos, vlim))
        self.state.pos += 0.35 * (pos - self.state.pos)

    def request_feedback(self) -> None:
        pass

    def get_state(self):
        return self.state


class _MockBusController:
    def __init__(self, init_positions: list[float]) -> None:
        self._init_pos = list(init_positions)
        self.motors: list[_MockDamiaoMotor] = []
        self.enabled = False
        self.disabled = False
        self.closed = False
        self.poll_count = 0

    def add_damiao_motor(self, motor_id: int, feedback_id: int, model: str):
        idx = len(self.motors)
        m = _MockDamiaoMotor(motor_id, feedback_id, model, init_pos=self._init_pos[idx])
        self.motors.append(m)
        return m

    def enable_all(self) -> None:
        self.enabled = True

    def disable_all(self) -> None:
        self.disabled = True

    def poll_feedback_once(self) -> None:
        self.poll_count += 1

    def shutdown(self) -> None:
        pass

    def close(self) -> None:
        self.closed = True


class DMControllerUnitTests(unittest.TestCase):
    def test_config_loads_and_enforces_dm_limits(self) -> None:
        hw, ext = load_dm_config(DEFAULT_DM_MIT_CONFIG_PATH)
        self.assertEqual(hw.channel, "/dev/ttyACM0")
        self.assertEqual(hw.control_mode, "mit")
        self.assertEqual(len(hw.motors), 7)
        self.assertEqual(hw.motors[0].model, "4340P")
        self.assertEqual(hw.motors[3].model, "4310")
        self.assertAlmostEqual(ext["gravity_payload_mass_kg"], 0.095, places=4)

        # J4 是 4310（峰值 7Nm），超限配置必须被拒绝
        with self.assertRaises(ValueError):
            validate_dm_mit_ext_cfg({"tau_limit_nm": [15.0, 21.0, 21.0, 9.0, 5.2, 5.2]})
        # J1 重力比例非零必须被拒绝
        with self.assertRaises(ValueError):
            validate_dm_mit_ext_cfg({"gravity_ff_scale": [0.2, 0.5, 0.5, 0.5, 0.0, 0.0]})

    def test_can_fault_monitor_window(self) -> None:
        mon = CanFaultMonitor(control_hz=200.0, fault_window_s=0.03, log_interval_s=1.0)
        self.assertEqual(mon.fail_threshold_ticks, 6)
        for k in range(5):
            fatal, _ = mon.record_failure(10.0 + k * 0.005)
            self.assertFalse(fatal)
        mon.record_success()
        self.assertEqual(mon.consecutive_failures, 0)
        for k in range(6):
            fatal, _ = mon.record_failure(12.0 + k * 0.005)
        self.assertTrue(fatal)

    def test_p1_foh_watchdog_and_j7_isolation(self) -> None:
        _, ext = load_dm_config(DEFAULT_DM_MIT_CONFIG_PATH)
        cmd, dq, _ = compute_p1_mit_step(
            target_rad_7=[0.1, -0.5, -0.5, 0.0, 0.0, 0.0, 1.5],
            command_rad_7=[0.0, -0.5, -0.5, 0.0, 0.0, 0.0, 1.0],
            max_speeds_rad_s_7=[2.0] * 6 + [5.0],
            vel_ff_raw_rad_s_7=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0],
            vel_ff_filt_rad_s_7=[0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            vel_ff_active=True,
            vel_ff_age_s=0.005,
            period_s=0.005,
            ext_cfg=ext,
        )
        self.assertGreater(dq[0], 0.0)
        self.assertEqual(dq[6], 0.0)  # J7 夹爪恒强制 0

        # 超时看门狗衰减：200ms 未刷新应几乎归零
        _, dq_stale, _ = compute_p1_mit_step(
            target_rad_7=[0.1, -0.5, -0.5, 0.0, 0.0, 0.0, 1.5],
            command_rad_7=[0.0, -0.5, -0.5, 0.0, 0.0, 0.0, 1.0],
            max_speeds_rad_s_7=[2.0] * 6 + [5.0],
            vel_ff_raw_rad_s_7=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            vel_ff_filt_rad_s_7=[0.0] * 7,
            vel_ff_active=True,
            vel_ff_age_s=0.300,
            period_s=0.005,
            ext_cfg=ext,
        )
        self.assertLess(abs(dq_stale[0]), 0.01)

    def test_p3_gravity_fade_for_dm_negative_angles_and_gain_sched(self) -> None:
        _, ext = load_dm_config(DEFAULT_DM_MIT_CONFIG_PATH)
        self.assertEqual(cradle_zero_fade([0.0, -0.02, -0.03, 0.0, 0.0, 0.0]), 0.0)
        self.assertEqual(cradle_zero_fade([0.0, -0.50, -0.50, 0.0, 0.0, 0.0]), 1.0)

        tau_zero = compute_p3_gravity_tau(
            raw_grav_nm_6=[2.0, -8.0, -7.0, -2.0, 0.0, 0.0],
            q_for_fade_6=[0.0, -0.02, -0.02, 0.0, 0.0, 0.0],
            prev_tau_6=[0.0] * 6,
            ramp_elapsed_s=1.0,
            period_s=0.005,
            ext_cfg=ext,
        )
        self.assertEqual(tau_zero, [0.0] * 6)

        tau_lifted = compute_p3_gravity_tau(
            raw_grav_nm_6=[2.0, -10.0, -10.0, -2.0, 0.0, 0.0],
            q_for_fade_6=[0.0, -0.50, -0.50, 0.0, 0.0, 0.0],
            prev_tau_6=[0.0, -5.0, -6.0, -1.0, 0.0, 0.0],
            ramp_elapsed_s=1.0,
            period_s=0.005,
            ext_cfg=ext,
        )
        self.assertEqual(tau_lifted[0], 0.0)  # J1 恒为 0
        self.assertLess(tau_lifted[1], -5.0)

        kp_out, kd_out = compute_p3_gains(
            base_kp_6=[120.0, 120.0, 120.0, 18.0, 18.0, 18.0],
            base_kd_6=[8.0, 8.0, 8.0, 2.0, 2.0, 2.0],
            prev_kd_6=[8.0, 8.0, 8.0, 2.0, 2.0, 2.0],
            reach_r_m=0.36,
            period_s=0.5,
            ext_cfg=ext,
        )
        self.assertAlmostEqual(kp_out[0], 120.0)
        self.assertAlmostEqual(kd_out[0], 11.0, places=3)

    def test_p4_safety_supervisor_latch_and_thermal_derate(self) -> None:
        _, ext = load_dm_config(DEFAULT_DM_MIT_CONFIG_PATH)
        sup = SafetySupervisor(ext, control_hz=200.0)
        # 79°C 刚好位于 70~88°C 中点 -> thermal_speed_scale = 0.70
        st = sup.update([0.0] * 6, [0.0] * 6, [0.0] * 6, 79.0, True)
        self.assertAlmostEqual(st["thermal_speed_scale"], 0.70, places=2)
        self.assertFalse(st["latched"])

        # J4 (4310) 过力矩 > 5.2 Nm 持续 >= 16 拍（80ms）必须锁存软停机
        for _ in range(17):
            st = sup.update([0.0] * 6, [0.0] * 6, [0.0, 0.0, 0.0, 6.0, 0.0, 0.0], 45.0, True)
        self.assertTrue(st["latched"])
        self.assertIn("over_torque_J4", st["reason"])
        self.assertTrue(sup.clear())
        self.assertFalse(sup.status()["latched"])

    def test_mock_dm_controller_mit_and_posvel_lifecycle(self) -> None:
        hw, ext = load_dm_config(DEFAULT_DM_MIT_CONFIG_PATH)
        for mode_name in ("mit", "posvel"):
            hw_m = type(hw)(**{**hw.__dict__, "control_mode": mode_name, "return_zero_speed_deg_s": 40.0})
            mock_bus = _MockBusController([0.0, -0.35, -0.35, 0.0, 0.0, 0.0, 1.0])
            ctrl = ReBotDMMITController(
                hw_cfg=hw_m,
                ext_cfg=dict(ext),
                controller_factory=lambda _cfg, b=mock_bus: b,
            )
            ctrl.start(install_signal_handlers=False)
            try:
                ctrl.set_joint_mit_targets([5.0, -25.0, -25.0, 0.0, 0.0, 0.0, 90.0], [10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 50.0])
                time.sleep(0.06)
                snap = ctrl.get_feedback_snapshot()
                self.assertTrue(all(snap["valid"][:6]))
                self.assertGreater(snap["seq"], 2)
                if mode_name == "mit":
                    self.assertGreater(len(mock_bus.motors[0].mit_cmds), 2)
                else:
                    self.assertGreater(len(mock_bus.motors[0].pv_cmds), 2)
                    # posvel 模式下夹爪仍走 MIT
                    self.assertGreater(len(mock_bus.motors[6].mit_cmds), 2)
            finally:
                ctrl.stop(return_to_zero=True, wait=True)
            self.assertTrue(mock_bus.disabled)
            self.assertTrue(mock_bus.closed)
            # 安全回零验证：最终回零接近 0
            for idx in range(6):
                self.assertLess(abs(mock_bus.motors[idx].state.pos), 0.06)

    def test_pinocchio_vs_pure_numpy_dm_kinematics_and_gravity_parity(self) -> None:
        pin_kin = _PinocchioKinematicsHelper(URDF_DEFAULT_PATH)
        np_kin = _PureNumpyURDFKinematics()
        rng = np.random.default_rng(123)
        for _ in range(10):
            q = rng.uniform(pin_kin.q_min + 0.1, pin_kin.q_max - 0.1)
            p1, R1, J1 = pin_kin.fk_and_jacobian(q)
            p2, R2, J2 = np_kin.fk_and_jacobian(q)
            g1 = pin_kin.gravity_torque(q)
            g2 = np_kin.gravity_torque(q)
            self.assertLess(float(np.max(np.abs(p1 - p2))), 1e-5)
            self.assertLess(float(np.max(np.abs(R1 - R2))), 1e-5)
            self.assertLess(float(np.max(np.abs(J1 - J2))), 1e-4)
            self.assertLess(float(np.max(np.abs(g1 - g2))), 2e-3)

    def test_probe_gravity_sysid_and_calib_tool_apis(self) -> None:
        """验证 mit_feedback_probe / p3_gravity_verify / joint_sysid / handeye 工具依赖的全部接口。"""
        hw, ext = load_dm_config(DEFAULT_DM_MIT_CONFIG_PATH)
        hw_m = type(hw)(**{**hw.__dict__, "return_zero_speed_deg_s": 40.0})
        mock_bus = _MockBusController([0.0, -0.35, -0.35, 0.0, 0.0, 0.0, 0.0])
        ctrl = ReBotDMMITController(
            hw_cfg=hw_m,
            ext_cfg=dict(ext),
            controller_factory=lambda _cfg, b=mock_bus: b,
        )
        ctrl.set_gravity_torque_fn(lambda _q: [0.0, -4.0, -3.0, -0.8, 0.0, 0.0])
        ctrl.start(install_signal_handlers=False)
        try:
            self.assertFalse(ctrl.is_stopped)
            ctrl.reset_feedback_change_stats()
            ctrl.set_vel_ff_enable(False)
            self.assertFalse(ctrl.ext_cfg["vel_ff_enable"])
            ctrl.set_vel_ff_enable(True)
            self.assertTrue(ctrl.ext_cfg["vel_ff_enable"])
            ctrl.set_joint_angles([5.0, -25.0, -25.0, 0.0, 0.0, 0.0, 0.0])
            time.sleep(0.06)
            stats = ctrl.get_loop_stats()
            for key in (
                "period_p50_ms",
                "period_p99_ms",
                "period_max_ms",
                "send_p99_ms",
                "sample_p99_ms",
                "feedback_errors",
                "feedback_change_ratio",
                "tau_ff_nm",
                "cur_kd",
            ):
                self.assertIn(key, stats)
            self.assertEqual(len(stats["tau_ff_nm"]), 6)
            self.assertEqual(len(stats["cur_kd"]), 6)
            self.assertEqual(len(ctrl.last_temperatures), 7)
        finally:
            ctrl.stop(return_to_zero=True, wait=True)
        self.assertTrue(ctrl.is_stopped)

        # 验证 handeye_joint_calib.FKModel 在 Pinocchio 与纯 NumPy 模式下完全一致
        tools_dir = os.path.join(ROOT, "tools")
        if tools_dir not in sys.path:
            sys.path.insert(0, tools_dir)
        from handeye_joint_calib import FKModel

        fk_pin = FKModel(use_pinocchio=True)
        fk_np = FKModel(use_pinocchio=False)
        q_test = np.array([0.2, -0.7, -0.6, 0.3, -0.2, 0.4], dtype=np.float64)
        delta_test = np.deg2rad([0.5, -0.8, 0.4, -1.2, 0.9, -0.3])
        T_pin = fk_pin.tcp(q_test, delta_test)
        T_np = fk_np.tcp(q_test, delta_test)
        self.assertLess(float(np.max(np.abs(T_pin - T_np))), 1e-5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
