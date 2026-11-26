r"""
可插拔机械臂后端接口：真机后端 + 离线数字孪生后端（B601-DM，servo_v14/robot_backend.py）
====================================================================================
定义统一的 ``RobotBackend`` 抽象契约，使 ``ServoV14Pipeline`` 与 ``SafeTeleopController``
在 **真实 B601-DM 机械臂** 与 **离线数字孪生仿真器** 上运行 100% 同一套代码：

1. ``HardwareRobotBackend``：封装 B601-DM ``RebotAdapter`` + ``dm_controller.py``（200Hz MIT / POS_VEL，P0~P4）；
2. ``DigitalTwinBackend``：加载真实 ``ReBot_Arm_DM.urdf`` + Pinocchio（无 Pinocchio 时回退逐参数一致的
   纯 NumPy DM 运动学/静重力引擎），挂载 ``0.095 kg`` Orbbec Gemini 305 + 相机壳 3.0 质点，
   以 200Hz 积分 Damiao 4340P/4310 MIT 二阶机电动力学。
"""

from __future__ import annotations

import json
import math
import os
from collections import deque
from typing import Deque, Optional, Protocol, Tuple

import numpy as np

from .config import SimConfig
from .types import RobotState, ShieldedCommand

# B601-DM 关节默认软限位（rad，由 ReBot_Arm_DM.urdf 向内收缩安全边距，注意 J2/J3 为负区间 [-3.14, 0]）
DEFAULT_Q_MIN = np.array([-2.625, -2.965, -2.965, -1.695, -1.395, -2.965], dtype=np.float64)
DEFAULT_Q_MAX = np.array([2.625, -0.175, -0.175, 1.395, 1.395, 2.965], dtype=np.float64)
DEFAULT_DM_HOME_Q = np.array([0.0, -0.50, -0.50, 0.0, 0.0, 0.0], dtype=np.float64)

_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LOCAL_DM_URDF = os.path.join(_PKG_ROOT, "urdf", "Rebot_B601_DM", "urdf", "ReBot_Arm_DM.urdf")
URDF_DEFAULT_PATH = _LOCAL_DM_URDF


class RobotBackend(Protocol):
    """真机与离线数字孪生共享的机械臂后端契约。"""

    @property
    def q_min(self) -> np.ndarray: ...

    @property
    def q_max(self) -> np.ndarray: ...

    def fk_and_jacobian(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]: ...
    def fk_only(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]: ...
    def get_state(self, now_s: float) -> RobotState: ...
    def apply_command(self, cmd: ShieldedCommand, dt_s: float) -> None: ...
    def set_gripper_deg(self, angle_deg: float) -> None: ...
    def get_gripper_deg(self) -> float: ...
    def set_joint_speed_limits(self, limits_rad: np.ndarray) -> None: ...
    def clear_safety_fault(self) -> bool: ...
    def move_to_joints(self, q_target: np.ndarray, speed_deg_s: float = 20.0, wait: bool = True) -> None: ...
    def hold(self) -> None: ...
    def stop(self) -> None: ...


class HardwareRobotBackend:
    """B601-DM 真机硬件后端：薄封装 ``RebotAdapter``（dm-serial / CAN + P0~P4）。"""

    def __init__(self, adapter=None) -> None:
        if adapter is None:
            from rebot_adapter import RebotAdapter
            adapter = RebotAdapter()
        self.adapter = adapter

    @property
    def q_min(self) -> np.ndarray:
        return np.asarray(self.adapter.q_min, dtype=np.float64)

    @property
    def q_max(self) -> np.ndarray:
        return np.asarray(self.adapter.q_max, dtype=np.float64)

    def fk_and_jacobian(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.adapter.fk_and_jacobian(np.asarray(q, dtype=np.float64))

    def fk_only(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        return self.adapter.fk_only(np.asarray(q, dtype=np.float64))

    def get_state(self, now_s: float) -> RobotState:
        q_target = getattr(self.adapter, "_q_target", None)
        if q_target is not None:
            q_cmd = np.asarray(q_target, dtype=np.float64).copy()
        else:
            q_cmd = np.asarray(self.adapter.get_command_pos(), dtype=np.float64)
        fb = self.adapter.get_feedback_state(max_age_s=0.05)
        sup = self.adapter.get_safety_status() or {}
        stats = self.adapter.get_mit_loop_stats() or {}

        if fb is not None and fb.get("valid", False):
            q_act = np.asarray(fb["q"], dtype=np.float64)
            dq_act = np.asarray(fb["dq"], dtype=np.float64)
            tau_act = np.asarray(fb["tau"], dtype=np.float64)
            fb_valid = True
            fb_age_ms = float(fb.get("age_ms", 0.0))
            t_mos = fb.get("t_mos")
            t_mos_max = float(np.nanmax(t_mos)) if (t_mos is not None and np.any(np.isfinite(t_mos))) else 25.0
            tau_ff = np.asarray(fb.get("tau_ff", np.zeros(6)), dtype=np.float64)
            kd_vec = fb.get("kd")
            kd_j1 = float(kd_vec[0]) if (kd_vec is not None and len(kd_vec) > 0) else 8.0
            period_p99 = float(stats.get("period_p99_ms", 5.0))
            can_fail = int(stats.get("can_fail_total", 0))
        else:
            q_act = q_cmd.copy()
            dq_act = np.zeros(6, dtype=np.float64)
            tau_act = np.zeros(6, dtype=np.float64)
            fb_valid = False
            fb_age_ms = 999.0
            t_mos_max = 25.0
            tau_ff = np.zeros(6, dtype=np.float64)
            kd_j1 = 8.0
            period_p99 = float(stats.get("period_p99_ms", 5.0))
            can_fail = int(stats.get("can_fail_total", 0))

        return RobotState(
            timestamp_s=float(now_s),
            q_cmd=q_cmd,
            q_act=q_act,
            dq_act=dq_act,
            tau_act=tau_act,
            feedback_valid=fb_valid,
            feedback_age_ms=fb_age_ms,
            t_mos_max_c=t_mos_max,
            supervisor_latched=bool(sup.get("latched", False)),
            supervisor_reason=str(sup.get("reason", "")),
            thermal_speed_scale=float(sup.get("thermal_speed_scale", 1.0)),
            mit_period_p99_ms=period_p99,
            can_fail_total=can_fail,
            tau_ff_nm=tau_ff,
            kd_j1=kd_j1,
        )

    def apply_command(self, cmd: ShieldedCommand, dt_s: float) -> None:
        del dt_s
        self.adapter.set_reach_radius(cmd.reach_radius_m)
        self.adapter.set_joint_targets_direct(cmd.q_cmd_next, cmd.dq_ff_safe)

    def set_gripper_deg(self, angle_deg: float) -> None:
        self.adapter.set_gripper_deg(float(angle_deg))

    def get_gripper_deg(self) -> float:
        return float(self.adapter.get_gripper_deg())

    def set_joint_speed_limits(self, limits_rad: np.ndarray) -> None:
        self.adapter.set_joint_speed_limits(np.asarray(limits_rad, dtype=np.float64))

    def clear_safety_fault(self) -> bool:
        sup = self.adapter.get_safety_status() or {}
        was = bool(sup.get("latched", False))
        self.adapter.clear_safety_fault()
        return was

    def move_to_joints(self, q_target: np.ndarray, speed_deg_s: float = 20.0, wait: bool = True) -> None:
        self.adapter.move_to_joints(np.asarray(q_target, dtype=np.float64), speed_deg_s=speed_deg_s, wait=wait)

    def hold(self) -> None:
        self.adapter.hold()

    def stop(self) -> None:
        self.adapter.hold()
        self.adapter.stop()


class _PinocchioKinematicsHelper:
    """加载真实 B601-DM URDF + Pinocchio（含关节零位偏置与 0.095kg Gemini 305 负载重力模型）。"""

    def __init__(self, urdf_path: str, joint_offset_path: Optional[str] = None) -> None:
        import pinocchio as pin

        self._pin = pin
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.grav_model = pin.buildModelFromUrdf(urdf_path)
        try:
            jid = self.grav_model.getJointId("joint6")
            if 0 < jid < self.grav_model.njoints:
                com = np.array([0.055, 0.000, 0.075], dtype=np.float64)
                inertia = pin.Inertia(0.095, com, np.eye(3, dtype=np.float64) * 5e-5)
                self.grav_model.appendBodyToJoint(jid, inertia, pin.SE3.Identity())
        except Exception:
            pass
        self.grav_data = self.grav_model.createData()

        self.joint_names = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
        self.last_jid = self.model.getJointId(self.joint_names[-1])
        self._idx_q = [self.model.joints[self.model.getJointId(n)].idx_q for n in self.joint_names]
        self._idx_v = [self.model.joints[self.model.getJointId(n)].idx_v for n in self.joint_names]
        self._grav_idx_q = [self.grav_model.joints[self.grav_model.getJointId(n)].idx_q for n in self.joint_names]
        self._grav_idx_v = [self.grav_model.joints[self.grav_model.getJointId(n)].idx_v for n in self.joint_names]

        self.joint_offset = np.zeros(6, dtype=np.float64)
        if joint_offset_path and os.path.exists(joint_offset_path):
            try:
                with open(joint_offset_path, encoding="utf-8") as stream:
                    raw = json.load(stream)
                off = raw.get("joint_offset_rad") or raw.get("offsets_rad") or raw.get("joint_offset")
                if off is not None and len(off) == 6:
                    self.joint_offset = np.asarray(off, dtype=np.float64)
            except Exception:
                pass

        margin = 0.05
        self.q_min = np.array(
            [self.model.lowerPositionLimit[iq] + margin for iq in self._idx_q],
            dtype=np.float64,
        )
        self.q_max = np.array(
            [self.model.upperPositionLimit[iq] - margin for iq in self._idx_q],
            dtype=np.float64,
        )

    def _q_full(self, q6: np.ndarray, *, grav: bool = False) -> np.ndarray:
        model = self.grav_model if grav else self.model
        idx_q = self._grav_idx_q if grav else self._idx_q
        q = np.zeros(model.nq, dtype=np.float64)
        for i, iq in enumerate(idx_q):
            q[iq] = float(q6[i]) + float(self.joint_offset[i])
        return q

    def fk_and_jacobian(self, q6: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pin = self._pin
        qf = self._q_full(q6, grav=False)
        pin.forwardKinematics(self.model, self.data, qf)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data, qf)
        oMi = self.data.oMi[self.last_jid]
        J_full = pin.getJointJacobian(
            self.model, self.data, self.last_jid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        )
        return oMi.translation.copy(), oMi.rotation.copy(), J_full[:, self._idx_v].copy()

    def fk_only(self, q6: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        pin = self._pin
        qf = self._q_full(q6, grav=False)
        pin.forwardKinematics(self.model, self.data, qf)
        oMi = self.data.oMi[self.last_jid]
        return oMi.translation.copy(), oMi.rotation.copy()

    def gravity_torque(self, q6: np.ndarray) -> np.ndarray:
        pin = self._pin
        qf = self._q_full(q6, grav=True)
        tau = pin.computeGeneralizedGravity(self.grav_model, self.grav_data, qf)
        return np.array([float(tau[iv]) for iv in self._grav_idx_v], dtype=np.float64)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return Rz @ Ry @ Rx


def _axis_angle_rot(axis: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    ax, ay, az = float(axis[0]), float(axis[1]), float(axis[2])
    K = np.array([[0.0, -az, ay], [az, 0.0, -ax], [-ay, ax, 0.0]], dtype=np.float64)
    return np.eye(3, dtype=np.float64) + s * K + (1.0 - c) * (K @ K)


class _PureNumpyURDFKinematics:
    """与 B601-DM ``ReBot_Arm_DM.urdf`` 逐参数精确一致的纯 NumPy 运动学/雅可比/静重力引擎。"""

    # 直接摘自 ReBot_Arm_DM.urdf 的 6 关节运动链定义：(xyz, rpy, axis, lower, upper)
    _JOINTS = (
        ((-8.416e-5, 0.0, 0.08465), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -2.80, 2.80),
        ((0.020084, 0.031625, 0.05555), (-1.5708, 0.0, 0.0), (0.0, 0.0, -1.0), -3.14, 0.00),
        ((-0.264, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -3.14, 0.00),
        ((0.2426, -0.054, -0.001625), (0.0, 0.0, 0.0), (0.0, 0.0, 1.0), -1.87, 1.57),
        ((0.078308, -0.0375, -0.030), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0), -1.57, 1.57),
        ((0.028008, 0.0, 0.040), (0.0, 1.5708, 0.0), (0.0, 0.0, 1.0), -3.14, 3.14),
    )

    # 机械臂本体各连杆 (link_idx, mass_kg, com_local_m) — 含 link1~link6 与固定 end_link
    _BASE_LINK_BODIES = (
        (0, 0.1613, (0.00011361, -0.00061632, 0.02364764)),      # link1
        (1, 1.3266, (-0.13225622, -0.00306170, -0.03083070)),    # link2
        (2, 0.8353, (0.12104004, -0.05362111, -0.03101379)),     # link3
        (3, 0.5200, (0.06082010, -0.05117119, -0.03029946)),     # link4
        (4, 0.3830, (-0.00502802, 0.00000174, 0.03862332)),      # link5
        (5, 0.3663, (0.00000376, -0.00010091, 0.02533086)),      # link6
        # end_link（固定于 link6 [0, 0, 0.15539], rpy=[0, -1.5708, 3.1415], mass=0.50）变换至 link6 系
        (5, 0.5000, (-0.00000704, 0.00000951, 0.08162457)),
    )
    # 兼容保留含默认 0.095kg 相机质点的完整列表
    _LINK_BODIES = _BASE_LINK_BODIES + (
        (5, 0.0950, (0.05500000, 0.00000000, 0.07500000)),
    )

    def __init__(
        self,
        joint_offset_path: Optional[str] = None,
        *,
        joint_offset_rad=None,
        margin_rad: float = 0.05,
        payload_mass_kg: float = 0.095,
        payload_com_link6_m=(0.055, 0.0, 0.075),
    ) -> None:
        self._fixed_t = [np.array(j[0], dtype=np.float64) for j in self._JOINTS]
        self._fixed_R = [_rpy_matrix(*j[1]) for j in self._JOINTS]
        self._axes = [np.array(j[2], dtype=np.float64) for j in self._JOINTS]
        margin = float(margin_rad)
        self.q_min = np.array([j[3] + margin for j in self._JOINTS], dtype=np.float64)
        self.q_max = np.array([j[4] - margin for j in self._JOINTS], dtype=np.float64)

        self._payload_mass_kg = max(0.0, float(payload_mass_kg))
        self._payload_com = tuple(float(x) for x in payload_com_link6_m[:3])

        self.joint_offset = np.zeros(6, dtype=np.float64)
        if joint_offset_rad is not None:
            self.joint_offset = np.asarray(joint_offset_rad, dtype=np.float64).reshape(6).copy()
        elif joint_offset_path and os.path.exists(joint_offset_path):
            try:
                with open(joint_offset_path, encoding="utf-8") as stream:
                    raw = json.load(stream)
                off = raw.get("joint_offset_rad") or raw.get("offsets_rad") or raw.get("joint_offset")
                if off is not None and len(off) == 6:
                    self.joint_offset = np.asarray(off, dtype=np.float64)
            except Exception:
                pass

    def _chain(self, q: np.ndarray) -> Tuple[list, list, list]:
        p_cur = np.zeros(3, dtype=np.float64)
        R_cur = np.eye(3, dtype=np.float64)
        p_list, R_list, z_list = [], [], []
        for i in range(6):
            p_cur = p_cur + R_cur @ self._fixed_t[i]
            R_joint0 = R_cur @ self._fixed_R[i]
            qi = float(q[i]) + float(self.joint_offset[i])
            R_cur = R_joint0 @ _axis_angle_rot(self._axes[i], qi)
            z_w = R_joint0 @ self._axes[i]
            p_list.append(p_cur.copy())
            R_list.append(R_cur.copy())
            z_list.append(z_w)
        return p_list, R_list, z_list

    def fk_only(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        p_list, R_list, _ = self._chain(np.asarray(q, dtype=np.float64))
        return p_list[-1], R_list[-1]

    def fk_and_jacobian(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        p_list, R_list, z_list = self._chain(np.asarray(q, dtype=np.float64))
        p_end = p_list[-1]
        R_end = R_list[-1]
        J = np.zeros((6, 6), dtype=np.float64)
        for i in range(6):
            J[:3, i] = np.cross(z_list[i], p_end - p_list[i])
            J[3:, i] = z_list[i]
        return p_end, R_end, J

    def gravity_torque(self, q: np.ndarray, *, include_payload: bool = True) -> np.ndarray:
        p_list, R_list, z_list = self._chain(np.asarray(q, dtype=np.float64))
        g_vec = np.array([0.0, 0.0, -9.81], dtype=np.float64)
        tau = np.zeros(6, dtype=np.float64)
        bodies = list(self._BASE_LINK_BODIES)
        if include_payload and self._payload_mass_kg > 1e-6:
            bodies.append((5, self._payload_mass_kg, self._payload_com))
        for link_idx, mass, com_local in bodies:
            c_w = p_list[link_idx] + R_list[link_idx] @ np.asarray(com_local, dtype=np.float64)
            f_g = mass * g_vec
            for i in range(link_idx + 1):
                tau[i] -= float(np.dot(z_list[i], np.cross(c_w - p_list[i], f_g)))
        return tau


PureNumpyDMKinematics = _PureNumpyURDFKinematics


class DigitalTwinBackend:
    """B601-DM 离线数字孪生后端（200Hz Damiao 4340P/4310 MIT 机电闭环动力学仿真）。"""

    def __init__(
        self,
        sim_cfg: Optional[SimConfig] = None,
        q_init: Optional[np.ndarray] = None,
        *,
        use_pinocchio: bool = True,
        urdf_path: str = URDF_DEFAULT_PATH,
        joint_offset_path: Optional[str] = None,
        rng_seed: Optional[int] = 42,
    ) -> None:
        self.sim_cfg = sim_cfg or SimConfig()
        self.kin = None
        self.using_pinocchio = False
        if use_pinocchio and os.path.exists(urdf_path):
            try:
                self.kin = _PinocchioKinematicsHelper(urdf_path, joint_offset_path)
                self.using_pinocchio = True
            except Exception:
                self.kin = None
        if self.kin is None:
            self.kin = _PureNumpyURDFKinematics(joint_offset_path=joint_offset_path)

        self._q_min = np.asarray(self.kin.q_min, dtype=np.float64)
        self._q_max = np.asarray(self.kin.q_max, dtype=np.float64)

        q0 = DEFAULT_DM_HOME_Q.copy() if q_init is None else np.asarray(q_init, dtype=np.float64).copy()
        q0 = np.clip(q0, self._q_min, self._q_max)

        self.q_cmd = q0.copy()
        self.q_act = q0.copy()
        self.dq_act = np.zeros(6, dtype=np.float64)
        self.tau_act = self.kin.gravity_torque(q0).copy()
        self.tau_ff_last = np.zeros(6, dtype=np.float64)
        self.kd_last = np.asarray(self.sim_cfg.kd_near, dtype=np.float64).copy()
        self.reach_radius_m: float = float(math.hypot(*self.fk_only(q0)[0][:2]))
        self.gripper_deg: float = 180.0

        self._delay_steps = max(0, int(round(self.sim_cfg.transport_delay_s * self.sim_cfg.mit_hz)))
        self._cmd_pipe: Deque[Tuple[np.ndarray, np.ndarray]] = deque()
        for _ in range(self._delay_steps):
            self._cmd_pipe.append((q0.copy(), np.zeros(6, dtype=np.float64)))

        self._rng = np.random.default_rng(rng_seed) if rng_seed is not None else None
        self.supervisor_latched: bool = False
        self.supervisor_reason: str = ""
        self._err_fault_s: float = 0.0
        self._tau_fault_s: float = 0.0
        self.sim_time_s: float = 0.0
        self.stopped: bool = False

    @property
    def q_min(self) -> np.ndarray:
        return self._q_min

    @property
    def q_max(self) -> np.ndarray:
        return self._q_max

    def fk_and_jacobian(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        return self.kin.fk_and_jacobian(np.asarray(q, dtype=np.float64))

    def fk_only(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        return self.kin.fk_only(np.asarray(q, dtype=np.float64))

    def get_state(self, now_s: float) -> RobotState:
        dq_meas = self.dq_act.copy()
        if self._rng is not None and self.sim_cfg.dq_noise_std > 0.0:
            dq_meas = dq_meas + self._rng.normal(0.0, self.sim_cfg.dq_noise_std, size=6)
        return RobotState(
            timestamp_s=float(now_s),
            q_cmd=self.q_cmd.copy(),
            q_act=self.q_act.copy(),
            dq_act=dq_meas,
            tau_act=self.tau_act.copy(),
            feedback_valid=True,
            feedback_age_ms=0.5,
            t_mos_max_c=35.0,
            supervisor_latched=self.supervisor_latched,
            supervisor_reason=self.supervisor_reason,
            thermal_speed_scale=1.0,
            mit_period_p99_ms=5.05,
            can_fail_total=0,
            tau_ff_nm=self.tau_ff_last.copy(),
            kd_j1=float(self.kd_last[0]),
        )

    def _compute_scheduled_kd(self, reach_r: float) -> np.ndarray:
        ratio = float(np.clip((reach_r - 0.18) / (0.36 - 0.18), 0.0, 1.0))
        kd_near = np.asarray(self.sim_cfg.kd_near, dtype=np.float64)
        kd_far = np.asarray(self.sim_cfg.kd_far, dtype=np.float64)
        return kd_near + ratio * (kd_far - kd_near)

    def apply_command(self, cmd: ShieldedCommand, dt_s: float) -> None:
        dt = max(1e-4, min(0.05, float(dt_s)))
        q_start = self.q_cmd.copy()
        q_end = np.clip(cmd.q_cmd_next, self._q_min, self._q_max)
        dq_ff_in = cmd.dq_ff_safe.copy()
        self.reach_radius_m = float(cmd.reach_radius_m)

        mit_dt = 1.0 / self.sim_cfg.mit_hz
        n_sub = max(1, int(round(dt / mit_dt)))
        sub_dt = dt / n_sub

        kp = np.asarray(self.sim_cfg.kp, dtype=np.float64)
        kd = self._compute_scheduled_kd(self.reach_radius_m)
        self.kd_last = kd
        vff_scale = np.asarray(self.sim_cfg.vel_ff_scale, dtype=np.float64)
        gff_scale = np.asarray(self.sim_cfg.gravity_ff_scale, dtype=np.float64)
        visc = np.asarray(self.sim_cfg.viscous_friction, dtype=np.float64)
        tau_lim = np.asarray(self.sim_cfg.tau_limit_nm, dtype=np.float64)
        peak_lim = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0], dtype=np.float64)

        j_eff = np.asarray(self.sim_cfg.j_eff, dtype=np.float64).copy()
        j_eff[0] += self.sim_cfg.j1_ext_coeff * max(0.0, self.reach_radius_m ** 2 - 0.15 ** 2)

        for k in range(1, n_sub + 1):
            frac = k / n_sub
            q_foh = q_start + frac * (q_end - q_start)

            if self._delay_steps > 0:
                self._cmd_pipe.append((q_foh, dq_ff_in))
                q_des, dq_ff = self._cmd_pipe.popleft()
            else:
                q_des, dq_ff = q_foh, dq_ff_in

            tau_grav = self.kin.gravity_torque(self.q_act)
            tau_ff = tau_grav * gff_scale
            self.tau_ff_last = tau_ff

            dq_des = dq_ff * vff_scale
            tau_motor = kp * (q_des - self.q_act) + kd * (dq_des - self.dq_act) + tau_ff
            tau_motor = np.clip(tau_motor, -peak_lim, peak_lim)
            self.tau_act = tau_motor

            ddq = (tau_motor - tau_grav - visc * self.dq_act) / j_eff
            self.dq_act = self.dq_act + ddq * sub_dt
            self.q_act = np.clip(self.q_act + self.dq_act * sub_dt, self._q_min, self._q_max)

            track_err_deg = float(np.max(np.abs(np.rad2deg(q_des - self.q_act))))
            if track_err_deg > self.sim_cfg.track_err_limit_deg:
                self._err_fault_s += sub_dt
            else:
                self._err_fault_s = 0.0
            if np.any(np.abs(tau_motor) > tau_lim):
                self._tau_fault_s += sub_dt
            else:
                self._tau_fault_s = 0.0
            if self._err_fault_s >= 0.08:
                self.supervisor_latched = True
                self.supervisor_reason = f"track_err>{self.sim_cfg.track_err_limit_deg:.1f}deg"
            elif self._tau_fault_s >= 0.08:
                self.supervisor_latched = True
                self.supervisor_reason = "torque_limit"

        self.q_cmd = q_end.copy()
        self.sim_time_s += dt

    def set_gripper_deg(self, angle_deg: float) -> None:
        self.gripper_deg = float(np.clip(angle_deg, -180.0, 400.0))

    def get_gripper_deg(self) -> float:
        return self.gripper_deg

    def set_joint_speed_limits(self, limits_rad: np.ndarray) -> None:
        del limits_rad

    def clear_safety_fault(self) -> bool:
        was = self.supervisor_latched
        self.supervisor_latched = False
        self.supervisor_reason = ""
        self._err_fault_s = 0.0
        self._tau_fault_s = 0.0
        return was

    def move_to_joints(self, q_target: np.ndarray, speed_deg_s: float = 20.0, wait: bool = True) -> None:
        del speed_deg_s, wait
        qt = np.clip(np.asarray(q_target, dtype=np.float64), self._q_min, self._q_max)
        self.q_cmd = qt.copy()
        self.q_act = qt.copy()
        self.dq_act[:] = 0.0
        self.tau_act = self.kin.gravity_torque(qt).copy()
        self._cmd_pipe.clear()
        for _ in range(self._delay_steps):
            self._cmd_pipe.append((qt.copy(), np.zeros(6, dtype=np.float64)))

    def hold(self) -> None:
        self.dq_act[:] = 0.0

    def stop(self) -> None:
        self.move_to_joints(np.zeros(6, dtype=np.float64))
        self.stopped = True
