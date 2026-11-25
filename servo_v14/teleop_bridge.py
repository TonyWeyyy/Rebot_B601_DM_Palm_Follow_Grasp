"""
v15.0 MuJoCo → 真机双向遥操作安全控制器（B601-DM，servo_v14/teleop_bridge.py）
===========================================================================
强制 5 道不可绕过的硬件安全防线：
  1. 显式解锁 + 8° 姿态对齐防跳变门控；
  2. 分轴速度硬包络 [1.5, 1.2, 1.2, 1.8, 1.8, 1.5] rad/s + 平滑加减速斜坡与无超调制动曲线；
  3. DM URDF 软限位（J2/J3 ∈ [-3.14, 0]）+ 0.42m 水平柱面软/硬墙 + 桌面 Z 安全净空；
  4. 250ms 失联看门狗 + 200Hz M6 安全监督联锁；
  5. 单总线 J7 夹爪限速通道。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

from .robot_backend import RobotBackend
from .types import RobotState, ShieldedCommand

# 遥操作专属关节硬限速 (rad/s)：承重 J2/J3 严守 <= 1.2 rad/s
HARD_TELEOP_JOINT_SPEED = (1.5, 1.2, 1.2, 1.8, 1.8, 1.5)

# Seeed reBot Arm B601-DM URDF 关节硬限位 (rad)（注意 J2/J3 为负区间 [-3.14, 0]）
B601_DM_JOINT_LOWER = (-2.80, -3.14, -3.14, -1.87, -1.57, -3.14)
B601_DM_JOINT_UPPER = (2.80, 0.00, 0.00, 1.57, 1.57, 3.14)
B601_JOINT_LOWER = B601_DM_JOINT_LOWER
B601_JOINT_UPPER = B601_DM_JOINT_UPPER


@dataclass(frozen=True)
class SafeTeleopConfig:
    """v15.0 MuJoCo 双向遥操作安全参数（不可变配置）。"""

    align_tol_deg: float = 8.0
    default_max_speed_deg_s: float = 25.0
    hard_max_speed_deg_s: float = 60.0
    max_joint_accel_rad_s2: float = 2.5
    joint_limit_margin_rad: float = 0.03
    workspace_max_radius_m: float = 0.42
    workspace_hard_overshoot_m: float = 0.05
    workspace_z_min_m: float = 0.05
    workspace_z_max_m: float = 0.65
    vel_ff_scale: float = 0.85
    gripper_min_deg: float = 0.0
    gripper_max_deg: float = 180.0
    gripper_deadband_deg: float = 1.5
    gripper_max_rate_deg_s: float = 240.0

    def validate(self) -> "SafeTeleopConfig":
        if not (0.5 <= self.align_tol_deg <= 20.0):
            raise ValueError(f"align_tol_deg 超出安全范围 [0.5, 20.0]: {self.align_tol_deg}")
        if not (5.0 <= self.hard_max_speed_deg_s <= 60.0):
            raise ValueError(f"hard_max_speed_deg_s 超出安全范围 [5, 60]: {self.hard_max_speed_deg_s}")
        if not (0.10 <= self.workspace_max_radius_m <= 0.42):
            raise ValueError(f"workspace_max_radius_m 违反 70% 臂展红线 (≤0.42m): {self.workspace_max_radius_m}")
        if self.workspace_z_min_m < 0.02:
            raise ValueError(f"workspace_z_min_m 过低存在撞桌风险: {self.workspace_z_min_m}")
        return self


@dataclass(frozen=True)
class TeleopCommand:
    seq: int
    armed: bool
    mode: str
    q_des: np.ndarray
    gripper_deg: float = 180.0
    max_speed_deg_s: float = 25.0
    clear_fault: bool = False


@dataclass(frozen=True)
class TeleopStepResult:
    armed: bool
    mode: str
    reason: str
    q_cmd_next: np.ndarray
    dq_cmd_safe: np.ndarray
    dq_ff_safe: np.ndarray
    reach_radius_m: float
    effective_speed_deg_s: float
    align_err_deg: float
    tracking_err_max_deg: float
    soft_wall_active: bool = False
    z_guard_active: bool = False
    limit_clamped: bool = False
    trigger_return_home: bool = False
    gripper_deg_cmd: Optional[float] = None
    cmd_seq_ack: int = 0


def parse_teleop_packet(pkt: Optional[dict]) -> Optional[TeleopCommand]:
    if pkt is None or not isinstance(pkt, dict):
        return None
    q_raw = pkt.get("q_des")
    if not isinstance(q_raw, (list, tuple)) or len(q_raw) != 6:
        return None
    q_arr = np.asarray(q_raw, dtype=np.float64)
    if not np.all(np.isfinite(q_arr)):
        return None
    g_deg = float(pkt.get("gripper_deg", 180.0))
    spd = float(pkt.get("max_speed_deg_s", 25.0))
    if not (math.isfinite(g_deg) and math.isfinite(spd)):
        return None
    return TeleopCommand(
        seq=int(pkt.get("seq", 0)),
        armed=bool(pkt.get("armed", False)),
        mode=str(pkt.get("mode", "HOLD")),
        q_des=q_arr,
        gripper_deg=g_deg,
        max_speed_deg_s=spd,
        clear_fault=bool(pkt.get("clear_fault", False)),
    )


class SafeTeleopController:
    """v15.0 100Hz MuJoCo → 真机安全遥操作控制器（B601-DM）。"""

    def __init__(
        self,
        cfg: SafeTeleopConfig,
        backend: RobotBackend,
        *,
        enable_gripper: bool = True,
    ) -> None:
        self.cfg = cfg.validate()
        self.backend = backend
        self.enable_gripper = bool(enable_gripper)

        self._q_min = np.asarray(backend.q_min, dtype=np.float64) + self.cfg.joint_limit_margin_rad
        self._q_max = np.asarray(backend.q_max, dtype=np.float64) - self.cfg.joint_limit_margin_rad
        self._hard_vel = np.asarray(HARD_TELEOP_JOINT_SPEED, dtype=np.float64)

        self._armed: bool = False
        self._dq_prev = np.zeros(6, dtype=np.float64)
        self._last_gripper_deg: Optional[float] = None
        self._last_cmd_seq: int = 0
        self._last_clear_fault: bool = False

    def reset(self) -> None:
        self._armed = False
        self._dq_prev[:] = 0.0

    @property
    def is_armed(self) -> bool:
        return self._armed

    def _clamp_joints(self, q: np.ndarray) -> Tuple[np.ndarray, bool]:
        q_c = np.clip(q, self._q_min, self._q_max)
        clamped = bool(np.any(np.abs(q_c - q) > 1e-9))
        return q_c, clamped

    def _update_gripper(self, target_deg: float, dt_s: float) -> Optional[float]:
        if not self.enable_gripper:
            return None
        lo = min(self.cfg.gripper_min_deg, self.cfg.gripper_max_deg)
        hi = max(self.cfg.gripper_min_deg, self.cfg.gripper_max_deg)
        tgt = float(np.clip(target_deg, lo, hi))
        cur = self._last_gripper_deg
        if cur is None:
            cur = float(self.backend.get_gripper_deg())
            self._last_gripper_deg = cur
        diff = tgt - cur
        if abs(diff) < self.cfg.gripper_deadband_deg:
            return None
        max_step = self.cfg.gripper_max_rate_deg_s * max(1e-4, dt_s)
        nxt = cur + float(np.clip(diff, -max_step, max_step))
        self._last_gripper_deg = nxt
        return nxt

    def step(
        self,
        cmd: Optional[TeleopCommand],
        robot_state: RobotState,
        dt_s: float,
        *,
        apply_to_backend: bool = True,
    ) -> TeleopStepResult:
        dt = max(1e-4, min(0.05, float(dt_s)))
        q_cmd = np.asarray(robot_state.q_cmd, dtype=np.float64)
        q_act = np.asarray(robot_state.q_act, dtype=np.float64)
        tcp_curr, _ = self.backend.fk_only(q_cmd)
        r_curr = float(math.hypot(float(tcp_curr[0]), float(tcp_curr[1])))
        z_curr = float(tcp_curr[2])
        track_err_deg = float(np.max(np.abs(np.degrees(q_cmd - q_act))))

        if cmd is not None and cmd.clear_fault and not self._last_clear_fault and not cmd.armed:
            self.backend.clear_safety_fault()
        self._last_clear_fault = bool(cmd.clear_fault) if cmd is not None else False

        if r_curr > self.cfg.workspace_max_radius_m + self.cfg.workspace_hard_overshoot_m:
            self._armed = False
            self._dq_prev[:] = 0.0
            if apply_to_backend:
                self.backend.hold()
            return TeleopStepResult(
                armed=False,
                mode="RETURN_HOME",
                reason=f"HARD_WALL_r={r_curr:.3f}m",
                q_cmd_next=q_cmd.copy(),
                dq_cmd_safe=np.zeros(6),
                dq_ff_safe=np.zeros(6),
                reach_radius_m=r_curr,
                effective_speed_deg_s=0.0,
                align_err_deg=0.0,
                tracking_err_max_deg=track_err_deg,
                trigger_return_home=True,
                cmd_seq_ack=self._last_cmd_seq,
            )

        if robot_state.supervisor_latched:
            self._armed = False
            self._dq_prev[:] = 0.0
            if apply_to_backend:
                self.backend.hold()
            return TeleopStepResult(
                armed=False,
                mode="FAULT_HOLD",
                reason=f"M6_FAULT:{robot_state.supervisor_reason or 'latched'}",
                q_cmd_next=q_cmd.copy(),
                dq_cmd_safe=np.zeros(6),
                dq_ff_safe=np.zeros(6),
                reach_radius_m=r_curr,
                effective_speed_deg_s=0.0,
                align_err_deg=0.0,
                tracking_err_max_deg=track_err_deg,
                cmd_seq_ack=self._last_cmd_seq,
            )

        if cmd is None:
            was_armed = self._armed
            self._armed = False
            self._dq_prev[:] = 0.0
            if apply_to_backend and was_armed:
                self.backend.hold()
            return TeleopStepResult(
                armed=False,
                mode="DISARMED",
                reason="WATCHDOG_TIMEOUT",
                q_cmd_next=q_cmd.copy(),
                dq_cmd_safe=np.zeros(6),
                dq_ff_safe=np.zeros(6),
                reach_radius_m=r_curr,
                effective_speed_deg_s=0.0,
                align_err_deg=0.0,
                tracking_err_max_deg=track_err_deg,
                cmd_seq_ack=self._last_cmd_seq,
            )

        self._last_cmd_seq = cmd.seq
        q_des_clamped, limit_clamped = self._clamp_joints(cmd.q_des)
        align_err_deg = float(np.max(np.abs(np.degrees(q_des_clamped - q_act))))

        if not cmd.armed or cmd.mode == "HOLD":
            was_armed = self._armed
            self._armed = False
            self._dq_prev[:] = 0.0
            if apply_to_backend and was_armed:
                self.backend.hold()
            return TeleopStepResult(
                armed=False,
                mode="DISARMED",
                reason="STANDBY_DISARMED",
                q_cmd_next=q_cmd.copy(),
                dq_cmd_safe=np.zeros(6),
                dq_ff_safe=np.zeros(6),
                reach_radius_m=r_curr,
                effective_speed_deg_s=0.0,
                align_err_deg=align_err_deg,
                tracking_err_max_deg=track_err_deg,
                limit_clamped=limit_clamped,
                cmd_seq_ack=cmd.seq,
            )

        if not self._armed:
            if align_err_deg > self.cfg.align_tol_deg:
                self._dq_prev[:] = 0.0
                return TeleopStepResult(
                    armed=False,
                    mode="DISARMED",
                    reason=f"ALIGN_REQUIRED({align_err_deg:.1f}>{self.cfg.align_tol_deg:.0f}deg)",
                    q_cmd_next=q_cmd.copy(),
                    dq_cmd_safe=np.zeros(6),
                    dq_ff_safe=np.zeros(6),
                    reach_radius_m=r_curr,
                    effective_speed_deg_s=0.0,
                    align_err_deg=align_err_deg,
                    tracking_err_max_deg=track_err_deg,
                    limit_clamped=limit_clamped,
                    cmd_seq_ack=cmd.seq,
                )
            self._armed = True
            self._dq_prev[:] = 0.0
            if apply_to_backend:
                self.backend.hold()

        req_spd_deg_s = float(np.clip(cmd.max_speed_deg_s, 5.0, self.cfg.hard_max_speed_deg_s))
        thermal = float(np.clip(robot_state.thermal_speed_scale, 0.4, 1.0))
        eff_spd_deg_s = req_spd_deg_s * thermal
        eff_spd_rad_s = math.radians(eff_spd_deg_s)
        per_axis_vmax = np.minimum(self._hard_vel, eff_spd_rad_s)

        err = q_des_clamped - q_cmd
        a_max = self.cfg.max_joint_accel_rad_s2
        v_brake = np.sqrt(2.0 * (0.80 * a_max) * np.abs(err))
        v_step = np.abs(err) / dt
        v_mag = np.minimum(per_axis_vmax, np.minimum(v_brake, v_step))
        dq_target = np.sign(err) * v_mag

        max_dv = a_max * dt
        dq_ramped = self._dq_prev + np.clip(dq_target - self._dq_prev, -max_dv, max_dv)
        dq_ramped = np.clip(dq_ramped, -per_axis_vmax, per_axis_vmax)

        small_mask = (np.abs(err) < 5e-4) & (np.abs(dq_ramped) < 5e-3)
        dq_ramped[small_mask] = 0.0

        q_next_raw = q_cmd + dq_ramped * dt
        q_next, next_clamped = self._clamp_joints(q_next_raw)
        limit_clamped = limit_clamped or next_clamped
        dq_safe = (q_next - q_cmd) / dt

        tcp_next, _ = self.backend.fk_only(q_next)
        r_next = float(math.hypot(float(tcp_next[0]), float(tcp_next[1])))
        z_next = float(tcp_next[2])

        soft_wall_active = False
        z_guard_active = False
        reason = "OK"

        if r_next > self.cfg.workspace_max_radius_m and r_next > r_curr + 1e-5:
            soft_wall_active = True
            q_next = q_cmd.copy()
            dq_safe = np.zeros(6, dtype=np.float64)
            reason = f"SOFT_WALL_r={r_next:.3f}m"
        elif z_next < self.cfg.workspace_z_min_m and z_next < z_curr - 1e-5:
            z_guard_active = True
            q_next = q_cmd.copy()
            dq_safe = np.zeros(6, dtype=np.float64)
            reason = f"Z_FLOOR_z={z_next:.3f}m"
        elif z_next > self.cfg.workspace_z_max_m and z_next > z_curr + 1e-5:
            z_guard_active = True
            q_next = q_cmd.copy()
            dq_safe = np.zeros(6, dtype=np.float64)
            reason = f"Z_CEIL_z={z_next:.3f}m"

        self._dq_prev = dq_safe.copy()

        at_lim = (q_next <= self._q_min + 1e-4) | (q_next >= self._q_max - 1e-4)
        dq_ff_safe = dq_safe * self.cfg.vel_ff_scale
        dq_ff_safe[at_lim] = 0.0

        g_cmd = self._update_gripper(cmd.gripper_deg, dt)
        if apply_to_backend:
            shield_cmd = ShieldedCommand(
                q_cmd_next=q_next,
                dq_cmd_safe=dq_safe,
                dq_ff_safe=dq_ff_safe,
                reach_radius_m=r_curr,
                trigger_return_home=False,
                soft_wall_active=soft_wall_active,
                limit_cushion_active=limit_clamped,
                singularity_active=False,
                speed_clamped=False,
                manipulability=1.0,
                shield_reason=reason,
            )
            self.backend.apply_command(shield_cmd, dt)
            if g_cmd is not None:
                self.backend.set_gripper_deg(g_cmd)

        out_mode = "TELEOP_IK" if cmd.mode == "IK_3D" else "TELEOP_JOINT"
        return TeleopStepResult(
            armed=True,
            mode=out_mode,
            reason=reason,
            q_cmd_next=q_next,
            dq_cmd_safe=dq_safe,
            dq_ff_safe=dq_ff_safe,
            reach_radius_m=r_curr,
            effective_speed_deg_s=eff_spd_deg_s,
            align_err_deg=align_err_deg,
            tracking_err_max_deg=track_err_deg,
            soft_wall_active=soft_wall_active,
            z_guard_active=z_guard_active,
            limit_clamped=limit_clamped,
            trigger_return_home=False,
            gripper_deg_cmd=g_cmd,
            cmd_seq_ack=cmd.seq,
        )
