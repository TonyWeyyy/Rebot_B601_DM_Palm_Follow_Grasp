#!/usr/bin/env python3
"""
reBot Arm B601-DM 底层实时控制器（Damiao 4340P + 4310 / motorbridge）
=====================================================================
专为 Seeed reBot Arm B601-DM 设计的独立 200Hz 实时控制引擎，直接封装 ``motorbridge``
（默认 USB 串口桥 ``/dev/ttyACM0 @ 921600``，亦支持 SocketCAN ``can0``），零依赖 RS 专属的
``MIT_control/rebot_control``，并完整内置 v13~v15 实机验收通过的 P0~P4 全部底层能力：

1. **双控制模式支持**（``hardware.control_mode``）：
   - ``"mit"``（推荐主模式）：7 轴全部运行 200Hz MIT 阻抗控制，完整支持 P0~P4；
   - ``"posvel"``（兼容回退模式）：arm 6 轴切换达妙 ``Mode.POS_VEL``（自动写入 25~28 号 PID
     寄存器并透传速度上限 ``vlim``），J7 夹爪仍独立维持 ``Mode.MIT`` 防硬夹堵转。
2. **P0 · 通信瞬时故障容错（``CanFaultMonitor``）与 200Hz 零额外请求反馈缓存**：
   - 达妙电机每发一帧 ``send_mit`` 即自动回复位置/速度/力矩/MOS与转子温度，无需阻塞式 SDO；
   - 单帧/偶发通信异常仅计数并限频告警，连续失败持续 ``>= can_fault_window_s``（30ms）才判故障，
     默认执行「缓慢安全回零 → 到位后失能」，杜绝半空失能坠落。
3. **P1 · MIT 速度前馈 ``dq_des`` + 200Hz FOH 一阶保持插值 + 40ms 看门狗**（``compute_p1_mit_step``）：
   - ``set_joint_mit_targets(angles_deg, velocities_deg_s)`` 与纯位置 ``set_joint_angles()`` 完全兼容；
   - J7 夹爪、``set_joint_angles()``、``hold()``、回零路径恒强制 ``dq_des = 0``。
4. **P3 · 静态重力前馈 ``tau_ff``（``compute_p3_gravity_tau``）与伸展自适应阻尼（``compute_p3_gains``）**：
   - 由 ``RebotAdapter`` 注入线程隔离的 Pinocchio DM+Gemini305 静态重力回调；
   - 0.5s 淡入、25 Nm/s 斜率限幅、按 4340P/4310 硬力矩封顶、近零位机械支架平滑淡出至 0 Nm。
5. **P4 · M6 200Hz 主动安全监督（``SafetySupervisor``）与三级过温保护**：
   - 持续监测 6 轴跟踪误差（>12° 持续 ≥80ms）与 4340P/4310 力矩上限，触发后原地软保持（维持重力
     补偿，绝不空中失能），排查后可调用 ``clear_safety_fault()`` 恢复；
   - 70~88°C 线性热降额 + 三级过温（报警 / 高温缓慢回零 / 紧急断开）。
6. **drop-proof 安全停机与信号处理**：
   - 正常退出、``stop(return_to_zero=True, wait=True)``、第 1 次 Ctrl+C 均走
     minimum-jerk / 限速平滑回零 → 到位确认 → 失能释放总线；回零途中第 2 次 Ctrl+C 立即急停失能。
"""

from __future__ import annotations

import math
import os
import signal
import threading
import time
from collections import deque
from contextlib import contextmanager
from dataclasses import dataclass, field
from typing import Any, Callable, Iterator, Optional, Sequence

import yaml

try:
    from motorbridge import Controller, Mode, CallError
except ImportError:  # pragma: no cover - 允许在无 motorbridge 的纯单测环境导入纯算法函数
    Controller = None  # type: ignore[assignment]
    Mode = None  # type: ignore[assignment]
    CallError = RuntimeError  # type: ignore[assignment]


DEFAULT_DM_MIT_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "config", "dm_mit.yaml"
)

# 代码级绝对硬件力矩硬天花板（Nm）：Damiao 4340P 峰值 27Nm，4310 峰值 7Nm
HARD_MOTOR_PEAK_TAU_NM = (27.0, 27.0, 27.0, 7.0, 7.0, 7.0)
HARD_MAX_GRAVITY_TAU_NM = (0.0, 11.0, 11.0, 2.5, 1.8, 1.0)

_CAN_FAULT_ACTIONS = ("return_to_zero", "emergency")
_CONTROL_MODES = ("mit", "posvel")


@dataclass(frozen=True)
class DMMotorConfig:
    """单台达妙电机配置。"""

    name: str
    motor_id: int
    feedback_id: int
    model: str
    kp: float
    kd: float
    vel_kp: float = 0.0125
    vel_ki: float = 0.004
    pos_kp: float = 150.0
    pos_ki: float = 0.5
    vlim: float = 3.0


@dataclass(frozen=True)
class TemperatureThresholds:
    """三级过温保护阈值（°C）。"""

    alarm_c: float = 75.0
    return_zero_c: float = 88.0
    disconnect_c: float = 98.0


@dataclass(frozen=True)
class DMHardwareConfig:
    """B601-DM 硬件与总线配置。"""

    name: str = "reBot_B601_DM"
    channel: str = "/dev/ttyACM0"
    baudrate: int = 921600
    control_mode: str = "mit"
    control_hz: float = 200.0
    telemetry_hz: float = 2.0
    return_zero_speed_deg_s: float = 18.0
    return_zero_tol_deg: float = 2.0
    return_zero_timeout_s: float = 20.0
    temperatures: TemperatureThresholds = field(default_factory=TemperatureThresholds)
    motors: tuple[DMMotorConfig, ...] = (
        DMMotorConfig("joint1", 0x01, 0x11, "4340P", 120.0, 8.0, 0.0125, 0.004, 150.0, 0.5, 5.0),
        DMMotorConfig("joint2", 0x02, 0x12, "4340P", 120.0, 8.0, 0.0125, 0.004, 150.0, 0.5, 5.0),
        DMMotorConfig("joint3", 0x03, 0x13, "4340P", 120.0, 8.0, 0.0125, 0.004, 150.0, 0.5, 5.0),
        DMMotorConfig("joint4", 0x04, 0x14, "4310", 18.0, 2.0, 0.0008, 0.002, 70.0, 1.0, 3.0),
        DMMotorConfig("joint5", 0x05, 0x15, "4310", 18.0, 2.0, 0.0008, 0.002, 60.0, 1.0, 3.0),
        DMMotorConfig("joint6", 0x06, 0x16, "4310", 18.0, 2.0, 0.0008, 0.002, 60.0, 1.0, 3.0),
        DMMotorConfig("gripper", 0x07, 0x17, "4310", 8.0, 1.0, 0.0008, 0.002, 50.0, 1.0, 3.0),
    )


DEFAULT_DM_MIT_EXT_CFG: dict[str, Any] = {
    "enable": True,
    "can_fault_window_s": 0.03,
    "can_fault_action": "return_to_zero",
    "can_error_log_interval_s": 1.0,
    "feedback_cache": True,
    "feedback_poll_once": True,
    "feedback_trusted": True,
    "feedback_max_age_s": 0.05,
    "record_max_s": 30.0,
    # P1
    "vel_ff_enable": True,
    "vel_ff_scale": [0.70, 0.85, 0.85, 0.85, 0.85, 0.85],
    "foh_interp_enable": True,
    "foh_upper_dt_s": 0.010,
    "vel_ff_watchdog_s": 0.040,
    "vel_ff_watchdog_tau_s": 0.030,
    "vel_ff_lpf_tau_s": 0.008,
    # P3 M2
    "gravity_ff_enable": True,
    "gravity_ff_scale": [0.0, 0.55, 0.65, 0.65, 0.0, 0.0],
    "gravity_ff_max_nm": [0.0, 10.0, 10.0, 2.2, 1.5, 0.8],
    "gravity_payload_mass_kg": 0.095,
    "gravity_payload_com_link6_m": [0.055, 0.000, 0.075],
    "gravity_ff_ramp_in_s": 0.50,
    "gravity_ff_slew_nm_s": 25.0,
    # P3 M3
    "gain_sched_enable": True,
    "gain_sched_r_min_m": 0.18,
    "gain_sched_r_max_m": 0.36,
    "gain_sched_kd_near": [8.0, 8.0, 8.0, 2.0, 2.0, 2.0],
    "gain_sched_kd_far": [11.0, 10.0, 10.0, 2.0, 2.0, 2.0],
    "gain_sched_kp_scale": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    "gain_sched_slew_per_s": 8.0,
    # P4 M6
    "supervisor_enable": True,
    "track_err_limit_deg": 12.0,
    "track_err_window_s": 0.08,
    "tau_limit_nm": [15.0, 21.0, 21.0, 5.2, 5.2, 5.2],
    "tau_limit_window_s": 0.08,
    "thermal_derate_start_c": 70.0,
    "thermal_derate_full_c": 88.0,
    "thermal_derate_min_scale": 0.40,
}


# ─────────────────────────────────────────────────────────────────────────────
# 配置校验与加载
# ─────────────────────────────────────────────────────────────────────────────

def _bounded_float(val: Any, lo: float, hi: float, name: str) -> float:
    if isinstance(val, bool) or not isinstance(val, (int, float)):
        raise ValueError(f"{name} 必须是数值，收到 {val!r}")
    out = float(val)
    if not (math.isfinite(out) and lo <= out <= hi):
        raise ValueError(f"{name}={out} 超出允许范围 [{lo}, {hi}]")
    return out


def _bounded_vec(val: Any, n: int, lo: float | Sequence[float], hi: float | Sequence[float], name: str) -> list[float]:
    if not isinstance(val, (list, tuple)) or len(val) != n:
        raise ValueError(f"{name} 必须是长度为 {n} 的列表")
    lo_seq = [lo] * n if isinstance(lo, (int, float)) else list(lo)
    hi_seq = [hi] * n if isinstance(hi, (int, float)) else list(hi)
    out: list[float] = []
    for i, item in enumerate(val):
        out.append(_bounded_float(item, float(lo_seq[i]), float(hi_seq[i]), f"{name}[{i}]"))
    return out


def validate_dm_mit_ext_cfg(cfg: Optional[dict[str, Any]]) -> dict[str, Any]:
    """校验并补全 DM MIT 扩展配置字典（严格锁死 4340P/4310 硬件安全上限）。"""
    if cfg is None:
        cfg = {}
    if not isinstance(cfg, dict):
        raise ValueError("mit_ext 配置必须是字典")
    unknown = set(cfg) - set(DEFAULT_DM_MIT_EXT_CFG)
    if unknown:
        raise ValueError(f"mit_ext 存在未知配置项: {sorted(unknown)}")
    merged = dict(DEFAULT_DM_MIT_EXT_CFG)
    merged.update(cfg)

    for key in (
        "enable", "feedback_cache", "feedback_poll_once", "feedback_trusted",
        "vel_ff_enable", "foh_interp_enable", "gravity_ff_enable",
        "gain_sched_enable", "supervisor_enable",
    ):
        if not isinstance(merged[key], bool):
            raise ValueError(f"mit_ext.{key} 必须是 true/false")

    merged["can_fault_window_s"] = _bounded_float(merged["can_fault_window_s"], 0.01, 0.15, "mit_ext.can_fault_window_s")
    if merged["can_fault_action"] not in _CAN_FAULT_ACTIONS:
        raise ValueError(f"mit_ext.can_fault_action 必须为 {_CAN_FAULT_ACTIONS}")
    merged["can_error_log_interval_s"] = _bounded_float(merged["can_error_log_interval_s"], 0.1, 10.0, "mit_ext.can_error_log_interval_s")
    merged["feedback_max_age_s"] = _bounded_float(merged["feedback_max_age_s"], 0.01, 0.50, "mit_ext.feedback_max_age_s")
    merged["record_max_s"] = _bounded_float(merged["record_max_s"], 1.0, 120.0, "mit_ext.record_max_s")
    if merged["feedback_trusted"] and not merged["feedback_cache"]:
        raise ValueError("mit_ext.feedback_trusted=true 需要同时启用 feedback_cache=true")

    # P1
    merged["vel_ff_scale"] = _bounded_vec(merged["vel_ff_scale"], 6, 0.0, 1.0, "mit_ext.vel_ff_scale")
    merged["foh_upper_dt_s"] = _bounded_float(merged["foh_upper_dt_s"], 0.002, 0.05, "mit_ext.foh_upper_dt_s")
    merged["vel_ff_watchdog_s"] = _bounded_float(merged["vel_ff_watchdog_s"], 0.015, 0.20, "mit_ext.vel_ff_watchdog_s")
    merged["vel_ff_watchdog_tau_s"] = _bounded_float(merged["vel_ff_watchdog_tau_s"], 0.005, 0.10, "mit_ext.vel_ff_watchdog_tau_s")
    merged["vel_ff_lpf_tau_s"] = _bounded_float(merged["vel_ff_lpf_tau_s"], 0.0, 0.05, "mit_ext.vel_ff_lpf_tau_s")

    # P3 M2
    merged["gravity_ff_scale"] = _bounded_vec(merged["gravity_ff_scale"], 6, 0.0, 1.0, "mit_ext.gravity_ff_scale")
    if merged["gravity_ff_scale"][0] != 0.0:
        raise ValueError("J1 为竖直回转轴，mit_ext.gravity_ff_scale[0] 必须为 0.0")
    merged["gravity_ff_max_nm"] = _bounded_vec(
        merged["gravity_ff_max_nm"], 6, 0.0, HARD_MAX_GRAVITY_TAU_NM, "mit_ext.gravity_ff_max_nm"
    )
    merged["gravity_payload_mass_kg"] = _bounded_float(merged["gravity_payload_mass_kg"], 0.0, 1.0, "mit_ext.gravity_payload_mass_kg")
    merged["gravity_payload_com_link6_m"] = _bounded_vec(merged["gravity_payload_com_link6_m"], 3, -0.35, 0.35, "mit_ext.gravity_payload_com_link6_m")
    merged["gravity_ff_ramp_in_s"] = _bounded_float(merged["gravity_ff_ramp_in_s"], 0.05, 5.0, "mit_ext.gravity_ff_ramp_in_s")
    merged["gravity_ff_slew_nm_s"] = _bounded_float(merged["gravity_ff_slew_nm_s"], 1.0, 60.0, "mit_ext.gravity_ff_slew_nm_s")

    # P3 M3
    merged["gain_sched_r_min_m"] = _bounded_float(merged["gain_sched_r_min_m"], 0.05, 0.40, "mit_ext.gain_sched_r_min_m")
    merged["gain_sched_r_max_m"] = _bounded_float(merged["gain_sched_r_max_m"], merged["gain_sched_r_min_m"] + 0.02, 0.50, "mit_ext.gain_sched_r_max_m")
    merged["gain_sched_kd_near"] = _bounded_vec(merged["gain_sched_kd_near"], 6, 0.2, 20.0, "mit_ext.gain_sched_kd_near")
    merged["gain_sched_kd_far"] = _bounded_vec(merged["gain_sched_kd_far"], 6, 0.2, 25.0, "mit_ext.gain_sched_kd_far")
    merged["gain_sched_kp_scale"] = _bounded_vec(merged["gain_sched_kp_scale"], 6, 0.5, 1.5, "mit_ext.gain_sched_kp_scale")
    merged["gain_sched_slew_per_s"] = _bounded_float(merged["gain_sched_slew_per_s"], 0.5, 50.0, "mit_ext.gain_sched_slew_per_s")

    # P4 M6
    merged["track_err_limit_deg"] = _bounded_float(merged["track_err_limit_deg"], 3.0, 35.0, "mit_ext.track_err_limit_deg")
    merged["track_err_window_s"] = _bounded_float(merged["track_err_window_s"], 0.02, 0.50, "mit_ext.track_err_window_s")
    merged["tau_limit_nm"] = _bounded_vec(merged["tau_limit_nm"], 6, 1.0, HARD_MOTOR_PEAK_TAU_NM, "mit_ext.tau_limit_nm")
    merged["tau_limit_window_s"] = _bounded_float(merged["tau_limit_window_s"], 0.02, 0.50, "mit_ext.tau_limit_window_s")
    merged["thermal_derate_start_c"] = _bounded_float(merged["thermal_derate_start_c"], 45.0, 100.0, "mit_ext.thermal_derate_start_c")
    merged["thermal_derate_full_c"] = _bounded_float(merged["thermal_derate_full_c"], merged["thermal_derate_start_c"] + 5.0, 120.0, "mit_ext.thermal_derate_full_c")
    merged["thermal_derate_min_scale"] = _bounded_float(merged["thermal_derate_min_scale"], 0.20, 1.0, "mit_ext.thermal_derate_min_scale")
    return merged


def load_dm_config(path: str = DEFAULT_DM_MIT_CONFIG_PATH) -> tuple[DMHardwareConfig, dict[str, Any]]:
    """加载并校验 ``config/dm_mit.yaml``，返回 ``(DMHardwareConfig, mit_ext_cfg)``。"""
    if not os.path.exists(path):
        return DMHardwareConfig(), validate_dm_mit_ext_cfg({})
    with open(path, encoding="utf-8") as stream:
        raw = yaml.safe_load(stream) or {}

    hw = raw.get("hardware") or {}
    mode = str(hw.get("control_mode", "mit")).strip().lower()
    if mode not in _CONTROL_MODES:
        raise ValueError(f"hardware.control_mode 必须为 {_CONTROL_MODES}，收到 {mode!r}")

    temps_raw = hw.get("temperatures") or {}
    alarm_c = _bounded_float(temps_raw.get("alarm_c", 75.0), 40.0, 110.0, "hardware.temperatures.alarm_c")
    ret_c = _bounded_float(temps_raw.get("return_zero_c", 88.0), alarm_c + 2.0, 120.0, "hardware.temperatures.return_zero_c")
    disc_c = _bounded_float(temps_raw.get("disconnect_c", 98.0), ret_c + 2.0, 130.0, "hardware.temperatures.disconnect_c")
    temps = TemperatureThresholds(alarm_c=alarm_c, return_zero_c=ret_c, disconnect_c=disc_c)

    motors_raw = hw.get("motors")
    if motors_raw:
        if len(motors_raw) != 7:
            raise ValueError(f"B601-DM 必须配置 7 路电机（6 关节 + 1 夹爪），当前 {len(motors_raw)}")
        motors = tuple(
            DMMotorConfig(
                name=str(m["name"]),
                motor_id=int(m["motor_id"]),
                feedback_id=int(m["feedback_id"]),
                model=str(m.get("model", "4340P")),
                kp=_bounded_float(m.get("kp", 20.0), 0.0, 300.0, f"motors[{i}].kp"),
                kd=_bounded_float(m.get("kd", 2.0), 0.0, 30.0, f"motors[{i}].kd"),
                vel_kp=float(m.get("vel_kp", 0.0125)),
                vel_ki=float(m.get("vel_ki", 0.004)),
                pos_kp=float(m.get("pos_kp", 100.0)),
                pos_ki=float(m.get("pos_ki", 0.5)),
                vlim=_bounded_float(m.get("vlim", 3.0), 0.1, 15.0, f"motors[{i}].vlim"),
            )
            for i, m in enumerate(motors_raw)
        )
    else:
        motors = DMHardwareConfig().motors

    hw_cfg = DMHardwareConfig(
        name=str(hw.get("name", "reBot_B601_DM")),
        channel=str(hw.get("channel", "/dev/ttyACM0")),
        baudrate=int(hw.get("baudrate", 921600)),
        control_mode=mode,
        control_hz=_bounded_float(hw.get("control_hz", 200.0), 50.0, 500.0, "hardware.control_hz"),
        telemetry_hz=_bounded_float(hw.get("telemetry_hz", 2.0), 0.5, 20.0, "hardware.telemetry_hz"),
        return_zero_speed_deg_s=_bounded_float(hw.get("return_zero_speed_deg_s", 18.0), 5.0, 45.0, "hardware.return_zero_speed_deg_s"),
        return_zero_tol_deg=_bounded_float(hw.get("return_zero_tol_deg", 2.0), 0.5, 10.0, "hardware.return_zero_tol_deg"),
        return_zero_timeout_s=_bounded_float(hw.get("return_zero_timeout_s", 20.0), 3.0, 60.0, "hardware.return_zero_timeout_s"),
        temperatures=temps,
        motors=motors,
    )
    ext_cfg = validate_dm_mit_ext_cfg(raw.get("mit_ext"))
    return hw_cfg, ext_cfg


# ─────────────────────────────────────────────────────────────────────────────
# P0 ~ P4 纯算法组件（无硬件依赖，可 100% 离线单元测试）
# ─────────────────────────────────────────────────────────────────────────────

class CanFaultMonitor:
    """P0 · S1 总线瞬时通信容错监控器（按控制频率推导时间窗，不硬编码拍数）。"""

    def __init__(self, control_hz: float, fault_window_s: float = 0.03, log_interval_s: float = 1.0) -> None:
        self.control_hz = max(10.0, float(control_hz))
        self.fault_window_s = max(0.01, float(fault_window_s))
        self.log_interval_s = max(0.1, float(log_interval_s))
        self.fail_threshold_ticks = max(2, int(round(self.fault_window_s * self.control_hz)))
        self.consecutive_failures: int = 0
        self.total_failures: int = 0
        self._last_log_mono: float = -1e9

    def record_success(self) -> None:
        self.consecutive_failures = 0

    def record_failure(self, now_mono: float) -> tuple[bool, bool]:
        """记录一次发送失败，返回 ``(is_fatal_fault, should_log_warning)``。"""
        self.consecutive_failures += 1
        self.total_failures += 1
        fatal = self.consecutive_failures >= self.fail_threshold_ticks
        should_log = (now_mono - self._last_log_mono) >= self.log_interval_s
        if should_log:
            self._last_log_mono = now_mono
        return fatal, should_log


class SafetySupervisor:
    """P4 · M6 200Hz 主动安全监督层（跟踪超差/过力矩软停机保持 + 温升线性降额）。"""

    def __init__(self, ext_cfg: dict[str, Any], control_hz: float = 200.0) -> None:
        self.enabled = bool(ext_cfg.get("supervisor_enable", True))
        self.control_hz = max(10.0, float(control_hz))
        self.track_err_limit_rad = math.radians(float(ext_cfg.get("track_err_limit_deg", 12.0)))
        self.track_err_ticks = max(2, int(round(float(ext_cfg.get("track_err_window_s", 0.08)) * self.control_hz)))
        self.tau_limit_nm = [float(x) for x in ext_cfg.get("tau_limit_nm", [15.0, 21.0, 21.0, 5.2, 5.2, 5.2])]
        self.tau_limit_ticks = max(2, int(round(float(ext_cfg.get("tau_limit_window_s", 0.08)) * self.control_hz)))
        self.thermal_start_c = float(ext_cfg.get("thermal_derate_start_c", 70.0))
        self.thermal_full_c = float(ext_cfg.get("thermal_derate_full_c", 88.0))
        self.thermal_min_scale = float(ext_cfg.get("thermal_derate_min_scale", 0.40))

        self.latched: bool = False
        self.reason: str = ""
        self.thermal_speed_scale: float = 1.0
        self.max_track_err_deg: float = 0.0
        self.max_tau_ratio: float = 0.0
        self._err_streak: int = 0
        self._tau_streak: int = 0

    def update(
        self,
        q_cmd_6: Sequence[float],
        q_act_6: Sequence[float],
        tau_act_6: Sequence[float],
        t_max_c: Optional[float],
        feedback_valid: bool,
    ) -> dict[str, Any]:
        # 温升前置线性降额（即使 supervisor_enable=False 也照常计算供监控）
        if t_max_c is not None and math.isfinite(t_max_c):
            if t_max_c <= self.thermal_start_c:
                self.thermal_speed_scale = 1.0
            elif t_max_c >= self.thermal_full_c:
                self.thermal_speed_scale = self.thermal_min_scale
            else:
                frac = (t_max_c - self.thermal_start_c) / max(1e-6, self.thermal_full_c - self.thermal_start_c)
                self.thermal_speed_scale = 1.0 - frac * (1.0 - self.thermal_min_scale)
        else:
            self.thermal_speed_scale = 1.0

        if not self.enabled or not feedback_valid:
            self._err_streak = 0
            self._tau_streak = 0
            return self.status()

        errs = [abs(float(q_cmd_6[i]) - float(q_act_6[i])) for i in range(6)]
        max_err_rad = max(errs)
        self.max_track_err_deg = math.degrees(max_err_rad)

        ratios = [abs(float(tau_act_6[i])) / max(0.1, self.tau_limit_nm[i]) for i in range(6)]
        self.max_tau_ratio = max(ratios)

        if max_err_rad > self.track_err_limit_rad:
            self._err_streak += 1
        else:
            self._err_streak = 0

        if self.max_tau_ratio > 1.0:
            self._tau_streak += 1
        else:
            self._tau_streak = 0

        if not self.latched:
            if self._err_streak >= self.track_err_ticks:
                worst_j = errs.index(max_err_rad) + 1
                self.latched = True
                self.reason = f"track_err_J{worst_j}={self.max_track_err_deg:.1f}deg"
            elif self._tau_streak >= self.tau_limit_ticks:
                worst_j = ratios.index(self.max_tau_ratio) + 1
                self.latched = True
                self.reason = f"over_torque_J{worst_j}={abs(float(tau_act_6[worst_j - 1])):.1f}Nm"

        return self.status()

    def clear(self) -> bool:
        was = self.latched
        self.latched = False
        self.reason = ""
        self._err_streak = 0
        self._tau_streak = 0
        return was

    def status(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "latched": self.latched,
            "reason": self.reason,
            "thermal_speed_scale": self.thermal_speed_scale,
            "max_track_err_deg": self.max_track_err_deg,
            "max_tau_ratio": self.max_tau_ratio,
        }


def cradle_zero_fade(q_rad_6: Sequence[float], fade_lo_rad: float = 0.05, fade_hi_rad: float = 0.16) -> float:
    """近零位机械支架平滑淡出系数 [0.0, 1.0]（对正负关节角对称，适配 DM 的 J2/J3 ∈ [-π, 0]）。

    当 J2 与 J3 同时靠近 0 零位支架（max(|J2|, |J3|) <= 0.05 rad ≈ 2.9°）时，淡出至 0.0，
    避免回零停靠支架时重力前馈顶撞机械止挡。
    """
    lift = max(abs(float(q_rad_6[1])), abs(float(q_rad_6[2])))
    if lift <= fade_lo_rad:
        return 0.0
    if lift >= fade_hi_rad:
        return 1.0
    s = (lift - fade_lo_rad) / max(1e-6, fade_hi_rad - fade_lo_rad)
    return s * s * (3.0 - 2.0 * s)


def compute_p1_mit_step(
    *,
    target_rad_7: Sequence[float],
    command_rad_7: Sequence[float],
    max_speeds_rad_s_7: Sequence[float],
    vel_ff_raw_rad_s_7: Sequence[float],
    vel_ff_filt_rad_s_7: Sequence[float],
    vel_ff_active: bool,
    vel_ff_age_s: float,
    period_s: float,
    ext_cfg: dict[str, Any],
) -> tuple[list[float], list[float], list[float]]:
    """计算单拍 200Hz P1 位置目标 + FOH 插值 + 速度前馈（含看门狗与 LPF）。

    Returns:
        ``(new_command_rad_7, dq_des_send_rad_s_7, new_vel_ff_filt_7)``
    """
    n = len(target_rad_7)
    new_cmd = [0.0] * n
    dq_send = [0.0] * n
    new_filt = list(vel_ff_filt_rad_s_7)

    vel_ff_on = bool(ext_cfg.get("vel_ff_enable", False)) and vel_ff_active
    foh_on = bool(ext_cfg.get("foh_interp_enable", True))
    upper_dt = float(ext_cfg.get("foh_upper_dt_s", 0.010))
    wd_s = float(ext_cfg.get("vel_ff_watchdog_s", 0.040))
    wd_tau = float(ext_cfg.get("vel_ff_watchdog_tau_s", 0.030))
    lpf_tau = float(ext_cfg.get("vel_ff_lpf_tau_s", 0.008))
    scales = ext_cfg.get("vel_ff_scale", [0.70, 0.85, 0.85, 0.85, 0.85, 0.85])

    # 看门狗衰减系数（上层超过 wd_s 未刷新，平滑衰减 dq_des 至 0）
    if vel_ff_on:
        if vel_ff_age_s <= wd_s:
            wd_factor = 1.0
        else:
            wd_factor = math.exp(-(vel_ff_age_s - wd_s) / max(1e-4, wd_tau))
    else:
        wd_factor = 0.0

    alpha = 1.0 if lpf_tau <= 1e-6 else (period_s / (lpf_tau + period_s))

    for i in range(n):
        tgt = float(target_rad_7[i])
        cmd = float(command_rad_7[i])
        vmax = max(1e-4, float(max_speeds_rad_s_7[i]))

        # J7 夹爪（索引 >= 6）或非前馈模式：纯位置斜坡，dq_des 恒为 0
        if i >= 6 or not vel_ff_on:
            err = tgt - cmd
            max_step = vmax * period_s
            new_cmd[i] = cmd + max(-max_step, min(max_step, err))
            new_filt[i] = 0.0
            dq_send[i] = 0.0
            continue

        raw_v = float(vel_ff_raw_rad_s_7[i]) * wd_factor
        raw_v = max(-vmax, min(vmax, raw_v))
        filt_v = new_filt[i] + alpha * (raw_v - new_filt[i])
        new_filt[i] = filt_v

        # FOH 向后插值：仅在目标点与上层到达时间窗内平滑插值，永不外推越过 tgt
        if foh_on and vel_ff_age_s < upper_dt:
            back_t = max(0.0, upper_dt - vel_ff_age_s)
            q_interp = tgt - filt_v * back_t
            # 反向冲突保护：保证插值点位于 cmd 与 tgt 之间或朝 tgt 收敛
            if (tgt - cmd) * (q_interp - cmd) < 0.0:
                q_interp = cmd
        else:
            q_interp = tgt

        err = q_interp - cmd
        max_step = vmax * period_s
        step = max(-max_step, min(max_step, err))
        new_cmd[i] = cmd + step

        # 方向冲突保护：目标误差极小或与前馈速度反向时清零前馈
        if abs(tgt - new_cmd[i]) < 1e-4 and abs(filt_v) < 0.02:
            dq_send[i] = 0.0
        elif (tgt - cmd) * filt_v < -1e-5:
            dq_send[i] = 0.0
        else:
            dq_send[i] = filt_v * float(scales[i])

    return new_cmd, dq_send, new_filt


def compute_p3_gravity_tau(
    *,
    raw_grav_nm_6: Sequence[float],
    q_for_fade_6: Sequence[float],
    prev_tau_6: Sequence[float],
    ramp_elapsed_s: float,
    period_s: float,
    ext_cfg: dict[str, Any],
) -> list[float]:
    """计算 P3 M2 静态重力前馈力矩（含启动淡入、零位支架淡出、硬封顶、斜率限幅）。"""
    if not ext_cfg.get("gravity_ff_enable", False):
        return [0.0] * 6

    ramp_s = max(0.05, float(ext_cfg.get("gravity_ff_ramp_in_s", 0.50)))
    ramp = max(0.0, min(1.0, ramp_elapsed_s / ramp_s))
    fade = cradle_zero_fade(q_for_fade_6)
    scales = ext_cfg.get("gravity_ff_scale", [0.0, 0.55, 0.65, 0.65, 0.0, 0.0])
    caps = ext_cfg.get("gravity_ff_max_nm", [0.0, 10.0, 10.0, 2.2, 1.5, 0.8])
    max_dtau = float(ext_cfg.get("gravity_ff_slew_nm_s", 25.0)) * period_s

    out = [0.0] * 6
    for i in range(6):
        if i == 0:
            out[i] = 0.0
            continue
        raw = float(raw_grav_nm_6[i])
        if not math.isfinite(raw):
            raw = 0.0
        target = raw * float(scales[i]) * ramp * fade
        hard_cap = min(float(caps[i]), float(HARD_MAX_GRAVITY_TAU_NM[i]))
        target = max(-hard_cap, min(hard_cap, target))
        prev = float(prev_tau_6[i])
        out[i] = prev + max(-max_dtau, min(max_dtau, target - prev))
    return out


def compute_p3_gains(
    *,
    base_kp_6: Sequence[float],
    base_kd_6: Sequence[float],
    prev_kd_6: Sequence[float],
    reach_r_m: float,
    period_s: float,
    ext_cfg: dict[str, Any],
) -> tuple[list[float], list[float]]:
    """计算 P3 M3 伸展度自适应 (kp, kd) 增益（带变化斜率上限与 [0.8x, 1.8x] 安全包络）。"""
    if not ext_cfg.get("gain_sched_enable", False):
        return [float(x) for x in base_kp_6], [float(x) for x in base_kd_6]

    r_min = float(ext_cfg.get("gain_sched_r_min_m", 0.18))
    r_max = float(ext_cfg.get("gain_sched_r_max_m", 0.36))
    ratio = max(0.0, min(1.0, (float(reach_r_m) - r_min) / max(1e-4, r_max - r_min)))
    kd_near = ext_cfg.get("gain_sched_kd_near", base_kd_6)
    kd_far = ext_cfg.get("gain_sched_kd_far", base_kd_6)
    kp_scale = ext_cfg.get("gain_sched_kp_scale", [1.0] * 6)
    max_dkd = float(ext_cfg.get("gain_sched_slew_per_s", 8.0)) * period_s

    kp_out = [0.0] * 6
    kd_out = [0.0] * 6
    for i in range(6):
        bkp = float(base_kp_6[i])
        bkd = float(base_kd_6[i])
        kp_out[i] = max(0.8 * bkp, min(1.5 * bkp, bkp * float(kp_scale[i])))
        tgt_kd = float(kd_near[i]) + ratio * (float(kd_far[i]) - float(kd_near[i]))
        tgt_kd = max(0.8 * bkd, min(1.8 * bkd, tgt_kd))
        prev = float(prev_kd_6[i])
        kd_out[i] = prev + max(-max_dkd, min(max_dkd, tgt_kd - prev))
    return kp_out, kd_out


# ─────────────────────────────────────────────────────────────────────────────
# B601-DM 实时控制器主类（200Hz MIT / POS_VEL + P0~P4 + 安全回零）
# ─────────────────────────────────────────────────────────────────────────────

class ReBotDMMITController:
    """Seeed reBot Arm B601-DM 7 轴统一底层控制器（单主机总线句柄）。

    外部角度接口统一使用【度 / 度每秒】（7 路：6 关节 + J7 夹爪），
    内部与 motorbridge 一律使用【弧度 / 弧度每秒 / Nm】。
    """

    _IO_LOCK_SHUTDOWN_TIMEOUT_S = 2.0
    _TEMP_ALARM_HYSTERESIS_C = 2.0

    def __init__(
        self,
        config_path: str = DEFAULT_DM_MIT_CONFIG_PATH,
        hw_cfg: Optional[DMHardwareConfig] = None,
        ext_cfg: Optional[dict[str, Any]] = None,
        *,
        controller_factory: Optional[Callable[[DMHardwareConfig], Any]] = None,
    ) -> None:
        if hw_cfg is None or ext_cfg is None:
            loaded_hw, loaded_ext = load_dm_config(config_path)
            hw_cfg = hw_cfg or loaded_hw
            ext_cfg = ext_cfg if ext_cfg is not None else loaded_ext
        else:
            ext_cfg = validate_dm_mit_ext_cfg(ext_cfg)

        self.config = hw_cfg
        self.ext_cfg = ext_cfg
        self.channel = hw_cfg.channel
        self.control_mode = hw_cfg.control_mode
        self.control_hz = float(hw_cfg.control_hz)
        self.telemetry_hz = float(hw_cfg.telemetry_hz)
        self.motor_count = len(hw_cfg.motors)
        self._controller_factory = controller_factory

        self.controller: Any = None
        self.motors: list[Any] = []

        self.target_positions = [0.0] * self.motor_count
        self.command_positions = [0.0] * self.motor_count
        self.max_speeds_rad_s = [math.radians(20.0)] * self.motor_count

        # P1 速度前馈状态
        self._vel_ff_raw = [0.0] * self.motor_count
        self._vel_ff_filt = [0.0] * self.motor_count
        self._vel_ff_active = False
        self._vel_ff_stamp_mono = 0.0

        # P3 重力前馈 & 增益调度状态
        self._gravity_fn: Optional[Callable[[Sequence[float]], Sequence[float]]] = None
        self._tau_ff_last = [0.0] * 6
        self._base_kp_6 = [float(m.kp) for m in hw_cfg.motors[:6]]
        self._base_kd_6 = [float(m.kd) for m in hw_cfg.motors[:6]]
        self._kd_last_6 = list(self._base_kd_6)
        self._reach_radius_m = 0.20
        self._start_mono = 0.0

        # P0 反馈缓存 & 故障监控 & 统计
        self._can_monitor = CanFaultMonitor(
            control_hz=self.control_hz,
            fault_window_s=float(ext_cfg["can_fault_window_s"]),
            log_interval_s=float(ext_cfg["can_error_log_interval_s"]),
        )
        self._supervisor = SafetySupervisor(ext_cfg, control_hz=self.control_hz)
        self._fb_lock = threading.Lock()
        self._fb_q = [0.0] * self.motor_count
        self._fb_dq = [0.0] * self.motor_count
        self._fb_tau = [0.0] * self.motor_count
        self._fb_tmos = [float("nan")] * self.motor_count
        self._fb_trotor = [float("nan")] * self.motor_count
        self._fb_valid = [False] * self.motor_count
        self._fb_stamp_mono = 0.0
        self._fb_seq = 0
        self._fb_err_streak = 0
        self._fb_errors_total = 0
        self._fb_disabled = False
        self._fb_prev_sig: Optional[tuple[float, ...]] = None
        self._fb_change_total = 0
        self._fb_change_hits = 0

        self._period_hist_ms: deque[float] = deque(maxlen=400)
        self._send_hist_ms: deque[float] = deque(maxlen=400)
        self._sample_hist_ms: deque[float] = deque(maxlen=400)
        self._rec_lock = threading.Lock()
        self._recording = False
        self._rec_rows: list[tuple[float, list[float], list[float], list[float], list[float]]] = []
        self._rec_start_mono = 0.0

        # 线程与生命周期同步
        self.target_lock = threading.RLock()
        self.io_lock = threading.RLock()
        self.shutdown_lock = threading.Lock()
        self.worker_stop_event = threading.Event()
        self.abort_return_event = threading.Event()
        self.shutdown_done_event = threading.Event()

        self.control_thread: Optional[threading.Thread] = None
        self.telemetry_thread: Optional[threading.Thread] = None
        self.shutdown_thread: Optional[threading.Thread] = None
        self.started = False
        self.shutdown_started = False
        self.signal_count = 0
        self.shutdown_reason = ""
        self.last_error: Optional[Exception] = None
        self._temp_alarm_active = [False] * self.motor_count

    @contextmanager
    def _io_lock_guard(self, timeout: Optional[float] = None) -> Iterator[None]:
        ok = self.io_lock.acquire() if timeout is None else self.io_lock.acquire(timeout=timeout)
        if not ok:
            raise TimeoutError(f"等待 DM 总线 io_lock 超时 ({timeout}s)")
        try:
            yield
        finally:
            self.io_lock.release()

    def _create_bus_controller(self) -> Any:
        if self._controller_factory is not None:
            return self._controller_factory(self.config)
        if Controller is None:
            raise ImportError("未安装 motorbridge，无法连接真实 B601-DM 机械臂")
        if self.channel.startswith("/dev/tty"):
            return Controller.from_dm_serial(self.channel, int(self.config.baudrate))
        return Controller(self.channel)

    def _configure_motor_modes(self) -> None:
        if Mode is None:
            return
        for idx, (motor, mcfg) in enumerate(zip(self.motors, self.config.motors)):
            # 夹爪 J7（idx == 6）始终采用 MIT 模式防堵转
            use_posvel = (self.control_mode == "posvel") and (idx < 6)
            with self._io_lock_guard():
                if use_posvel:
                    motor.write_register_f32(25, float(mcfg.vel_kp))
                    motor.write_register_f32(26, float(mcfg.vel_ki))
                    motor.write_register_f32(27, float(mcfg.pos_kp))
                    motor.write_register_f32(28, float(mcfg.pos_ki))
                    time.sleep(0.02)
                    motor.ensure_mode(Mode.POS_VEL, 1000)
                else:
                    motor.ensure_mode(Mode.MIT, 1000)
            time.sleep(0.03)

    def connect(self) -> None:
        """连接 dm-serial / CAN、注册 7 路达妙电机、配置模式、使能并读取当前位姿锚定目标。"""
        if self.controller is not None:
            return
        print(f"[DM-Connect] 打开总线: {self.channel} (mode={self.control_mode}, rate={self.control_hz:.0f}Hz)")
        self.controller = self._create_bus_controller()
        try:
            for mcfg in self.config.motors:
                mot = self.controller.add_damiao_motor(mcfg.motor_id, mcfg.feedback_id, mcfg.model)
                self.motors.append(mot)
                print(f"[DM-Connect] 注册电机 {mcfg.name}: id={mcfg.motor_id:#04x} fb={mcfg.feedback_id:#04x} ({mcfg.model})")

            self._configure_motor_modes()

            with self._io_lock_guard():
                self.controller.enable_all()
            time.sleep(0.25)

            # 使能后读取当前实际角度作为初始目标，杜绝上电突跳
            q_init = self._read_positions_rad()
            with self.target_lock:
                self.target_positions[:] = q_init
                self.command_positions[:] = q_init

            print(f"[DM-Connect] 全部电机已使能，当前角度(deg): {[round(math.degrees(x), 2) for x in q_init]}")
        except Exception:
            self._disable_and_close()
            raise

    def start(self, *, enable_esc: bool = False, install_signal_handlers: bool = True) -> None:
        """连接硬件并启动 200Hz 实时控制线程与温度保护线程。"""
        del enable_esc  # 热键由上层主程序统一接管
        if self.started:
            return
        self.connect()
        self.worker_stop_event.clear()
        self.abort_return_event.clear()
        self.shutdown_done_event.clear()
        self.shutdown_started = False
        self.signal_count = 0
        self.last_error = None
        self._start_mono = time.monotonic()
        self.started = True

        self.control_thread = threading.Thread(target=self._control_loop, name="rebot-dm-ctrl-200hz", daemon=True)
        self.control_thread.start()

        self.telemetry_thread = threading.Thread(target=self._temperature_loop, name="rebot-dm-temp", daemon=True)
        self.telemetry_thread.start()

        if install_signal_handlers and threading.current_thread() is threading.main_thread():
            signal.signal(signal.SIGINT, self._signal_handler)
            signal.signal(signal.SIGTERM, self._signal_handler)

    def _signal_handler(self, signum: int, _frame: Any) -> None:
        self.signal_count += 1
        if self.signal_count == 1:
            print(f"\n[DM-Signal] 收到信号 {signum}，开始安全缓慢回零后失能（再按一次 Ctrl+C 立即急停失能）...")
            self.request_stop(reason=f"signal_{signum}", return_to_zero=True, wait=False)
        else:
            print("\n[DM-Signal] 收到第二次中断，立即中止回零并失能！")
            self.abort_return_event.set()
            self.request_stop(reason="double_interrupt", return_to_zero=False, emergency=True, wait=False)

    # ── 200Hz 实时控制主循环 ──────────────────────────────────────────────

    def _sample_feedback_under_io_lock(self, now_mono: float) -> tuple[bool, Optional[float]]:
        if not self.ext_cfg.get("feedback_cache", True) or self._fb_disabled:
            return False, None
        try:
            if self.ext_cfg.get("feedback_poll_once", True) and self.controller is not None:
                self.controller.poll_feedback_once()
            q = [0.0] * self.motor_count
            dq = [0.0] * self.motor_count
            tau = [0.0] * self.motor_count
            t_mos = [float("nan")] * self.motor_count
            t_rot = [float("nan")] * self.motor_count
            valid = [False] * self.motor_count
            t_max_vals: list[float] = []
            for i, motor in enumerate(self.motors):
                st = motor.get_state()
                if st is None:
                    continue
                pos = float(getattr(st, "pos", float("nan")))
                vel = float(getattr(st, "vel", 0.0))
                trq = float(getattr(st, "torq", 0.0))
                tm = float(getattr(st, "t_mos", float("nan")))
                tr = float(getattr(st, "t_rotor", float("nan")))
                if math.isfinite(pos) and math.isfinite(vel) and math.isfinite(trq):
                    q[i], dq[i], tau[i] = pos, vel, trq
                    valid[i] = True
                if math.isfinite(tm):
                    t_mos[i] = tm
                    t_max_vals.append(tm)
                if math.isfinite(tr):
                    t_rot[i] = tr
                    t_max_vals.append(tr)

            with self._fb_lock:
                self._fb_q = q
                self._fb_dq = dq
                self._fb_tau = tau
                self._fb_tmos = t_mos
                self._fb_trotor = t_rot
                self._fb_valid = valid
                self._fb_stamp_mono = now_mono
                self._fb_seq += 1
                if all(valid[:6]):
                    sig = tuple(q[:6]) + tuple(dq[:6]) + tuple(tau[:6])
                    if self._fb_prev_sig is not None:
                        self._fb_change_total += 1
                        if sig != self._fb_prev_sig:
                            self._fb_change_hits += 1
                    self._fb_prev_sig = sig
            self._fb_err_streak = 0
            all_6_ok = all(valid[:6])
            t_max = max(t_max_vals) if t_max_vals else None
            return all_6_ok, t_max
        except Exception:
            self._fb_err_streak += 1
            self._fb_errors_total += 1
            if self._fb_err_streak >= int(self.control_hz):
                self._fb_disabled = True
                print("[DM-FB] 反馈缓存连续异常 1s，已自动降级关闭缓存")
            return False, None

    def _compute_gravity_6(self, q_rad_6: Sequence[float]) -> list[float]:
        fn = self._gravity_fn
        if fn is None:
            return [0.0] * 6
        try:
            vals = fn(q_rad_6)
            return [float(vals[i]) if math.isfinite(float(vals[i])) else 0.0 for i in range(6)]
        except Exception:
            return [0.0] * 6

    def _send_frame(
        self,
        cmd_7: Sequence[float],
        dq_ff_7: Sequence[float],
        kp_6: Sequence[float],
        kd_6: Sequence[float],
        tau_ff_6: Sequence[float],
        *,
        lock_timeout: Optional[float] = None,
    ) -> tuple[bool, Optional[float]]:
        now_mono = time.monotonic()
        with self._io_lock_guard(lock_timeout):
            t_send0 = time.perf_counter()
            for idx, (motor, mcfg) in enumerate(zip(self.motors, self.config.motors)):
                if idx < 6:
                    if self.control_mode == "posvel":
                        vlim = min(float(mcfg.vlim), max(0.05, float(self.max_speeds_rad_s[idx])))
                        motor.send_pos_vel(float(cmd_7[idx]), vlim)
                    else:
                        motor.send_mit(
                            float(cmd_7[idx]),
                            float(dq_ff_7[idx]),
                            float(kp_6[idx]),
                            float(kd_6[idx]),
                            float(tau_ff_6[idx]),
                        )
                else:
                    # J7 夹爪恒为纯位置 MIT（dq=0, tau=0）
                    motor.send_mit(float(cmd_7[idx]), 0.0, float(mcfg.kp), float(mcfg.kd), 0.0)
            t_send1 = time.perf_counter()
            self._send_hist_ms.append((t_send1 - t_send0) * 1000.0)
            res = self._sample_feedback_under_io_lock(now_mono)
            self._sample_hist_ms.append((time.perf_counter() - t_send1) * 1000.0)
            return res

    def _control_loop(self) -> None:
        period = 1.0 / self.control_hz
        next_tick = time.perf_counter()
        prev_tick = next_tick

        while not self.worker_stop_event.is_set():
            now_perf = time.perf_counter()
            self._period_hist_ms.append((now_perf - prev_tick) * 1000.0)
            prev_tick = now_perf
            now_mono = time.monotonic()

            with self.target_lock:
                # 若 M6 主动安全监督锁存，强制原地软保持（target 钉死在当前 command，dq_ff=0，重力维持）
                if self._supervisor.latched:
                    self.target_positions[:6] = self.command_positions[:6]
                    self._vel_ff_active = False

                cmd_7, dq_ff_7, self._vel_ff_filt = compute_p1_mit_step(
                    target_rad_7=self.target_positions,
                    command_rad_7=self.command_positions,
                    max_speeds_rad_s_7=self.max_speeds_rad_s,
                    vel_ff_raw_rad_s_7=self._vel_ff_raw,
                    vel_ff_filt_rad_s_7=self._vel_ff_filt,
                    vel_ff_active=self._vel_ff_active,
                    vel_ff_age_s=max(0.0, now_mono - self._vel_ff_stamp_mono),
                    period_s=period,
                    ext_cfg=self.ext_cfg,
                )
                self.command_positions[:] = cmd_7
                reach_r = self._reach_radius_m

            # 读取最近一拍反馈用于重力模型与 M6 监督
            with self._fb_lock:
                fb_ok_prev = all(self._fb_valid[:6]) and (now_mono - self._fb_stamp_mono) <= float(self.ext_cfg["feedback_max_age_s"])
                q_for_grav = list(self._fb_q[:6]) if fb_ok_prev else list(cmd_7[:6])

            raw_grav = self._compute_gravity_6(q_for_grav) if self.control_mode == "mit" else [0.0] * 6
            self._tau_ff_last = compute_p3_gravity_tau(
                raw_grav_nm_6=raw_grav,
                q_for_fade_6=q_for_grav,
                prev_tau_6=self._tau_ff_last,
                ramp_elapsed_s=max(0.0, now_mono - self._start_mono),
                period_s=period,
                ext_cfg=self.ext_cfg,
            )
            kp_6, self._kd_last_6 = compute_p3_gains(
                base_kp_6=self._base_kp_6,
                base_kd_6=self._base_kd_6,
                prev_kd_6=self._kd_last_6,
                reach_r_m=reach_r,
                period_s=period,
                ext_cfg=self.ext_cfg,
            )

            try:
                fb_ok, t_max = self._send_frame(cmd_7, dq_ff_7, kp_6, self._kd_last_6, self._tau_ff_last)
                self._can_monitor.record_success()
                with self._fb_lock:
                    q_act_6 = list(self._fb_q[:6])
                    dq_act_6 = list(self._fb_dq[:6])
                    tau_act_6 = list(self._fb_tau[:6])
                self._supervisor.update(cmd_7[:6], q_act_6, tau_act_6, t_max, fb_ok)

                if self._recording:
                    with self._rec_lock:
                        if self._recording:
                            if (now_mono - self._rec_start_mono) <= float(self.ext_cfg["record_max_s"]):
                                self._rec_rows.append((now_mono, list(cmd_7), q_act_6, dq_act_6, tau_act_6))
                            else:
                                self._recording = False
            except Exception as err:
                self.last_error = err
                fatal, should_log = self._can_monitor.record_failure(now_mono)
                if should_log:
                    print(f"\n[DM-Bus] 瞬时通信异常 ({self._can_monitor.consecutive_failures}/{self._can_monitor.fail_threshold_ticks}): {err}")
                if fatal:
                    ret_zero = self.ext_cfg["can_fault_action"] == "return_to_zero"
                    print(f"\n[DM-Bus] 连续通信故障超 {self.ext_cfg['can_fault_window_s']*1000:.0f}ms，执行 {'安全回零' if ret_zero else '紧急失能'}！")
                    self.request_stop(reason=f"bus_fault:{err}", return_to_zero=ret_zero, emergency=not ret_zero, wait=False)
                    return

            next_tick += period
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()

    # ── 外部控制接口（与 RebotAdapter 100% 对齐）─────────────────────────

    def set_joint_angles(self, angles_deg: Sequence[float]) -> None:
        if self.shutdown_started:
            return
        if len(angles_deg) != self.motor_count:
            raise ValueError(f"必须提供 {self.motor_count} 个关节角度(deg)")
        pos_rad = [math.radians(float(a)) for a in angles_deg]
        if not all(math.isfinite(x) for x in pos_rad):
            raise ValueError("angles_deg 含非有限值")
        with self.target_lock:
            self.target_positions[:] = pos_rad
            self._vel_ff_active = False
            self._vel_ff_raw[:] = [0.0] * self.motor_count

    def set_joint_mit_targets(self, angles_deg: Sequence[float], velocities_deg_s: Sequence[float]) -> None:
        if self.shutdown_started:
            return
        if len(angles_deg) != self.motor_count or len(velocities_deg_s) != self.motor_count:
            raise ValueError(f"必须提供 {self.motor_count} 路角度与速度前馈")
        pos_rad = [math.radians(float(a)) for a in angles_deg]
        vel_rad = [math.radians(float(v)) for v in velocities_deg_s]
        if not (all(math.isfinite(x) for x in pos_rad) and all(math.isfinite(x) for x in vel_rad)):
            raise ValueError("MIT 目标含非有限值")
        vel_rad[6] = 0.0  # J7 夹爪速度前馈恒为 0
        with self.target_lock:
            self.target_positions[:] = pos_rad
            self._vel_ff_raw[:] = vel_rad
            self._vel_ff_active = True
            self._vel_ff_stamp_mono = time.monotonic()

    def set_max_speeds(self, speeds_deg_s: Sequence[float]) -> None:
        if len(speeds_deg_s) != self.motor_count:
            raise ValueError(f"必须提供 {self.motor_count} 个速度上限(deg/s)")
        vals = [math.radians(float(v)) for v in speeds_deg_s]
        if not all(math.isfinite(v) and v > 0.0 for v in vals):
            raise ValueError("速度上限必须全为有限正数")
        with self.target_lock:
            self.max_speeds_rad_s[:] = vals

    def get_target_angles(self) -> list[float]:
        with self.target_lock:
            return [math.degrees(x) for x in self.target_positions]

    def get_command_angles(self) -> list[float]:
        with self.target_lock:
            return [math.degrees(x) for x in self.command_positions]

    def _read_positions_rad(self) -> list[float]:
        """主动请求一次反馈并读取全部 7 路电机当前角度（rad）。"""
        positions: list[float] = []
        with self._io_lock_guard():
            for _ in range(5):
                for motor in self.motors:
                    try:
                        motor.request_feedback()
                    except Exception:
                        pass
                if self.controller is not None:
                    try:
                        self.controller.poll_feedback_once()
                    except Exception:
                        pass
                if all(m.get_state() is not None for m in self.motors):
                    break
                time.sleep(0.01)

            for mcfg, motor in zip(self.config.motors, self.motors):
                st = motor.get_state()
                if st is None or not math.isfinite(float(st.pos)):
                    raise RuntimeError(f"达妙电机 {mcfg.name} (id={mcfg.motor_id:#04x}) 无有效反馈")
                positions.append(float(st.pos))
        return positions

    def read_joint_angles(self) -> list[float]:
        return [math.degrees(x) for x in self._read_positions_rad()]

    def get_feedback_snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._fb_lock:
            age_s = max(0.0, now - self._fb_stamp_mono) if self._fb_seq > 0 else 999.0
            with self.target_lock:
                cmd = list(self.command_positions)
            return {
                "q": list(self._fb_q),
                "dq": list(self._fb_dq),
                "tau": list(self._fb_tau),
                "cmd": cmd,
                "valid": list(self._fb_valid),
                "t_mos": [
                    max(a, b) if (math.isfinite(a) and math.isfinite(b)) else (a if math.isfinite(a) else b)
                    for a, b in zip(self._fb_tmos, self._fb_trotor)
                ],
                "t_rotor": list(self._fb_trotor),
                "tau_ff": list(self._tau_ff_last),
                "kd": list(self._kd_last_6),
                "age_s": age_s,
                "age_ms": age_s * 1000.0,
                "seq": self._fb_seq,
            }

    def get_loop_stats(self) -> dict[str, Any]:
        hist = list(self._period_hist_ms)
        if hist:
            hist_sorted = sorted(hist)
            p50 = hist_sorted[min(len(hist_sorted) - 1, int(0.50 * len(hist_sorted)))]
            p99 = hist_sorted[min(len(hist_sorted) - 1, int(0.99 * len(hist_sorted)))]
            pmax = hist_sorted[-1]
            avg = sum(hist) / len(hist)
        else:
            p50 = 1000.0 / self.control_hz
            p99 = p50
            pmax = p50
            avg = p50

        def _p99(buf: deque[float]) -> Optional[float]:
            vals = sorted(buf)
            if not vals:
                return None
            return vals[min(len(vals) - 1, int(0.99 * len(vals)))]

        with self._fb_lock:
            fb_change_ratio = (
                (self._fb_change_hits / self._fb_change_total)
                if self._fb_change_total > 0
                else None
            )
            tau_ff = list(self._tau_ff_last)
            cur_kd = list(self._kd_last_6)

        return {
            "control_mode": self.control_mode,
            "control_hz": self.control_hz,
            "period_avg_ms": avg,
            "period_p50_ms": p50,
            "period_p99_ms": p99,
            "period_max_ms": pmax,
            "send_p99_ms": _p99(self._send_hist_ms),
            "sample_p99_ms": _p99(self._sample_hist_ms),
            "can_fail_total": self._can_monitor.total_failures,
            "can_fail_consecutive": self._can_monitor.consecutive_failures,
            "feedback_errors": self._fb_errors_total,
            "feedback_change_ratio": fb_change_ratio,
            "tau_ff_nm": tau_ff,
            "cur_kd": cur_kd,
        }

    @property
    def is_stopped(self) -> bool:
        """与历史标定/工具脚本兼容：控制器已停机或正在收尾关闭。"""
        return self.shutdown_done_event.is_set() or (self.shutdown_started and not self.started)

    @property
    def last_temperatures(self) -> list[Optional[float]]:
        """返回最近一拍 7 路电机温度（max(t_mos, t_rotor)，无有效读数时为 None）。"""
        snap = self.get_feedback_snapshot()
        return [float(t) if math.isfinite(t) else None for t in snap["t_mos"]]

    def reset_feedback_change_stats(self) -> None:
        """重置反馈变化率与周期分布统计（供 mit_feedback_probe.py 分模式对比使用）。"""
        with self._fb_lock:
            self._fb_prev_sig = None
            self._fb_change_total = 0
            self._fb_change_hits = 0
        self._period_hist_ms.clear()
        self._send_hist_ms.clear()
        self._sample_hist_ms.clear()

    def set_vel_ff_enable(self, enabled: bool) -> None:
        """运行时切换 P1 速度前馈开关（供 joint_sysid.py P0 vs P1 对比使用）。"""
        with self.target_lock:
            self.ext_cfg["vel_ff_enable"] = bool(enabled)
            if not enabled:
                self._vel_ff_active = False
                self._vel_ff_raw[:] = [0.0] * self.motor_count
                self._vel_ff_filt[:] = [0.0] * self.motor_count

    def set_gravity_torque_fn(self, fn: Optional[Callable[[Sequence[float]], Sequence[float]]]) -> None:
        self._gravity_fn = fn

    def set_reach_radius(self, reach_r_m: float) -> None:
        if math.isfinite(float(reach_r_m)):
            with self.target_lock:
                self._reach_radius_m = max(0.0, float(reach_r_m))

    def set_feedback_poll_once(self, enabled: bool) -> None:
        """运行时切换 200Hz 循环中是否顺带调用 poll_feedback_once（供 mit_feedback_probe.py 使用）。"""
        with self.target_lock:
            self.ext_cfg["feedback_poll_once"] = bool(enabled)

    def set_gravity_ff_runtime(self, *, enabled: bool, scale: Optional[Sequence[float]] = None) -> None:
        """运行时切换静态重力前馈使能与分轴比例（供 p3_gravity_verify.py 分级验证使用）。"""
        with self.target_lock:
            self.ext_cfg["gravity_ff_enable"] = bool(enabled)
            if scale is not None:
                vec = [float(x) for x in scale]
                if len(vec) != 6 or not all(math.isfinite(x) and 0.0 <= x <= 1.0 for x in vec):
                    raise ValueError("gravity_ff_scale 必须为 6 维 [0.0, 1.0] 向量")
                vec[0] = 0.0
                self.ext_cfg["gravity_ff_scale"] = vec
            if not enabled:
                self._tau_ff_last = [0.0] * 6

    def get_safety_status(self) -> dict[str, Any]:
        return self._supervisor.status()

    def clear_safety_fault(self) -> bool:
        return self._supervisor.clear()

    def start_recording(self) -> None:
        with self._rec_lock:
            self._rec_rows.clear()
            self._rec_start_mono = time.monotonic()
            self._recording = True

    def stop_recording(self) -> list[tuple[float, list[float], list[float], list[float], list[float]]]:
        with self._rec_lock:
            self._recording = False
            rows = list(self._rec_rows)
            self._rec_rows.clear()
            return rows

    # ── 温度监控线程（三级过温保护）──────────────────────────────────────

    def _temperature_loop(self) -> None:
        period = 1.0 / self.telemetry_hz
        t_cfg = self.config.temperatures
        while not self.worker_stop_event.wait(period):
            snap = self.get_feedback_snapshot()
            temps = snap["t_mos"]
            valid = [(i, t) for i, t in enumerate(temps) if math.isfinite(t)]
            if not valid:
                continue
            hot_i, hot_t = max(valid, key=lambda x: x[1])
            hot_name = self.config.motors[hot_i].name

            if hot_t >= t_cfg.disconnect_c:
                print(f"\n[DM-Temp] 紧急过温 {hot_name}={hot_t:.1f}°C >= {t_cfg.disconnect_c:.1f}°C，立即失能！")
                self.request_stop(reason=f"temp_disconnect:{hot_name}", return_to_zero=False, emergency=True, wait=False)
                return
            if hot_t >= t_cfg.return_zero_c:
                print(f"\n[DM-Temp] 高温回零 {hot_name}={hot_t:.1f}°C >= {t_cfg.return_zero_c:.1f}°C，缓慢回零后失能！")
                self.request_stop(reason=f"temp_return_zero:{hot_name}", return_to_zero=True, wait=False)
                return

            for idx, val in valid:
                if val >= t_cfg.alarm_c and not self._temp_alarm_active[idx]:
                    self._temp_alarm_active[idx] = True
                    print(f"\n[DM-Temp] 温度告警 {self.config.motors[idx].name}={val:.1f}°C")
                elif val < t_cfg.alarm_c - self._TEMP_ALARM_HYSTERESIS_C:
                    self._temp_alarm_active[idx] = False

    # ── 安全停机链（绝不半空失能）────────────────────────────────────────

    def _safe_return_to_zero(self) -> None:
        """以低速平滑 minimum-jerk 轨迹将 6 个手臂关节归零，重力前馈随轨迹平滑淡出至 0 Nm。"""
        if self.controller is None or not self.motors:
            return
        with self.target_lock:
            q_start = list(self.command_positions)
        # 尝试用新鲜实测角作为起点（更贴近真实姿态）
        snap = self.get_feedback_snapshot()
        if all(snap["valid"][:6]) and snap["age_s"] <= 0.20:
            q_start[:6] = snap["q"][:6]

        q_goal = [0.0] * 6 + [q_start[6]]  # 夹爪保持原位，不强行夹闭
        max_err = max(abs(q_goal[i] - q_start[i]) for i in range(6))
        speed_rad_s = math.radians(self.config.return_zero_speed_deg_s)
        tol_rad = math.radians(self.config.return_zero_tol_deg)

        if max_err >= tol_rad:
            duration = min(self.config.return_zero_timeout_s, max(1.0, (max_err / max(0.05, speed_rad_s)) * 1.8))
            dt = 1.0 / min(100.0, self.control_hz)
            steps = max(2, int(round(duration / dt)))
            print(f"[DM-SafeHome] 平滑回零轨迹: max_err={math.degrees(max_err):.1f}°, {duration:.1f}s ({steps} 步)")
            t_deadline = time.monotonic() + self.config.return_zero_timeout_s
            for k in range(1, steps + 1):
                if self.abort_return_event.is_set() or time.monotonic() > t_deadline:
                    break
                s = k / steps
                mj = 10.0 * (s ** 3) - 15.0 * (s ** 4) + 6.0 * (s ** 5)
                q_step = [q_start[i] + (q_goal[i] - q_start[i]) * mj for i in range(7)]
                raw_g = self._compute_gravity_6(q_step[:6]) if self.control_mode == "mit" else [0.0] * 6
                self._tau_ff_last = compute_p3_gravity_tau(
                    raw_grav_nm_6=raw_g,
                    q_for_fade_6=q_step[:6],
                    prev_tau_6=self._tau_ff_last,
                    ramp_elapsed_s=1.0,
                    period_s=dt,
                    ext_cfg=self.ext_cfg,
                )
                try:
                    self._send_frame(q_step, [0.0] * 7, self._base_kp_6, self._base_kd_6, self._tau_ff_last, lock_timeout=0.05)
                except Exception:
                    pass
                time.sleep(dt)

            # 在零位短暂稳态保持，让机械臂完全落座支架后再失能
            for _ in range(15):
                if self.abort_return_event.is_set():
                    break
                try:
                    self._send_frame(q_goal, [0.0] * 7, self._base_kp_6, self._base_kd_6, [0.0] * 6, lock_timeout=0.05)
                except Exception:
                    pass
                time.sleep(0.02)

    def _disable_and_close(self) -> None:
        if self.controller is None:
            return
        try:
            with self._io_lock_guard(self._IO_LOCK_SHUTDOWN_TIMEOUT_S):
                try:
                    self.controller.disable_all()
                except Exception:
                    pass
                time.sleep(0.15)
                try:
                    self.controller.shutdown()
                except Exception:
                    pass
                time.sleep(0.05)
                try:
                    self.controller.close()
                except Exception:
                    pass
        except TimeoutError:
            pass
        self.controller = None
        self.motors.clear()

    def request_stop(
        self,
        *,
        reason: str = "user_stop",
        return_to_zero: bool = True,
        emergency: bool = False,
        wait: bool = True,
    ) -> None:
        with self.shutdown_lock:
            if self.shutdown_started:
                if emergency:
                    self.abort_return_event.set()
                t = self.shutdown_thread
            else:
                self.shutdown_started = True
                self.shutdown_reason = reason
                if emergency:
                    self.abort_return_event.set()

                def _worker() -> None:
                    try:
                        self.worker_stop_event.set()
                        if self.control_thread is not None and self.control_thread.is_alive() and threading.current_thread() is not self.control_thread:
                            self.control_thread.join(timeout=1.5)
                        if self.telemetry_thread is not None and self.telemetry_thread.is_alive() and threading.current_thread() is not self.telemetry_thread:
                            self.telemetry_thread.join(timeout=1.0)
                        if return_to_zero and not emergency and not self.abort_return_event.is_set():
                            self._safe_return_to_zero()
                    finally:
                        self._disable_and_close()
                        self.started = False
                        self.shutdown_done_event.set()

                t = threading.Thread(target=_worker, name="rebot-dm-shutdown", daemon=True)
                self.shutdown_thread = t
                t.start()

        if wait:
            self.shutdown_done_event.wait(timeout=self.config.return_zero_timeout_s + 5.0)

    def stop(self, *, return_to_zero: bool = True, wait: bool = True) -> None:
        """安全停机入口（默认强制缓慢回零到位再失能）。"""
        self.request_stop(reason="stop()", return_to_zero=return_to_zero, wait=wait)
