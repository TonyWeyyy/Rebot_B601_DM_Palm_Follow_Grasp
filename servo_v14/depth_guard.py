"""
4 层深度保护链独立模块（servo_v14/depth_guard.py，适配 Orbbec Gemini 305）
=======================================================================
严格实现 4 层深度保护链：
    保护层 1：7×7 → 13×13 → 19×19 前景优先中值窗口回退（支持 Gemini 305 0.07m 起近距）
    保护层 2：帧间跳变拒收 |Δz| > max_jump_m（连续拒收超 anti_lock_frames=15 帧防锁死重锚）
    保护层 3：长丢失 / 启动后 5 帧一致恢复校验（reacq_buf）+ 背景深度过滤
    保护层 4：单帧变化率硬限幅 ±max_rate_m + 速度自适应 One Euro 滤波
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from palm_detection import OneEuroFilter, get_median_depth
from .config import DepthGuardConfig


@dataclass(frozen=True)
class DepthGuardResult:
    valid: bool
    frozen: bool
    z_m: float
    raw_z_m: float
    reject_reason: str = ""


class DepthProtectionChain:
    """严格实现 4 层深度保护链的有状态滤波器。"""

    def __init__(self, cfg: DepthGuardConfig, target_distance_m: float = 0.35) -> None:
        self.cfg = cfg
        self.target_distance_m = float(target_distance_m)
        self._oe = OneEuroFilter(min_cutoff=cfg.oe_min_cutoff, beta=cfg.oe_beta)
        self._z_ref: float = 0.0
        self._z_filt: float = 0.0
        self._jump_reject_count: int = 0
        self._reacq_buf: List[float] = []
        self._freeze_start_s: Optional[float] = None

    def reset(self) -> None:
        self._oe.reset()
        self._z_ref = 0.0
        self._z_filt = 0.0
        self._jump_reject_count = 0
        self._reacq_buf.clear()
        self._freeze_start_s = None

    def extract_raw_depth(
        self,
        depth_img: Optional[np.ndarray],
        u: int,
        v: int,
        depth_scale: float,
    ) -> float:
        if depth_img is None:
            return 0.0
        z = float(get_median_depth(
            depth_img,
            float(depth_scale),
            int(u),
            int(v),
            ref_depth=self._z_ref,
            max_jump=self.cfg.max_jump_m,
            min_depth_m=self.cfg.min_valid_m,
            max_depth_m=self.cfg.max_valid_m,
        ))
        return z if (self.cfg.min_valid_m <= z <= self.cfg.max_valid_m) else 0.0

    def update(self, raw_z_m: float, timestamp_s: float) -> DepthGuardResult:
        z_raw = float(raw_z_m) if math.isfinite(raw_z_m) else 0.0
        in_range = self.cfg.min_valid_m <= z_raw <= self.cfg.max_valid_m

        if in_range:
            if self._z_ref > 0.0:
                delta = abs(z_raw - self._z_ref)
                if delta > self.cfg.max_jump_m:
                    self._jump_reject_count += 1
                    if self._jump_reject_count < self.cfg.anti_lock_frames:
                        return self._handle_freeze(z_raw, timestamp_s, reason="jump_reject")
                    self._jump_reject_count = 0
                    self._oe.reset()
                    self._z_ref = z_raw
                    self._z_filt = self._oe(z_raw, timestamp_s)
                    self._freeze_start_s = None
                    return DepthGuardResult(
                        valid=True,
                        frozen=False,
                        z_m=self._z_filt,
                        raw_z_m=z_raw,
                        reject_reason="anti_lock_reanchor",
                    )

                self._jump_reject_count = 0
                self._freeze_start_s = None
                z_clamped = float(np.clip(
                    z_raw,
                    self._z_ref - self.cfg.max_rate_m,
                    self._z_ref + self.cfg.max_rate_m,
                ))
                self._z_ref = z_clamped
                self._z_filt = self._oe(z_clamped, timestamp_s)
                return DepthGuardResult(
                    valid=True,
                    frozen=False,
                    z_m=self._z_filt,
                    raw_z_m=z_raw,
                )

            bg_cap = self.target_distance_m + self.cfg.reacq_bg_margin_m
            if z_raw > bg_cap:
                self._reacq_buf.clear()
                return DepthGuardResult(
                    valid=False,
                    frozen=False,
                    z_m=0.0,
                    raw_z_m=z_raw,
                    reject_reason="bg_depth_reject",
                )

            self._reacq_buf.append(z_raw)
            if len(self._reacq_buf) > self.cfg.reacq_frames:
                self._reacq_buf.pop(0)
            if len(self._reacq_buf) < self.cfg.reacq_frames:
                return DepthGuardResult(
                    valid=False,
                    frozen=False,
                    z_m=0.0,
                    raw_z_m=z_raw,
                    reject_reason="reacq_collecting",
                )
            if (max(self._reacq_buf) - min(self._reacq_buf)) > self.cfg.reacq_tol_m:
                self._reacq_buf.pop(0)
                return DepthGuardResult(
                    valid=False,
                    frozen=False,
                    z_m=0.0,
                    raw_z_m=z_raw,
                    reject_reason="reacq_inconsistent",
                )

            z_init = float(np.median(self._reacq_buf))
            self._reacq_buf.clear()
            self._jump_reject_count = 0
            self._freeze_start_s = None
            self._oe.reset()
            self._z_ref = z_init
            self._z_filt = self._oe(z_init, timestamp_s)
            return DepthGuardResult(
                valid=True,
                frozen=False,
                z_m=self._z_filt,
                raw_z_m=z_raw,
            )

        return self._handle_freeze(z_raw, timestamp_s, reason="invalid_depth")

    def _handle_freeze(self, z_raw: float, timestamp_s: float, reason: str) -> DepthGuardResult:
        if self._z_filt <= 0.0:
            return DepthGuardResult(
                valid=False,
                frozen=False,
                z_m=0.0,
                raw_z_m=z_raw,
                reject_reason=reason,
            )
        if self._freeze_start_s is None:
            self._freeze_start_s = timestamp_s
        if (timestamp_s - self._freeze_start_s) <= self.cfg.z_freeze_max_s:
            return DepthGuardResult(
                valid=True,
                frozen=True,
                z_m=self._z_filt,
                raw_z_m=z_raw,
                reject_reason=reason,
            )
        self.reset()
        return DepthGuardResult(
            valid=False,
            frozen=False,
            z_m=0.0,
            raw_z_m=z_raw,
            reject_reason="freeze_timeout",
        )
