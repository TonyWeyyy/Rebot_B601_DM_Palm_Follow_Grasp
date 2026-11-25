#!/usr/bin/env python3
"""B601-DM + Gemini 305 + 相机壳3.0 STL 的 MuJoCo 4.0+ 数字孪生编译与 IK 自检脚本。"""

from __future__ import annotations

import os
import sys
import unittest

import mujoco
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TWIN_DIR = os.path.join(ROOT, "mujoco_twin")
for p in (ROOT, TWIN_DIR):
    if p not in sys.path:
        sys.path.insert(0, p)

import scene_builder  # noqa: E402


class MuJoCoDMTwinTests(unittest.TestCase):
    def test_dm_spec_compiles_with_bracket_stl_and_gemini305(self) -> None:
        spec = scene_builder.build_b601_dm_spec()
        model = spec.compile()
        data = mujoco.MjData(model)

        # 验证 DM 6 关节 + 平行夹爪 2 关节齐全
        for jn in ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "finger_left", "finger_right"):
            self.assertGreaterEqual(model.joint(jn).id, 0)

        # 验证 相机壳3.0 STL 与 Gemini 305 机身/光轴 site 成功挂载
        self.assertGreaterEqual(model.geom("gemini305_mount_v3").id, 0)
        self.assertGreaterEqual(model.geom("gemini305_housing").id, 0)
        self.assertGreaterEqual(model.site("gemini305_cam").id, 0)
        for mname in ("palm_target", "cmd_tcp_marker", "workspace_ring", "ik_target"):
            self.assertGreaterEqual(int(model.body(mname).mocapid[0]), 0)

        # 加载 home 关键帧并验证 TCP 位置与对称夹爪
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        tcp = data.body("link6").xpos
        self.assertAlmostEqual(float(tcp[0]), 0.137, delta=0.01)
        self.assertAlmostEqual(float(tcp[2]), 0.318, delta=0.01)
        self.assertAlmostEqual(float(data.joint("finger_left").qpos[0]), 0.0285, delta=1e-4)
        self.assertAlmostEqual(float(data.joint("finger_right").qpos[0]), -0.0285, delta=1e-4)

        # 验证 Gemini 305 相机光轴 (+Z) 在 HOME 姿态下前视朝向世界 +X（0° 倾角）
        cam_z_world = data.site("gemini305_cam").xmat.reshape(3, 3)[:, 2]
        self.assertGreater(float(cam_z_world[0]), 0.99)
        self.assertLess(abs(float(cam_z_world[2])), 0.05)


if __name__ == "__main__":
    unittest.main(verbosity=2)
