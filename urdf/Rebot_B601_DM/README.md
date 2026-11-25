# ReBot Arm B601-DM 内置机械描述包

本目录内置于主项目中，提供完整的 Seeed reBot Arm B601-DM 运动学/动力学 URDF、多材质视觉网格、碰撞网格以及 Orbbec Gemini 305 3D 打印相机支架网格（`gemini305_mount_v3.stl`），无需依赖任何外部仓库或 ROS package URI。

```text
urdf/Rebot_B601_DM/
├── urdf/
│   └── ReBot_Arm_DM.urdf          # B601-DM 机械臂 + 官方刚性平行夹爪主模型
└── meshes/
    ├── visual/                    # URDF 多材质视觉分件 + gemini305_mount_v3.stl
    ├── collision/                 # URDF 碰撞整件
    ├── shared/                    # URDF 渲染与 MuJoCo 碰撞共用
    └── mujoco_collision/          # MuJoCo 夹指分段碰撞网格
```

- `urdf/ReBot_Arm_DM.urdf` 全部使用 `../meshes/...` 相对路径，随项目拷贝到任意路径均可直接被 Pinocchio 与 MuJoCo 4.0+ 加载。
- 详细参数与使用方法见项目根目录 [`README.md`](../../README.md)。
