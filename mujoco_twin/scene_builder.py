"""
Seeed reBot Arm B601-DM + Orbbec Gemini 305 MuJoCo 4.0+ 数字孪生场景构建器（MjSpec）
==================================================================================
以官方 ``ReBot_Arm_DM.urdf``（含多材质精模 STL 与官方刚性平行夹爪 ``finger_left/right``）
为基础，通过 ``mujoco.MjSpec`` 在内存中装配完整的实时数字孪生 3D 场景：

  - 保留全部多材质视觉网格（``group=1``），将原始碰撞壳（``group=0``）移至 ``group=3``；
  - 在 ``link6`` 上按 ``config/hand_eye_calibration.json`` 的真实手眼外参 ``T_tcp_camera``
    精确挂载：
      1. 3D 打印新支架 ``gemini305_mount_v3.stl``（由根目录 ``相机壳3.0.3MF`` 无损转换）；
      2. Orbbec Gemini 305 超紧凑机身（42×42×23 mm）与双目玻璃面板；
      3. 0.35m 光轴期望跟随距离瞄准线与停靠目标点。
  - 挂载 4 个零物理开销 mocap 叠加体：
      ``palm_target``、``cmd_tcp_marker``、``workspace_ring`` (r=0.42m)、``ik_target``。
  - 注册 DM 安全 ``home`` 关键帧（对应 ``config/home_pose.json`` + ``config/joint_offset.json``）。
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import re
from typing import Sequence

import mujoco
import numpy as np

PROJECT_ROOT = pathlib.Path(os.path.abspath(__file__)).parent.parent
LOCAL_DM_URDF = str(PROJECT_ROOT / "urdf" / "Rebot_B601_DM" / "urdf" / "ReBot_Arm_DM.urdf")
BRACKET_STL_PATH = str(
    PROJECT_ROOT / "urdf" / "Rebot_B601_DM" / "meshes" / "visual" / "gemini305_mount_v3.stl"
)
URDF_PATH = LOCAL_DM_URDF

DEFAULT_HOME_Q = np.array([0.0, -0.50, -0.50, 0.0, 0.0, 0.0], dtype=np.float64)
ARM_JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
GRIPPER_JOINT_NAMES = ("finger_left", "finger_right")
GRIPPER_MAX_STROKE_M = 0.0285

# 0° 倾角前视理论外参（camera +Z 沿 link6 +Z 朝前，+X 朝右，+Y 朝下，安装于手腕上方）
DEFAULT_T_TCP_CAM = np.array(
    [
        [0.0, -1.0, 0.0, 0.055],
        [1.0, 0.0, 0.0, 0.000],
        [0.0, 0.0, 1.0, 0.075],
        [0.0, 0.0, 0.0, 1.000],
    ],
    dtype=np.float64,
)


def resolve_urdf_path(custom_path: str = "") -> str:
    if custom_path:
        ap = os.path.abspath(os.path.expanduser(custom_path))
        if os.path.isfile(ap):
            return ap
    if not os.path.isfile(LOCAL_DM_URDF):
        raise FileNotFoundError(f"未找到内置 B601-DM URDF: {LOCAL_DM_URDF}")
    return LOCAL_DM_URDF


def load_joint_offset_rad() -> np.ndarray:
    path = PROJECT_ROOT / "config" / "joint_offset.json"
    if path.is_file():
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            off = np.asarray(data["joint_offset_rad"], dtype=np.float64)
            if off.shape == (6,) and np.all(np.isfinite(off)):
                return off
        except Exception:
            pass
    return np.zeros(6, dtype=np.float64)


def load_home_q_rad() -> np.ndarray:
    path = PROJECT_ROOT / "config" / "home_pose.json"
    if path.is_file():
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            q = np.asarray(data["home_joint_pos"], dtype=np.float64)
            if q.shape == (6,) and np.all(np.isfinite(q)):
                return q
        except Exception:
            pass
    return DEFAULT_HOME_Q.copy()


def load_hand_eye_matrix() -> np.ndarray:
    path = PROJECT_ROOT / "config" / "hand_eye_calibration.json"
    if path.is_file():
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            T = np.asarray(data["T_tcp_camera"], dtype=np.float64)
            if T.shape == (4, 4) and np.all(np.isfinite(T)):
                return T
        except Exception:
            pass
    return DEFAULT_T_TCP_CAM.copy()


def load_gripper_deg_bounds() -> tuple[float, float]:
    """从 ``config/grasp.yaml`` 读取 J7 夹爪标定闭合/全开角度（无 yaml 依赖时用正则兜底）。"""
    path = PROJECT_ROOT / "config" / "grasp.yaml"
    if path.is_file():
        txt = path.read_text(encoding="utf-8")
        m_c = re.search(r"closed_deg\s*:\s*([-+]?\d+(?:\.\d+)?)", txt)
        m_o = re.search(r"open_deg\s*:\s*([-+]?\d+(?:\.\d+)?)", txt)
        if m_c and m_o:
            return float(m_c.group(1)), float(m_o.group(1))
    return 0.0, 180.0


def _rotmat_to_quat_wxyz(R: np.ndarray) -> list[float]:
    quat = np.zeros(4, dtype=np.float64)
    mujoco.mju_mat2Quat(quat, np.asarray(R, dtype=np.float64).reshape(9))
    norm = float(np.linalg.norm(quat))
    if norm < 1e-12:
        return [1.0, 0.0, 0.0, 0.0]
    return (quat / norm).tolist()


def gripper_deg_to_prismatic_m(
    gripper_deg: float,
    closed_deg: float = 0.0,
    open_deg: float = 180.0,
    max_stroke_m: float = GRIPPER_MAX_STROKE_M,
) -> float:
    """将 DM J7 夹爪角度（deg）线性映射到单指滑移量（m，0=闭合，0.0285=全开）。"""
    span = float(open_deg) - float(closed_deg)
    if abs(span) < 1e-3:
        return 0.0
    ratio = float(np.clip((float(gripper_deg) - float(closed_deg)) / span, 0.0, 1.0))
    return ratio * float(max_stroke_m)


def _prepare_visual_urdf_string(urdf_path: str) -> str:
    urdf_dir = os.path.dirname(os.path.abspath(urdf_path))
    xml = pathlib.Path(urdf_path).read_text(encoding="utf-8")

    def _abs_mesh(match: re.Match[str]) -> str:
        rel = match.group(1)
        absp = os.path.abspath(os.path.join(urdf_dir, rel))
        return f'filename="{absp}"'

    xml = re.sub(r'filename="([^"]+)"', _abs_mesh, xml)
    compiler_block = (
        '<mujoco><compiler discardvisual="false" strippath="false" '
        'fusestatic="false"/></mujoco>'
    )
    xml_out, count = re.subn(r"(<robot\b[^>]*>)", r"\1" + compiler_block, xml, count=1)
    if count == 0:
        raise ValueError(f"无法在 URDF 中找到 <robot> 根标签: {urdf_path}")
    return xml_out


def build_b601_dm_spec(
    *,
    urdf_path: str | None = None,
    workspace_radius_m: float = 0.42,
    target_distance_m: float = 0.35,
    show_camera_ray: bool = True,
    home_q_sdk: Sequence[float] | None = None,
    joint_offset_rad: Sequence[float] | None = None,
    T_tcp_camera: np.ndarray | None = None,
) -> mujoco.MjSpec:
    """构建完整的 reBot Arm B601-DM + Gemini 305 数字孪生 ``MjSpec``。"""
    if not urdf_path:
        urdf_path = resolve_urdf_path()
    xml_str = _prepare_visual_urdf_string(urdf_path)
    spec = mujoco.MjSpec.from_string(xml_str)
    spec.modelname = "rebot_b601_dm_digital_twin"

    spec.option.timestep = 0.01
    spec.option.gravity = [0.0, 0.0, 0.0]
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

    spec.visual.headlight.diffuse = [0.58, 0.60, 0.62]
    spec.visual.headlight.ambient = [0.28, 0.30, 0.32]
    spec.visual.headlight.specular = [0.15, 0.15, 0.15]
    spec.visual.rgba.haze = [0.12, 0.16, 0.22, 1.0]
    spec.visual.global_.azimuth = 135.0
    spec.visual.global_.elevation = -22.0
    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 720
    spec.stat.center = [0.16, 0.0, 0.28]
    spec.stat.extent = 0.75

    spec.add_texture(
        name="studio_skybox",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.18, 0.24, 0.32],
        rgb2=[0.04, 0.06, 0.09],
        width=512,
        height=3072,
    )
    spec.add_texture(
        name="workbench_checker",
        type=mujoco.mjtTexture.mjTEXTURE_2D,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER,
        mark=mujoco.mjtMark.mjMARK_EDGE,
        rgb1=[0.16, 0.19, 0.23],
        rgb2=[0.12, 0.15, 0.19],
        markrgb=[0.28, 0.33, 0.39],
        width=300,
        height=300,
    )
    floor_mat = spec.add_material(
        name="workbench_mat",
        texrepeat=[8.0, 8.0],
        texuniform=True,
        reflectance=0.14,
        roughness=0.55,
    )
    floor_mat.textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = "workbench_checker"

    # 若转换后的相机壳 3.0 STL 存在，注册为 mesh 资产
    has_bracket_mesh = os.path.isfile(BRACKET_STL_PATH)
    if has_bracket_mesh:
        spec.add_mesh(name="gemini305_mount_v3_mesh", file=BRACKET_STL_PATH)

    # 分离视觉精模（group=1）与碰撞壳（移至 group=3 默认隐藏）
    for geom in spec.geoms:
        if geom.group == 0:
            geom.group = 3
        geom.contype = 0
        geom.conaffinity = 0

    world = spec.worldbody
    world.add_light(
        name="key_light",
        pos=[0.8, 0.9, 1.8],
        dir=[-0.4, -0.45, -1.0],
        diffuse=[0.85, 0.85, 0.82],
        specular=[0.35, 0.35, 0.35],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
    )
    world.add_light(
        name="rim_light",
        pos=[-0.9, -0.8, 1.3],
        dir=[0.5, 0.45, -0.75],
        diffuse=[0.35, 0.42, 0.52],
        specular=[0.15, 0.18, 0.22],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
    )
    world.add_geom(
        name="workbench_floor",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        pos=[0.0, 0.0, 0.0],
        size=[0.0, 0.0, 0.05],
        material="workbench_mat",
        group=1,
        contype=0,
        conaffinity=0,
    )

    # 在 link6 上按手眼外参 T_tcp_camera 精确安装 Orbbec Gemini 305 + 相机壳 3.0
    T_cam = load_hand_eye_matrix() if T_tcp_camera is None else np.asarray(T_tcp_camera, dtype=np.float64)
    cam_pos = T_cam[:3, 3].tolist()
    cam_quat = _rotmat_to_quat_wxyz(T_cam[:3, :3])
    link6_body = spec.body("link6")
    if link6_body is not None:
        link6_body.add_site(
            name="tcp_site",
            pos=[0.0, 0.0, 0.0],
            size=[0.008, 0.008, 0.008],
            rgba=[0.95, 0.85, 0.15, 0.9],
            group=2,
        )
        cam_body = link6_body.add_body(
            name="gemini305_camera_body",
            pos=cam_pos,
            quat=cam_quat,
        )
        # 相机光学中心 site（+Z 为光轴）
        cam_body.add_site(
            name="gemini305_cam",
            pos=[0.0, 0.0, 0.0],
            size=[0.006, 0.006, 0.006],
            rgba=[0.1, 0.85, 1.0, 0.9],
            group=2,
        )
        # 3D 打印外壳（相机壳3.0.3MF：内腔 42×42×26mm，Y 负侧为底部安装槽；相机系 +Y 朝下，
        # 因此绕 Z 轴转 180° 使安装槽朝 +Y 下方贴合夹爪，Z 偏移 -0.025m 包裹机身）
        if has_bracket_mesh:
            cam_body.add_geom(
                name="gemini305_mount_v3",
                type=mujoco.mjtGeom.mjGEOM_MESH,
                meshname="gemini305_mount_v3_mesh",
                pos=[0.0, 0.0, -0.025],
                quat=[0.0, 0.0, 0.0, 1.0],
                rgba=[0.18, 0.20, 0.22, 1.0],
                group=1,
                contype=0,
                conaffinity=0,
            )
        # Orbbec Gemini 305 机身（42 × 42 × 23 mm -> 半尺寸 0.021 × 0.021 × 0.0115 m）
        cam_body.add_geom(
            name="gemini305_housing",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[0.0, 0.0, -0.0115],
            size=[0.021, 0.021, 0.0115],
            rgba=[0.09, 0.11, 0.13, 1.0],
            group=1,
            contype=0,
            conaffinity=0,
        )
        # 前置深色双目玻璃面板
        cam_body.add_geom(
            name="gemini305_visor",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            pos=[0.0, 0.0, -0.0005],
            size=[0.019, 0.019, 0.0012],
            rgba=[0.02, 0.03, 0.04, 1.0],
            group=1,
            contype=0,
            conaffinity=0,
        )
        # 双目镜头（基线 18 mm -> ±9 mm）
        for side, lx in (("left", -0.009), ("right", 0.009)):
            cam_body.add_geom(
                name=f"gemini305_lens_{side}",
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[lx, 0.0, 0.0008],
                size=[0.0042, 0.0012, 0.0],
                rgba=[0.08, 0.25, 0.38, 1.0],
                group=1,
                contype=0,
                conaffinity=0,
            )
        # 0.35m 光轴期望跟随距离瞄准线与停靠目标点
        if show_camera_ray:
            ray_len = max(0.05, float(target_distance_m))
            cam_body.add_geom(
                name="gemini305_aim_ray",
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                pos=[0.0, 0.0, 0.5 * ray_len],
                size=[0.0018, 0.5 * ray_len, 0.0],
                rgba=[0.15, 0.85, 1.0, 0.30],
                group=2,
                contype=0,
                conaffinity=0,
            )
            cam_body.add_geom(
                name="gemini305_aim_tip",
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                pos=[0.0, 0.0, ray_len],
                size=[0.010, 0.0, 0.0],
                rgba=[0.15, 0.85, 1.0, 0.55],
                group=2,
                contype=0,
                conaffinity=0,
            )

    # Mocap 1: 实时 3D 手掌目标球
    palm_body = world.add_body(name="palm_target", pos=[0.0, 0.0, -10.0], mocap=True)
    palm_body.add_geom(
        name="palm_target_halo", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.038, 0.0, 0.0],
        rgba=[0.18, 0.96, 0.48, 0.45], group=2, contype=0, conaffinity=0,
    )
    palm_body.add_geom(
        name="palm_target_core", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.015, 0.0, 0.0],
        rgba=[0.30, 1.0, 0.55, 0.92], group=2, contype=0, conaffinity=0,
    )

    # Mocap 2: 指令 TCP 幽灵标记球
    cmd_body = world.add_body(name="cmd_tcp_marker", pos=[0.0, 0.0, -10.0], mocap=True)
    cmd_body.add_geom(
        name="cmd_tcp_sphere", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.015, 0.0, 0.0],
        rgba=[0.10, 0.78, 1.0, 0.65], group=2, contype=0, conaffinity=0,
    )

    # Mocap 3: r = 0.42m 水平工作空间软墙指示环
    ring_body = world.add_body(name="workspace_ring", pos=[0.0, 0.0, 0.004], mocap=True)
    n_seg = 36
    r = float(workspace_radius_m)
    for k in range(n_seg):
        a0 = 2.0 * math.pi * k / n_seg
        a1 = 2.0 * math.pi * (k + 1) / n_seg
        ring_body.add_geom(
            name=f"ws_ring_seg_{k:02d}",
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[r * math.cos(a0), r * math.sin(a0), 0.0, r * math.cos(a1), r * math.sin(a1), 0.0],
            size=[0.0028, 0.0, 0.0],
            rgba=[1.0, 0.68, 0.16, 0.55],
            group=2,
            contype=0,
            conaffinity=0,
        )

    # Mocap 4: v15 3D 拖拽 IK 目标球
    ik_body = world.add_body(name="ik_target", pos=[0.0, 0.0, -10.0], mocap=True)
    ik_body.add_geom(
        name="ik_target_shell", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.028, 0.0, 0.0],
        rgba=[1.0, 0.52, 0.08, 0.65], group=1, contype=0, conaffinity=0,
    )
    ik_body.add_geom(
        name="ik_target_core", type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.011, 0.0, 0.0],
        rgba=[1.0, 0.86, 0.18, 0.96], group=1, contype=0, conaffinity=0,
    )
    for axis_name, end_pt, rgba in (
        ("ik_axis_x", [0.055, 0.0, 0.0], [0.95, 0.22, 0.22, 0.9]),
        ("ik_axis_y", [0.0, 0.055, 0.0], [0.22, 0.92, 0.32, 0.9]),
        ("ik_axis_z", [0.0, 0.0, 0.055], [0.22, 0.58, 1.00, 0.9]),
    ):
        ik_body.add_geom(
            name=axis_name, type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[0.0, 0.0, 0.0] + end_pt, size=[0.0024, 0.0, 0.0],
            rgba=rgba, group=1, contype=0, conaffinity=0,
        )

    # 注册 DM 安全 HOME 关键帧（注意 finger_left = +stroke, finger_right = -stroke）
    q_home = load_home_q_rad() if home_q_sdk is None else np.asarray(home_q_sdk, dtype=np.float64)
    q_off = load_joint_offset_rad() if joint_offset_rad is None else np.asarray(joint_offset_rad, dtype=np.float64)
    q_urdf_home = q_home + q_off
    g_closed, g_open_deg = load_gripper_deg_bounds()
    g_open = gripper_deg_to_prismatic_m(g_open_deg, closed_deg=g_closed, open_deg=g_open_deg)
    joint_map = {
        "joint1": float(q_urdf_home[0]),
        "joint2": float(q_urdf_home[1]),
        "joint3": float(q_urdf_home[2]),
        "joint4": float(q_urdf_home[3]),
        "joint5": float(q_urdf_home[4]),
        "joint6": float(q_urdf_home[5]),
        "finger_left": g_open,
        "finger_right": -g_open,
    }
    key_qpos = [joint_map.get(j.name, 0.0) for j in spec.joints]
    spec.add_key(name="home", qpos=key_qpos)

    return spec


# 向后兼容别名
build_b601_spec = build_b601_dm_spec
