#!/usr/bin/env python3
"""
Seeed reBot Arm B601-DM + Gemini 305 MuJoCo Studio 3D 实时数字孪生独立启动器（v15.0）
==================================================================================
支持本地原生桌面窗口（--native）与独立浏览器 WebViewer（--web）两种模式。
"""

from __future__ import annotations

import argparse
import os
import pathlib
import sys

TWIN_DIR = pathlib.Path(os.path.abspath(__file__)).parent
if str(TWIN_DIR) not in sys.path:
    sys.path.insert(0, str(TWIN_DIR))


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="reBot Arm B601-DM MuJoCo Studio 3D 实时数字孪生独立启动器"
    )
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--native", action="store_true", help="本地原生桌面 3D 窗口模式")
    mode.add_argument("--web", action="store_true", help="浏览器 WebViewer 模式（默认 http://localhost:8080/）")
    p.add_argument("--udp-bind", default="127.0.0.1", help="状态接收绑定地址（本机 127.0.0.1；跨机接收 Jetson 请填 0.0.0.0）")
    p.add_argument("--udp-port", type=int, default=19870, help="状态接收 UDP 端口（通道 A，默认 19870）")
    p.add_argument("--cmd-host", default="127.0.0.1", help="遥操作指令目标主机（本机 127.0.0.1；跨机控 Jetson 请填 Jetson IP）")
    p.add_argument("--cmd-port", type=int, default=19871, help="遥操作指令发送 UDP 端口（通道 B，默认 19871）")
    p.add_argument("--allow-state-host", default="", help="可选：跨机模式下仅接收该来源 IP 的状态包（如 Jetson IP）")
    p.add_argument("--http-port", type=int, default=8080, help="WebViewer HTTP 端口（仅 --web 生效）")
    p.add_argument("--open-browser", action="store_true", help="启动 --web 后自动打开浏览器")
    p.add_argument("--gfx", default="", help="指定原生图形后端（默认 opengl_headless）")
    p.add_argument("--width", type=int, default=1440, help="窗口初始宽度（默认 1440）")
    p.add_argument("--height", type=int, default=900, help="窗口初始高度（默认 900）")
    p.add_argument("--urdf", default="", help="可选：指定自定义 B601-DM URDF 文件路径")
    p.add_argument("--no-hot-reload", action="store_true", help="禁用插件热重载监听")
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    use_web = bool(args.web)

    from mujoco.studio import app as studio_app
    from mujoco.studio import launch_native
    from mujoco.studio import launch_web
    from mujoco.studio import plugin
    from mujoco.studio import viewer_protocol

    user_plugins = plugin.load_plugins(
        [TWIN_DIR / "sim_plugin.py", TWIN_DIR / "viewer_plugin.py"]
    )
    for plg in user_plugins:
        if hasattr(plg, "udp_bind"):
            plg.udp_bind = str(args.udp_bind)
        if hasattr(plg, "udp_port"):
            plg.udp_port = int(args.udp_port)
        if hasattr(plg, "cmd_host"):
            plg.cmd_host = str(args.cmd_host)
        if hasattr(plg, "cmd_port"):
            plg.cmd_port = int(args.cmd_port)
        if hasattr(plg, "allow_state_host"):
            plg.allow_state_host = str(args.allow_state_host)
        if hasattr(plg, "custom_urdf_path"):
            plg.custom_urdf_path = str(args.urdf)

    effective_gfx = str(args.gfx) if args.gfx else ("" if use_web else "opengl_headless")

    config = viewer_protocol.ViewerConfig(
        title="reBot Arm B601-DM Digital Twin v15.0",
        width=int(args.width),
        height=int(args.height),
        gfx=effective_gfx,
        http_port=int(args.http_port) if use_web else 0,
        open_browser=bool(args.open_browser) if use_web else False,
        hot_reload="" if args.no_hot_reload else "all",
    )
    all_plugins = [studio_app.App(), *studio_app.SIM_PLUGINS, *user_plugins]

    if use_web:
        print(
            f"[MuJoCo Twin DM v15.0] WebViewer 模式启动 -> http://localhost:{args.http_port}/ "
            f"| UDP 状态:{args.udp_bind}:{args.udp_port} <-> 指令:{args.cmd_host}:{args.cmd_port}",
            flush=True,
        )
        launch_web.run(config, plugins=all_plugins)
    else:
        print(
            f"[MuJoCo Twin DM v15.0] 本地原生桌面窗口启动 ({args.width}x{args.height}) "
            f"| UDP 状态:{args.udp_bind}:{args.udp_port} <-> 指令:{args.cmd_host}:{args.cmd_port}",
            flush=True,
        )
        launch_native.run(config, plugins=all_plugins)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
