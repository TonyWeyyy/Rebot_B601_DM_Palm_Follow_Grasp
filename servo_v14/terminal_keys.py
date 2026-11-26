"""
跨桌面 X11 与无头 SSH 终端的统一按键监听模块（servo_v14/terminal_keys.py）
==========================================================================
在桌面环境（存在 ``DISPLAY`` 且 ``pynput`` 可用）优先使用 ``pynput`` 全局热键；
在 Jetson 无显示器 / SSH 无头终端下自动回退至零依赖的 POSIX ``termios + select``
非阻塞标准输入监听，确保 ``g`` / ``Space`` / ``h`` / ``q`` / ``Esc`` 在任意部署形态
下均可可靠响应（包括急停解除武装与安全回零退出），退出时自动恢复终端状态。
"""

from __future__ import annotations

import os
import select
import sys
import threading
from typing import Callable, Optional

try:
    import termios
    import tty
except ImportError:  # pragma: no cover - 非 POSIX 平台
    termios = None
    tty = None


def normalize_posix_bytes(buf: bytes) -> list[str]:
    """将终端原始字节流解析为统一按键名（'esc' / 'space' / 小写单字符）。"""
    keys: list[str] = []
    i = 0
    n = len(buf)
    while i < n:
        b = buf[i]
        if b == 0x1B:
            # 跳过常见 ANSI 转义序列（如方向键 ESC [ A），仅将独立 ESC 视作急停退出键
            if i + 1 < n and buf[i + 1] in (ord("["), ord("O")):
                i += 2
                while i < n and not (0x40 <= buf[i] <= 0x7E):
                    i += 1
                i += 1
                continue
            keys.append("esc")
            i += 1
            continue
        if b == 0x20:
            keys.append("space")
            i += 1
            continue
        if b in (0x03,):  # Ctrl+C 原始字节（若 cbreak 意外收到）
            keys.append("ctrl_c")
            i += 1
            continue
        if 0x21 <= b <= 0x7E:
            keys.append(chr(b).lower())
        i += 1
    return keys


class UnifiedKeyListener:
    """统一键盘监听器：桌面优先 ``pynput``，无头/SSH 自动回退 POSIX ``termios``。"""

    def __init__(
        self,
        on_key: Callable[[str], Optional[bool]],
        *,
        prefer_pynput: bool = True,
        fd: Optional[int] = None,
    ) -> None:
        self._on_key = on_key
        self._prefer_pynput = bool(prefer_pynput)
        self._custom_fd = fd
        self._stop_event = threading.Event()
        self._pynput_listener = None
        self._thread: Optional[threading.Thread] = None
        self._saved_termios = None
        self._tty_fd: Optional[int] = None
        self.backend: str = "none"

    def start(self) -> str:
        """启动监听器并返回实际使用的后端名：``'pynput'`` / ``'posix_tty'`` / ``'none'``。"""
        if self._prefer_pynput and self._custom_fd is None and os.environ.get("DISPLAY"):
            if self._try_start_pynput():
                self.backend = "pynput"
                return self.backend

        if self._try_start_posix():
            self.backend = "posix_tty"
            return self.backend

        self.backend = "none"
        return self.backend

    def _try_start_pynput(self) -> bool:
        try:
            from pynput import keyboard as pynput_kb

            def _on_press(key):
                if key == pynput_kb.Key.esc:
                    name = "esc"
                elif key == pynput_kb.Key.space:
                    name = "space"
                else:
                    ch = getattr(key, "char", None)
                    if not ch:
                        return None
                    name = str(ch).lower()
                return self._on_key(name)

            listener = pynput_kb.Listener(on_press=_on_press)
            listener.start()
            self._pynput_listener = listener
            return True
        except Exception:
            self._pynput_listener = None
            return False

    def _try_start_posix(self) -> bool:
        if self._custom_fd is not None:
            fd = int(self._custom_fd)
            is_tty = os.isatty(fd)
        else:
            try:
                if sys.stdin is None or sys.stdin.closed or not sys.stdin.isatty():
                    return False
                fd = sys.stdin.fileno()
                is_tty = True
            except Exception:
                return False

        if is_tty and termios is not None and tty is not None:
            try:
                self._saved_termios = termios.tcgetattr(fd)
                self._tty_fd = fd
                tty.setcbreak(fd)
            except Exception:
                self._saved_termios = None
                self._tty_fd = None

        self._stop_event.clear()

        def _reader() -> None:
            try:
                while not self._stop_event.is_set():
                    try:
                        ready, _, _ = select.select([fd], [], [], 0.05)
                    except (OSError, ValueError):
                        break
                    if not ready:
                        continue
                    try:
                        chunk = os.read(fd, 32)
                    except OSError:
                        break
                    if not chunk:
                        break
                    for key_name in normalize_posix_bytes(chunk):
                        try:
                            keep = self._on_key(key_name)
                        except Exception:
                            keep = True
                        if keep is False:
                            self._stop_event.set()
                            return
            finally:
                self._restore_termios()

        self._thread = threading.Thread(target=_reader, name="rebot-posix-keys", daemon=True)
        self._thread.start()
        return True

    def _restore_termios(self) -> None:
        if self._saved_termios is not None and self._tty_fd is not None and termios is not None:
            try:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._saved_termios)
            except Exception:
                pass
            self._saved_termios = None
            self._tty_fd = None

    def stop(self) -> None:
        self._stop_event.set()
        if self._pynput_listener is not None:
            try:
                self._pynput_listener.stop()
            except Exception:
                pass
            self._pynput_listener = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=0.3)
        self._thread = None
        self._restore_termios()
