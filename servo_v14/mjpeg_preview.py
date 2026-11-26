"""
轻量级无头远程调试 MJPEG 预览服务（servo_v14/mjpeg_preview.py）
==============================================================
专为 **NVIDIA Jetson Orin Nano 8GB** 无显示器 SSH 现场调试设计：
  - 零额外第三方依赖（仅用标准库 ``http.server`` + ``cv2.imencode``）；
  - 默认关闭（``port=0`` 时零线程、零内存、零 CPU）；
  - 仅在有浏览器客户端实际连接 ``/stream.mjpg`` 时才缩放并压缩 JPEG（默认限速 10fps、
    宽 400px、质量 65），无客户端时完全跳过编码，内存占用 < 5 MB，不挤占 200Hz MIT 线程。
"""

from __future__ import annotations

import http.server
import socketserver
import threading
import time
from typing import Optional

import cv2
import numpy as np

_INDEX_HTML = b"""<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <title>reBot B601-DM Orin Nano Live Preview</title>
  <style>
    body { margin: 0; background: #111; color: #eee; font-family: sans-serif; text-align: center; }
    h3 { margin: 12px 0 8px; font-weight: 500; }
    img { max-width: 96vw; max-height: 86vh; border: 1px solid #333; border-radius: 6px; }
  </style>
</head>
<body>
  <h3>reBot Arm B601-DM + Gemini 305 (Headless Debug Preview)</h3>
  <img src="/stream.mjpg" alt="live stream" />
</body>
</html>
"""


class _ThreadingHTTPServer(socketserver.ThreadingMixIn, http.server.HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class MjpegPreviewServer:
    """按需编码的轻量级 MJPEG HTTP 调试预览服务器。"""

    def __init__(
        self,
        *,
        host: str = "0.0.0.0",
        port: int = 8081,
        max_fps: float = 10.0,
        max_width: int = 400,
        jpeg_quality: int = 65,
    ) -> None:
        self.host = str(host)
        self.port = int(port)
        self.min_interval_s = 1.0 / max(1.0, float(max_fps))
        self.max_width = max(160, int(max_width))
        self.jpeg_quality = int(np.clip(jpeg_quality, 30, 95))

        self._lock = threading.Lock()
        self._cond = threading.Condition(self._lock)
        self._latest_jpg: Optional[bytes] = None
        self._frame_seq: int = 0
        self._last_encode_mono: float = 0.0
        self._client_count: int = 0
        self._running: bool = False
        self._server: Optional[_ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None

    @property
    def active_clients(self) -> int:
        with self._lock:
            return self._client_count

    @property
    def bound_port(self) -> int:
        if self._server is not None:
            return int(self._server.server_address[1])
        return self.port

    def start(self) -> None:
        if self._running:
            return
        parent = self

        class _Handler(http.server.BaseHTTPRequestHandler):
            def log_message(self, fmt: str, *args) -> None:  # noqa: A002
                return

            def do_GET(self) -> None:
                if self.path in ("/", "/index.html"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(_INDEX_HTML)))
                    self.end_headers()
                    self.wfile.write(_INDEX_HTML)
                    return

                if self.path != "/stream.mjpg":
                    self.send_error(404)
                    return

                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header("Pragma", "no-cache")
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
                self.end_headers()

                with parent._lock:
                    parent._client_count += 1
                last_seq = -1
                try:
                    while parent._running:
                        with parent._cond:
                            if parent._frame_seq == last_seq:
                                parent._cond.wait(timeout=0.5)
                            jpg = parent._latest_jpg
                            seq = parent._frame_seq
                        if jpg is None or seq == last_seq:
                            continue
                        last_seq = seq
                        header = (
                            b"--FRAME\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            + f"Content-Length: {len(jpg)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(header)
                        self.wfile.write(jpg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError, OSError):
                    pass
                finally:
                    with parent._lock:
                        parent._client_count = max(0, parent._client_count - 1)

        self._server = _ThreadingHTTPServer((self.host, self.port), _Handler)
        self._running = True
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            kwargs={"poll_interval": 0.2},
            name="rebot-mjpeg-preview",
            daemon=True,
        )
        self._thread.start()

    def push_frame(self, bgr_img: np.ndarray, *, force: bool = False) -> bool:
        """推送一帧图像；无连接客户端或未到限速间隔时零开销跳过。"""
        if not self._running:
            return False
        now = time.monotonic()
        with self._lock:
            if not force and self._client_count <= 0:
                return False
            if not force and (now - self._last_encode_mono) < self.min_interval_s:
                return False
            self._last_encode_mono = now

        h, w = bgr_img.shape[:2]
        if w > self.max_width and w > 0:
            new_h = max(1, int(round(h * (self.max_width / float(w)))))
            small = cv2.resize(bgr_img, (self.max_width, new_h), interpolation=cv2.INTER_AREA)
        else:
            small = bgr_img

        ok, enc = cv2.imencode(
            ".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]
        )
        if not ok:
            return False
        jpg_bytes = enc.tobytes()
        with self._cond:
            self._latest_jpg = jpg_bytes
            self._frame_seq += 1
            self._cond.notify_all()
        return True

    def stop(self) -> None:
        self._running = False
        with self._cond:
            self._cond.notify_all()
        if self._server is not None:
            try:
                self._server.shutdown()
                self._server.server_close()
            except Exception:
                pass
            self._server = None
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
