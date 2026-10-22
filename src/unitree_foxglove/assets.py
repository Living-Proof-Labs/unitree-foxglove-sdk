"""Serve the robot description (URDF and meshes) to the Foxglove 3D panel.

The web app fetches the URDF layer over HTTP. Browsers treat ``http://localhost``
as a secure origin, so an https page may load it, but the request is cross-origin
and needs a CORS header. Mesh paths inside the URDF are relative, so they are
served from the same directory.
"""

from __future__ import annotations

import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


class _CorsHandler(SimpleHTTPRequestHandler):
    def end_headers(self) -> None:
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002 - stdlib name
        return None


def serve_directory(directory: Path, host: str, port: int) -> ThreadingHTTPServer:
    """Start a background HTTP server rooted at ``directory``; returns it for ``shutdown``."""
    handler = partial(_CorsHandler, directory=str(directory))
    server = ThreadingHTTPServer((host, port), handler)
    thread = threading.Thread(target=server.serve_forever, name="assets", daemon=True)
    thread.start()
    return server
