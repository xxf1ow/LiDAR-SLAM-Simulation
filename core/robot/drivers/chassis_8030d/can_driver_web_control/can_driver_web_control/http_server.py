from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from .control_state import ControlConflict, ControlError, ControlState


def _handler_for(state: ControlState, html_path: Path):
    class ControlRequestHandler(BaseHTTPRequestHandler):
        def _send_bytes(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, status: int, payload: dict[str, object]) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self._send_bytes(status, "application/json; charset=utf-8", body)

        def _read_json(self) -> dict[str, object]:
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise ControlError("invalid Content-Length") from exc
            if length <= 0 or length > 4096:
                raise ControlError("request body must contain 1 to 4096 bytes")
            try:
                payload = json.loads(self.rfile.read(length))
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ControlError("request body must be valid JSON") from exc
            if not isinstance(payload, dict):
                raise ControlError("request JSON must be an object")
            return payload

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/":
                self._send_bytes(
                    200,
                    "text/html; charset=utf-8",
                    html_path.read_bytes(),
                )
                return
            if path == "/api/status":
                self._send_json(200, state.snapshot())
                return
            self._send_json(404, {"error": "not found"})

        def do_POST(self) -> None:
            path = urlsplit(self.path).path
            if path not in {"/api/command", "/api/driver"}:
                self._send_json(404, {"error": "not found"})
                return
            try:
                payload = self._read_json()
                if path == "/api/command":
                    state.set_command(
                        payload.get("direction"),
                        payload.get("speed_rpm"),
                    )
                else:
                    state.set_enabled(payload.get("enabled"))
            except ControlConflict as exc:
                self._send_json(409, {"error": str(exc)})
                return
            except ControlError as exc:
                self._send_json(400, {"error": str(exc)})
                return
            self._send_json(200, state.snapshot())

        def log_message(self, format: str, *args: object) -> None:
            return

    return ControlRequestHandler


def create_server(
    state: ControlState,
    html_path: Path,
    host: str = "0.0.0.0",
    port: int = 8080,
) -> ThreadingHTTPServer:
    server = ThreadingHTTPServer((host, port), _handler_for(state, html_path))
    server.daemon_threads = True
    return server
