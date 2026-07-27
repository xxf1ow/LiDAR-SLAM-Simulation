from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlsplit


REQUEST_TIMEOUT = 0.5


class ActionUnavailable(RuntimeError):
    pass


class _ActionWithMode(RuntimeError):
    def __init__(self, message: str, mode: str | None) -> None:
        super().__init__(message)
        self.mode = mode


class ActionConflict(_ActionWithMode):
    pass


class ActionPending(_ActionWithMode):
    pass


def _handler_for(actions, html_path: Path):
    class RobotWebRequestHandler(BaseHTTPRequestHandler):
        def setup(self) -> None:
            super().setup()
            self.connection.settimeout(REQUEST_TIMEOUT)

        def _send_bytes(
            self,
            status: int,
            content_type: str,
            body: bytes,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, status: int, payload: dict[str, object]) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self._send_bytes(
                status,
                "application/json; charset=utf-8",
                body,
            )

        def _read_json(self) -> dict[str, object]:
            if self.headers.get_content_type() != "application/json":
                raise ValueError("Content-Type must be application/json")
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError as exc:
                raise ValueError("invalid Content-Length") from exc
            if length <= 0 or length > 4096:
                raise ValueError(
                    "request body must contain 1 to 4096 bytes"
                )
            try:
                body = self.rfile.read(length)
            except TimeoutError as exc:
                self.close_connection = True
                raise ValueError("request body timed out") from exc
            try:
                payload = json.loads(body)
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError("request body must be valid JSON") from exc
            if not isinstance(payload, dict):
                raise ValueError("request JSON must be an object")
            return payload

        def do_GET(self) -> None:
            if urlsplit(self.path).path == "/":
                self._send_bytes(
                    200,
                    "text/html; charset=utf-8",
                    html_path.read_bytes(),
                )
                return
            self._send_json(404, {"error": "not found"})

        def do_POST(self) -> None:
            path = urlsplit(self.path).path
            if path not in {
                "/api/manual-command",
                "/api/takeover-manual",
                "/api/resume-automatic",
            }:
                self._send_json(404, {"error": "not found"})
                return

            try:
                payload = self._read_json()
                if path == "/api/manual-command":
                    mode = actions.manual_command(
                        payload.get("direction"),
                        payload.get("speed_percent"),
                    )
                elif path == "/api/takeover-manual":
                    mode = actions.takeover_manual()
                else:
                    mode = actions.resume_automatic()
            except ValueError as exc:
                self._send_json(400, {"error": str(exc)})
                return
            except ActionConflict as exc:
                self._send_json(
                    409,
                    {"error": str(exc), "mode": exc.mode},
                )
                return
            except ActionPending as exc:
                self._send_json(
                    202,
                    {
                        "ok": False,
                        "pending": True,
                        "error": str(exc),
                        "mode": exc.mode,
                    },
                )
                return
            except ActionUnavailable as exc:
                self._send_json(503, {"error": str(exc)})
                return
            self._send_json(200, {"ok": True, "mode": mode})

        def log_message(self, format: str, *args: object) -> None:
            return

    return RobotWebRequestHandler


def create_server(
    actions,
    html_path: Path,
    host: str = "0.0.0.0",
    port: int = 8080,
) -> HTTPServer:
    return HTTPServer((host, port), _handler_for(actions, html_path))
