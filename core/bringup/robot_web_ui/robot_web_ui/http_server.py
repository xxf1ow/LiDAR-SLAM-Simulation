from __future__ import annotations

import json
import socket
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit


REQUEST_TIMEOUT = 0.5
ASSET_PATHS = {
    "/api/map/static": "static",
    "/api/map/global-costmap": "global_costmap",
    "/api/map/local-costmap": "local_costmap",
    "/api/navigation-path": "path",
}


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


class RobotWebHttpServer(ThreadingHTTPServer):
    daemon_threads = True


def _handler_for(actions, html_path: Path):
    class RobotWebRequestHandler(BaseHTTPRequestHandler):
        def setup(self) -> None:
            super().setup()
            self.connection.settimeout(REQUEST_TIMEOUT)

        def _send_bytes(
            self,
            status: int,
            content_type: str,
            body: bytes | None,
            *,
            cache_control: str = "no-store",
            headers: dict[str, str] | None = None,
        ) -> None:
            try:
                self.send_response(status)
                if content_type:
                    self.send_header("Content-Type", content_type)
                if body is not None:
                    self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", cache_control)
                for name, value in (headers or {}).items():
                    self.send_header(name, value)
                self.end_headers()
                if body:
                    self.wfile.write(body)
            except (
                BrokenPipeError,
                ConnectionResetError,
                socket.timeout,
            ):
                self.close_connection = True

        def _send_json(self, status: int, payload: dict[str, object]) -> None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self._send_bytes(
                status,
                "application/json; charset=utf-8",
                body,
            )

        def _send_asset(self, snapshot) -> None:
            if snapshot is None:
                self._send_json(404, {"error": "not found"})
                return
            headers = {"ETag": snapshot.etag}
            if self.headers.get("If-None-Match") == snapshot.etag:
                self._send_bytes(
                    304,
                    "",
                    None,
                    cache_control="no-cache",
                    headers=headers,
                )
                return
            self._send_bytes(
                200,
                snapshot.media_type,
                snapshot.gzip_data,
                cache_control="no-cache",
                headers={**headers, "Content-Encoding": "gzip"},
            )

        def _send_action_json(
            self,
            status: int,
            payload: dict[str, object],
        ) -> None:
            self._send_json(
                status,
                {**payload, **actions.motion_status()},
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
            path = urlsplit(self.path).path
            if path == "/":
                self._send_bytes(
                    200,
                    "text/html; charset=utf-8",
                    html_path.read_bytes(),
                )
                return
            if path == "/api/navigation-state":
                self._send_json(200, actions.navigation_state())
                return
            asset_name = ASSET_PATHS.get(path)
            if asset_name is not None:
                self._send_asset(actions.navigation_asset(asset_name))
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
                self._send_action_json(
                    409,
                    {"error": str(exc), "mode": exc.mode},
                )
                return
            except ActionPending as exc:
                self._send_action_json(
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
            self._send_action_json(200, {"ok": True, "mode": mode})

        def log_message(self, format: str, *args: object) -> None:
            return

    return RobotWebRequestHandler


def create_server(
    actions,
    html_path: Path,
    host: str = "0.0.0.0",
    port: int = 8080,
) -> HTTPServer:
    return RobotWebHttpServer((host, port), _handler_for(actions, html_path))
