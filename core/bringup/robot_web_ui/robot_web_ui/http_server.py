from __future__ import annotations

import json
import secrets
import socket
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from .navigation_request import MapRevisionConflict


REQUEST_TIMEOUT = 0.5
MANUAL_SEQUENCE_MAX = 9_007_199_254_740_991
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


def _manual_command_payload(payload):
    if set(payload) != {
        "session_id",
        "sequence",
        "direction",
        "speed_percent",
    }:
        raise ValueError("manual command fields are invalid")
    session_id, sequence, direction, speed_percent = (
        payload["session_id"],
        payload["sequence"],
        payload["direction"],
        payload["speed_percent"],
    )
    if not (isinstance(session_id, str) and session_id):
        raise ValueError("session_id must be a nonempty string")
    if (
        not isinstance(sequence, int)
        or isinstance(sequence, bool)
        or not 1 <= sequence <= MANUAL_SEQUENCE_MAX
    ):
        raise ValueError("sequence must be a positive safe integer")
    return session_id, sequence, direction, speed_percent


def _handler_for(actions, html_path: Path):
    manual_lock = threading.Lock()
    active_session_id = None
    last_sequence = 0
    manual_mode = None

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
            try:
                body = json.dumps(
                    payload,
                    separators=(",", ":"),
                    allow_nan=False,
                ).encode("utf-8")
            except (TypeError, ValueError):
                status = 500
                body = b'{"error":"response serialization failed"}'
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
            if path == "/map_view.js":
                self._send_bytes(
                    200,
                    "application/javascript; charset=utf-8",
                    html_path.with_name("map_view.js").read_bytes(),
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
                "/api/manual-session",
                "/api/manual-command",
                "/api/takeover-manual",
                "/api/resume-automatic",
                "/api/initial-pose",
                "/api/navigation-goal",
                "/api/navigation-cancel",
            }:
                self._send_json(404, {"error": "not found"})
                return

            manual_command_context = None
            try:
                payload = self._read_json()
                if path == "/api/manual-session":
                    if payload != {}:
                        raise ValueError("manual session request must be {}")
                    nonlocal active_session_id, last_sequence, manual_mode
                    with manual_lock:
                        mode = actions.manual_command("stop", 0)
                        session_id = secrets.token_urlsafe(16)
                        active_session_id = session_id
                        last_sequence = 0
                        manual_mode = mode
                    self._send_action_json(
                        200,
                        {
                            "ok": True,
                            "session_id": session_id,
                            "mode": mode,
                        },
                    )
                    return
                if path == "/api/manual-command":
                    (
                        session_id,
                        sequence,
                        direction,
                        speed_percent,
                    ) = _manual_command_payload(payload)
                    manual_response = None
                    manual_status = 200
                    manual_response_with_status = False
                    with manual_lock:
                        previous_sequence = last_sequence
                        manual_command_context = (
                            sequence,
                            previous_sequence,
                        )
                        if active_session_id != session_id:
                            manual_status = 409
                            manual_response = {
                                "error": "inactive manual session",
                                "accepted": False,
                                "reason": "inactive_session",
                                "sequence": sequence,
                            }
                        elif sequence <= last_sequence:
                            manual_response_with_status = True
                            manual_response = {
                                "ok": True,
                                "accepted": False,
                                "reason": "stale_sequence",
                                "sequence": sequence,
                                "last_sequence": last_sequence,
                                "mode": manual_mode,
                            }
                        else:
                            mode = actions.manual_command(
                                direction,
                                speed_percent,
                            )
                            last_sequence = sequence
                            manual_mode = mode
                            manual_response_with_status = True
                            manual_response = {
                                "ok": True,
                                "accepted": True,
                                "sequence": sequence,
                                "last_sequence": sequence,
                                "mode": mode,
                            }
                    if manual_response_with_status:
                        self._send_action_json(
                            manual_status,
                            manual_response,
                        )
                    else:
                        self._send_json(manual_status, manual_response)
                    return
                if path == "/api/takeover-manual":
                    mode = actions.takeover_manual()
                elif path == "/api/resume-automatic":
                    mode = actions.resume_automatic()
                elif path == "/api/initial-pose":
                    actions.publish_initial_pose(payload)
                    self._send_json(200, {"ok": True})
                    return
                elif path == "/api/navigation-goal":
                    goal_status = actions.send_navigation_goal(payload)
                    self._send_json(
                        202,
                        {"ok": True, "goal_status": goal_status},
                    )
                    return
                else:
                    if payload != {}:
                        raise ValueError("navigation cancel request must be {}")
                    goal_status = actions.cancel_navigation()
                    self._send_json(
                        202,
                        {"ok": True, "goal_status": goal_status},
                    )
                    return
            except MapRevisionConflict as exc:
                self._send_json(409, {"error": str(exc)})
                return
            except ValueError as exc:
                if manual_command_context is None:
                    self._send_json(400, {"error": str(exc)})
                else:
                    sequence, previous_sequence = manual_command_context
                    self._send_action_json(
                        400,
                        {
                            "error": str(exc),
                            "accepted": False,
                            "sequence": sequence,
                            "last_sequence": previous_sequence,
                            "mode": manual_mode,
                        },
                    )
                return
            except ActionConflict as exc:
                if manual_command_context is None:
                    self._send_action_json(
                        409,
                        {"error": str(exc), "mode": exc.mode},
                    )
                else:
                    sequence, previous_sequence = manual_command_context
                    self._send_action_json(
                        409,
                        {
                            "error": str(exc),
                            "accepted": False,
                            "sequence": sequence,
                            "last_sequence": previous_sequence,
                            "mode": exc.mode,
                        },
                    )
                return
            except ActionPending as exc:
                if manual_command_context is None:
                    self._send_action_json(
                        202,
                        {
                            "ok": False,
                            "pending": True,
                            "error": str(exc),
                            "mode": exc.mode,
                        },
                    )
                else:
                    sequence, previous_sequence = manual_command_context
                    self._send_action_json(
                        202,
                        {
                            "ok": False,
                            "pending": True,
                            "error": str(exc),
                            "accepted": False,
                            "sequence": sequence,
                            "last_sequence": previous_sequence,
                            "mode": exc.mode,
                        },
                    )
                return
            except ActionUnavailable as exc:
                if manual_command_context is None:
                    self._send_json(503, {"error": str(exc)})
                else:
                    sequence, previous_sequence = manual_command_context
                    self._send_action_json(
                        503,
                        {
                            "error": str(exc),
                            "accepted": False,
                            "sequence": sequence,
                            "last_sequence": previous_sequence,
                            "mode": manual_mode,
                        },
                    )
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
