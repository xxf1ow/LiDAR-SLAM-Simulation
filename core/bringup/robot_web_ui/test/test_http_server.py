import json
import socket
import threading
import time
import urllib.error
import urllib.request
from contextlib import contextmanager
from http.server import HTTPServer, ThreadingHTTPServer
from pathlib import Path

import pytest

import robot_web_ui.http_server as http_server
from robot_web_ui.http_server import ActionUnavailable, create_server
from robot_web_ui.manual_command import command_values


SYNTHETIC_MAX_LINEAR_SPEED = 1.7
SYNTHETIC_MAX_ANGULAR_SPEED = 2.3


WEB_UI_NODE_PATH = (
    Path(__file__).parents[1]
    / "robot_web_ui"
    / "web_ui_node.py"
)


class FakeActions:
    def __init__(self):
        self.calls = []
        self.unavailable = False
        self.conflict = False
        self.pending = False
        self.mode = "manual"
        self.motion_status_calls = 0

    def motion_status(self):
        self.motion_status_calls += 1
        return {
            "linear_x": 0.25,
            "angular_z": -0.1,
            "feedback_fresh": True,
        }

    def manual_command(self, direction, speed_percent):
        command_values(
            direction,
            speed_percent,
            SYNTHETIC_MAX_LINEAR_SPEED,
            SYNTHETIC_MAX_ANGULAR_SPEED,
        )
        if self.conflict:
            raise http_server.ActionConflict(
                "manual control is not active",
                self.mode,
            )
        if self.unavailable:
            raise ActionUnavailable("publisher unavailable")
        self.calls.append(("manual_command", direction, speed_percent))
        return self.mode

    def takeover_manual(self):
        if self.unavailable:
            raise ActionUnavailable("manual service unavailable")
        if self.pending:
            raise http_server.ActionPending(
                "manual takeover unconfirmed",
                self.mode,
            )
        self.calls.append(("takeover_manual",))
        self.mode = "manual"
        return self.mode

    def resume_automatic(self):
        if self.unavailable:
            raise ActionUnavailable("automatic service unavailable")
        if self.pending:
            raise http_server.ActionPending(
                "automatic resume unconfirmed",
                self.mode,
            )
        self.calls.append(("resume_automatic",))
        self.mode = "automatic"
        return self.mode


def test_odometry_freshness_uses_only_the_ros_node_clock():
    source = WEB_UI_NODE_PATH.read_text(encoding="utf-8")

    assert "time.monotonic" not in source
    assert "platform" not in source
    assert "backend" not in source
    assert "ODOM_TIMEOUT = 0.5" in source


@contextmanager
def running_server(actions, html_path):
    server = create_server(actions, html_path, host="127.0.0.1", port=0)
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)


def request(base_url, path, body=None, content_type="application/json"):
    data = None if body is None else body
    req = urllib.request.Request(base_url + path, data=data)
    if body is not None:
        req.add_header("Content-Type", content_type)
    try:
        return urllib.request.urlopen(req)
    except urllib.error.HTTPError as exc:
        return exc


def post_json(base_url, path, payload):
    return request(
        base_url,
        path,
        json.dumps(payload).encode("utf-8"),
    )


def response_json(response):
    return json.loads(response.read())


def test_root_returns_exact_asset_without_caching(tmp_path):
    html = b"<!doctype html><title>robot</title>"
    html_path = tmp_path / "index.html"
    html_path.write_bytes(html)
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        response = request(base_url, "/")

        assert response.status == 200
        assert response.headers["Cache-Control"] == "no-store"
        assert response.read() == html
        assert actions.calls == []


def test_manual_command_calls_action_once(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        response = post_json(
            base_url,
            "/api/manual-command",
            {"direction": "forward", "speed_percent": 20},
        )

        assert response.status == 200
        assert response_json(response) == {
            "ok": True,
            "mode": "manual",
            "linear_x": 0.25,
            "angular_z": -0.1,
            "feedback_fresh": True,
        }
        assert actions.calls == [("manual_command", "forward", 20)]
        assert actions.motion_status_calls == 1


@pytest.mark.parametrize(
    ("path", "expected_call"),
    [
        ("/api/takeover-manual", ("takeover_manual",)),
        ("/api/resume-automatic", ("resume_automatic",)),
    ],
)
def test_mode_endpoints_call_corresponding_action_once(
    tmp_path, path, expected_call
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        response = post_json(base_url, path, {})

        assert response.status == 200
        assert response_json(response) == {
            "ok": True,
            "mode": (
                "manual"
                if path == "/api/takeover-manual"
                else "automatic"
            ),
            "linear_x": 0.25,
            "angular_z": -0.1,
            "feedback_fresh": True,
        }
        assert actions.calls == [expected_call]
        assert actions.motion_status_calls == 1


@pytest.mark.parametrize(
    "body",
    [
        b"{",
        json.dumps({"direction": "diagonal", "speed_percent": 20}).encode(),
        json.dumps({"direction": "forward", "speed_percent": -1}).encode(),
        json.dumps({"direction": "forward", "speed_percent": 101}).encode(),
        b'{"direction":"forward","speed_percent":NaN}',
        b"x" * 4097,
    ],
)
def test_bad_manual_requests_return_400_without_actions(
    tmp_path, body
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        response = request(base_url, "/api/manual-command", body)

        assert response.status == 400
        assert "error" in response_json(response)
        assert actions.calls == []


def test_unknown_routes_return_404_without_actions(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        get_response = request(base_url, "/missing")
        post_response = request(base_url, "/api/status", b"not-json")

        assert get_response.status == 404
        assert post_response.status == 404
        assert actions.calls == []


def test_non_json_content_type_is_rejected_without_actions(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    body = json.dumps(
        {"direction": "forward", "speed_percent": 20}
    ).encode()

    with running_server(actions, html_path) as base_url:
        response = request(
            base_url,
            "/api/manual-command",
            body,
            content_type="text/plain",
        )

        assert response.status == 400
        assert actions.calls == []


def test_partial_body_times_out_with_request_error(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    server = create_server(
        actions,
        html_path,
        host="127.0.0.1",
        port=0,
    )
    thread = threading.Thread(target=server.serve_forever)
    thread.start()
    client = socket.create_connection(
        ("127.0.0.1", server.server_port),
        timeout=1.0,
    )
    client.settimeout(1.0)

    try:
        client.sendall(
            b"POST /api/manual-command HTTP/1.1\r\n"
            b"Host: localhost\r\n"
            b"Content-Type: application/json\r\n"
            b"Content-Length: 64\r\n"
            b"\r\n"
            b"{"
        )
        started = time.monotonic()
        try:
            response = client.recv(4096)
        except socket.timeout:
            response = b""
        elapsed = time.monotonic() - started

        assert response.startswith(b"HTTP/1.0 400")
        assert elapsed < 1.0
        assert actions.calls == []
    finally:
        client.close()
        server.shutdown()
        server.server_close()
        thread.join(timeout=1.0)


def test_action_unavailable_returns_503(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    actions.unavailable = True

    with running_server(actions, html_path) as base_url:
        response = post_json(base_url, "/api/takeover-manual", {})

        assert response.status == 503
        assert response_json(response) == {
            "error": "manual service unavailable"
        }
        assert actions.calls == []


def test_manual_command_conflict_returns_409(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    actions.conflict = True
    actions.mode = "automatic"

    with running_server(actions, html_path) as base_url:
        response = post_json(
            base_url,
            "/api/manual-command",
            {"direction": "forward", "speed_percent": 20},
        )

        assert response.status == 409
        assert response_json(response) == {
            "error": "manual control is not active",
            "mode": "automatic",
            "linear_x": 0.25,
            "angular_z": -0.1,
            "feedback_fresh": True,
        }
        assert actions.calls == []
        assert actions.motion_status_calls == 1


def test_pending_mode_switch_returns_202_with_observed_mode(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    actions.pending = True
    actions.mode = None

    with running_server(actions, html_path) as base_url:
        response = post_json(base_url, "/api/takeover-manual", {})

        assert response.status == 202
        assert response_json(response) == {
            "ok": False,
            "pending": True,
            "error": "manual takeover unconfirmed",
            "mode": None,
            "linear_x": 0.25,
            "angular_z": -0.1,
            "feedback_fresh": True,
        }
        assert actions.calls == []
        assert actions.motion_status_calls == 1


def test_create_server_is_serial_http_server(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")

    server = create_server(
        FakeActions(), html_path, host="127.0.0.1", port=0
    )
    try:
        assert isinstance(server, HTTPServer)
        assert not isinstance(server, ThreadingHTTPServer)
    finally:
        server.server_close()
