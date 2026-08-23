import ast
import json
import socket
import struct
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import Future, ThreadPoolExecutor
from contextlib import contextmanager
from http.server import HTTPServer, ThreadingHTTPServer
from pathlib import Path

import pytest

import robot_web_ui.http_server as http_server
from robot_web_ui.http_server import ActionUnavailable, create_server
from robot_web_ui.manual_command import command_values
from robot_web_ui.map_snapshot import BinarySnapshot
from robot_web_ui.navigation_request import MapRevisionConflict


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
        self.asset_read_started = threading.Event()
        self.allow_asset_read = threading.Event()
        self.blocked_asset_name = None
        self.navigation_action_started = threading.Event()
        self.allow_navigation_action = threading.Event()
        self.blocked_navigation_action = None
        self.navigation_state_override = None
        self.assets = {
            "static": BinarySnapshot(
                3,
                '"static-revision-3"',
                "application/octet-stream",
                b"static-map",
                b"gzip-static-map",
            ),
            "global_costmap": BinarySnapshot(
                5,
                '"global-costmap-revision-5"',
                "application/octet-stream",
                b"global-costmap",
                b"gzip-global-costmap",
            ),
            "local_costmap": BinarySnapshot(
                7,
                '"local-costmap-revision-7"',
                "application/octet-stream",
                b"local-costmap",
                b"gzip-local-costmap",
            ),
            "path": BinarySnapshot(
                11,
                '"path-revision-11"',
                "application/octet-stream",
                b"path-points",
                b"gzip-path-points",
            ),
        }

    def motion_status(self):
        self.motion_status_calls += 1
        return {
            "linear_x": 0.25,
            "angular_z": -0.1,
            "feedback_fresh": True,
        }

    def navigation_state(self):
        if self.navigation_state_override is not None:
            return self.navigation_state_override
        return {
            "map_error": None,
            "localized": True,
            "layers": {
                name: {
                    "revision": snapshot.revision,
                    "etag": snapshot.etag,
                }
                for name, snapshot in self.assets.items()
            },
        }

    def navigation_asset(self, name):
        if name == self.blocked_asset_name:
            self.asset_read_started.set()
            if not self.allow_asset_read.wait(timeout=1.0):
                raise AssertionError("test did not release blocked asset")
        return self.assets.get(name)

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

    def publish_initial_pose(self, payload):
        self.calls.append(("publish_initial_pose", payload))

    def send_navigation_goal(self, payload):
        self.calls.append(("send_navigation_goal", payload))
        if self.blocked_navigation_action == "send_navigation_goal":
            self.navigation_action_started.set()
            if not self.allow_navigation_action.wait(timeout=1.0):
                raise AssertionError("test did not release blocked navigation")
        return "sending"

    def cancel_navigation(self):
        self.calls.append(("cancel_navigation",))
        if self.blocked_navigation_action == "cancel_navigation":
            self.navigation_action_started.set()
            if not self.allow_navigation_action.wait(timeout=1.0):
                raise AssertionError("test did not release blocked navigation")
        return "canceling"


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


def request(
    base_url,
    path,
    body=None,
    content_type="application/json",
    headers=None,
):
    data = None if body is None else body
    req = urllib.request.Request(base_url + path, data=data)
    if body is not None:
        req.add_header("Content-Type", content_type)
    for name, value in (headers or {}).items():
        req.add_header(name, value)
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


def manual_payload(session_id, sequence, direction="forward", speed_percent=20):
    return {
        "session_id": session_id,
        "sequence": sequence,
        "direction": direction,
        "speed_percent": speed_percent,
    }


def start_manual_session(base_url):
    body = response_json(post_json(base_url, "/api/manual-session", {}))
    assert body["ok"] is True and body["mode"] == "manual"
    assert isinstance(body["session_id"], str)
    return body["session_id"]


def send_manual(
    base_url, session_id, sequence, direction="forward", speed_percent=20
):
    response = post_json(
        base_url,
        "/api/manual-command",
        manual_payload(session_id, sequence, direction, speed_percent),
    )
    return response, response_json(response)


def test_manual_session_sequence_and_concurrency_are_atomic(
    tmp_path, monkeypatch
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        session_a = start_manual_session(base_url)
        first, first_body = send_manual(base_url, session_a, 1)
        assert first_body["accepted"] is True
        assert first_body.get("reason") is None
        calls_after_first = len(actions.calls)

        second, second_body = send_manual(
            base_url, session_a, 2, "stop", 0
        )
        assert second_body["accepted"] is True

        delayed, delayed_body = send_manual(base_url, session_a, 1)
        assert delayed_body.get("reason") == "stale_sequence"
        assert len(actions.calls) == calls_after_first + 1

        def fail_manual(_direction, _speed_percent):
            raise ActionUnavailable("publisher unavailable")

        monkeypatch.setattr(actions, "manual_command", fail_manual)
        failed, failed_body = send_manual(base_url, session_a, 3)
        assert failed.status == 503
        assert failed_body["accepted"] is False
        assert failed_body["last_sequence"] == 2
        monkeypatch.undo()

        retry, retry_body = send_manual(base_url, session_a, 3)
        assert retry_body["accepted"] is True

        session_b = start_manual_session(base_url)
        inactive, inactive_body = send_manual(base_url, session_a, 4)
        assert inactive.status == 409
        assert inactive_body == {
            "error": "inactive manual session",
            "accepted": False,
            "reason": "inactive_session",
            "sequence": 4,
        }

        with ThreadPoolExecutor(max_workers=2) as pool:
            duplicate_bodies = list(
                pool.map(
                    lambda _: send_manual(base_url, session_b, 1)[1],
                    range(2),
                )
            )
        assert sum(body["accepted"] for body in duplicate_bodies) == 1
        assert sum(
            body.get("reason") == "stale_sequence"
            for body in duplicate_bodies
        ) == 1

    assert actions.calls == [
        ("manual_command", "stop", 0),
        ("manual_command", "forward", 20),
        ("manual_command", "stop", 0),
        ("manual_command", "forward", 20),
        ("manual_command", "stop", 0),
        ("manual_command", "forward", 20),
    ]
    assert session_b != session_a


def navigation_pose_payload():
    return {
        "x": 1.25,
        "y": -0.75,
        "yaw": 0.5,
        "map_revision": 3,
    }


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


def test_map_view_is_served_as_an_exact_javascript_asset(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    map_view = html_path.with_name("map_view.js")
    map_view.write_bytes(b"globalThis.RobotMapView = {};")

    with running_server(FakeActions(), html_path) as base_url:
        response = request(base_url, "/map_view.js")

        assert response.status == 200
        assert response.headers["Content-Type"] == (
            "application/javascript; charset=utf-8"
        )
        assert response.read() == map_view.read_bytes()


def test_navigation_state_returns_small_no_store_json(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        response = request(base_url, "/api/navigation-state")
        body = response.read()

        assert response.status == 200
        assert response.headers["Cache-Control"] == "no-store"
        assert response.headers.get("Content-Encoding") is None
        assert body == json.dumps(
            actions.navigation_state(), separators=(",", ":")
        ).encode()
        assert json.loads(body) == actions.navigation_state()
        assert b'"data"' not in body
        assert b"gzip" not in body


@pytest.mark.parametrize(
    "bad_value",
    [
        pytest.param(object(), id="non-json-object"),
        pytest.param(float("nan"), id="non-finite-float"),
    ],
)
def test_navigation_state_serialization_failure_returns_500_and_server_recovers(
    tmp_path, bad_value
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    actions.navigation_state_override = {"bad": bad_value}

    with running_server(actions, html_path) as base_url:
        try:
            response = request(base_url, "/api/navigation-state")
        except Exception as exc:
            pytest.fail(f"server closed without a 500 response: {exc}")

        assert response.status == 500
        assert (
            response.headers["Content-Type"]
            == "application/json; charset=utf-8"
        )
        assert response.read() == b'{"error":"response serialization failed"}'

        actions.navigation_state_override = None
        session_id = start_manual_session(base_url)
        manual = post_json(
            base_url,
            "/api/manual-command",
            manual_payload(session_id, 1),
        )
        assert manual.status == 200
        assert response_json(manual)["ok"] is True
        assert actions.calls == [
            ("manual_command", "stop", 0),
            ("manual_command", "forward", 20),
        ]


@pytest.mark.parametrize(
    ("path", "asset_name"),
    [
        ("/api/map/static", "static"),
        ("/api/map/global-costmap", "global_costmap"),
        ("/api/map/local-costmap", "local_costmap"),
        ("/api/navigation-path", "path"),
    ],
)
def test_binary_assets_send_cached_gzip_etag_and_exact_length(
    tmp_path, path, asset_name
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    snapshot = actions.assets[asset_name]

    with running_server(actions, html_path) as base_url:
        response = request(base_url, path)

        assert response.status == 200
        assert response.headers["Content-Type"] == snapshot.media_type
        assert response.headers["Content-Encoding"] == "gzip"
        assert response.headers["ETag"] == snapshot.etag
        assert response.headers["Cache-Control"] == "no-cache"
        assert response.headers["Content-Length"] == str(
            len(snapshot.gzip_data)
        )
        assert response.read() == snapshot.gzip_data


def test_matching_if_none_match_returns_304_without_body(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    snapshot = actions.assets["static"]

    with running_server(actions, html_path) as base_url:
        response = request(
            base_url,
            "/api/map/static",
            headers={"If-None-Match": snapshot.etag},
        )

        assert response.status == 304
        assert response.headers["ETag"] == snapshot.etag
        assert response.headers["Cache-Control"] == "no-cache"
        assert response.headers.get("Content-Encoding") is None
        assert response.headers.get("Content-Length") is None
        assert response.read() == b""


def test_missing_asset_returns_404_without_fabricated_metadata(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    actions.assets.pop("static")

    with running_server(actions, html_path) as base_url:
        response = request(base_url, "/api/map/static")

        assert response.status == 404
        assert response_json(response) == {"error": "not found"}
        assert response.headers.get("ETag") is None
        assert response.headers.get("Content-Encoding") is None


def test_unknown_get_and_existing_post_contracts_are_unchanged(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        get_response = request(base_url, "/missing")
        session_id = start_manual_session(base_url)
        post_response = post_json(
            base_url,
            "/api/manual-command",
            manual_payload(session_id, 1),
        )

        assert get_response.status == 404
        assert response_json(get_response) == {"error": "not found"}
        assert post_response.status == 200
        expected_body = {
            "ok": True,
            "accepted": True,
            "sequence": 1,
            "last_sequence": 1,
            "mode": "manual",
            "linear_x": 0.25,
            "angular_z": -0.1,
            "feedback_fresh": True,
        }
        assert post_response.read() == json.dumps(
            expected_body,
            separators=(",", ":"),
        ).encode()
        assert actions.calls == [
            ("manual_command", "stop", 0),
            ("manual_command", "forward", 20),
        ]


def test_map_download_does_not_block_manual_post(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    actions.blocked_asset_name = "static"
    results = {}
    download_done = threading.Event()
    manual_done = threading.Event()

    def download(base_url):
        try:
            response = request(base_url, "/api/map/static")
            results["download"] = (response.status, response.read())
        except BaseException as exc:
            results["download_error"] = exc
        finally:
            download_done.set()

    def manual_post(base_url):
        try:
            response = post_json(
                base_url,
                "/api/manual-command",
                manual_payload(session_id, 1),
            )
            results["manual"] = (response.status, response_json(response))
        except BaseException as exc:
            results["manual_error"] = exc
        finally:
            manual_done.set()

    with running_server(actions, html_path) as base_url:
        session_id = start_manual_session(base_url)
        download_thread = threading.Thread(target=download, args=(base_url,))
        manual_thread = threading.Thread(target=manual_post, args=(base_url,))
        download_thread.start()
        try:
            assert actions.asset_read_started.wait(timeout=1.0)
            manual_thread.start()
            assert manual_done.wait(timeout=1.0)
            assert "manual_error" not in results
            assert results["manual"][0] == 200
            assert not download_done.is_set()
        finally:
            actions.allow_asset_read.set()
            download_thread.join(timeout=1.0)
            if manual_thread.is_alive():
                manual_thread.join(timeout=1.0)

    assert not download_thread.is_alive()
    assert not manual_thread.is_alive()
    assert "download_error" not in results
    assert results["download"] == (200, actions.assets["static"].gzip_data)


@pytest.mark.parametrize(
    "write_error",
    [BrokenPipeError, ConnectionResetError, TimeoutError],
)
def test_send_bytes_absorbs_client_write_errors(write_error, tmp_path):
    class FailingWriter:
        def write(self, _body):
            raise write_error()

    handler = object.__new__(
        http_server._handler_for(FakeActions(), tmp_path / "index.html")
    )
    handler.wfile = FailingWriter()
    handler.close_connection = False
    handler.send_response = lambda _status: None
    handler.send_header = lambda _name, _value: None
    handler.end_headers = lambda: None

    handler._send_bytes(200, "application/octet-stream", b"asset")

    assert handler.close_connection is True


def test_abandoned_map_download_does_not_disrupt_manual_post(
    tmp_path, monkeypatch
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    actions.blocked_asset_name = "static"
    response_handled = threading.Event()
    original_handler_for = http_server._handler_for

    def handler_for(*args):
        handler = original_handler_for(*args)
        original_do_get = handler.do_GET

        def do_GET(request_handler):
            try:
                original_do_get(request_handler)
            finally:
                response_handled.set()

        handler.do_GET = do_GET
        return handler

    monkeypatch.setattr(http_server, "_handler_for", handler_for)

    with running_server(actions, html_path) as base_url:
        host, port = base_url.removeprefix("http://").split(":")
        client = socket.create_connection((host, int(port)), timeout=1.0)
        try:
            client.sendall(
                b"GET /api/map/static HTTP/1.1\r\n"
                b"Host: localhost\r\n"
                b"Connection: close\r\n"
                b"\r\n"
            )
            assert actions.asset_read_started.wait(timeout=1.0)
        finally:
            client.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_LINGER,
                struct.pack("ii", 1, 0),
            )
            client.close()

        actions.allow_asset_read.set()
        assert response_handled.wait(timeout=1.0)
        session_id = start_manual_session(base_url)
        response = post_json(
            base_url,
            "/api/manual-command",
            manual_payload(session_id, 1),
        )

        assert response.status == 200
        assert response_json(response)["ok"] is True


def test_manual_command_calls_action_once(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        session_id = start_manual_session(base_url)
        response = post_json(
            base_url,
            "/api/manual-command",
            manual_payload(session_id, 1),
        )

        assert response.status == 200
        assert response_json(response) == {
            "ok": True,
            "accepted": True,
            "sequence": 1,
            "last_sequence": 1,
            "mode": "manual",
            "linear_x": 0.25,
            "angular_z": -0.1,
            "feedback_fresh": True,
        }
        assert actions.calls == [
            ("manual_command", "stop", 0),
            ("manual_command", "forward", 20),
        ]
        assert actions.motion_status_calls == 2


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
        expected_body = {
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
        assert response.read() == json.dumps(
            expected_body,
            separators=(",", ":"),
        ).encode()
        assert actions.calls == [expected_call]
        assert actions.motion_status_calls == 1


@pytest.mark.parametrize(
    "body",
    [
        b"{",
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


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"session_id": "session", "sequence": 1, "direction": "forward"},
        {
            "session_id": "session",
            "sequence": 1,
            "direction": "forward",
            "speed_percent": 20,
            "extra": True,
        },
        manual_payload("", 1),
        manual_payload("session", True),
        manual_payload("session", 9_007_199_254_740_992),
    ],
    ids=[
        "missing-fields",
        "missing-speed",
        "extra-field",
        "empty-session",
        "boolean-sequence",
        "unsafe-sequence",
    ],
)
def test_manual_command_envelope_validation_returns_400_without_actions(
    tmp_path, payload
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        response = post_json(base_url, "/api/manual-command", payload)

        assert response.status == 400
        assert "error" in response_json(response)
        assert actions.calls == []


@pytest.mark.parametrize(
    "payload",
    [
        manual_payload("session", 1, "diagonal"),
        manual_payload("session", 1, "forward", -1),
        manual_payload("session", 1, "forward", 101),
    ],
    ids=["invalid-direction", "negative-speed", "over-max-speed"],
)
def test_manual_command_action_validation_does_not_advance_sequence(
    tmp_path, payload
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        session_id = start_manual_session(base_url)
        payload["session_id"] = session_id
        response = post_json(base_url, "/api/manual-command", payload)
        body = response_json(response)

        assert response.status == 400
        assert body["accepted"] is False
        assert body["sequence"] == 1
        assert body["last_sequence"] == 0
        assert actions.calls == [("manual_command", "stop", 0)]


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


@pytest.mark.parametrize(
    ("path", "payload", "expected_status", "expected_body", "call"),
    [
        (
            "/api/initial-pose",
            navigation_pose_payload(),
            200,
            b'{"ok":true}',
            ("publish_initial_pose", navigation_pose_payload()),
        ),
        (
            "/api/navigation-goal",
            navigation_pose_payload(),
            202,
            b'{"ok":true,"goal_status":"sending"}',
            ("send_navigation_goal", navigation_pose_payload()),
        ),
        (
            "/api/navigation-cancel",
            {},
            202,
            b'{"ok":true,"goal_status":"canceling"}',
            ("cancel_navigation",),
        ),
    ],
)
def test_navigation_routes_dispatch_once_with_exact_responses(
    tmp_path, path, payload, expected_status, expected_body, call
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    with running_server(actions, html_path) as base_url:
        response = post_json(base_url, path, payload)

        assert response.status == expected_status
        assert response.read() == expected_body
        assert actions.calls == [call]


@pytest.mark.parametrize(
    ("path", "body", "action_name"),
    [
        ("/api/initial-pose", b"{", None),
        ("/api/navigation-goal", b'{"x":1}', "send_navigation_goal"),
        ("/api/navigation-cancel", b'{"unexpected":true}', None),
    ],
)
def test_bad_navigation_requests_return_400_without_actions(
    tmp_path, monkeypatch, path, body, action_name
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    if action_name is not None:
        def fail(*_args):
            raise ValueError("invalid pose")

        monkeypatch.setattr(actions, action_name, fail)

    with running_server(actions, html_path) as base_url:
        response = request(base_url, path, body)

        assert response.status == 400
        assert "error" in response_json(response)
        assert actions.calls == []


@pytest.mark.parametrize(
    ("path", "payload", "action_name", "error", "expected_status"),
    [
        (
            "/api/initial-pose",
            navigation_pose_payload(),
            "publish_initial_pose",
            ValueError("invalid pose"),
            400,
        ),
        (
            "/api/navigation-goal",
            navigation_pose_payload(),
            "send_navigation_goal",
            MapRevisionConflict("map revision changed"),
            409,
        ),
        (
            "/api/navigation-goal",
            navigation_pose_payload(),
            "send_navigation_goal",
            http_server.ActionConflict("navigation goal is active", "automatic"),
            409,
        ),
        (
            "/api/navigation-cancel",
            {},
            "cancel_navigation",
            ActionUnavailable("navigation action unavailable"),
            503,
        ),
    ],
)
def test_navigation_action_errors_use_the_existing_http_status_mapping(
    tmp_path,
    monkeypatch,
    path,
    payload,
    action_name,
    error,
    expected_status,
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    def fail(*_args):
        raise error

    monkeypatch.setattr(actions, action_name, fail)

    with running_server(actions, html_path) as base_url:
        response = post_json(base_url, path, payload)

        assert response.status == expected_status
        assert response_json(response)["error"] == str(error)
        assert actions.calls == []


def test_navigation_conflict_preserves_existing_motion_status_response(
    tmp_path, monkeypatch
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()

    def fail(*_args):
        raise http_server.ActionConflict(
            "navigation goal is active",
            "automatic",
        )

    monkeypatch.setattr(actions, "send_navigation_goal", fail)

    with running_server(actions, html_path) as base_url:
        response = post_json(
            base_url,
            "/api/navigation-goal",
            navigation_pose_payload(),
        )

        assert response.status == 409
        assert response.read() == (
            b'{"error":"navigation goal is active","mode":"automatic",'
            b'"linear_x":0.25,"angular_z":-0.1,'
            b'"feedback_fresh":true}'
        )
        assert actions.motion_status_calls == 1
        assert actions.calls == []


@pytest.mark.parametrize(
    ("path", "payload", "action_name", "expected_status"),
    [
        (
            "/api/navigation-goal",
            navigation_pose_payload(),
            "send_navigation_goal",
            202,
        ),
        (
            "/api/navigation-cancel",
            {},
            "cancel_navigation",
            202,
        ),
    ],
)
def test_blocked_navigation_request_does_not_block_manual_post(
    tmp_path, path, payload, action_name, expected_status
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    actions.blocked_navigation_action = action_name
    results = {}
    navigation_done = threading.Event()
    manual_done = threading.Event()

    def navigation_post(base_url):
        try:
            response = post_json(base_url, path, payload)
            results["navigation"] = (response.status, response.read())
        except BaseException as exc:
            results["navigation_error"] = exc
        finally:
            navigation_done.set()

    def manual_post(base_url):
        try:
            response = post_json(
                base_url,
                "/api/manual-command",
                manual_payload(session_id, 1),
            )
            results["manual"] = (response.status, response_json(response))
        except BaseException as exc:
            results["manual_error"] = exc
        finally:
            manual_done.set()

    with running_server(actions, html_path) as base_url:
        session_id = start_manual_session(base_url)
        navigation_thread = threading.Thread(
            target=navigation_post,
            args=(base_url,),
        )
        manual_thread = threading.Thread(target=manual_post, args=(base_url,))
        navigation_thread.start()
        try:
            assert actions.navigation_action_started.wait(timeout=1.0)
            manual_thread.start()
            assert manual_done.wait(timeout=1.0)
            assert "manual_error" not in results
            assert results["manual"][0] == 200
            assert not navigation_done.is_set()
        finally:
            actions.allow_navigation_action.set()
            navigation_thread.join(timeout=1.0)
            if manual_thread.is_alive():
                manual_thread.join(timeout=1.0)

    assert not navigation_thread.is_alive()
    assert not manual_thread.is_alive()
    assert "navigation_error" not in results
    assert results["navigation"][0] == expected_status


def test_abandoned_navigation_response_does_not_disrupt_later_post(
    tmp_path, monkeypatch
):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    response_write_reached = threading.Event()
    allow_response_write = threading.Event()
    response_handled = threading.Event()
    original_handler_for = http_server._handler_for

    def handler_for(*args):
        handler = original_handler_for(*args)
        original_do_post = handler.do_POST
        original_send_bytes = handler._send_bytes

        def send_bytes(request_handler, status, content_type, body, **kwargs):
            if (
                status == 202
                and body == b'{"ok":true,"goal_status":"sending"}'
            ):
                response_write_reached.set()
                if not allow_response_write.wait(timeout=1.0):
                    raise AssertionError("test did not release response write")
            return original_send_bytes(
                request_handler,
                status,
                content_type,
                body,
                **kwargs,
            )

        def do_POST(request_handler):
            try:
                original_do_post(request_handler)
            finally:
                response_handled.set()

        handler._send_bytes = send_bytes
        handler.do_POST = do_POST
        return handler

    monkeypatch.setattr(http_server, "_handler_for", handler_for)
    payload = json.dumps(navigation_pose_payload()).encode("utf-8")

    with running_server(actions, html_path) as base_url:
        host, port = base_url.removeprefix("http://").split(":")
        client = socket.create_connection((host, int(port)), timeout=1.0)
        try:
            client.sendall(
                b"POST /api/navigation-goal HTTP/1.1\r\n"
                b"Host: localhost\r\n"
                b"Content-Type: application/json\r\n"
                + f"Content-Length: {len(payload)}\r\n".encode()
                + b"Connection: close\r\n\r\n"
                + payload
            )
            assert response_write_reached.wait(timeout=1.0)
        finally:
            client.setsockopt(
                socket.SOL_SOCKET,
                socket.SO_LINGER,
                struct.pack("ii", 1, 0),
            )
            client.close()
            allow_response_write.set()

        assert response_handled.wait(timeout=1.0)
        session_id = start_manual_session(base_url)
        response = post_json(
            base_url,
            "/api/manual-command",
            manual_payload(session_id, 1),
        )

        assert response.status == 200
        assert response_json(response)["ok"] is True
        assert actions.calls == [
            ("send_navigation_goal", navigation_pose_payload()),
            ("manual_command", "stop", 0),
            ("manual_command", "forward", 20),
        ]


def test_navigation_http_branch_has_no_blocking_action_control_flow():
    source = Path(http_server.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    do_post = next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "do_POST"
    )
    calls = [
        node
        for node in ast.walk(do_post)
        if isinstance(node, ast.Call)
    ]
    called_attributes = {
        node.func.attr
        for node in calls
        if isinstance(node.func, ast.Attribute)
    }

    assert not any(
        isinstance(node, (ast.For, ast.AsyncFor, ast.While))
        for node in ast.walk(do_post)
    )
    assert called_attributes.isdisjoint({
        "sleep",
        "spin_until_future_complete",
        "result",
        "wait",
        "send_goal_async",
        "get_result_async",
        "cancel_goal_async",
        "add_done_callback",
    })
    assert not any(
        isinstance(node, ast.Name) and node.id == "ActionClient"
        for node in ast.walk(do_post)
    )


def test_navigation_http_responds_while_action_future_is_unresolved(tmp_path):
    class PendingFutureActions(FakeActions):
        def __init__(self):
            super().__init__()
            self.action_future = Future()

        def send_navigation_goal(self, payload):
            self.calls.append(("send_navigation_goal", payload))
            return "sending"

    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = PendingFutureActions()

    with running_server(actions, html_path) as base_url:
        started = time.monotonic()
        response = post_json(
            base_url,
            "/api/navigation-goal",
            navigation_pose_payload(),
        )
        body = response.read()
        elapsed = time.monotonic() - started

    assert response.status == 202
    assert body == b'{"ok":true,"goal_status":"sending"}'
    assert elapsed < 0.5
    assert not actions.action_future.done()
    assert actions.calls == [
        ("send_navigation_goal", navigation_pose_payload())
    ]


def test_non_json_content_type_is_rejected_without_actions(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")
    actions = FakeActions()
    body = json.dumps(
        manual_payload("session", 1)
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

    with running_server(actions, html_path) as base_url:
        session_id = start_manual_session(base_url)
        actions.mode = "automatic"
        actions.conflict = True
        response = post_json(
            base_url,
            "/api/manual-command",
            manual_payload(session_id, 1),
        )

        assert response.status == 409
        assert response_json(response) == {
            "error": "manual control is not active",
            "accepted": False,
            "sequence": 1,
            "last_sequence": 0,
            "mode": "automatic",
            "linear_x": 0.25,
            "angular_z": -0.1,
            "feedback_fresh": True,
        }
        assert actions.calls == [("manual_command", "stop", 0)]
        assert actions.motion_status_calls == 2


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


def test_create_server_uses_daemon_threading_http_server(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("ok")

    server = create_server(
        FakeActions(), html_path, host="127.0.0.1", port=0
    )
    try:
        assert isinstance(server, HTTPServer)
        assert isinstance(server, ThreadingHTTPServer)
        assert server.daemon_threads is True
    finally:
        server.server_close()
