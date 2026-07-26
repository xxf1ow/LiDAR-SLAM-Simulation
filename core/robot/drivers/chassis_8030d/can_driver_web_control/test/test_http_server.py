import json
import threading
import urllib.error
import urllib.request

import pytest

from can_driver_web_control.control_state import ControlState
from can_driver_web_control.http_server import create_server


@pytest.fixture
def running_server(tmp_path):
    html_path = tmp_path / "index.html"
    html_path.write_text("<h1>8030D control</h1>", encoding="utf-8")
    state = ControlState()
    server = create_server(state, html_path, host="127.0.0.1", port=0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base_url = f"http://127.0.0.1:{server.server_address[1]}"
    yield state, base_url
    server.shutdown()
    server.server_close()
    thread.join(timeout=1.0)


def request_json(url, method="GET", body=None):
    data = None if body is None else json.dumps(body).encode("utf-8")
    request = urllib.request.Request(
        url,
        data=data,
        method=method,
        headers={"Content-Type": "application/json"},
    )
    with urllib.request.urlopen(request, timeout=1.0) as response:
        return response.status, json.loads(response.read())


def test_root_serves_packaged_page(running_server):
    _, base_url = running_server
    with urllib.request.urlopen(base_url + "/", timeout=1.0) as response:
        assert response.status == 200
        assert b"8030D control" in response.read()


def test_enable_command_and_status_round_trip(running_server):
    state, base_url = running_server
    assert request_json(
        base_url + "/api/driver", "POST", {"enabled": True}
    )[0] == 200
    assert request_json(
        base_url + "/api/command",
        "POST",
        {"direction": "left", "speed_rpm": 20},
    )[0] == 200
    _, status = request_json(base_url + "/api/status")
    assert status["enabled"] is True
    assert status["enable_confirmed"] is None
    assert status["command"] == [20, -20]
    assert state.safe_command() == (20, -20)


def test_disabled_motion_returns_409_without_mutation(running_server):
    state, base_url = running_server
    request = urllib.request.Request(
        base_url + "/api/command",
        data=json.dumps({"direction": "forward", "speed_rpm": 20}).encode(),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request, timeout=1.0)
    assert error.value.code == 409
    assert state.snapshot()["direction"] == "stop"


@pytest.mark.parametrize(
    "body",
    [
        {"direction": "diagonal", "speed_rpm": 20},
        {"direction": [], "speed_rpm": 20},
        {"direction": "forward", "speed_rpm": 101},
        {"direction": "forward", "speed_rpm": "20"},
    ],
)
def test_invalid_command_returns_400(running_server, body):
    state, base_url = running_server
    request_json(base_url + "/api/driver", "POST", {"enabled": True})
    before = state.snapshot()
    request = urllib.request.Request(
        base_url + "/api/command",
        data=json.dumps(body).encode(),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request, timeout=1.0)
    assert error.value.code == 400
    assert state.snapshot() == before


def test_unknown_route_returns_404(running_server):
    _, base_url = running_server
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(base_url + "/missing", timeout=1.0)
    assert error.value.code == 404


@pytest.mark.parametrize("body", [b"", b"{"])
def test_unknown_post_route_returns_404_regardless_of_body(running_server, body):
    _, base_url = running_server
    request = urllib.request.Request(
        base_url + "/missing",
        data=body,
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request, timeout=1.0)
    assert error.value.code == 404


def test_malformed_json_returns_400(running_server):
    state, base_url = running_server
    before = state.snapshot()
    request = urllib.request.Request(
        base_url + "/api/driver",
        data=b"{",
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request, timeout=1.0)
    assert error.value.code == 400
    assert state.snapshot() == before


def test_invalid_driver_value_returns_400_without_mutation(running_server):
    state, base_url = running_server
    before = state.snapshot()
    request = urllib.request.Request(
        base_url + "/api/driver",
        data=json.dumps({"enabled": "true"}).encode(),
        method="POST",
        headers={"Content-Type": "application/json"},
    )
    with pytest.raises(urllib.error.HTTPError) as error:
        urllib.request.urlopen(request, timeout=1.0)
    assert error.value.code == 400
    assert state.snapshot() == before
