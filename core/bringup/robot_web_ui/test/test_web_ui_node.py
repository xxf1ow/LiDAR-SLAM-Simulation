import importlib
import sys
import threading
import types

import pytest
import yaml

from robot_web_ui.map_snapshot import BinarySnapshot, GridInfo, GridSnapshot


SYNTHETIC_MAX_LINEAR_SPEED = 1.7
SYNTHETIC_MAX_ANGULAR_SPEED = 2.3


@pytest.fixture
def node_module(monkeypatch):
    class FakeNode:
        def __init__(self, *_args):
            self.declared_parameters = {}
            self.publishers = []
            self.destroyed = False

        def declare_parameter(self, name, _parameter_type):
            values = {
                "max_linear_speed": SYNTHETIC_MAX_LINEAR_SPEED,
                "max_angular_speed": SYNTHETIC_MAX_ANGULAR_SPEED,
                "host": "127.0.0.1",
                "port": 0,
                "map_yaml_path": "map.yaml",
            }
            value = values[name]
            self.declared_parameters[name] = value
            return types.SimpleNamespace(value=value)

        def create_publisher(self, *_args):
            publisher = FakePublisher()
            self.publishers.append(publisher)
            return publisher

        def create_subscription(self, *_args):
            return object()

        def create_client(self, *_args):
            return object()

        def get_clock(self):
            return types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(
                    to_msg=lambda: object(), nanoseconds=0
                )
            )

        def get_logger(self):
            return types.SimpleNamespace(info=lambda _message: None)

        def destroy_node(self):
            self.base_destroy_calls = (
                getattr(self, "base_destroy_calls", 0) + 1
            )

    class FakeParameter:
        class Type:
            DOUBLE = object()
            STRING = object()
            INTEGER = object()

    class FakeTwistStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.twist = types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0),
                angular=types.SimpleNamespace(z=0.0),
            )

    class FakeTrigger:
        class Request:
            pass

    class FakeString:
        def __init__(self, data=""):
            self.data = data

    class FakeOdometry:
        def __init__(self):
            self.twist = types.SimpleNamespace(
                twist=types.SimpleNamespace(
                    linear=types.SimpleNamespace(x=0.0),
                    angular=types.SimpleNamespace(z=0.0),
                )
            )

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_parameter = types.ModuleType("rclpy.parameter")
    rclpy_parameter.Parameter = FakeParameter
    rclpy_qos = types.ModuleType("rclpy.qos")
    class FakeQoSProfile:
        def __init__(self, **kwargs):
            self.settings = kwargs

    rclpy_qos.QoSProfile = FakeQoSProfile
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=object())
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=object())
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(
        TRANSIENT_LOCAL=object()
    )
    ament = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = lambda _name: ""
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.TwistStamped = FakeTwistStamped
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = FakeOdometry
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = FakeTrigger
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = FakeString

    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.parameter": rclpy_parameter,
        "rclpy.qos": rclpy_qos,
        "ament_index_python": ament,
        "ament_index_python.packages": ament_packages,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    module_name = "robot_web_ui.web_ui_node"
    sys.modules.pop(module_name, None)
    module = importlib.import_module(module_name)
    class FakeServer:
        def serve_forever(self):
            return None

        def shutdown(self):
            return None

        def server_close(self):
            return None

    monkeypatch.setattr(
        module, "create_server", lambda *args, **kwargs: FakeServer()
    )
    yield module
    sys.modules.pop(module_name, None)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeFuture:
    def __init__(
        self,
        *,
        response=None,
        error=None,
        complete=True,
        delay=0.0,
    ):
        self.response = response
        self.error = error
        self.complete = complete
        self.delay = delay

    def add_done_callback(self, callback):
        if not self.complete:
            return
        if self.delay:
            timer = threading.Timer(self.delay, callback, args=(self,))
            timer.start()
            return
        callback(self)

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response


class FakeClient:
    def __init__(self, future, *, ready=True):
        self.future = future
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def response(success=True, message=""):
    return types.SimpleNamespace(success=success, message=message)


def bare_node(module):
    return object.__new__(module.WebUiNode)


def _static_snapshot():
    info = GridInfo(2, 1, 0.05, -1.0, 2.0, 0.25, "map")
    binary = BinarySnapshot(
        3, '"etag-static"', "application/octet-stream", b"\x00\x64", b"gzip"
    )
    return GridSnapshot(info, binary)


def test_static_map_load_success_exposes_state_and_asset(node_module, monkeypatch):
    snapshot = _static_snapshot()
    monkeypatch.setattr(node_module, "load_nav2_pgm", lambda _path: snapshot)

    node = node_module.WebUiNode()

    assert node.navigation_state() == {
        "map_error": None,
        "localized": False,
        "layers": {
            "static": {
                **snapshot.info.as_dict(),
                "revision": snapshot.binary.revision,
                "etag": snapshot.binary.etag,
            },
            "global_costmap": None,
            "local_costmap": None,
            "path": None,
        },
    }
    assert node.navigation_asset("static") is snapshot.binary
    node.destroy_node()


@pytest.mark.parametrize(
    "loader_error",
    [
        OSError("missing"),
        TypeError("bad type"),
        ValueError("bad map"),
        yaml.YAMLError("bad yaml"),
    ],
)
def test_static_map_load_failure_is_reported_without_breaking_manual_controls(
    node_module, monkeypatch, loader_error
):
    def fail(_path):
        raise loader_error

    monkeypatch.setattr(node_module, "load_nav2_pgm", fail)
    node = node_module.WebUiNode()
    node._gate_mode = "automatic"

    result = node.manual_command("stop", 20)

    assert result == "automatic"
    message = node._manual_publisher.messages[-1]
    assert message.header.stamp is not None
    assert message.header.frame_id == "base_link"
    assert message.twist.linear.x == 0.0
    assert message.twist.angular.z == 0.0
    assert node._static_map is None
    assert loader_error.__class__.__name__ in node._map_error
    node.destroy_node()


def test_navigation_asset_rejects_unknown_names(node_module):
    node = bare_node(node_module)
    node._static_map = None

    with pytest.raises(KeyError):
        node.navigation_asset("unknown")


def set_clock(node, *seconds):
    readings = iter(seconds)
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(
            nanoseconds=int(next(readings) * 1_000_000_000)
        )
    )


def test_manual_command_publishes_one_stamped_base_link_message(node_module):
    node = bare_node(node_module)
    publisher = FakePublisher()
    stamp = object()
    node._max_linear = SYNTHETIC_MAX_LINEAR_SPEED
    node._max_angular = SYNTHETIC_MAX_ANGULAR_SPEED
    node._gate_mode = "manual"
    node._manual_publisher = publisher
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: stamp)
    )

    assert node.manual_command("forward", 20) == "manual"

    assert len(publisher.messages) == 1
    message = publisher.messages[0]
    assert message.header.stamp is stamp
    assert message.header.frame_id == "base_link"
    assert message.twist.linear.x == pytest.approx(
        SYNTHETIC_MAX_LINEAR_SPEED * 20 / 100.0
    )
    assert message.twist.angular.z == 0.0


def test_nonzero_manual_command_is_rejected_outside_manual_mode(node_module):
    node = bare_node(node_module)
    publisher = FakePublisher()
    node._max_linear = SYNTHETIC_MAX_LINEAR_SPEED
    node._max_angular = SYNTHETIC_MAX_ANGULAR_SPEED
    node._gate_mode = "automatic"
    node._manual_publisher = publisher

    with pytest.raises(
        node_module.ActionConflict,
        match="manual control is not active",
    ) as raised:
        node.manual_command("forward", 20)

    assert raised.value.mode == "automatic"
    assert publisher.messages == []


def test_zero_manual_command_remains_a_safe_noop_outside_manual_mode(
    node_module,
):
    node = bare_node(node_module)
    publisher = FakePublisher()
    node._max_linear = SYNTHETIC_MAX_LINEAR_SPEED
    node._max_angular = SYNTHETIC_MAX_ANGULAR_SPEED
    node._gate_mode = "automatic"
    node._manual_publisher = publisher
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: object())
    )

    assert node.manual_command("stop", 20) == "automatic"

    assert len(publisher.messages) == 1
    assert publisher.messages[0].twist.linear.x == 0.0
    assert publisher.messages[0].twist.angular.z == 0.0


def test_gate_mode_callback_tracks_authoritative_mode(node_module):
    node = bare_node(node_module)
    node._gate_mode = None

    node._gate_mode_callback(types.SimpleNamespace(data="manual"))

    assert node._gate_mode == "manual"


def test_motion_status_is_absent_before_odometry_feedback(node_module):
    node = bare_node(node_module)
    node._odom_feedback = None

    assert node.motion_status() == {
        "linear_x": None,
        "angular_z": None,
        "feedback_fresh": False,
    }


def test_motion_status_returns_fresh_odometry_snapshot(
    node_module,
):
    node = bare_node(node_module)
    set_clock(node, 10.0, 10.49)
    message = node_module.Odometry()
    message.twist.twist.linear.x = 0.3
    message.twist.twist.angular.z = -0.2

    node._odom_callback(message)

    assert node._odom_feedback == (0.3, -0.2, 10.0)
    assert node.motion_status() == {
        "linear_x": 0.3,
        "angular_z": -0.2,
        "feedback_fresh": True,
    }


def test_motion_status_expires_at_timeout_boundary(
    node_module,
):
    node = bare_node(node_module)
    set_clock(node, 10.0, 10.5)
    message = node_module.Odometry()
    message.twist.twist.linear.x = 0.3
    message.twist.twist.angular.z = -0.2

    node._odom_callback(message)

    assert node.motion_status() == {
        "linear_x": None,
        "angular_z": None,
        "feedback_fresh": False,
    }


@pytest.mark.parametrize("reported_mode", ["", "stopped", "MANUAL"])
def test_gate_mode_callback_ignores_non_stable_modes(
    node_module, reported_mode
):
    node = bare_node(node_module)
    node._gate_mode = "automatic"

    node._gate_mode_callback(types.SimpleNamespace(data=reported_mode))

    assert node._gate_mode == "automatic"


def test_mode_service_rejects_unavailable_client(node_module):
    node = bare_node(node_module)
    client = FakeClient(FakeFuture(), ready=False)
    node._takeover_client = client

    with pytest.raises(node_module.ActionUnavailable, match="unavailable"):
        node.takeover_manual()

    assert client.requests == []


def test_mode_service_propagates_success_false(node_module):
    node = bare_node(node_module)
    client = FakeClient(
        FakeFuture(response=response(False, "gate denied"))
    )
    node._takeover_client = client

    with pytest.raises(node_module.ActionUnavailable, match="gate denied"):
        node.takeover_manual()

    assert len(client.requests) == 1


def test_mode_service_rejects_none_response(node_module):
    node = bare_node(node_module)
    node._gate_mode = "automatic"
    node._takeover_client = FakeClient(FakeFuture(response=None))

    with pytest.raises(
        node_module.ActionUnavailable,
        match="manual takeover service returned no response",
    ):
        node.takeover_manual()


def test_future_exception_is_success_when_target_is_observed(node_module):
    node = bare_node(node_module)
    node._gate_mode = "manual"
    node._takeover_client = FakeClient(
        FakeFuture(error=RuntimeError("transport failed"))
    )

    assert node.takeover_manual() == "manual"
    assert len(node._takeover_client.requests) == 1


def test_future_exception_is_pending_when_target_is_not_observed(
    node_module,
):
    node = bare_node(node_module)
    node._gate_mode = "automatic"
    node._takeover_client = FakeClient(
        FakeFuture(error=RuntimeError("transport failed"))
    )

    with pytest.raises(
        node_module.ActionPending,
        match="transport failed",
    ) as raised:
        node.takeover_manual()

    assert raised.value.mode == "automatic"
    assert len(node._takeover_client.requests) == 1


def test_timeout_is_success_when_target_is_observed(
    node_module, monkeypatch
):
    waits = []

    class TimeoutEvent:
        def set(self):
            pass

        def wait(self, timeout):
            waits.append(timeout)
            return False

    monkeypatch.setattr(node_module.threading, "Event", TimeoutEvent)
    node = bare_node(node_module)
    node._gate_mode = "manual"
    node._takeover_client = FakeClient(FakeFuture(complete=False))

    assert node.takeover_manual() == "manual"
    assert waits == [1.0]
    assert len(node._takeover_client.requests) == 1


def test_timeout_is_pending_when_target_is_not_observed(
    node_module, monkeypatch
):
    waits = []

    class TimeoutEvent:
        def set(self):
            pass

        def wait(self, timeout):
            waits.append(timeout)
            return False

    monkeypatch.setattr(node_module.threading, "Event", TimeoutEvent)
    node = bare_node(node_module)
    node._gate_mode = None
    node._takeover_client = FakeClient(FakeFuture(complete=False))

    with pytest.raises(
        node_module.ActionPending,
        match="timed out",
    ) as raised:
        node.takeover_manual()

    assert waits == [1.0]
    assert raised.value.mode is None
    assert len(node._takeover_client.requests) == 1


def test_mode_service_accepts_successful_async_completion(node_module):
    node = bare_node(node_module)
    node._gate_mode = None
    client = FakeClient(
        FakeFuture(response=response(), delay=0.01)
    )
    node._resume_client = client

    assert node.resume_automatic() == "automatic"

    assert len(client.requests) == 1


def test_successful_mode_services_update_local_observation(node_module):
    node = bare_node(node_module)
    node._gate_mode = "automatic"
    node._takeover_client = FakeClient(FakeFuture(response=response()))
    node._resume_client = FakeClient(FakeFuture(response=response()))

    assert node.takeover_manual() == "manual"
    assert node._gate_mode == "manual"

    assert node.resume_automatic() == "automatic"
    assert node._gate_mode == "automatic"


def test_destroy_cleans_up_once_and_is_idempotent(node_module):
    class FakeServer:
        def __init__(self):
            self.shutdown_calls = 0
            self.close_calls = 0

        def shutdown(self):
            self.shutdown_calls += 1

        def server_close(self):
            self.close_calls += 1

    class FakeThread:
        def __init__(self):
            self.alive_checks = 0
            self.join_timeouts = []

        def is_alive(self):
            self.alive_checks += 1
            return True

        def join(self, timeout):
            self.join_timeouts.append(timeout)

    node = bare_node(node_module)
    server = FakeServer()
    thread = FakeThread()
    node._destroy_started = False
    node._http_server = server
    node._http_thread = thread

    node.destroy_node()
    node.destroy_node()

    assert server.shutdown_calls == 1
    assert server.close_calls == 1
    assert thread.alive_checks == 1
    assert thread.join_timeouts == [1.0]
    assert node.base_destroy_calls == 1
