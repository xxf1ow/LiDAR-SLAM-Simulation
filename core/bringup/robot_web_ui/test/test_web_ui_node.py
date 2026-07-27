import importlib
import sys
import threading
import types

import pytest


@pytest.fixture
def node_module(monkeypatch):
    class FakeNode:
        def destroy_node(self):
            self.base_destroy_calls = (
                getattr(self, "base_destroy_calls", 0) + 1
            )

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

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    ament = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = lambda _name: ""
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.TwistStamped = FakeTwistStamped
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = FakeTrigger

    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "ament_index_python": ament,
        "ament_index_python.packages": ament_packages,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    module_name = "robot_web_ui.web_ui_node"
    sys.modules.pop(module_name, None)
    module = importlib.import_module(module_name)
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


def test_manual_command_publishes_one_stamped_base_link_message(node_module):
    node = bare_node(node_module)
    publisher = FakePublisher()
    stamp = object()
    node._max_linear = 1.5
    node._max_angular = 2.0
    node._manual_publisher = publisher
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: stamp)
    )

    node.manual_command("forward", 20)

    assert len(publisher.messages) == 1
    message = publisher.messages[0]
    assert message.header.stamp is stamp
    assert message.header.frame_id == "base_link"
    assert message.twist.linear.x == pytest.approx(0.3)
    assert message.twist.angular.z == 0.0


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


def test_mode_service_propagates_future_exception(node_module):
    node = bare_node(node_module)
    client = FakeClient(
        FakeFuture(error=RuntimeError("transport failed"))
    )
    node._takeover_client = client

    with pytest.raises(
        node_module.ActionUnavailable,
        match="transport failed",
    ):
        node.takeover_manual()

    assert len(client.requests) == 1


def test_mode_service_waits_at_most_one_second(node_module, monkeypatch):
    waits = []

    class TimeoutEvent:
        def set(self):
            pass

        def wait(self, timeout):
            waits.append(timeout)
            return False

    monkeypatch.setattr(node_module.threading, "Event", TimeoutEvent)
    node = bare_node(node_module)
    client = FakeClient(FakeFuture(complete=False))
    node._takeover_client = client

    with pytest.raises(node_module.ActionUnavailable, match="timed out"):
        node.takeover_manual()

    assert waits == [1.0]
    assert len(client.requests) == 1


def test_mode_service_accepts_successful_async_completion(node_module):
    node = bare_node(node_module)
    client = FakeClient(
        FakeFuture(response=response(), delay=0.01)
    )
    node._resume_client = client

    node.resume_automatic()

    assert len(client.requests) == 1


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
