import importlib.util
import sys
import types
from pathlib import Path
from unittest.mock import patch


FAKE_PATH = Path(__file__).resolve().parent / "fake_8030d_node.py"


class StubExternalShutdownException(Exception):
    pass


class StubNode:
    destroy_calls = 0

    def __init__(self, name):
        self.name = name

    def create_subscription(self, message_type, topic, callback, qos):
        return (message_type, topic, callback, qos)

    def create_publisher(self, message_type, topic, qos):
        return (message_type, topic, qos)

    def create_timer(self, period, callback):
        return (period, callback)

    def destroy_node(self):
        StubNode.destroy_calls += 1


def load_fake_module(spin_error):
    calls = {"init": 0, "try_shutdown": 0}
    StubNode.destroy_calls = 0

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: calls.__setitem__("init", calls["init"] + 1)
    rclpy.spin = lambda node: (_ for _ in ()).throw(spin_error)
    rclpy.shutdown = lambda: None
    rclpy.try_shutdown = lambda: calls.__setitem__(
        "try_shutdown", calls["try_shutdown"] + 1
    )

    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = StubNode
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_executors.ExternalShutdownException = StubExternalShutdownException

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.Int16MultiArray = type("Int16MultiArray", (), {})
    std_msgs_msg.Int8 = type("Int8", (), {})

    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.executors": rclpy_executors,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location("fake_8030d_node", FAKE_PATH)
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        spec.loader.exec_module(module)
    return module, calls


def run_main(spin_error):
    module, calls = load_fake_module(spin_error)
    escaped = None
    try:
        module.main()
    except BaseException as error:
        escaped = error
    return escaped, calls


def test_main_handles_keyboard_interrupt_as_clean_shutdown():
    escaped, calls = run_main(KeyboardInterrupt())
    assert escaped is None
    assert calls == {"init": 1, "try_shutdown": 1}
    assert StubNode.destroy_calls == 1


def test_main_handles_external_shutdown_as_clean_shutdown():
    escaped, calls = run_main(StubExternalShutdownException())
    assert escaped is None
    assert calls == {"init": 1, "try_shutdown": 1}
    assert StubNode.destroy_calls == 1
