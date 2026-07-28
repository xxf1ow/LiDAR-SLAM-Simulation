import importlib
import sys
from types import ModuleType
from unittest.mock import Mock

import pytest


def _load_gate_node(monkeypatch):
    rclpy = ModuleType("rclpy")
    rclpy.init = Mock()
    rclpy.spin = Mock()
    rclpy.shutdown = Mock()
    rclpy.try_shutdown = Mock()

    executors = ModuleType("rclpy.executors")

    class ExternalShutdownException(Exception):
        pass

    executors.ExternalShutdownException = ExternalShutdownException
    rclpy.executors = executors

    node = ModuleType("rclpy.node")
    node.Node = object

    qos = ModuleType("rclpy.qos")
    qos.DurabilityPolicy = object
    qos.HistoryPolicy = object
    qos.QoSProfile = object
    qos.ReliabilityPolicy = object

    geometry_msgs = ModuleType("geometry_msgs")
    geometry_msgs_msg = ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.TwistStamped = object
    geometry_msgs.msg = geometry_msgs_msg

    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.String = object
    std_msgs.msg = std_msgs_msg

    std_srvs = ModuleType("std_srvs")
    std_srvs_srv = ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = object
    std_srvs.srv = std_srvs_srv

    for name, module in {
        "rclpy": rclpy,
        "rclpy.executors": executors,
        "rclpy.node": node,
        "rclpy.qos": qos,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    monkeypatch.delitem(sys.modules, "cmd_vel_gate.gate_node", raising=False)
    return importlib.import_module("cmd_vel_gate.gate_node"), rclpy


@pytest.mark.parametrize(
    "exception_name",
    ["keyboard_interrupt", "external_shutdown"],
)
def test_main_cleans_up_after_shutdown_exception(monkeypatch, exception_name):
    gate_node, rclpy = _load_gate_node(monkeypatch)
    exception = (
        KeyboardInterrupt()
        if exception_name == "keyboard_interrupt"
        else gate_node.rclpy.executors.ExternalShutdownException()
    )
    rclpy.spin.side_effect = exception
    node = Mock()
    monkeypatch.setattr(gate_node, "CmdVelGate", Mock(return_value=node))

    escaped = None
    try:
        gate_node.main()
    except BaseException as exception:
        escaped = exception

    node.destroy_node.assert_called_once_with()
    rclpy.try_shutdown.assert_called_once_with()
    rclpy.shutdown.assert_not_called()
    assert escaped is None
