import importlib.util
import inspect
from pathlib import Path
import sys
import time
import types

import pytest


ROOT = Path(__file__).resolve().parents[4]
READY_GATE = (
    ROOT / "core/bringup/system_bringup/system_bringup/ready_gate.py"
)


class _Action:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


def _load_ready_gate(monkeypatch):
    launch = types.ModuleType("launch")
    actions = types.ModuleType("launch.actions")
    handlers = types.ModuleType("launch.event_handlers")
    actions.ExecuteProcess = _Action
    actions.OpaqueFunction = _Action
    actions.RegisterEventHandler = _Action
    handlers.OnProcessExit = _Action
    monkeypatch.setitem(sys.modules, "launch", launch)
    monkeypatch.setitem(sys.modules, "launch.actions", actions)
    monkeypatch.setitem(sys.modules, "launch.event_handlers", handlers)

    spec = importlib.util.spec_from_file_location("ready_gate_under_test", READY_GATE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class _RosTime:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class _Clock:
    def __init__(self, values):
        values = iter(values)
        self._current = next(values, 0)
        self._pending = values

    def now(self):
        return _RosTime(self._current)

    def advance(self):
        self._current = next(self._pending, self._current)


def _run_script(monkeypatch, script, clock_values, topics):
    clock = _Clock(clock_values)

    class _Node:
        def __init__(self, name):
            assert name == "ready_gate"

        def get_topic_names_and_types(self):
            return [(topic, []) for topic in topics]

        def get_clock(self):
            return clock

    rclpy = types.ModuleType("rclpy")
    rclpy.init = lambda: None
    spins = []

    def _spin_once(node, timeout_sec):
        spins.append(timeout_sec)
        clock.advance()

    rclpy.spin_once = _spin_once
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = _Node
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.node", rclpy_node)

    ticks = iter(range(100))
    monkeypatch.setattr(time, "monotonic", lambda: float(next(ticks)))
    monkeypatch.setattr(time, "sleep", lambda duration: None)

    with pytest.raises(SystemExit) as exc_info:
        exec(compile(script, "<ready_gate>", "exec"), {})
    return exc_info.value.code, len(spins)


@pytest.mark.parametrize(
    ("use_sim_time", "expected"),
    [(True, "true"), (False, "false"), ("true", "true"), ("false", "false")],
)
def test_ready_gate_requires_and_forwards_one_clock_value(
    monkeypatch, use_sim_time, expected
):
    module = _load_ready_gate(monkeypatch)
    signature = inspect.signature(module.ready_gate)
    assert signature.parameters["use_sim_time"].default is inspect.Parameter.empty

    waiter, _ = module.ready_gate(
        ["/points_raw"],
        300.0,
        "simulation",
        [object()],
        use_sim_time=use_sim_time,
    )
    assert waiter.kwargs["cmd"][-3:] == [
        "--ros-args",
        "-p",
        f"use_sim_time:={expected}",
    ]


def test_generated_script_uses_wall_supervision_and_ros_time_settling(monkeypatch):
    module = _load_ready_gate(monkeypatch)
    script = module._gate_script(["/ready"], timeout=60.0, settling=3.0)

    assert "DISCOVERY_DEADLINE = time.monotonic() + TIMEOUT" in script
    assert "SETTLING_WALL_DEADLINE = time.monotonic() + TIMEOUT" in script
    assert "SETTLING_START = n.get_clock().now().nanoseconds" in script
    assert "n.get_clock().now().nanoseconds - SETTLING_START" in script
    assert "rclpy.spin_once(n, timeout_sec=0.1)" in script
    assert "time.sleep(SETTLING)" not in script
    assert not any(word in script.lower() for word in ("platform", "backend", "mode"))


def test_generated_script_releases_after_ros_time_settling(monkeypatch):
    module = _load_ready_gate(monkeypatch)
    script = module._gate_script(["/ready"], timeout=10.0, settling=1.0)

    result, _ = _run_script(
        monkeypatch,
        script,
        clock_values=[0, 500_000_000, 1_000_000_000, 1_500_000_000],
        topics=["/ready"],
    )
    assert result == 0


def test_generated_script_does_not_count_queued_sim_uptime_as_settling(monkeypatch):
    module = _load_ready_gate(monkeypatch)
    script = module._gate_script(["/ready"], timeout=10.0, settling=1.0)

    result, spins = _run_script(
        monkeypatch,
        script,
        clock_values=[
            0,
            100_000_000_000,
            100_500_000_000,
            101_000_000_000,
        ],
        topics=["/ready"],
    )

    assert result == 0
    assert spins == 3


def test_generated_script_fails_when_ros_clock_is_frozen(monkeypatch):
    module = _load_ready_gate(monkeypatch)
    script = module._gate_script(["/ready"], timeout=3.0, settling=1.0)

    result, _ = _run_script(
        monkeypatch,
        script,
        clock_values=[0],
        topics=["/ready"],
    )
    assert result == 1
