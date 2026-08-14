import importlib
import sys
import types
from pathlib import Path

import pytest


ROOT = Path(__file__).parents[1]
NODE_PATH = ROOT / "system_bringup" / "sensor_gate_node.py"


class FakeClock:
    def __init__(self, seconds):
        self.seconds = seconds

    def now(self):
        return types.SimpleNamespace(
            nanoseconds=int(self.seconds * 1_000_000_000)
        )


class FakeTimer:
    def __init__(self):
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


@pytest.fixture
def node_module(monkeypatch):
    clock = FakeClock(10.0)
    generated = {
        "expected_points_per_scan": 17 * 1700,
        "expected_point_hz": 37.5,
        "expected_imu_hz": 275.0,
        "minimum_point_rate_ratio": 0.64,
        "minimum_imu_rate_ratio": 0.44,
        "max_stamp_age": 0.25,
        "rate_window": 1.5,
        "stable_duration": 1.0,
        "timeout": 42.0,
    }

    class FakeNode:
        def __init__(self, _name):
            self.name = _name
            self._clock = clock
            self._logger = FakeLogger()
            self.declared_parameters = []
            self.subscriptions = []

        def declare_parameter(self, name, parameter_type):
            self.declared_parameters.append((name, parameter_type))
            if name not in generated:
                raise RuntimeError(f"missing required parameter: {name}")
            return types.SimpleNamespace(value=generated[name])

        def create_subscription(self, *args, **kwargs):
            self.subscriptions.append((args, kwargs))
            return types.SimpleNamespace(args=args, kwargs=kwargs)

        def create_timer(self, _period, _callback):
            return FakeTimer()

        def get_clock(self):
            return self._clock

        def get_logger(self):
            return self._logger

    class FakePointCloud2:
        pass

    class FakeImu:
        pass

    rclpy = types.ModuleType("rclpy")

    class FakeParameter:
        class Type:
            INTEGER = object()
            DOUBLE = object()

    rclpy_parameter = types.ModuleType("rclpy.parameter")
    rclpy_parameter.Parameter = FakeParameter
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.qos_profile_sensor_data = object()
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.Imu = FakeImu
    sensor_msgs_msg.PointCloud2 = FakePointCloud2

    for name, module in {
        "rclpy": rclpy,
        "rclpy.parameter": rclpy_parameter,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
    }.items():
        monkeypatch.setitem(sys.modules, name, module)

    module_name = "system_bringup.sensor_gate_node"
    sys.modules.pop(module_name, None)
    module = importlib.import_module(module_name)
    yield module, clock, generated
    sys.modules.pop(module_name, None)


def _stamp(seconds):
    whole = int(seconds)
    return types.SimpleNamespace(
        sec=whole,
        nanosec=int((seconds - whole) * 1_000_000_000),
    )


def _point_message(stamp):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(
            stamp=_stamp(stamp), frame_id="vanjee_lidar"
        ),
        height=16,
        width=1800,
        fields=[
            types.SimpleNamespace(name=name)
            for name in ("x", "y", "z", "intensity", "ring", "time")
        ],
    )


def _imu_message(stamp):
    return types.SimpleNamespace(
        header=types.SimpleNamespace(stamp=_stamp(stamp), frame_id="imu_link")
    )


class RecordingRate:
    def __init__(self, hz):
        self.value = hz
        self.times = []

    def hz(self, now):
        self.times.append(now)
        return self.value


class RecordingState:
    def __init__(self, *, ready=False, reason="waiting"):
        self.ready = ready
        self.reason = reason
        self.point_calls = []
        self.imu_calls = []
        self.status_times = []
        self.point_rate = RecordingRate(12.0)
        self.imu_rate = RecordingRate(180.0)

    def observe_point(self, **kwargs):
        self.point_calls.append(kwargs)

    def observe_imu(self, **kwargs):
        self.imu_calls.append(kwargs)

    def status(self, now):
        self.status_times.append(now)
        return self.ready, self.reason


def test_node_loads_generated_contract_parameters_and_uses_neutral_name(node_module):
    module, _clock, generated = node_module
    node = module.SensorGateNode()

    assert node.name == "sensor_contract_gate"
    assert [name for name, _default in node.declared_parameters] == list(generated)
    assert node.timeout == generated["timeout"]
    assert node.state.expected_points_per_scan == generated["expected_points_per_scan"]
    assert node.state.minimum_point_hz == pytest.approx(
        generated["expected_point_hz"] * generated["minimum_point_rate_ratio"]
    )
    assert node.state.minimum_imu_hz == pytest.approx(
        generated["expected_imu_hz"] * generated["minimum_imu_rate_ratio"]
    )
    assert node.state.max_stamp_age == generated["max_stamp_age"]
    assert node.state.point_rate.window == generated["rate_window"]
    assert node.state.stable_duration == generated["stable_duration"]


def test_node_declares_only_typed_required_generated_parameters(node_module):
    module, _clock, generated = node_module
    node = module.SensorGateNode()

    assert {name for name, _parameter_type in node.declared_parameters} == set(
        generated
    )
    assert (
        dict(node.declared_parameters)["expected_points_per_scan"]
        is module.Parameter.Type.INTEGER
    )
    assert all(
        parameter_type is module.Parameter.Type.DOUBLE
        for name, parameter_type in node.declared_parameters
        if name != "expected_points_per_scan"
    )


def test_node_subscribes_to_the_stable_sensor_protocol_topics(node_module):
    module, _clock, _generated = node_module
    node = module.SensorGateNode()

    assert [args[1] for args, _kwargs in node.subscriptions] == [
        "/points_raw",
        "/imu/data",
    ]


def test_startup_and_samples_use_one_ros_clock_domain(node_module):
    module, clock, _generated = node_module
    node = module.SensorGateNode()

    assert node.started_at == 10.0
    assert node.last_report_at == 10.0
    assert node.timeout == 42.0

    state = RecordingState()
    node.state = state
    clock.seconds = 12.5
    node._on_points(_point_message(12.4))
    node._on_imu(_imu_message(12.45))

    assert state.point_calls[0]["received"] == 12.5
    assert state.point_calls[0]["now_ros"] == 12.5
    assert state.point_calls[0]["stamp"] == pytest.approx(12.4)
    assert state.imu_calls[0]["received"] == 12.5
    assert state.imu_calls[0]["now_ros"] == 12.5
    assert state.imu_calls[0]["stamp"] == pytest.approx(12.45)


def test_status_and_rate_windows_use_the_ros_clock(node_module):
    module, clock, _generated = node_module
    node = module.SensorGateNode()
    state = RecordingState(ready=True, reason="ready")
    node.state = state
    clock.seconds = 14.25

    node._check_status()

    assert state.status_times == [14.25]
    assert state.point_rate.times == [14.25]
    assert state.imu_rate.times == [14.25]
    assert node.exit_code == 0
    assert node.finished
    assert node.timer.cancelled
    assert node.get_logger().messages == [
        "sensor contract ready (point=12.0 Hz, imu=180.0 Hz)"
    ]


def test_reporting_and_final_timeout_use_the_ros_clock(node_module):
    module, clock, _generated = node_module
    reporting_node = module.SensorGateNode()
    reporting_state = RecordingState(reason="still waiting")
    reporting_node.state = reporting_state

    clock.seconds = 14.99
    reporting_node._check_status()
    assert reporting_node.get_logger().messages == []

    clock.seconds = 15.0
    reporting_node._check_status()
    assert reporting_state.status_times == [14.99, 15.0]
    assert reporting_node.last_report_at == 15.0
    assert reporting_node.get_logger().messages == [
        "sensor contract waiting: still waiting"
    ]

    timeout_node = module.SensorGateNode()
    timeout_state = RecordingState(reason="sensor missing")
    timeout_node.state = timeout_state
    clock.seconds = 57.0
    timeout_node._check_status()

    assert timeout_state.status_times == [57.0]
    assert timeout_node.exit_code == 1
    assert timeout_node.finished
    assert timeout_node.timer.cancelled
    assert timeout_node.get_logger().messages == [
        "sensor contract timed out: sensor missing"
    ]


def test_functional_timing_has_no_wall_clock_or_clock_branch():
    source = NODE_PATH.read_text(encoding="utf-8")

    assert "time.monotonic" not in source
    assert "import time" not in source
    assert "platform" not in source
    assert "backend" not in source
    assert "ClockType" not in source
    assert "fallback" not in source.lower()
