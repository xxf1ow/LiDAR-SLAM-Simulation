import pytest

from can_driver_web_control.control_state import (
    ControlConflict,
    ControlError,
    ControlState,
)


class FakeClock:
    def __init__(self):
        self.now = 10.0

    def __call__(self):
        return self.now


@pytest.mark.parametrize(
    ("direction", "expected"),
    [
        ("forward", (20, 20)),
        ("backward", (-20, -20)),
        # /motor_speed is ordered as [right, left].
        ("left", (20, -20)),
        ("right", (-20, 20)),
        ("stop", (0, 0)),
    ],
)
def test_direction_mapping(direction, expected):
    state = ControlState()
    state.set_enabled(True)
    state.set_command(direction, 20)
    assert state.safe_command() == expected


def test_movement_is_rejected_while_disabled_without_mutation():
    state = ControlState()
    before = state.snapshot()
    with pytest.raises(ControlConflict):
        state.set_command("forward", 20)
    assert state.snapshot() == before


@pytest.mark.parametrize("speed", [-1, 101, 1.5, True, "20"])
def test_invalid_speed_is_rejected_without_replacing_valid_command(speed):
    state = ControlState()
    state.set_enabled(True)
    state.set_command("forward", 20)
    with pytest.raises(ControlError):
        state.set_command("forward", speed)
    assert state.safe_command() == (20, 20)


def test_watchdog_stops_after_300_ms():
    clock = FakeClock()
    state = ControlState(clock=clock)
    state.set_enabled(True)
    state.set_command("forward", 20)
    clock.now += 0.3001
    assert state.safe_command() == (0, 0)
    assert state.snapshot()["direction"] == "stop"


def test_disabling_clears_motion_and_reenable_stays_stopped():
    state = ControlState()
    state.set_enabled(True)
    state.set_command("forward", 20)
    state.set_enabled(False)
    state.set_enabled(True)
    assert state.safe_command() == (0, 0)
    assert state.snapshot()["direction"] == "stop"


def test_snapshot_contains_raw_feedback_and_driver_connection():
    state = ControlState()
    state.update_feedback([123, -456, 999])
    state.set_driver_connected(True)
    snapshot = state.snapshot()
    assert snapshot["feedback"] == [123, -456]
    assert snapshot["driver_connected"] is True
    assert snapshot["hardware_feedback"] is True
    assert snapshot["enable_confirmed"] is None


def test_hardware_feedback_expires_independently_from_ros_driver_connection():
    clock = FakeClock()
    state = ControlState(clock=clock)
    state.set_driver_connected(True)
    assert state.snapshot()["driver_connected"] is True
    assert state.snapshot()["hardware_feedback"] is False

    state.update_feedback([0, 0])
    assert state.snapshot()["hardware_feedback"] is True

    clock.now += 1.01
    assert state.snapshot()["driver_connected"] is True
    assert state.snapshot()["hardware_feedback"] is False


def test_short_feedback_is_rejected():
    state = ControlState()
    with pytest.raises(ControlError):
        state.update_feedback([1])
