import pytest

from robot_web_ui.manual_command import command_values


def test_forward_scales_linear_only():
    assert command_values("forward", 20, 1.5, 2.0) == pytest.approx((0.3, 0.0))


def test_left_scales_angular_only():
    assert command_values("left", 25, 1.5, 2.0) == pytest.approx((0.0, 0.5))


def test_reverse_and_right_are_negative():
    assert command_values("backward", 100, 1.5, 2.0) == pytest.approx(
        (-1.5, 0.0)
    )
    assert command_values("right", 100, 1.5, 2.0) == pytest.approx((0.0, -2.0))


def test_stop_is_exact_zero():
    assert command_values("stop", 100, 1.5, 2.0) == (0.0, 0.0)


def test_rejects_invalid_direction_or_percentage():
    with pytest.raises(ValueError):
        command_values("diagonal", 20, 1.5, 2.0)
    with pytest.raises(ValueError):
        command_values("forward", 101, 1.5, 2.0)
    with pytest.raises(ValueError):
        command_values("forward", float("nan"), 1.5, 2.0)
