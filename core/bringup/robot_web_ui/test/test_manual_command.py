import pytest

from robot_web_ui.manual_command import command_values


SYNTHETIC_MAX_LINEAR_SPEED = 1.7
SYNTHETIC_MAX_ANGULAR_SPEED = 2.3


def test_forward_scales_linear_only():
    percent = 20
    assert command_values(
        "forward", percent, SYNTHETIC_MAX_LINEAR_SPEED,
        SYNTHETIC_MAX_ANGULAR_SPEED,
    ) == pytest.approx(
        (SYNTHETIC_MAX_LINEAR_SPEED * percent / 100.0, 0.0)
    )


def test_left_scales_angular_only():
    percent = 25
    assert command_values(
        "left", percent, SYNTHETIC_MAX_LINEAR_SPEED,
        SYNTHETIC_MAX_ANGULAR_SPEED,
    ) == pytest.approx(
        (0.0, SYNTHETIC_MAX_ANGULAR_SPEED * percent / 100.0)
    )


def test_reverse_and_right_are_negative():
    assert command_values(
        "backward", 100, SYNTHETIC_MAX_LINEAR_SPEED,
        SYNTHETIC_MAX_ANGULAR_SPEED,
    ) == pytest.approx(
        (-SYNTHETIC_MAX_LINEAR_SPEED, 0.0)
    )
    assert command_values(
        "right", 100, SYNTHETIC_MAX_LINEAR_SPEED,
        SYNTHETIC_MAX_ANGULAR_SPEED,
    ) == pytest.approx(
        (0.0, -SYNTHETIC_MAX_ANGULAR_SPEED)
    )


def test_stop_is_exact_zero():
    assert command_values(
        "stop", 100, SYNTHETIC_MAX_LINEAR_SPEED, SYNTHETIC_MAX_ANGULAR_SPEED
    ) == (0.0, 0.0)


def test_rejects_invalid_direction_or_percentage():
    with pytest.raises(ValueError):
        command_values(
            "diagonal", 20, SYNTHETIC_MAX_LINEAR_SPEED,
            SYNTHETIC_MAX_ANGULAR_SPEED,
        )
    with pytest.raises(ValueError):
        command_values(
            "forward", 101, SYNTHETIC_MAX_LINEAR_SPEED,
            SYNTHETIC_MAX_ANGULAR_SPEED,
        )
    with pytest.raises(ValueError):
        command_values(
            "forward", float("nan"), SYNTHETIC_MAX_LINEAR_SPEED,
            SYNTHETIC_MAX_ANGULAR_SPEED,
        )
