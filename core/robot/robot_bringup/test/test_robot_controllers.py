from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config" / "robot_controllers.yaml"


def test_base_controller_uses_measured_velocity_feedback_without_odom_tf():
    config = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
    params = config["base_controller"]["ros__parameters"]

    assert params["left_wheel_names"] == ["left_wheel_joint"]
    assert params["right_wheel_names"] == ["right_wheel_joint"]
    assert params["open_loop"] is False
    assert params["position_feedback"] is False
    assert params["enable_odom_tf"] is False
    assert params["cmd_vel_timeout"] == 0.5
    assert "use_stamped_vel" not in params
    assert "velocity_rolling_window_size" not in params
