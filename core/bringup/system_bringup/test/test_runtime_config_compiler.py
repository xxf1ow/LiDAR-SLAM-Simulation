from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
TEMPLATE_DIR = CONFIG_DIR / "templates"


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_shared_templates_are_complete_mappings():
    for name in ("robot_controllers.yaml", "robot_web_ui.yaml", "nav2.yaml"):
        data = _load_yaml(TEMPLATE_DIR / name)
        assert isinstance(data, dict)
        assert data


def test_controller_template_contains_all_owned_target_leaves():
    data = _load_yaml(TEMPLATE_DIR / "robot_controllers.yaml")
    manager = data["controller_manager"]["ros__parameters"]
    base = data["base_controller"]["ros__parameters"]
    assert isinstance(manager["use_sim_time"], bool)
    assert isinstance(base["use_sim_time"], bool)
    assert base["linear.x.min_acceleration"] == -1.0
    assert base["linear.x.has_acceleration_limits"] is True
    assert base["angular.z.has_acceleration_limits"] is True
    assert base["linear.x.has_jerk_limits"] is False
    assert base["angular.z.has_jerk_limits"] is False


def test_web_ui_template_is_a_complete_native_parameter_file():
    assert _load_yaml(TEMPLATE_DIR / "robot_web_ui.yaml") == {
        "robot_web_ui": {
            "ros__parameters": {
                "use_sim_time": True,
                "max_linear_speed": 1.5,
                "max_angular_speed": 2.0,
                "host": "0.0.0.0",
                "port": 8080,
            }
        }
    }


def test_nav2_template_preserves_complete_current_sim_baseline():
    source = _load_yaml(
        PACKAGE_ROOT.parents[1]
        / "navigation/robot_navigation/config/nav2_params.yaml"
    )
    template = _load_yaml(TEMPLATE_DIR / "nav2.yaml")
    assert template == source
    assert set(template) == {
        "map_server",
        "planner_server",
        "controller_server",
        "global_costmap",
        "local_costmap",
        "behavior_server",
        "bt_navigator",
    }
