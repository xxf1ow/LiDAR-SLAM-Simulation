import shutil
from pathlib import Path

import pytest
import yaml

from system_bringup import profile_compiler as pc
from system_bringup import runtime_config_compiler as rcc


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
TEMPLATE_DIR = CONFIG_DIR / "templates"


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


class _RuntimeTree:
    def __init__(self, config):
        self.config = config

    def set_bringup_value(self, key, value):
        config = _load_yaml(self.config)
        config[key] = value
        self.config.write_text(
            yaml.safe_dump(config, sort_keys=False), encoding="utf-8"
        )

    def set_profile_value(self, platform, path, value):
        profile_path = self.config.parent / "profiles" / f"{platform}.yaml"
        profile = _load_yaml(profile_path)
        target = profile
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value
        profile_path.write_text(
            yaml.safe_dump(profile, sort_keys=False), encoding="utf-8"
        )


@pytest.fixture
def runtime_tree(tmp_path):
    config_dir = tmp_path / "config"
    shutil.copytree(CONFIG_DIR / "profiles", config_dir / "profiles")
    shutil.copytree(CONFIG_DIR / "templates", config_dir / "templates")
    config = config_dir / "bringup.yaml"
    config.write_text(
        yaml.safe_dump(
            {
                "platform": "sim",
                "mode": "navigation",
                "profiles": {
                    "sim": "profiles/sim.yaml",
                    "real": "profiles/real.yaml",
                },
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return _RuntimeTree(config)


@pytest.mark.parametrize("mode", ["mapping", "navigation"])
def test_runtime_accepts_only_supported_modes(runtime_tree, mode):
    runtime_tree.set_bringup_value("mode", mode)

    assert rcc._load_runtime_inputs(runtime_tree.config)["mode"] == mode


@pytest.mark.parametrize("mode", [None, True, 1, "", "localization"])
def test_runtime_rejects_invalid_mode(runtime_tree, mode):
    runtime_tree.set_bringup_value("mode", mode)

    with pytest.raises(ValueError, match="mode"):
        rcc._load_runtime_inputs(runtime_tree.config)


@pytest.mark.parametrize("profile_name", ["sim", "real"])
@pytest.mark.parametrize("key", rcc.MOTION_KEYS)
@pytest.mark.parametrize(
    "value",
    [
        None,
        True,
        False,
        0,
        0.0,
        -0.1,
        float("inf"),
        float("-inf"),
        float("nan"),
    ],
)
def test_runtime_rejects_invalid_motion_in_either_profile(
    runtime_tree, profile_name, key, value
):
    runtime_tree.set_profile_value(profile_name, ("motion", key), value)

    with pytest.raises(ValueError, match=rf"{profile_name}.*motion\.{key}"):
        rcc._load_runtime_inputs(runtime_tree.config)


def test_runtime_reads_bringup_yaml_exactly_once(runtime_tree, monkeypatch):
    calls = []
    original = pc._read_yaml_mapping

    def counted(path, label):
        if label == "bringup config":
            calls.append(Path(path))
        return original(path, label)

    monkeypatch.setattr(pc, "_read_yaml_mapping", counted)

    rcc._load_runtime_inputs(runtime_tree.config)

    assert calls == [runtime_tree.config]


@pytest.mark.parametrize("name", rcc.TEMPLATE_FILENAMES)
def test_runtime_rejects_missing_template(runtime_tree, name):
    path = runtime_tree.config.parent / "templates" / rcc.TEMPLATE_FILENAMES[name]
    path.unlink()

    with pytest.raises(ValueError, match="template file does not exist"):
        rcc._load_template(path, name)


@pytest.mark.parametrize(
    "name,text",
    [
        ("controllers", ""),
        ("web_ui", "- item\n"),
        ("nav2", "not-a-mapping\n"),
    ],
)
def test_runtime_rejects_non_mapping_template(runtime_tree, name, text):
    path = runtime_tree.config.parent / "templates" / rcc.TEMPLATE_FILENAMES[name]
    path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match="template root must be a mapping"):
        rcc._load_template(path, name)


@pytest.mark.parametrize("name", rcc.TEMPLATE_FILENAMES)
def test_runtime_inputs_reject_missing_source_template(runtime_tree, name):
    path = runtime_tree.config.parent / "templates" / rcc.TEMPLATE_FILENAMES[name]
    path.unlink()

    with pytest.raises(ValueError, match="template file does not exist"):
        rcc._load_runtime_inputs(runtime_tree.config)


@pytest.mark.parametrize(
    "name,text",
    [
        ("controllers", ""),
        ("web_ui", "- item\n"),
        ("nav2", "not-a-mapping\n"),
    ],
)
def test_runtime_inputs_reject_non_mapping_source_template(
    runtime_tree, name, text
):
    path = runtime_tree.config.parent / "templates" / rcc.TEMPLATE_FILENAMES[name]
    path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match="template root must be a mapping"):
        rcc._load_runtime_inputs(runtime_tree.config)


def test_runtime_inputs_return_validated_template_mappings(runtime_tree):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    assert inputs["templates"] == {
        name: _load_yaml(
            runtime_tree.config.parent / "templates" / filename
        )
        for name, filename in rcc.TEMPLATE_FILENAMES.items()
    }


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
