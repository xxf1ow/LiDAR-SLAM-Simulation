import shutil
from copy import deepcopy
import json
from math import cos, sin, sqrt
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


def _rendered(runtime_tree, platform, mode="navigation"):
    runtime_tree.set_bringup_value("platform", platform)
    runtime_tree.set_bringup_value("mode", mode)
    return rcc._render_runtime_configs(rcc._load_runtime_inputs(runtime_tree.config))


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_renderer_maps_profile_motion_and_geometry_to_all_modules(
    runtime_tree, platform
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    rendered = rcc._render_runtime_configs(inputs)
    effective = inputs["effective"]
    geometry = effective["derived"]["geometry"]
    motion = effective["profile"]["motion"]
    controllers = rendered["controllers"]
    base = controllers["base_controller"]["ros__parameters"]
    web_ui = rendered["web_ui"]["robot_web_ui"]["ros__parameters"]
    follow_path = rendered["nav2"]["controller_server"]["ros__parameters"][
        "FollowPath"
    ]

    assert base["wheel_radius"] == geometry["drive"]["wheel_radius"]
    assert base["wheel_separation"] == geometry["drive"]["wheel_separation"]
    assert "wheel_width" not in base
    assert base["linear.x.max_velocity"] == motion["max_linear_velocity"]
    assert base["linear.x.min_velocity"] == -motion["max_linear_velocity"]
    assert base["linear.x.max_acceleration"] == motion["max_linear_acceleration"]
    assert base["linear.x.min_acceleration"] == -motion["max_linear_acceleration"]
    assert base["angular.z.max_velocity"] == motion["max_angular_velocity"]
    assert base["angular.z.min_velocity"] == -motion["max_angular_velocity"]
    assert base["angular.z.max_acceleration"] == motion["max_angular_acceleration"]
    assert base["angular.z.min_acceleration"] == -motion["max_angular_acceleration"]
    assert base["linear.x.has_acceleration_limits"] is True
    assert base["angular.z.has_acceleration_limits"] is True
    assert base["linear.x.has_jerk_limits"] is False
    assert base["angular.z.has_jerk_limits"] is False

    assert web_ui["max_linear_speed"] == motion["max_linear_velocity"]
    assert web_ui["max_angular_speed"] == motion["max_angular_velocity"]
    assert web_ui["host"] == "0.0.0.0"
    assert web_ui["port"] == 8080

    assert follow_path["vx_max"] == motion["max_linear_velocity"]
    assert follow_path["wz_max"] == motion["max_angular_velocity"]
    assert follow_path["vx_min"] == -0.35
    assert {"ax_max", "ax_min", "az_max"}.isdisjoint(follow_path)
    assert follow_path["vx_std"] == 0.2
    assert follow_path["wz_std"] == 0.6
    assert rendered["nav2"]["controller_server"]["ros__parameters"][
        "controller_frequency"
    ] == 10.0
    assert rendered["nav2"]["behavior_server"]["ros__parameters"][
        "spin"
    ] == {"plugin": "nav2_behaviors/Spin"}
    assert rendered["nav2"]["behavior_server"]["ros__parameters"][
        "backup"
    ] == {"plugin": "nav2_behaviors/BackUp"}
    assert rendered["nav2"]["controller_server"]["ros__parameters"][
        "FollowPath"
    ]["ConstraintCritic"] == {"enabled": True, "cost_power": 1, "cost_weight": 4.0}

    footprints = [
        rendered["nav2"]["global_costmap"]["global_costmap"]["ros__parameters"][
            "footprint"
        ],
        rendered["nav2"]["local_costmap"]["local_costmap"]["ros__parameters"][
            "footprint"
        ],
    ]
    assert all(isinstance(value, str) for value in footprints)
    assert footprints[0] == footprints[1]
    assert json.loads(footprints[0]) == geometry["footprint"]

    expected_time = platform == "sim"
    for mapping, paths in (
        (controllers, rcc.CONTROLLER_TIME_PATHS),
        (rendered["web_ui"], rcc.WEB_UI_TIME_PATHS),
        (rendered["nav2"], rcc.NAV2_TIME_PATHS),
    ):
        for path in paths:
            node = mapping
            for key in path:
                node = node[key]
            assert node is expected_time


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_renderer_does_not_depend_on_mode(runtime_tree, platform):
    assert _rendered(runtime_tree, platform, "mapping") == _rendered(
        runtime_tree, platform, "navigation"
    )


def test_renderer_does_not_mutate_loaded_templates(runtime_tree):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    original_templates = deepcopy(inputs["templates"])

    rcc._render_runtime_configs(inputs)

    assert inputs["templates"] == original_templates


@pytest.mark.parametrize(
    "mutate",
    [
        lambda rendered: rendered["controllers"]["base_controller"][
            "ros__parameters"
        ].__setitem__("wheel_width", 0.06),
        lambda rendered: rendered["web_ui"]["robot_web_ui"]["ros__parameters"].__setitem__(
            "max_linear_speed", 99.0
        ),
        lambda rendered: rendered["nav2"]["controller_server"]["ros__parameters"][
            "FollowPath"
        ].__setitem__("ax_max", 1.0),
    ],
)
def test_generated_config_validator_rejects_cross_module_drift(
    runtime_tree, mutate
):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    rendered = rcc._render_runtime_configs(inputs)
    mutate(rendered)

    with pytest.raises(ValueError):
        rcc._validate_generated_configs(
            inputs["effective"],
            rendered["controllers"],
            rendered["web_ui"],
            rendered["nav2"],
        )


CONTROLLER_RENDER_PATHS = (
    ("base_controller", "ros__parameters", "wheel_radius"),
    ("base_controller", "ros__parameters", "wheel_separation"),
    ("base_controller", "ros__parameters", "linear.x.max_velocity"),
    ("base_controller", "ros__parameters", "linear.x.min_velocity"),
    ("base_controller", "ros__parameters", "linear.x.max_acceleration"),
    ("base_controller", "ros__parameters", "linear.x.min_acceleration"),
    ("base_controller", "ros__parameters", "angular.z.max_velocity"),
    ("base_controller", "ros__parameters", "angular.z.min_velocity"),
    ("base_controller", "ros__parameters", "angular.z.max_acceleration"),
    ("base_controller", "ros__parameters", "angular.z.min_acceleration"),
) + rcc.CONTROLLER_TIME_PATHS
WEB_UI_RENDER_PATHS = (
    ("robot_web_ui", "ros__parameters", "max_linear_speed"),
    ("robot_web_ui", "ros__parameters", "max_angular_speed"),
) + rcc.WEB_UI_TIME_PATHS
NAV2_RENDER_PATHS = (
    ("controller_server", "ros__parameters", "FollowPath", "vx_max"),
    ("controller_server", "ros__parameters", "FollowPath", "wz_max"),
    ("global_costmap", "global_costmap", "ros__parameters", "footprint"),
    ("local_costmap", "local_costmap", "ros__parameters", "footprint"),
) + rcc.NAV2_TIME_PATHS


def _mutate_path(mapping, path, mutation):
    node = mapping
    for key in path[:-1]:
        node = node[key]
    if mutation == "missing_leaf":
        del node[path[-1]]
    elif mutation == "wrong_type":
        node[path[-1]] = "wrong" if isinstance(node[path[-1]], bool) else True
    else:
        parent = path[-2]
        grandparent = mapping
        for key in path[:-2]:
            grandparent = grandparent[key]
        del grandparent[parent]


@pytest.mark.parametrize(
    ("label", "path"),
    [("controllers", path) for path in CONTROLLER_RENDER_PATHS]
    + [("web_ui", path) for path in WEB_UI_RENDER_PATHS]
    + [("nav2", path) for path in NAV2_RENDER_PATHS],
)
@pytest.mark.parametrize("mutation", ["missing_parent", "missing_leaf", "wrong_type"])
def test_renderer_rejects_template_target_drift(
    runtime_tree, label, path, mutation
):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    inputs["templates"] = deepcopy(inputs["templates"])
    template = inputs["templates"][label]
    node = template
    for key in path:
        node = node[key]
    expected_type = (
        bool if isinstance(node, bool) else str if isinstance(node, str) else float
    )
    _mutate_path(template, path, mutation)

    with pytest.raises(ValueError, match=rf"{label} template"):
        rcc._render_runtime_configs(inputs)

    with pytest.raises(ValueError, match=rf"{label} template.*{'.'.join(path)}"):
        rcc._set_template_existing(label, template, path, None, expected_type)


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_robot_launch_arguments_map_effective_geometry_exactly(
    runtime_tree, platform
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    geometry = inputs["effective"]["derived"]["geometry"]
    lidar_from_base_link = geometry["mounts_relative_to_base_link"]["lidar"]

    assert rcc._derive_robot_launch_arguments(inputs["effective"]) == {
        "base_length": str(geometry["body"]["length"]),
        "base_width": str(geometry["body"]["width"]),
        "base_height": str(geometry["body"]["height"]),
        "base_link_height": str(geometry["body"]["base_link_height"]),
        "wheel_radius": str(geometry["drive"]["wheel_radius"]),
        "wheel_width": str(geometry["drive"]["wheel_width"]),
        "wheel_separation": str(geometry["drive"]["wheel_separation"]),
        "sensor_x": str(lidar_from_base_link["x"]),
        "sensor_y": str(lidar_from_base_link["y"]),
        "sensor_z": str(lidar_from_base_link["z"]),
        "sensor_roll": str(lidar_from_base_link["roll"]),
        "sensor_pitch": str(lidar_from_base_link["pitch"]),
        "sensor_yaw": str(lidar_from_base_link["yaw"]),
    }
    assert "wheel_width" not in _rendered(runtime_tree, platform)["controllers"][
        "base_controller"
    ]["ros__parameters"]


@pytest.mark.parametrize(
    ("platform", "expected"),
    [
        (
            "sim",
            {
                "x": "0.0",
                "y": "0.0",
                "z": "-0.556",
                "qx": "0.0",
                "qy": "0.0",
                "qz": "0.0",
                "qw": "1.0",
            },
        ),
        (
            "real",
            {
                "x": "-0.443",
                "y": "0.0",
                "z": "-0.905",
                "qx": "0.0",
                "qy": "0.0",
                "qz": "0.0",
                "qw": "1.0",
            },
        ),
    ],
)
def test_compatibility_body_weld_arguments_match_profile_baselines(
    runtime_tree, platform, expected
):
    runtime_tree.set_bringup_value("platform", platform)
    profile = rcc._load_runtime_inputs(runtime_tree.config)["selected_profile"]

    assert rcc._derive_compatibility_body_weld_arguments(profile) == expected


def _quaternion_matrix(qx, qy, qz, qw):
    return (
        (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy - qz * qw),
            2.0 * (qx * qz + qy * qw),
        ),
        (
            2.0 * (qx * qy + qz * qw),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz - qx * qw),
        ),
        (
            2.0 * (qx * qz - qy * qw),
            2.0 * (qy * qz + qx * qw),
            1.0 - 2.0 * (qx * qx + qy * qy),
        ),
    )


def _compose_transform(left, right):
    left_rotation, left_translation = left
    right_rotation, right_translation = right
    rotation = tuple(
        tuple(
            sum(left_rotation[row][k] * right_rotation[k][column] for k in range(3))
            for column in range(3)
        )
        for row in range(3)
    )
    translation = tuple(
        left_translation[row]
        + sum(left_rotation[row][k] * right_translation[k] for k in range(3))
        for row in range(3)
    )
    return rotation, translation


def test_compatibility_body_weld_is_full_se3_inverse_for_nonzero_rpy():
    mount = {
        "x": 0.4,
        "y": -0.2,
        "z": 0.8,
        "roll": 0.3,
        "pitch": -0.4,
        "yaw": 0.7,
    }
    cr, sr = cos(mount["roll"] / 2.0), sin(mount["roll"] / 2.0)
    cp, sp = cos(mount["pitch"] / 2.0), sin(mount["pitch"] / 2.0)
    cy, sy = cos(mount["yaw"] / 2.0), sin(mount["yaw"] / 2.0)
    forward_quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    norm = sqrt(sum(value * value for value in forward_quaternion))
    forward_rotation = _quaternion_matrix(
        *(value / norm for value in forward_quaternion)
    )

    weld = rcc._derive_compatibility_body_weld_arguments(
        {"robot": {"mounts": {"lidar": mount}}}
    )
    inverse_rotation = _quaternion_matrix(
        float(weld["qx"]),
        float(weld["qy"]),
        float(weld["qz"]),
        float(weld["qw"]),
    )
    composed = _compose_transform(
        (forward_rotation, (mount["x"], mount["y"], mount["z"])),
        (
            inverse_rotation,
            (float(weld["x"]), float(weld["y"]), float(weld["z"])),
        ),
    )

    for row in range(3):
        for column in range(3):
            assert composed[0][row][column] == pytest.approx(
                1.0 if row == column else 0.0, abs=1e-12
            )
    assert composed[1] == pytest.approx((0.0, 0.0, 0.0), abs=1e-12)


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_runtime_report_records_temporary_and_deferred_compatibility(
    runtime_tree, platform
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    effective_before = deepcopy(inputs["effective"])
    weld = rcc._derive_compatibility_body_weld_transform(
        inputs["selected_profile"]
    )

    report = rcc._build_runtime_report(inputs["effective"], weld)

    assert inputs["effective"] == effective_before
    assert all(report[key] == value for key, value in effective_before.items())
    assert report["compatibility"] == {
        "body_to_base_footprint": {
            "status": "temporary",
            "assumption": "FAST-LIO body is colocated with the lidar/IMU origin",
            "follow_up_section": 5,
            "translation": {key: weld[key] for key in ("x", "y", "z")},
            "rotation": {key: weld[key] for key in ("qx", "qy", "qz", "qw")},
        }
    }
    assert report["deferred_compatibility"] == [
        {
            "component": "nav2.behavior_server",
            "status": "deferred_to_section_9",
            "template_values": {
                "max_rotational_vel": 1.0,
                "min_rotational_vel": 0.4,
                "rotational_acc_lim": 3.2,
            },
            "profile_values": {
                "max_angular_velocity": effective_before["profile"]["motion"][
                    "max_angular_velocity"
                ],
                "max_angular_acceleration": effective_before["profile"]["motion"][
                    "max_angular_acceleration"
                ],
            },
            "reason": "Humble Nav2 behavior capability and semantics require target-version review",
        }
    ]


def test_real_runtime_report_preserves_deferred_compatibility_difference(runtime_tree):
    runtime_tree.set_bringup_value("platform", "real")
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    weld = rcc._derive_compatibility_body_weld_transform(
        inputs["selected_profile"]
    )

    debt = rcc._build_runtime_report(inputs["effective"], weld)[
        "deferred_compatibility"
    ][0]

    assert debt["template_values"] == {
        "max_rotational_vel": 1.0,
        "min_rotational_vel": 0.4,
        "rotational_acc_lim": 3.2,
    }
    assert debt["profile_values"] == {
        "max_angular_velocity": 0.4,
        "max_angular_acceleration": 0.3,
    }
