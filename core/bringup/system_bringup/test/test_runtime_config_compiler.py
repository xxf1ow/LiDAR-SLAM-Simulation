import shutil
from copy import deepcopy
import json
from math import cos, sin, sqrt
from pathlib import Path
import tempfile

import pytest
import yaml

from system_bringup import profile_compiler as pc
from system_bringup import runtime_config_compiler as rcc


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
TEMPLATE_DIR = CONFIG_DIR / "templates"
SIM_FOOTPRINT = [
    [0.375, 0.275],
    [0.375, -0.275],
    [-0.375, -0.275],
    [-0.375, 0.275],
]
REAL_FOOTPRINT = [
    [0.48, 0.305],
    [0.48, -0.305],
    [-0.48, -0.305],
    [-0.48, 0.305],
]


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

    def mutate_template(self, label, path, mutation):
        template_path = (
            self.config.parent
            / "templates"
            / rcc.TEMPLATE_FILENAMES[label]
        )
        template = _load_yaml(template_path)
        _mutate_path(template, path, mutation)
        template_path.write_text(
            yaml.safe_dump(template, sort_keys=False), encoding="utf-8"
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


@pytest.mark.parametrize("mode", [[], {}], ids=["list", "mapping"])
def test_public_runtime_compile_rejects_non_string_mode_as_value_error(
    runtime_tree, tmp_path, mode
):
    runtime_tree.set_bringup_value("mode", mode)

    with pytest.raises(
        ValueError,
        match="bringup config mode must be 'mapping' or 'navigation'",
    ):
        rcc.compile_runtime_configs(runtime_tree.config, tmp_path / "output")


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
    source = _load_yaml(
        PACKAGE_ROOT.parents[1]
        / "robot/robot_bringup/config/robot_controllers.yaml"
    )
    expected = deepcopy(source)
    expected["controller_manager"]["ros__parameters"]["use_sim_time"] = True
    expected["base_controller"]["ros__parameters"]["use_sim_time"] = True
    expected["base_controller"]["ros__parameters"][
        "linear.x.min_acceleration"
    ] = -1.0

    assert _load_yaml(TEMPLATE_DIR / "robot_controllers.yaml") == expected


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
def test_public_runtime_compile_rejects_source_template_target_drift(
    runtime_tree, tmp_path, label, path, mutation
):
    runtime_tree.mutate_template(label, path, mutation)
    output = tmp_path / "output"
    expected_error = rf"{label} template"
    if mutation != "missing_parent":
        expected_error += rf".*{'.'.join(path)}"

    with pytest.raises(ValueError, match=expected_error):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert not output.exists()


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


def _temporary_files(output):
    return sorted(output.glob(".*.tmp"))


def _yaml_schema(value):
    if isinstance(value, dict):
        return {key: _yaml_schema(child) for key, child in value.items()}
    if isinstance(value, list):
        return [_yaml_schema(child) for child in value]
    return type(value)


def _profile_owned_runtime_values(manifest):
    controllers = _load_yaml(manifest["controllers_path"])
    base = controllers["base_controller"]["ros__parameters"]
    web_ui_config = _load_yaml(manifest["web_ui_path"])
    web_ui = web_ui_config["robot_web_ui"]["ros__parameters"]
    nav2 = _load_yaml(manifest["nav2_path"])
    follow_path = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    footprints = [
        nav2["global_costmap"]["global_costmap"]["ros__parameters"][
            "footprint"
        ],
        nav2["local_costmap"]["local_costmap"]["ros__parameters"][
            "footprint"
        ],
    ]
    return {
        "controllers": {
            "wheel_radius": base["wheel_radius"],
            "wheel_separation": base["wheel_separation"],
            "max_linear_velocity": base["linear.x.max_velocity"],
            "min_linear_velocity": base["linear.x.min_velocity"],
            "max_linear_acceleration": base["linear.x.max_acceleration"],
            "min_linear_acceleration": base["linear.x.min_acceleration"],
            "max_angular_velocity": base["angular.z.max_velocity"],
            "min_angular_velocity": base["angular.z.min_velocity"],
            "max_angular_acceleration": base["angular.z.max_acceleration"],
            "min_angular_acceleration": base["angular.z.min_acceleration"],
        },
        "web_ui": {
            "max_linear_speed": web_ui["max_linear_speed"],
            "max_angular_speed": web_ui["max_angular_speed"],
        },
        "nav2": {
            "vx_max": follow_path["vx_max"],
            "wz_max": follow_path["wz_max"],
            "footprints": [json.loads(value) for value in footprints],
        },
        "use_sim_time": {
            "controllers": [
                _path_value(controllers, path)
                for path in rcc.CONTROLLER_TIME_PATHS
            ],
            "web_ui": [
                _path_value(web_ui_config, path)
                for path in rcc.WEB_UI_TIME_PATHS
            ],
            "nav2": [_path_value(nav2, path) for path in rcc.NAV2_TIME_PATHS],
        },
    }


def _path_value(mapping, path):
    for key in path:
        mapping = mapping[key]
    return mapping


def test_sim_and_real_public_compiles_remain_schema_and_value_isolated(
    runtime_tree, tmp_path
):
    manifests = {platform: {} for platform in ("sim", "real")}
    generated = {platform: {} for platform in ("sim", "real")}
    for platform in ("sim", "real"):
        for mode in ("mapping", "navigation"):
            runtime_tree.set_bringup_value("platform", platform)
            runtime_tree.set_bringup_value("mode", mode)
            manifests[platform][mode] = rcc.compile_runtime_configs(
                runtime_tree.config, tmp_path / platform / mode
            )
            generated[platform][mode] = {
                name: _load_yaml(manifests[platform][mode][f"{name}_path"])
                for name in (
                    "controllers",
                    "web_ui",
                    "nav2",
                    "effective_profile",
                )
            }

        for name in ("controllers", "web_ui", "nav2"):
            assert generated[platform]["mapping"][name] == generated[platform][
                "navigation"
            ][name]

    for name in ("controllers", "web_ui", "nav2"):
        assert _yaml_schema(generated["sim"]["navigation"][name]) == _yaml_schema(
            generated["real"]["navigation"][name]
        )

    assert _profile_owned_runtime_values(manifests["sim"]["navigation"]) == {
        "controllers": {
            "wheel_radius": 0.12,
            "wheel_separation": 0.55,
            "max_linear_velocity": 1.0,
            "min_linear_velocity": -1.0,
            "max_linear_acceleration": 1.0,
            "min_linear_acceleration": -1.0,
            "max_angular_velocity": 1.8,
            "min_angular_velocity": -1.8,
            "max_angular_acceleration": 1.0,
            "min_angular_acceleration": -1.0,
        },
        "web_ui": {"max_linear_speed": 1.0, "max_angular_speed": 1.8},
        "nav2": {
            "vx_max": 1.0,
            "wz_max": 1.8,
            "footprints": [SIM_FOOTPRINT, SIM_FOOTPRINT],
        },
        "use_sim_time": {
            "controllers": [True, True],
            "web_ui": [True],
            "nav2": [True] * len(rcc.NAV2_TIME_PATHS),
        },
    }
    assert _profile_owned_runtime_values(manifests["real"]["navigation"]) == {
        "controllers": {
            "wheel_radius": 0.1025,
            "wheel_separation": 0.463,
            "max_linear_velocity": 1.0,
            "min_linear_velocity": -1.0,
            "max_linear_acceleration": 1.0,
            "min_linear_acceleration": -1.0,
            "max_angular_velocity": 0.4,
            "min_angular_velocity": -0.4,
            "max_angular_acceleration": 0.3,
            "min_angular_acceleration": -0.3,
        },
        "web_ui": {"max_linear_speed": 1.0, "max_angular_speed": 0.4},
        "nav2": {
            "vx_max": 1.0,
            "wz_max": 0.4,
            "footprints": [REAL_FOOTPRINT, REAL_FOOTPRINT],
        },
        "use_sim_time": {
            "controllers": [False, False],
            "web_ui": [False],
            "nav2": [False] * len(rcc.NAV2_TIME_PATHS),
        },
    }

    legacy_report_keys = {"platform", "source_profile", "profile", "derived"}
    runtime_only_keys = {
        "generated_configs",
        "compatibility",
        "deferred_compatibility",
    }
    for platform_values in generated.values():
        for values in platform_values.values():
            assert set(values["effective_profile"]) == (
                legacy_report_keys | runtime_only_keys
            )

    profile_path = pc.compile_profile(runtime_tree.config, tmp_path / "profile")
    profile_report = _load_yaml(profile_path)
    assert set(profile_report) == legacy_report_keys
    assert runtime_only_keys.isdisjoint(profile_report)


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_compile_runtime_configs_writes_owned_files_and_stable_manifest(
    runtime_tree, tmp_path, platform, monkeypatch
):
    runtime_tree.set_bringup_value("platform", platform)
    output = tmp_path / f"{platform}-output"
    loaded = []
    original = rcc._load_runtime_inputs

    def capture_inputs(path):
        inputs = original(path)
        loaded.append(inputs)
        return inputs

    monkeypatch.setattr(rcc, "_load_runtime_inputs", capture_inputs)

    manifest = rcc.compile_runtime_configs(runtime_tree.config, output)

    assert len(loaded) == 1
    paths = {
        key: output / filename for key, filename in rcc.OUTPUT_FILENAMES.items()
    }
    assert set(output.iterdir()) == set(paths.values())
    assert set(manifest) == {
        "bringup_config_path",
        "bringup_config",
        "platform",
        "mode",
        "use_sim_time",
        "effective_profile_path",
        "controllers_path",
        "web_ui_path",
        "nav2_path",
        "robot_launch_arguments",
        "compatibility_body_weld_arguments",
    }
    assert manifest["bringup_config_path"] == runtime_tree.config.resolve()
    assert manifest["bringup_config"] == loaded[0]["config"]
    assert manifest["bringup_config"] is not loaded[0]["config"]
    assert (
        manifest["bringup_config"]["profiles"]
        is not loaded[0]["config"]["profiles"]
    )
    source_config = runtime_tree.config.read_bytes()
    selected_profile = loaded[0]["config"]["profiles"][platform]
    manifest["bringup_config"]["profiles"][platform] = "mutated"
    assert loaded[0]["config"]["profiles"][platform] == selected_profile
    assert runtime_tree.config.read_bytes() == source_config
    assert manifest["platform"] == platform
    assert manifest["mode"] == "navigation"
    assert manifest["use_sim_time"] is (platform == "sim")
    assert manifest["effective_profile_path"] == paths["effective_profile"]
    assert manifest["controllers_path"] == paths["controllers"]
    assert manifest["web_ui_path"] == paths["web_ui"]
    assert manifest["nav2_path"] == paths["nav2"]
    assert manifest["robot_launch_arguments"] == rcc._derive_robot_launch_arguments(
        loaded[0]["effective"]
    )
    assert manifest["compatibility_body_weld_arguments"] == (
        rcc._derive_compatibility_body_weld_arguments(
            loaded[0]["selected_profile"]
        )
    )

    report = _load_yaml(paths["effective_profile"])
    assert report["generated_configs"] == {
        "controllers": str(paths["controllers"]),
        "web_ui": str(paths["web_ui"]),
        "nav2": str(paths["nav2"]),
    }
    assert _temporary_files(output) == []


def test_compile_runtime_configs_uses_integrated_renderer(
    runtime_tree, tmp_path, monkeypatch
):
    original = rcc._render_runtime_configs
    calls = []

    def render_with_marker(inputs):
        calls.append(inputs)
        generated = original(inputs)
        generated["web_ui"]["robot_web_ui"]["ros__parameters"][
            "host"
        ] = "renderer-used"
        return generated

    monkeypatch.setattr(rcc, "_render_runtime_configs", render_with_marker)

    manifest = rcc.compile_runtime_configs(
        runtime_tree.config, tmp_path / "output"
    )

    assert len(calls) == 1
    assert _load_yaml(manifest["web_ui_path"])["robot_web_ui"][
        "ros__parameters"
    ]["host"] == "renderer-used"


def test_compile_runtime_configs_uses_unique_private_temp_directories(runtime_tree):
    first = rcc.compile_runtime_configs(runtime_tree.config)
    second = rcc.compile_runtime_configs(runtime_tree.config)
    first_dir = first["effective_profile_path"].parent
    second_dir = second["effective_profile_path"].parent

    assert first_dir != second_dir
    for output in (first_dir, second_dir):
        assert output.name.startswith("system_bringup-runtime-")
        assert output.is_absolute()
        assert output.parent == Path(tempfile.gettempdir()).resolve()
        assert set(path.name for path in output.iterdir()) == set(
            rcc.OUTPUT_FILENAMES.values()
        )


def test_explicit_runtime_output_preserves_unowned_files_across_compiles(
    runtime_tree, tmp_path
):
    output = tmp_path / "output"
    output.mkdir()
    keep = output / "keep.txt"
    keep.write_text("keep exactly", encoding="utf-8")

    first = rcc.compile_runtime_configs(runtime_tree.config, output)
    first_report = first["effective_profile_path"].read_bytes()
    runtime_tree.set_bringup_value("platform", "real")
    second = rcc.compile_runtime_configs(runtime_tree.config, output)

    assert first["effective_profile_path"] == second["effective_profile_path"]
    assert second["effective_profile_path"].read_bytes() != first_report
    assert keep.read_text(encoding="utf-8") == "keep exactly"
    assert set(path.name for path in output.iterdir()) == {
        "keep.txt",
        *rcc.OUTPUT_FILENAMES.values(),
    }
    assert _temporary_files(output) == []


def test_runtime_compilation_does_not_modify_source_or_formal_files(
    runtime_tree, tmp_path
):
    core_dir = PACKAGE_ROOT.parents[1]
    protected = [
        *(TEMPLATE_DIR / filename for filename in rcc.TEMPLATE_FILENAMES.values()),
        *(
            runtime_tree.config.parent / "templates" / filename
            for filename in rcc.TEMPLATE_FILENAMES.values()
        ),
        core_dir / "robot/robot_bringup/config/robot_controllers.yaml",
        core_dir / "navigation/robot_navigation/config/nav2_params.yaml",
        core_dir / "navigation/robot_navigation/config/nav2_params_real.yaml",
        PACKAGE_ROOT / "launch/bringup.launch.py",
    ]
    before = {path: path.read_bytes() for path in protected}

    rcc.compile_runtime_configs(runtime_tree.config, tmp_path / "output")

    assert {path: path.read_bytes() for path in protected} == before


def test_runtime_resources_resolve_from_config_for_absolute_and_relative_paths(
    runtime_tree, tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    absolute = rcc.compile_runtime_configs(
        runtime_tree.config.resolve(), tmp_path / "absolute-output"
    )
    relative_config = runtime_tree.config.relative_to(tmp_path)
    relative = rcc.compile_runtime_configs(
        relative_config, tmp_path / "relative-output"
    )

    assert _load_yaml(absolute["controllers_path"]) == _load_yaml(
        relative["controllers_path"]
    )
    assert absolute["bringup_config_path"] == relative["bringup_config_path"]


def test_in_memory_validation_failure_creates_no_completion_marker(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"

    def fail_validation(*args):
        raise ValueError("in-memory drift")

    monkeypatch.setattr(rcc, "_validate_generated_configs", fail_validation)

    with pytest.raises(ValueError, match="in-memory drift"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert not (output / rcc.OUTPUT_FILENAMES["effective_profile"]).exists()
    assert not output.exists()


def test_staging_write_failure_cleans_every_temporary_file(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    original = rcc.yaml.safe_dump
    calls = 0

    def fail_fourth_dump(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 4:
            raise OSError("staging write failed")
        return original(*args, **kwargs)

    monkeypatch.setattr(rcc.yaml, "safe_dump", fail_fourth_dump)

    with pytest.raises(OSError, match="staging write failed"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert not (output / rcc.OUTPUT_FILENAMES["effective_profile"]).exists()
    assert _temporary_files(output) == []
    assert list(output.iterdir()) == []


def test_staged_reload_validation_failure_precedes_all_replacements(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    rcc.compile_runtime_configs(runtime_tree.config, output)
    before = {path: path.read_bytes() for path in output.iterdir()}
    runtime_tree.set_bringup_value("platform", "real")
    original = rcc._validate_generated_configs
    validations = 0

    def fail_second_validation(*args):
        nonlocal validations
        validations += 1
        if validations == 2:
            raise ValueError("staged drift")
        return original(*args)

    monkeypatch.setattr(rcc, "_validate_generated_configs", fail_second_validation)

    with pytest.raises(ValueError, match="staged drift"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert validations == 2
    assert {path: path.read_bytes() for path in output.iterdir()} == before
    assert _temporary_files(output) == []


def test_mid_replace_failure_does_not_update_existing_completion_marker(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    runtime_tree.set_bringup_value("platform", "real")
    original = rcc.os.replace
    replaced = []

    def fail_second_replace(source, destination):
        replaced.append(Path(destination).name)
        if len(replaced) == 2:
            raise OSError("replace interrupted")
        return original(source, destination)

    monkeypatch.setattr(rcc.os, "replace", fail_second_replace)

    with pytest.raises(OSError, match="replace interrupted"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert replaced == [
        rcc.OUTPUT_FILENAMES["controllers"],
        rcc.OUTPUT_FILENAMES["web_ui"],
    ]
    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


def test_effective_report_is_replaced_last_before_manifest_is_returned(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    original = rcc.os.replace
    replaced = []

    def record_replace(source, destination):
        replaced.append(Path(destination).name)
        return original(source, destination)

    monkeypatch.setattr(rcc.os, "replace", record_replace)

    manifest = rcc.compile_runtime_configs(runtime_tree.config, output)

    assert replaced == [
        rcc.OUTPUT_FILENAMES["controllers"],
        rcc.OUTPUT_FILENAMES["web_ui"],
        rcc.OUTPUT_FILENAMES["nav2"],
        rcc.OUTPUT_FILENAMES["effective_profile"],
    ]
    assert manifest["effective_profile_path"].exists()
    assert _temporary_files(output) == []


def test_runtime_cli_prints_one_absolute_report_path(runtime_tree, tmp_path, capsys):
    output = tmp_path / "output"

    result = rcc.main(
        [
            "--bringup-config",
            str(runtime_tree.config),
            "--output-dir",
            str(output),
        ]
    )

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert captured.out == (
        f"{(output / rcc.OUTPUT_FILENAMES['effective_profile']).resolve()}\n"
    )


def test_runtime_cli_help_remains_standard(capsys):
    with pytest.raises(SystemExit) as exc_info:
        rcc.main(["--help"])

    captured = capsys.readouterr()
    assert exc_info.value.code == 0
    assert captured.err == ""
    assert captured.out.startswith("usage:")
    assert "--bringup-config" in captured.out
    assert "--output-dir" in captured.out


def test_runtime_cli_reports_one_actionable_error_without_traceback(
    runtime_tree, capsys
):
    runtime_tree.set_bringup_value("platform", "invalid")

    with pytest.raises(SystemExit) as exc_info:
        rcc.main(["--bringup-config", str(runtime_tree.config)])

    captured = capsys.readouterr()
    assert exc_info.value.code == 2
    assert captured.out == ""
    assert captured.err == (
        "compile_runtime_configs: "
        f"{runtime_tree.config.resolve()}: platform must be 'sim' or 'real'\n"
    )
    assert "Traceback" not in captured.err


@pytest.mark.parametrize("mode", [[], {}], ids=["list", "mapping"])
def test_runtime_cli_reports_non_string_mode_as_one_actionable_line(
    runtime_tree, mode, capsys
):
    runtime_tree.set_bringup_value("mode", mode)

    with pytest.raises(SystemExit) as exc_info:
        rcc.main(["--bringup-config", str(runtime_tree.config)])

    captured = capsys.readouterr()
    assert exc_info.value.code == 2
    assert captured.out == ""
    assert captured.err == (
        "compile_runtime_configs: "
        "bringup config mode must be 'mapping' or 'navigation'\n"
    )
    assert "Traceback" not in captured.err


def test_runtime_cli_missing_required_argument_is_one_line(capsys):
    with pytest.raises(SystemExit) as exc_info:
        rcc.main([])

    captured = capsys.readouterr()
    assert exc_info.value.code == 2
    assert captured.out == ""
    assert captured.err == (
        "compile_runtime_configs: the following arguments are required: "
        "--bringup-config\n"
    )
    assert "Traceback" not in captured.err


def test_runtime_cli_unknown_argument_is_one_line(runtime_tree, capsys):
    with pytest.raises(SystemExit) as exc_info:
        rcc.main(
            [
                "--bringup-config",
                str(runtime_tree.config),
                "--unexpected",
            ]
        )

    captured = capsys.readouterr()
    assert exc_info.value.code == 2
    assert captured.out == ""
    assert captured.err == (
        "compile_runtime_configs: unrecognized arguments: --unexpected\n"
    )
    assert "Traceback" not in captured.err


def test_formal_bringup_does_not_import_profile_runtime_compilers():
    launch_source = (PACKAGE_ROOT / "launch/bringup.launch.py").read_text(
        encoding="utf-8"
    )

    for name in (
        "profile_compiler",
        "runtime_config_compiler",
        "compile_profile",
        "compile_runtime_configs",
    ):
        assert name not in launch_source
