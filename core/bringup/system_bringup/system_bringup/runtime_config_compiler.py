import argparse
from copy import deepcopy
import json
from math import cos, degrees, isfinite, sin, sqrt
import os
from pathlib import Path
import tempfile

import yaml

from system_bringup import profile_compiler as pc


SUPPORTED_MODES = {"mapping", "navigation"}
MOTION_KEYS = (
    "max_linear_velocity",
    "max_angular_velocity",
    "max_linear_acceleration",
    "max_angular_acceleration",
)
TEMPLATE_FILENAMES = {
    "controllers": "robot_controllers.yaml",
    "web_ui": "robot_web_ui.yaml",
    "nav2": "nav2.yaml",
}
SENSOR_TEMPLATE_FILENAMES = {
    "sim": {
        "lidar_adapter": "lidar_adapter.yaml",
        "sensor_gate": "sensor_gate.yaml",
    },
    "real": {
        "vanjee_lidar": "vanjee_lidar.yaml",
        "sensor_gate": "sensor_gate.yaml",
    },
}
COMMON_OUTPUT_FILENAMES = {
    "controllers": "robot_controllers.generated.yaml",
    "web_ui": "robot_web_ui.generated.yaml",
    "nav2": "nav2.generated.yaml",
}
SENSOR_OUTPUT_FILENAMES = {
    "sim": {
        "lidar_adapter": "lidar_adapter.generated.yaml",
        "sensor_gate": "sensor_gate.generated.yaml",
    },
    "real": {
        "vanjee_lidar": "vanjee_lidar.generated.yaml",
        "sensor_gate": "sensor_gate.generated.yaml",
    },
}
EFFECTIVE_PROFILE_FILENAME = "effective_profile.generated.yaml"
CONTROLLER_TIME_PATHS = (
    ("controller_manager", "ros__parameters", "use_sim_time"),
    ("base_controller", "ros__parameters", "use_sim_time"),
)
WEB_UI_TIME_PATHS = (("robot_web_ui", "ros__parameters", "use_sim_time"),)
NAV2_TIME_PATHS = (
    ("map_server", "ros__parameters", "use_sim_time"),
    ("planner_server", "ros__parameters", "use_sim_time"),
    ("controller_server", "ros__parameters", "use_sim_time"),
    ("global_costmap", "global_costmap", "ros__parameters", "use_sim_time"),
    ("local_costmap", "local_costmap", "ros__parameters", "use_sim_time"),
    ("behavior_server", "ros__parameters", "use_sim_time"),
    ("bt_navigator", "ros__parameters", "use_sim_time"),
)


class _OneLineArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        self.exit(2, f"compile_runtime_configs: {message}\n")


def _output_filenames(platform):
    if platform not in SENSOR_OUTPUT_FILENAMES:
        raise ValueError("platform must be 'sim' or 'real'")
    return {
        **COMMON_OUTPUT_FILENAMES,
        **SENSOR_OUTPUT_FILENAMES[platform],
        "effective_profile": EFFECTIVE_PROFILE_FILENAME,
    }


def _validate_mode(config):
    mode = config.get("mode")
    if not isinstance(mode, str) or mode not in SUPPORTED_MODES:
        raise ValueError("bringup config mode must be 'mapping' or 'navigation'")
    return mode


def _validate_motion_pair(profiles):
    for platform, (_, profile) in profiles.items():
        motion = profile["motion"]
        for key in MOTION_KEYS:
            value = motion[key]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not isfinite(value)
                or value <= 0
            ):
                raise ValueError(
                    f"profile {platform} motion.{key} must be a finite number > 0"
                )


def _load_runtime_inputs(bringup_config_path):
    """Load and validate one immutable-by-convention runtime input snapshot."""
    source, config, platform, profile_paths = pc.load_bringup_context(
        bringup_config_path
    )
    mode = _validate_mode(config)
    profiles = {
        name: (path, pc.load_profile(path)) for name, path in profile_paths.items()
    }
    pc.validate_profile_pair(profiles)
    _validate_motion_pair(profiles)
    selected_profile_path, selected_profile = profiles[platform]
    effective = pc.derive_effective_profile(
        platform, selected_profile_path, selected_profile
    )
    templates = {
        name: _load_template(source.parent / "templates" / filename, name)[1]
        for name, filename in TEMPLATE_FILENAMES.items()
    }
    sensor_templates = {
        name: _load_template(source.parent / "templates" / filename, name)[1]
        for name, filename in SENSOR_TEMPLATE_FILENAMES[platform].items()
    }
    return {
        "source": source,
        "config": config,
        "platform": platform,
        "mode": mode,
        "profiles": profiles,
        "selected_profile_path": selected_profile_path,
        "selected_profile": selected_profile,
        "effective": effective,
        "templates": templates,
        "sensor_templates": sensor_templates,
    }


def _load_template(path, label):
    source, data = pc._read_yaml_mapping(path, f"{label} template")
    return source, deepcopy(data)


def _set_existing(mapping, path, value, expected_type):
    node = mapping
    dotted = ".".join(path)
    for key in path[:-1]:
        if not isinstance(node, dict) or key not in node:
            raise ValueError(f"template target missing: {dotted}")
        node = node[key]
    leaf = path[-1]
    if not isinstance(node, dict) or leaf not in node:
        raise ValueError(f"template target missing: {dotted}")
    current = node[leaf]
    if expected_type is bool:
        valid = isinstance(current, bool)
    elif expected_type is str:
        valid = isinstance(current, str)
    else:
        valid = isinstance(current, (int, float)) and not isinstance(current, bool)
    if not valid:
        raise ValueError(f"template target has wrong type: {dotted}")
    node[leaf] = value


def _set_template_existing(label, mapping, path, value, expected_type):
    try:
        _set_existing(mapping, path, value, expected_type)
    except ValueError as exc:
        raise ValueError(f"{label} template: {exc}") from exc


def _get_existing(mapping, path):
    node = mapping
    dotted = ".".join(path)
    for key in path:
        if not isinstance(node, dict) or key not in node:
            raise ValueError(f"generated config target missing: {dotted}")
        node = node[key]
    return node


def _format_footprint(footprint):
    return json.dumps(footprint, ensure_ascii=False, separators=(", ", ": "))


def _derive_robot_launch_arguments(effective):
    geometry = effective["derived"]["geometry"]
    body = geometry["body"]
    drive = geometry["drive"]
    lidar = geometry["mounts_relative_to_base_link"]["lidar"]
    return {
        "base_length": str(body["length"]),
        "base_width": str(body["width"]),
        "base_height": str(body["height"]),
        "base_link_height": str(body["base_link_height"]),
        "wheel_radius": str(drive["wheel_radius"]),
        "wheel_width": str(drive["wheel_width"]),
        "wheel_separation": str(drive["wheel_separation"]),
        "sensor_x": str(lidar["x"]),
        "sensor_y": str(lidar["y"]),
        "sensor_z": str(lidar["z"]),
        "sensor_roll": str(lidar["roll"]),
        "sensor_pitch": str(lidar["pitch"]),
        "sensor_yaw": str(lidar["yaw"]),
    }


def _stable_float(value):
    return 0.0 if abs(value) < 1e-15 else value


def _derive_compatibility_body_weld_transform(profile):
    """Invert the raw base_footprint-to-lidar mount for the temporary body weld."""
    mount = profile["robot"]["mounts"]["lidar"]
    cr, sr = cos(mount["roll"] / 2.0), sin(mount["roll"] / 2.0)
    cp, sp = cos(mount["pitch"] / 2.0), sin(mount["pitch"] / 2.0)
    cy, sy = cos(mount["yaw"] / 2.0), sin(mount["yaw"] / 2.0)
    quaternion = (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )
    norm = sqrt(sum(value * value for value in quaternion))
    qx, qy, qz, qw = (value / norm for value in quaternion)

    rotation = (
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
    translation = (mount["x"], mount["y"], mount["z"])
    inverse_translation = tuple(
        -sum(rotation[row][column] * translation[row] for row in range(3))
        for column in range(3)
    )
    values = inverse_translation + (-qx, -qy, -qz, qw)
    return {
        key: _stable_float(value)
        for key, value in zip(("x", "y", "z", "qx", "qy", "qz", "qw"), values)
    }


def _derive_compatibility_body_weld_arguments(profile):
    transform = _derive_compatibility_body_weld_transform(profile)
    return {key: str(value) for key, value in transform.items()}


def _build_runtime_report(effective, body_weld):
    report = deepcopy(effective)
    selected_motion = effective["profile"]["motion"]
    report["compatibility"] = {
        "body_to_base_footprint": {
            "status": "temporary",
            "assumption": "FAST-LIO body is colocated with the lidar/IMU origin",
            "follow_up_section": 5,
            "translation": {
                key: body_weld[key] for key in ("x", "y", "z")
            },
            "rotation": {
                key: body_weld[key] for key in ("qx", "qy", "qz", "qw")
            },
        }
    }
    report["deferred_compatibility"] = [
        {
            "component": "nav2.behavior_server",
            "status": "deferred_to_section_9",
            "template_values": {
                "max_rotational_vel": 1.0,
                "min_rotational_vel": 0.4,
                "rotational_acc_lim": 3.2,
            },
            "profile_values": {
                "max_angular_velocity": selected_motion["max_angular_velocity"],
                "max_angular_acceleration": selected_motion[
                    "max_angular_acceleration"
                ],
            },
            "reason": "Humble Nav2 behavior capability and semantics require target-version review",
        }
    ]
    return report


def _render_controller(template, effective):
    controllers = deepcopy(template)
    geometry = effective["derived"]["geometry"]["drive"]
    motion = effective["profile"]["motion"]
    for key, value in (
        ("wheel_radius", geometry["wheel_radius"]),
        ("wheel_separation", geometry["wheel_separation"]),
        ("linear.x.max_velocity", motion["max_linear_velocity"]),
        ("linear.x.min_velocity", -motion["max_linear_velocity"]),
        ("linear.x.max_acceleration", motion["max_linear_acceleration"]),
        ("linear.x.min_acceleration", -motion["max_linear_acceleration"]),
        ("angular.z.max_velocity", motion["max_angular_velocity"]),
        ("angular.z.min_velocity", -motion["max_angular_velocity"]),
        ("angular.z.max_acceleration", motion["max_angular_acceleration"]),
        ("angular.z.min_acceleration", -motion["max_angular_acceleration"]),
    ):
        _set_template_existing(
            "controllers",
            controllers,
            ("base_controller", "ros__parameters", key),
            value,
            float,
        )
    for path in CONTROLLER_TIME_PATHS:
        _set_template_existing(
            "controllers", controllers, path, effective["derived"]["use_sim_time"], bool
        )
    return controllers


def _render_web_ui(template, effective):
    web_ui = deepcopy(template)
    motion = effective["profile"]["motion"]
    _set_template_existing(
        "web_ui",
        web_ui,
        ("robot_web_ui", "ros__parameters", "max_linear_speed"),
        motion["max_linear_velocity"],
        float,
    )
    _set_template_existing(
        "web_ui",
        web_ui,
        ("robot_web_ui", "ros__parameters", "max_angular_speed"),
        motion["max_angular_velocity"],
        float,
    )
    for path in WEB_UI_TIME_PATHS:
        _set_template_existing(
            "web_ui", web_ui, path, effective["derived"]["use_sim_time"], bool
        )
    return web_ui


def _render_nav2(template, effective):
    nav2 = deepcopy(template)
    motion = effective["profile"]["motion"]
    footprint = _format_footprint(effective["derived"]["geometry"]["footprint"])
    _set_template_existing(
        "nav2",
        nav2,
        ("controller_server", "ros__parameters", "FollowPath", "vx_max"),
        motion["max_linear_velocity"],
        float,
    )
    _set_template_existing(
        "nav2",
        nav2,
        ("controller_server", "ros__parameters", "FollowPath", "wz_max"),
        motion["max_angular_velocity"],
        float,
    )
    for path in (
        ("global_costmap", "global_costmap", "ros__parameters", "footprint"),
        ("local_costmap", "local_costmap", "ros__parameters", "footprint"),
    ):
        _set_template_existing("nav2", nav2, path, footprint, str)
    for path in NAV2_TIME_PATHS:
        _set_template_existing(
            "nav2", nav2, path, effective["derived"]["use_sim_time"], bool
        )
    return nav2


def _render_lidar_adapter(template, effective):
    adapter = deepcopy(template)
    for key, value, expected_type in (
        ("use_sim_time", effective["derived"]["use_sim_time"], bool),
        (
            "scan_period",
            effective["derived"]["sensor_contract"]["scan_period"],
            float,
        ),
    ):
        _set_template_existing(
            "lidar_adapter",
            adapter,
            ("lidar_pointcloud_adapter", "ros__parameters", key),
            value,
            expected_type,
        )
    return adapter


def _render_vanjee_lidar(template, effective):
    vanjee = deepcopy(template)
    hardware = effective["profile"]["hardware"]["lidar"]
    lidar = effective["profile"]["sensors"]["lidar"]
    values = (
        ("lidar_type", hardware["model"], str),
        ("host_address", hardware["host_address"], str),
        ("lidar_address", hardware["device_address"], str),
        ("host_msop_port", hardware["host_msop_port"], int),
        ("lidar_msop_port", hardware["device_msop_port"], int),
        ("start_angle", degrees(lidar["horizontal_start_angle"]), float),
        ("end_angle", degrees(lidar["horizontal_end_angle"]), float),
        ("min_distance", lidar["min_range"], float),
        ("max_distance", lidar["max_range"], float),
    )
    for key, value, expected_type in values:
        _set_template_existing(
            "vanjee_lidar",
            vanjee,
            ("vanjee_lidar", "ros__parameters", key),
            value,
            expected_type,
        )
    return vanjee


def _render_sensor_gate(template, effective):
    gate = deepcopy(template)
    lidar = effective["profile"]["sensors"]["lidar"]
    imu = effective["profile"]["sensors"]["imu"]
    values = (
        ("use_sim_time", effective["derived"]["use_sim_time"], bool),
        (
            "expected_points_per_scan",
            effective["derived"]["sensor_contract"]["points_per_scan"],
            int,
        ),
        ("expected_point_hz", lidar["scan_rate_hz"], float),
        ("expected_imu_hz", imu["rate_hz"], float),
    )
    for key, value, expected_type in values:
        _set_template_existing(
            "sensor_gate",
            gate,
            ("sensor_contract_gate", "ros__parameters", key),
            value,
            expected_type,
        )
    return gate


def _validate_sensor_generated_configs(platform, effective, generated):
    expected_keys = set(SENSOR_TEMPLATE_FILENAMES[platform])
    if set(generated) != expected_keys:
        raise ValueError(
            f"generated sensor configs mismatch: expected {sorted(expected_keys)}"
        )

    lidar = effective["profile"]["sensors"]["lidar"]
    if platform == "sim":
        adapter = generated["lidar_adapter"]
        adapter_path = ("lidar_pointcloud_adapter", "ros__parameters")
        expected_adapter = {
            "use_sim_time": effective["derived"]["use_sim_time"],
            "input_topic": "/lidar/points",
            "output_topic": "/points_raw",
            "output_frame": "velodyne",
            "scan_period": effective["derived"]["sensor_contract"][
                "scan_period"
            ],
        }
        for key, expected in expected_adapter.items():
            path = adapter_path + (key,)
            if _get_existing(adapter, path) != expected:
                raise ValueError(
                    f"generated lidar_adapter mismatch: {'.'.join(path)}"
                )
    else:
        hardware = effective["profile"]["hardware"]["lidar"]
        vanjee = generated["vanjee_lidar"]
        vanjee_path = ("vanjee_lidar", "ros__parameters")
        expected_vanjee = {
            "lidar_type": hardware["model"],
            "host_address": hardware["host_address"],
            "lidar_address": hardware["device_address"],
            "host_msop_port": hardware["host_msop_port"],
            "lidar_msop_port": hardware["device_msop_port"],
            "start_angle": degrees(lidar["horizontal_start_angle"]),
            "end_angle": degrees(lidar["horizontal_end_angle"]),
            "min_distance": lidar["min_range"],
            "max_distance": lidar["max_range"],
            "lidar_frame": "velodyne",
            "imu_frame": "imu_link",
            "point_cloud_topic": "/points_raw",
            "imu_topic": "/imu/data",
        }
        for key, expected in expected_vanjee.items():
            path = vanjee_path + (key,)
            if _get_existing(vanjee, path) != expected:
                raise ValueError(
                    f"generated vanjee_lidar mismatch: {'.'.join(path)}"
                )

    gate = generated["sensor_gate"]
    gate_path = ("sensor_contract_gate", "ros__parameters")
    expected_gate = {
        "use_sim_time": effective["derived"]["use_sim_time"],
        "expected_points_per_scan": effective["derived"]["sensor_contract"][
            "points_per_scan"
        ],
        "expected_point_hz": lidar["scan_rate_hz"],
        "expected_imu_hz": effective["profile"]["sensors"]["imu"]["rate_hz"],
    }
    for key, expected in expected_gate.items():
        path = gate_path + (key,)
        if _get_existing(gate, path) != expected:
            raise ValueError(f"generated sensor_gate mismatch: {'.'.join(path)}")

    for key in ("minimum_point_rate_ratio", "minimum_imu_rate_ratio"):
        path = gate_path + (key,)
        value = _get_existing(gate, path)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not isfinite(value)
            or not 0 < value <= 1
        ):
            raise ValueError(
                f"generated sensor_gate ratio must be in (0, 1]: {'.'.join(path)}"
            )
    for key in ("max_stamp_age", "rate_window", "stable_duration", "timeout"):
        path = gate_path + (key,)
        value = _get_existing(gate, path)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not isfinite(value)
            or value <= 0
        ):
            raise ValueError(
                f"generated sensor_gate value must be > 0: {'.'.join(path)}"
            )


def _render_sensor_configs(inputs):
    effective = inputs["effective"]
    templates = inputs["sensor_templates"]
    if inputs["platform"] == "sim":
        generated = {
            "lidar_adapter": _render_lidar_adapter(
                templates["lidar_adapter"], effective
            ),
            "sensor_gate": _render_sensor_gate(
                templates["sensor_gate"], effective
            ),
        }
    else:
        generated = {
            "vanjee_lidar": _render_vanjee_lidar(
                templates["vanjee_lidar"], effective
            ),
            "sensor_gate": _render_sensor_gate(
                templates["sensor_gate"], effective
            ),
        }
    _validate_sensor_generated_configs(inputs["platform"], effective, generated)
    return generated


def _validate_generated_configs(effective, controllers, web_ui, nav2):
    drive = effective["derived"]["geometry"]["drive"]
    motion = effective["profile"]["motion"]
    base_path = ("base_controller", "ros__parameters")
    expected_controller = {
        "wheel_radius": drive["wheel_radius"],
        "wheel_separation": drive["wheel_separation"],
        "linear.x.max_velocity": motion["max_linear_velocity"],
        "linear.x.min_velocity": -motion["max_linear_velocity"],
        "linear.x.max_acceleration": motion["max_linear_acceleration"],
        "linear.x.min_acceleration": -motion["max_linear_acceleration"],
        "angular.z.max_velocity": motion["max_angular_velocity"],
        "angular.z.min_velocity": -motion["max_angular_velocity"],
        "angular.z.max_acceleration": motion["max_angular_acceleration"],
        "angular.z.min_acceleration": -motion["max_angular_acceleration"],
    }
    for key, expected in expected_controller.items():
        if _get_existing(controllers, base_path + (key,)) != expected:
            raise ValueError(f"generated controllers mismatch: {'.'.join(base_path + (key,))}")
    base = _get_existing(controllers, base_path)
    if "wheel_width" in base:
        raise ValueError("generated controllers must not contain wheel_width")

    for key, expected in (
        ("max_linear_speed", motion["max_linear_velocity"]),
        ("max_angular_speed", motion["max_angular_velocity"]),
    ):
        path = ("robot_web_ui", "ros__parameters", key)
        if _get_existing(web_ui, path) != expected:
            raise ValueError(f"generated web_ui mismatch: {'.'.join(path)}")

    follow_path = (
        "controller_server",
        "ros__parameters",
        "FollowPath",
    )
    for key, expected in (
        ("vx_max", motion["max_linear_velocity"]),
        ("wz_max", motion["max_angular_velocity"]),
    ):
        path = follow_path + (key,)
        if _get_existing(nav2, path) != expected:
            raise ValueError(f"generated nav2 mismatch: {'.'.join(path)}")
    follow_values = _get_existing(nav2, follow_path)
    unsupported = {"ax_max", "ax_min", "az_max"}.intersection(follow_values)
    if unsupported:
        raise ValueError(f"generated nav2 has unsupported keys: {sorted(unsupported)}")

    footprint = _format_footprint(effective["derived"]["geometry"]["footprint"])
    for path in (
        ("global_costmap", "global_costmap", "ros__parameters", "footprint"),
        ("local_costmap", "local_costmap", "ros__parameters", "footprint"),
    ):
        if _get_existing(nav2, path) != footprint:
            raise ValueError(f"generated nav2 mismatch: {'.'.join(path)}")

    expected_time = effective["derived"]["use_sim_time"]
    for mapping, paths in (
        (controllers, CONTROLLER_TIME_PATHS),
        (web_ui, WEB_UI_TIME_PATHS),
        (nav2, NAV2_TIME_PATHS),
    ):
        for path in paths:
            if _get_existing(mapping, path) is not expected_time:
                raise ValueError(f"generated config mismatch: {'.'.join(path)}")


def _render_runtime_configs(inputs):
    """Render and cross-check the three runtime modules without writing files."""
    effective = inputs["effective"]
    templates = inputs["templates"]
    controllers = _render_controller(templates["controllers"], effective)
    web_ui = _render_web_ui(templates["web_ui"], effective)
    nav2 = _render_nav2(templates["nav2"], effective)
    _validate_generated_configs(effective, controllers, web_ui, nav2)
    return {"controllers": controllers, "web_ui": web_ui, "nav2": nav2}


def _prepare_output_dir(output_dir):
    if output_dir is None:
        return Path(tempfile.mkdtemp(prefix="system_bringup-runtime-"))
    target = Path(output_dir).expanduser().resolve()
    target.mkdir(parents=True, exist_ok=True)
    return target


def _stage_yaml(path, data):
    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            delete=False,
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            mode="w",
            encoding="utf-8",
        ) as stream:
            temporary_path = Path(stream.name)
            yaml.safe_dump(
                data,
                stream,
                sort_keys=False,
                allow_unicode=True,
            )
            stream.flush()
        return temporary_path
    except Exception:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise


def _load_staged_yaml(path, label):
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"staged {label} root must be a mapping")
    return data


def compile_runtime_configs(bringup_config_path, output_dir=None):
    inputs = _load_runtime_inputs(bringup_config_path)
    effective = inputs["effective"]
    generated = _render_runtime_configs(inputs)
    generated.update(_render_sensor_configs(inputs))
    robot_launch_arguments = _derive_robot_launch_arguments(inputs["effective"])
    body_weld = _derive_compatibility_body_weld_transform(
        inputs["selected_profile"]
    )
    body_weld_arguments = {
        key: str(value) for key, value in body_weld.items()
    }

    output = _prepare_output_dir(output_dir)
    output_filenames = _output_filenames(inputs["platform"])
    paths = {
        key: output / filename for key, filename in output_filenames.items()
    }
    report = _build_runtime_report(effective, body_weld)
    report["generated_configs"] = {
        key: str(paths[key])
        for key in output_filenames
        if key != "effective_profile"
    }
    manifest = {
        "bringup_config_path": inputs["source"],
        "bringup_config": deepcopy(inputs["config"]),
        "platform": inputs["platform"],
        "mode": inputs["mode"],
        "use_sim_time": effective["derived"]["use_sim_time"],
        "effective_profile_path": paths["effective_profile"],
        "controllers_path": paths["controllers"],
        "web_ui_path": paths["web_ui"],
        "nav2_path": paths["nav2"],
        "robot_launch_arguments": robot_launch_arguments,
        "compatibility_body_weld_arguments": body_weld_arguments,
    }
    for key in SENSOR_OUTPUT_FILENAMES[inputs["platform"]]:
        manifest[f"{key}_path"] = paths[key]
    staged_data = {**generated, "effective_profile": report}
    staged_paths = {}
    try:
        for key in output_filenames:
            staged_paths[key] = _stage_yaml(paths[key], staged_data[key])

        reloaded = {
            key: _load_staged_yaml(staged_paths[key], key)
            for key in output_filenames
        }
        _validate_generated_configs(
            reloaded["effective_profile"],
            reloaded["controllers"],
            reloaded["web_ui"],
            reloaded["nav2"],
        )
        _validate_sensor_generated_configs(
            inputs["platform"],
            reloaded["effective_profile"],
            {
                key: reloaded[key]
                for key in SENSOR_OUTPUT_FILENAMES[inputs["platform"]]
            },
        )

        for key in output_filenames:
            os.replace(staged_paths[key], paths[key])
        return manifest
    finally:
        for temporary_path in staged_paths.values():
            temporary_path.unlink(missing_ok=True)


def main(argv=None):
    parser = _OneLineArgumentParser()
    parser.add_argument("--bringup-config", required=True, type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args(argv)
    try:
        manifest = compile_runtime_configs(
            args.bringup_config,
            output_dir=args.output_dir,
        )
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.exit(2, f"compile_runtime_configs: {exc}\n")
    print(manifest["effective_profile_path"].resolve())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
