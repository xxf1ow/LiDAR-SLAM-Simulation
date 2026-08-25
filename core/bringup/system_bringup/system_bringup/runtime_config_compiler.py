import argparse
from copy import deepcopy
import json
from math import degrees, isfinite
import os
from pathlib import Path
import tempfile

import yaml

from system_bringup import profile_compiler as pc


SUPPORTED_MODES = {"mapping", "navigation"}
MAP_ARTIFACT_KEYS = {"lio_sam_work_dir", "prior_pcd", "nav2_map"}
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
    "fast_lio": "fast_lio.yaml",
    "lio_sam": "lio_sam.yaml",
    "gicp": "gicp.yaml",
}
FAST_LIO_ROOT = ("/**", "ros__parameters")
GICP_ROOT = ("gicp_localization", "ros__parameters")
LIO_SAM_ROOT = ("/**", "ros__parameters")
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
    "fast_lio": "fast_lio.generated.yaml",
    "lio_sam": "lio_sam.generated.yaml",
    "gicp": "gicp.generated.yaml",
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
NAV2_STVL_ROOTS = (
    ("global_costmap", "global_costmap", "ros__parameters", "stvl_layer"),
    ("local_costmap", "local_costmap", "ros__parameters", "stvl_layer"),
)
SENSOR_POINT_TOPIC = "/points_raw"
SENSOR_IMU_TOPIC = "/imu/data"
LIDAR_FRAME = "velodyne"
IMU_FRAME = "imu_link"
SIM_LIDAR_INPUT_TOPIC = "/lidar/points"


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


def runtime_template_filenames(platform):
    if platform not in SENSOR_TEMPLATE_FILENAMES:
        raise ValueError("platform must be 'sim' or 'real'")
    return {
        **TEMPLATE_FILENAMES,
        **SENSOR_TEMPLATE_FILENAMES[platform],
    }


def runtime_manifest_artifacts(platform):
    return {
        f"{artifact_name}_path": artifact_name
        for artifact_name in _output_filenames(platform)
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


def _validate_map_artifacts(config):
    artifacts = config.get("map_artifacts")
    if not isinstance(artifacts, dict) or set(artifacts) != MAP_ARTIFACT_KEYS:
        raise ValueError(
            "bringup config map_artifacts must contain exactly "
            "lio_sam_work_dir, prior_pcd, and nav2_map"
        )

    work_dir = artifacts["lio_sam_work_dir"]
    if not isinstance(work_dir, str):
        raise ValueError("map_artifacts.lio_sam_work_dir must be a string")
    components = work_dir[1:-1].split("/") if len(work_dir) >= 2 else []
    if (
        not work_dir.startswith("/")
        or not work_dir.endswith("/")
        or work_dir == "/"
        or len(components) < 2
        or any(part in {"", ".", ".."} or "~" in part for part in components)
    ):
        raise ValueError(
            "map_artifacts.lio_sam_work_dir must be an absolute-style literal "
            "with at least two safe directory segments and a trailing slash"
        )

    for key in ("prior_pcd", "nav2_map"):
        value = artifacts[key]
        if not isinstance(value, str) or not value.strip():
            raise ValueError(f"map_artifacts.{key} must be a non-empty string")

    return deepcopy(artifacts)


def _load_runtime_inputs(bringup_config_path):
    """Load and validate one immutable-by-convention runtime input snapshot."""
    source, config, platform, profile_paths = pc.load_bringup_context(
        bringup_config_path
    )
    map_artifacts = _validate_map_artifacts(config)
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
        "map_artifacts": map_artifacts,
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
    if expected_type is float:
        valid = type(current) is float
    else:
        valid = type(current) is expected_type
    if not valid:
        raise ValueError(f"template target has wrong type: {dotted}")
    node[leaf] = value


def _set_template_existing(label, mapping, path, value, expected_type):
    try:
        _set_existing(mapping, path, value, expected_type)
    except ValueError as exc:
        raise ValueError(f"{label} template: {exc}") from exc


def _apply_template_overrides(label, mapping, root, overrides):
    for suffix, value, expected_type in overrides:
        _set_template_existing(
            label, mapping, root + suffix, value, expected_type
        )


def _get_existing(mapping, path):
    node = mapping
    dotted = ".".join(path)
    for key in path:
        if not isinstance(node, dict) or key not in node:
            raise ValueError(f"generated config target missing: {dotted}")
        node = node[key]
    return node


def _same_typed_value(actual, expected):
    return type(actual) is type(expected) and actual == expected


def _same_typed_tree(actual, expected):
    if type(actual) is not type(expected):
        return False
    if isinstance(expected, dict):
        return set(actual) == set(expected) and all(
            _same_typed_tree(actual[key], expected[key]) for key in expected
        )
    if isinstance(expected, list):
        return len(actual) == len(expected) and all(
            _same_typed_tree(actual_item, expected_item)
            for actual_item, expected_item in zip(actual, expected)
        )
    return actual == expected


def _finite_float_list(value, length, label):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{label} must be a {length}-element list")
    if any(
        isinstance(item, bool)
        or not isinstance(item, (int, float))
        or not isfinite(item)
        for item in value
    ):
        raise ValueError(f"{label} must contain finite numbers")
    return [float(item) for item in value]


def _rotation_matrix_from_xyzw(value):
    qx, qy, qz, qw = _finite_float_list(value, 4, "imu_from_lidar.rotation_xyzw")
    norm_squared = qx*qx + qy*qy + qz*qz + qw*qw
    if abs(norm_squared - 1.0) > 1e-9:
        raise ValueError("imu_from_lidar.rotation_xyzw must be normalized")
    return [
        1.0 - 2.0*(qy*qy + qz*qz), 2.0*(qx*qy - qz*qw),
        2.0*(qx*qz + qy*qw), 2.0*(qx*qy + qz*qw),
        1.0 - 2.0*(qx*qx + qz*qz), 2.0*(qy*qz - qx*qw),
        2.0*(qx*qz - qy*qw), 2.0*(qy*qz + qx*qw),
        1.0 - 2.0*(qx*qx + qy*qy),
    ]


def _inverse_rotation_matrix_from_xyzw(value):
    qx, qy, qz, qw = _finite_float_list(
        value, 4, "imu_from_lidar.rotation_xyzw"
    )
    return _rotation_matrix_from_xyzw([-qx, -qy, -qz, qw])


def _validate_rotation_matrix(values, label):
    values = _finite_float_list(values, 9, label)
    matrix = [values[0:3], values[3:6], values[6:9]]
    for row in range(3):
        for column in range(3):
            dot = sum(matrix[index][row] * matrix[index][column]
                      for index in range(3))
            expected = 1.0 if row == column else 0.0
            if abs(dot - expected) > 1e-9:
                raise ValueError(f"{label} is not orthonormal")
    determinant = (
        matrix[0][0] * (matrix[1][1]*matrix[2][2] - matrix[1][2]*matrix[2][1])
        - matrix[0][1] * (matrix[1][0]*matrix[2][2] - matrix[1][2]*matrix[2][0])
        + matrix[0][2] * (matrix[1][0]*matrix[2][1] - matrix[1][1]*matrix[2][0])
    )
    if abs(determinant - 1.0) > 1e-9:
        raise ValueError(f"{label} determinant must be 1")


def _fast_lio_scan_rate(lidar):
    value = lidar["scan_rate_hz"]
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not isfinite(value)
        or value <= 0
        or not float(value).is_integer()
    ):
        raise ValueError("FAST-LIO scan_rate_hz must be a finite positive integer value")
    return int(value)


def _fast_lio_scan_lines(lidar):
    value = lidar["scan_lines"]
    if type(value) is not int or value <= 0:
        raise ValueError("FAST-LIO scan_lines must be a positive integer")
    return value


def _format_footprint(footprint):
    return json.dumps(footprint, ensure_ascii=False, separators=(", ", ": "))


def _derive_robot_launch_arguments(effective):
    geometry = effective["derived"]["geometry"]
    body = geometry["body"]
    drive = geometry["drive"]
    mounts = geometry["mounts_relative_to_base_link"]
    lidar_mount = mounts["lidar"]
    imu_mount = mounts["imu"]
    lidar = effective["profile"]["sensors"]["lidar"]
    imu = effective["profile"]["sensors"]["imu"]
    return {
        "base_length": str(body["length"]),
        "base_width": str(body["width"]),
        "base_height": str(body["height"]),
        "base_link_height": str(body["base_link_height"]),
        "wheel_radius": str(drive["wheel_radius"]),
        "wheel_width": str(drive["wheel_width"]),
        "wheel_separation": str(drive["wheel_separation"]),
        "lidar_x": str(lidar_mount["x"]),
        "lidar_y": str(lidar_mount["y"]),
        "lidar_z": str(lidar_mount["z"]),
        "lidar_roll": str(lidar_mount["roll"]),
        "lidar_pitch": str(lidar_mount["pitch"]),
        "lidar_yaw": str(lidar_mount["yaw"]),
        "imu_x": str(imu_mount["x"]),
        "imu_y": str(imu_mount["y"]),
        "imu_z": str(imu_mount["z"]),
        "imu_roll": str(imu_mount["roll"]),
        "imu_pitch": str(imu_mount["pitch"]),
        "imu_yaw": str(imu_mount["yaw"]),
        "lidar_scan_lines": str(lidar["scan_lines"]),
        "lidar_columns_per_scan": str(lidar["columns_per_scan"]),
        "lidar_scan_rate_hz": str(lidar["scan_rate_hz"]),
        "lidar_min_range": str(lidar["min_range"]),
        "lidar_max_range": str(lidar["max_range"]),
        "lidar_horizontal_start_angle": str(lidar["horizontal_start_angle"]),
        "lidar_horizontal_end_angle": str(lidar["horizontal_end_angle"]),
        "imu_rate_hz": str(imu["rate_hz"]),
    }


def _stable_float(value):
    return 0.0 if abs(value) < 1e-15 else value


def _derive_fast_lio_body_bridge_arguments(effective):
    transform = effective["derived"]["geometry"]["relative_transforms"][
        "imu_from_base_footprint"
    ]
    translation = _finite_float_list(
        transform["translation"], 3, "imu_from_base_footprint.translation"
    )
    rotation = _finite_float_list(
        transform["rotation_xyzw"], 4, "imu_from_base_footprint.rotation_xyzw"
    )
    if abs(sum(value * value for value in rotation) - 1.0) > 1e-9:
        raise ValueError("imu_from_base_footprint.rotation_xyzw must be normalized")
    values = [*translation, *rotation]
    return {
        key: str(_stable_float(float(value)))
        for key, value in zip(("x", "y", "z", "qx", "qy", "qz", "qw"), values)
    }


def _build_runtime_report(effective):
    return deepcopy(effective)


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
    perception = effective["profile"]["perception"]
    lidar = effective["profile"]["sensors"]["lidar"]
    footprint = _format_footprint(effective["derived"]["geometry"]["footprint"])

    fixed_behavior_paths = {
        "vx_min": (
            "controller_server", "ros__parameters", "FollowPath", "vx_min"
        ),
        "max_rotational_vel": (
            "behavior_server", "ros__parameters", "max_rotational_vel"
        ),
        "min_rotational_vel": (
            "behavior_server", "ros__parameters", "min_rotational_vel"
        ),
        "rotational_acc_lim": (
            "behavior_server", "ros__parameters", "rotational_acc_lim"
        ),
    }
    fixed_behavior = {}
    for name, path in fixed_behavior_paths.items():
        try:
            value = _get_existing(nav2, path)
        except ValueError as exc:
            raise ValueError(f"nav2 template: {exc}") from exc
        if type(value) is not float or not isfinite(value):
            raise ValueError(f"nav2 template {name} must be a finite float")
        fixed_behavior[name] = value

    if abs(fixed_behavior["vx_min"]) > motion["max_linear_velocity"]:
        raise ValueError("nav2 template vx_min exceeds profile linear capability")
    if fixed_behavior["min_rotational_vel"] > motion["max_angular_velocity"]:
        raise ValueError(
            "nav2 template min_rotational_vel exceeds profile angular capability"
        )
    if fixed_behavior["max_rotational_vel"] > motion["max_angular_velocity"]:
        raise ValueError(
            "nav2 template max_rotational_vel exceeds profile angular capability"
        )
    if fixed_behavior["rotational_acc_lim"] > motion["max_angular_acceleration"]:
        raise ValueError(
            "nav2 template rotational_acc_lim exceeds profile angular acceleration"
        )

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
    min_height = perception["obstacle_height"]["min"]
    max_height = perception["obstacle_height"]["max"]
    clearing_range = perception["clearing_range"]
    lidar_mount_z = effective["profile"]["robot"]["mounts"]["lidar"]["z"]
    min_obstacle_z = min_height - lidar_mount_z
    max_obstacle_z = max_height - lidar_mount_z
    vertical_fov = lidar["vertical_fov_angle"]
    horizontal_fov = (
        lidar["horizontal_end_angle"] - lidar["horizontal_start_angle"]
    )
    for root in NAV2_STVL_ROOTS:
        for suffix, value in (
            (("pointcloud_mark", "obstacle_range"), float(clearing_range["max"])),
            (("pointcloud_mark", "min_obstacle_height"), float(min_obstacle_z)),
            (("pointcloud_mark", "max_obstacle_height"), float(max_obstacle_z)),
            (("pointcloud_clear", "min_z"), float(clearing_range["min"])),
            (("pointcloud_clear", "max_z"), float(clearing_range["max"])),
            (("pointcloud_clear", "vertical_fov_angle"), float(vertical_fov)),
            (("pointcloud_clear", "horizontal_fov_angle"), float(horizontal_fov)),
        ):
            _set_template_existing("nav2", nav2, root + suffix, value, float)
    for path in NAV2_TIME_PATHS:
        _set_template_existing(
            "nav2", nav2, path, effective["derived"]["use_sim_time"], bool
        )
    return nav2


def _render_fast_lio(template, effective):
    fast_lio = deepcopy(template)
    lidar = effective["profile"]["sensors"]["lidar"]
    if lidar["point_time_unit"] != "seconds":
        raise ValueError("FAST-LIO supports only point_time_unit seconds")

    relative = effective["derived"]["geometry"]["relative_transforms"][
        "imu_from_lidar"
    ]
    values = (
        (("preprocess", "scan_line"), _fast_lio_scan_lines(lidar), int),
        (("preprocess", "scan_rate"), _fast_lio_scan_rate(lidar), int),
        (("preprocess", "timestamp_unit"), 0, int),
        (("mapping", "extrinsic_T"),
         _finite_float_list(relative["translation"], 3, "imu_from_lidar.translation"),
         list),
        (("mapping", "extrinsic_R"),
         _rotation_matrix_from_xyzw(relative["rotation_xyzw"]), list),
    )
    for suffix, value, expected_type in values:
        _set_template_existing(
            "fast_lio", fast_lio, FAST_LIO_ROOT + suffix, value, expected_type
        )
    return fast_lio


def _validate_fast_lio_generated(effective, template, generated):
    expected = _render_fast_lio(template, effective)
    _validate_rotation_matrix(
        _get_existing(generated, FAST_LIO_ROOT + ("mapping", "extrinsic_R")),
        "generated fast_lio extrinsic_R",
    )
    if not _same_typed_tree(generated, expected):
        raise ValueError("generated fast_lio does not match template plus overrides")


def _gicp_overrides(effective):
    return (
        (("use_sim_time",), effective["derived"]["use_sim_time"], bool),
    )


def _validate_gicp_generated(effective, template, generated):
    expected = deepcopy(template)
    _apply_template_overrides(
        "gicp", expected, GICP_ROOT, _gicp_overrides(effective)
    )
    if not _same_typed_tree(generated, expected):
        raise ValueError("generated gicp does not match template plus overrides")


def _render_gicp(template, effective):
    gicp = deepcopy(template)
    _apply_template_overrides(
        "gicp", gicp, GICP_ROOT, _gicp_overrides(effective)
    )
    _validate_gicp_generated(effective, template, gicp)
    return gicp


def _lio_sam_overrides(effective, map_artifacts):
    lidar = effective["profile"]["sensors"]["lidar"]
    relative = effective["derived"]["geometry"]["relative_transforms"][
        "imu_from_lidar"
    ]
    return (
        (("use_sim_time",), effective["derived"]["use_sim_time"], bool),
        (("N_SCAN",), lidar["scan_lines"], int),
        (("Horizon_SCAN",), lidar["columns_per_scan"], int),
        (("savePCDDirectory",), map_artifacts["lio_sam_work_dir"], str),
        (("extrinsicTrans",), _finite_float_list(
            relative["translation"], 3, "imu_from_lidar.translation"
        ), list),
        (("extrinsicRot",), _inverse_rotation_matrix_from_xyzw(
            relative["rotation_xyzw"]
        ), list),
    )


def _validate_lio_sam_generated(
    effective, map_artifacts, template, generated
):
    expected = deepcopy(template)
    _apply_template_overrides(
        "lio_sam",
        expected,
        LIO_SAM_ROOT,
        _lio_sam_overrides(effective, map_artifacts),
    )
    rotation = _get_existing(generated, LIO_SAM_ROOT + ("extrinsicRot",))
    _validate_rotation_matrix(rotation, "generated lio_sam extrinsicRot")
    if not _same_typed_tree(generated, expected):
        raise ValueError("generated lio_sam does not match template plus overrides")


def _render_lio_sam(template, effective, map_artifacts):
    lio_sam = deepcopy(template)
    _apply_template_overrides(
        "lio_sam",
        lio_sam,
        LIO_SAM_ROOT,
        _lio_sam_overrides(effective, map_artifacts),
    )
    _validate_lio_sam_generated(
        effective, map_artifacts, template, lio_sam
    )
    return lio_sam


def _render_lidar_adapter(template, effective):
    adapter = deepcopy(template)
    for key, value, expected_type in (
        ("use_sim_time", effective["derived"]["use_sim_time"], bool),
        (
            "scan_period",
            float(effective["derived"]["sensor_contract"]["scan_period"]),
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
        ("start_angle", float(degrees(lidar["horizontal_start_angle"])), float),
        ("end_angle", float(degrees(lidar["horizontal_end_angle"])), float),
        ("min_distance", float(lidar["min_range"]), float),
        ("max_distance", float(lidar["max_range"]), float),
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
        ("expected_point_hz", float(lidar["scan_rate_hz"]), float),
        ("expected_imu_hz", float(imu["rate_hz"]), float),
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


def _validate_sensor_generated_configs(platform, effective, templates, generated):
    expected = {"sensor_gate": _render_sensor_gate(
        templates["sensor_gate"], effective
    )}
    if platform == "sim":
        expected["lidar_adapter"] = _render_lidar_adapter(
            templates["lidar_adapter"], effective
        )
    elif platform == "real":
        expected["vanjee_lidar"] = _render_vanjee_lidar(
            templates["vanjee_lidar"], effective
        )
    else:
        raise ValueError("platform must be 'sim' or 'real'")

    if set(generated) != set(expected):
        raise ValueError(
            f"generated sensor configs mismatch: expected {sorted(expected)}"
        )
    for name, expected_tree in expected.items():
        if not _same_typed_tree(generated[name], expected_tree):
            raise ValueError(
                f"generated {name} does not match template plus overrides"
            )

    gate = generated["sensor_gate"]
    gate_path = ("sensor_contract_gate", "ros__parameters")
    for key in ("minimum_point_rate_ratio", "minimum_imu_rate_ratio"):
        path = gate_path + (key,)
        value = _get_existing(gate, path)
        if (
            type(value) is not float
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
            type(value) is not float
            or not isfinite(value)
            or value <= 0
        ):
            raise ValueError(
                f"generated sensor_gate value must be > 0: {'.'.join(path)}"
            )


def _runtime_protocol_value(tree, path, expected, label):
    actual = _get_existing(tree, path)
    if type(actual) is not type(expected) or actual != expected:
        raise ValueError(
            f"runtime protocol {label} must be {expected!r}; got {actual!r}"
        )
    return actual


def _validate_runtime_protocol(platform, generated):
    fast_lio = generated["fast_lio"]
    fast_lio_point_topic = _get_existing(
        fast_lio, FAST_LIO_ROOT + ("common", "lid_topic")
    )
    fast_lio_imu_topic = _get_existing(
        fast_lio, FAST_LIO_ROOT + ("common", "imu_topic")
    )

    if platform == "sim":
        adapter = generated["lidar_adapter"]
        adapter_root = ("lidar_pointcloud_adapter", "ros__parameters")
        _runtime_protocol_value(
            adapter,
            adapter_root + ("input_topic",),
            SIM_LIDAR_INPUT_TOPIC,
            "lidar adapter input_topic",
        )
        adapter_point_topic = _get_existing(
            adapter, adapter_root + ("output_topic",)
        )
        if adapter_point_topic != fast_lio_point_topic:
            raise ValueError(
                "runtime protocol lidar adapter output_topic must match "
                "FAST-LIO common.lid_topic"
            )
        _runtime_protocol_value(
            adapter,
            adapter_root + ("output_frame",),
            LIDAR_FRAME,
            "lidar adapter output_frame",
        )
    elif platform == "real":
        vanjee = generated["vanjee_lidar"]
        vanjee_root = ("vanjee_lidar", "ros__parameters")
        vanjee_point_topic = _get_existing(
            vanjee, vanjee_root + ("point_cloud_topic",)
        )
        vanjee_imu_topic = _get_existing(
            vanjee, vanjee_root + ("imu_topic",)
        )
        if vanjee_point_topic != fast_lio_point_topic:
            raise ValueError(
                "runtime protocol Vanjee point_cloud_topic must match "
                "FAST-LIO common.lid_topic"
            )
        if vanjee_imu_topic != fast_lio_imu_topic:
            raise ValueError(
                "runtime protocol Vanjee imu_topic must match "
                "FAST-LIO common.imu_topic"
            )
        _runtime_protocol_value(
            vanjee,
            vanjee_root + ("lidar_frame",),
            LIDAR_FRAME,
            "Vanjee lidar_frame",
        )
        _runtime_protocol_value(
            vanjee,
            vanjee_root + ("imu_frame",),
            IMU_FRAME,
            "Vanjee imu_frame",
        )
    else:
        raise ValueError("platform must be 'sim' or 'real'")

    _runtime_protocol_value(
        fast_lio,
        FAST_LIO_ROOT + ("common", "lid_topic"),
        SENSOR_POINT_TOPIC,
        "FAST-LIO point topic / sensor-gate point topic",
    )
    _runtime_protocol_value(
        fast_lio,
        FAST_LIO_ROOT + ("common", "imu_topic"),
        SENSOR_IMU_TOPIC,
        "FAST-LIO IMU topic / sensor-gate IMU topic",
    )

    gicp = generated["gicp"]
    for key, expected in (
        ("map_frame", "map"),
        ("odom_frame", "camera_init"),
        ("base_frame", "body"),
        ("cloud_topic", "/cloud_registered_body"),
        ("odom_topic", "/Odometry"),
    ):
        _runtime_protocol_value(
            gicp,
            GICP_ROOT + (key,),
            expected,
            f"GICP {key}",
        )

    _runtime_protocol_value(
        generated["controllers"],
        ("base_controller", "ros__parameters", "base_frame_id"),
        "base_footprint",
        "controller base_frame_id",
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
    _validate_sensor_generated_configs(
        inputs["platform"], effective, templates, generated
    )
    return generated


def _validate_generated_configs(effective, templates, controllers, web_ui, nav2):
    expected = {
        "controllers": _render_controller(templates["controllers"], effective),
        "web_ui": _render_web_ui(templates["web_ui"], effective),
        "nav2": _render_nav2(templates["nav2"], effective),
    }
    actual = {
        "controllers": controllers,
        "web_ui": web_ui,
        "nav2": nav2,
    }
    for name in expected:
        if not _same_typed_tree(actual[name], expected[name]):
            raise ValueError(
                f"generated {name} does not match template plus overrides"
            )

    base = _get_existing(controllers, ("base_controller", "ros__parameters"))
    if "wheel_width" in base:
        raise ValueError("generated controllers must not contain wheel_width")
    follow = _get_existing(
        nav2, ("controller_server", "ros__parameters", "FollowPath")
    )
    unsupported = {"ax_max", "ax_min", "az_max"}.intersection(follow)
    if unsupported:
        raise ValueError(f"generated nav2 has unsupported keys: {sorted(unsupported)}")


def _render_runtime_configs(inputs):
    """Render and cross-check shared runtime modules without writing files."""
    effective = inputs["effective"]
    templates = inputs["templates"]
    controllers = _render_controller(templates["controllers"], effective)
    web_ui = _render_web_ui(templates["web_ui"], effective)
    nav2 = _render_nav2(templates["nav2"], effective)
    fast_lio = _render_fast_lio(templates["fast_lio"], effective)
    lio_sam = _render_lio_sam(
        templates["lio_sam"], effective, inputs["map_artifacts"]
    )
    gicp = _render_gicp(templates["gicp"], effective)
    _validate_generated_configs(effective, templates, controllers, web_ui, nav2)
    _validate_fast_lio_generated(effective, templates["fast_lio"], fast_lio)
    _validate_lio_sam_generated(
        effective, inputs["map_artifacts"], templates["lio_sam"], lio_sam
    )
    _validate_gicp_generated(effective, templates["gicp"], gicp)
    return {
        "controllers": controllers,
        "web_ui": web_ui,
        "nav2": nav2,
        "fast_lio": fast_lio,
        "lio_sam": lio_sam,
        "gicp": gicp,
    }


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
    _validate_runtime_protocol(inputs["platform"], generated)
    robot_launch_arguments = _derive_robot_launch_arguments(inputs["effective"])
    fast_lio_body_bridge_arguments = _derive_fast_lio_body_bridge_arguments(
        effective
    )
    output = _prepare_output_dir(output_dir)
    output_filenames = _output_filenames(inputs["platform"])
    paths = {
        key: output / filename for key, filename in output_filenames.items()
    }
    report = _build_runtime_report(effective)
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
        "robot_launch_arguments": robot_launch_arguments,
        "fast_lio_body_bridge_arguments": fast_lio_body_bridge_arguments,
    }
    manifest.update(
        {
            manifest_key: paths[artifact_name]
            for manifest_key, artifact_name in runtime_manifest_artifacts(
                inputs["platform"]
            ).items()
        }
    )
    staged_data = {**generated, "effective_profile": report}
    staged_paths = {}
    try:
        for key in output_filenames:
            staged_paths[key] = _stage_yaml(paths[key], staged_data[key])

        reloaded = {
            key: _load_staged_yaml(staged_paths[key], key)
            for key in output_filenames
        }
        if not _same_typed_tree(reloaded["effective_profile"], report):
            raise ValueError(
                "staged effective_profile does not match in-memory report"
            )
        _validate_generated_configs(
            reloaded["effective_profile"],
            inputs["templates"],
            reloaded["controllers"],
            reloaded["web_ui"],
            reloaded["nav2"],
        )
        _validate_fast_lio_generated(
            reloaded["effective_profile"],
            inputs["templates"]["fast_lio"],
            reloaded["fast_lio"],
        )
        _validate_lio_sam_generated(
            reloaded["effective_profile"],
            inputs["map_artifacts"],
            inputs["templates"]["lio_sam"],
            reloaded["lio_sam"],
        )
        _validate_gicp_generated(
            reloaded["effective_profile"],
            inputs["templates"]["gicp"],
            reloaded["gicp"],
        )
        _validate_sensor_generated_configs(
            inputs["platform"],
            reloaded["effective_profile"],
            inputs["sensor_templates"],
            {
                key: reloaded[key]
                for key in SENSOR_OUTPUT_FILENAMES[inputs["platform"]]
            },
        )
        _validate_runtime_protocol(inputs["platform"], reloaded)
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
