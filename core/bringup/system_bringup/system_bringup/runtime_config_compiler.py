from copy import deepcopy
import json
from math import isfinite

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

def _validate_mode(config):
    mode = config.get("mode")
    if mode not in SUPPORTED_MODES or isinstance(mode, bool):
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
