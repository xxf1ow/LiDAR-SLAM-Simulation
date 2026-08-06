from copy import deepcopy
from math import isfinite
from pathlib import Path

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
        name: source.parent / "templates" / filename
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
