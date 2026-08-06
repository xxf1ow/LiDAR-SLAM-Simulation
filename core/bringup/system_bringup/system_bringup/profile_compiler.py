import math
from pathlib import Path

import yaml


_STRING = ((str,), False)
_NULLABLE_STRING = ((str,), True)
_INTEGER = ((int,), False)
_NULLABLE_INTEGER = ((int,), True)
_NUMBER = ((int, float), False)
_NULLABLE_NUMBER = ((int, float), True)

_MOUNT_SCHEMA = {
    "x": _NUMBER,
    "y": _NUMBER,
    "z": _NUMBER,
    "roll": _NUMBER,
    "pitch": _NUMBER,
    "yaw": _NUMBER,
}

_PROFILE_SCHEMA = {
    "hardware": {
        "chassis": {
            "backend": _STRING,
        },
        "lidar": {
            "backend": _STRING,
            "model": _STRING,
            "host_address": _NULLABLE_STRING,
            "device_address": _NULLABLE_STRING,
            "host_msop_port": _NULLABLE_INTEGER,
            "device_msop_port": _NULLABLE_INTEGER,
        },
    },
    "robot": {
        "body": {
            "front_extent": _NUMBER,
            "rear_extent": _NUMBER,
            "left_extent": _NUMBER,
            "right_extent": _NUMBER,
            "height": _NUMBER,
            "ground_clearance": _NUMBER,
        },
        "drive": {
            "wheel_radius": _NUMBER,
            "wheel_width": _NUMBER,
            "wheel_separation": _NUMBER,
        },
        "mounts": {
            "lidar": _MOUNT_SCHEMA,
            "imu": _MOUNT_SCHEMA,
        },
    },
    "sensors": {
        "lidar": {
            "scan_lines": _INTEGER,
            "columns_per_scan": _INTEGER,
            "scan_rate_hz": _NUMBER,
            "min_range": _NUMBER,
            "max_range": _NUMBER,
            "horizontal_start_angle": _NUMBER,
            "horizontal_end_angle": _NUMBER,
            "point_time_field": _STRING,
            "point_time_unit": _STRING,
            "point_time_reference": _STRING,
        },
        "imu": {
            "rate_hz": _NUMBER,
        },
    },
    "motion": {
        "max_linear_velocity": _NULLABLE_NUMBER,
        "max_angular_velocity": _NULLABLE_NUMBER,
        "max_linear_acceleration": _NULLABLE_NUMBER,
        "max_angular_acceleration": _NULLABLE_NUMBER,
    },
    "perception": {
        "obstacle_height": {
            "min": _NULLABLE_NUMBER,
            "max": _NULLABLE_NUMBER,
        },
    },
}


def _read_yaml_mapping(path, label):
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise ValueError(f"{source}: {label} file does not exist")
    try:
        value = yaml.safe_load(source.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"{source}: invalid YAML: {exc}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{source}: {label} root must be a mapping")
    return source, value


def load_bringup_selection(path):
    source, config = _read_yaml_mapping(path, "bringup config")
    platform = config.get("platform")
    if platform not in ("sim", "real") or isinstance(platform, bool):
        raise ValueError(f"{source}: platform must be 'sim' or 'real'")

    selected = config.get("profiles")
    if not isinstance(selected, dict):
        raise ValueError(f"{source}: profiles must be a mapping")
    missing = sorted({"sim", "real"} - set(selected))
    extra = sorted(set(selected) - {"sim", "real"})
    if missing or extra:
        raise ValueError(
            f"{source}: profiles keys invalid; missing={missing}, unexpected={extra}"
        )

    paths = {}
    for name in ("sim", "real"):
        raw = selected[name]
        if not isinstance(raw, str) or not raw.strip():
            raise ValueError(f"{source}: profiles.{name} must be a non-empty string")
        relative = Path(raw)
        if relative.is_absolute():
            raise ValueError(f"{source}: profiles.{name} must be relative")
        resolved = (source.parent / relative).resolve()
        if not resolved.is_file():
            raise ValueError(f"{source}: profiles.{name} does not exist: {resolved}")
        paths[name] = resolved
    return platform, paths


def load_profile(path):
    return _read_yaml_mapping(path, "profile")[1]


def _field_name(path):
    return ".".join(path) if path else "<root>"


def _validate_node(value, schema, source, path=()):
    if isinstance(schema, dict):
        if not isinstance(value, dict):
            raise ValueError(f"{source}: {_field_name(path)} must be a mapping")
        missing = sorted(set(schema) - set(value))
        extra = sorted(set(value) - set(schema))
        if missing:
            raise ValueError(f"{source}: {_field_name(path)} missing keys: {missing}")
        if extra:
            raise ValueError(f"{source}: {_field_name(path)} unexpected keys: {extra}")
        for key, child_schema in schema.items():
            _validate_node(value[key], child_schema, source, path + (key,))
        return

    allowed_types, nullable = schema
    if value is None:
        if nullable:
            return
        raise ValueError(f"{source}: {_field_name(path)} must not be null")
    if isinstance(value, bool) and any(
        expected in (int, float) for expected in allowed_types
    ):
        raise ValueError(f"{source}: {_field_name(path)} must be numeric, not bool")
    if not isinstance(value, allowed_types):
        expected = "/".join(item.__name__ for item in allowed_types)
        raise ValueError(f"{source}: {_field_name(path)} must have type {expected}")
    if isinstance(value, str) and not value.strip():
        raise ValueError(f"{source}: {_field_name(path)} must be non-empty")
    if isinstance(value, (int, float)) and not math.isfinite(value):
        raise ValueError(f"{source}: {_field_name(path)} must be finite")


def _leaf_paths(value, prefix=()):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from _leaf_paths(child, prefix + (key,))
        return
    yield prefix


def validate_profile_pair(profiles):
    if set(profiles) != {"sim", "real"}:
        raise ValueError("profile pair must contain exactly sim and real")
    for name in ("sim", "real"):
        source, profile = profiles[name]
        _validate_node(profile, _PROFILE_SCHEMA, Path(source))
    sim_paths = set(_leaf_paths(profiles["sim"][1]))
    real_paths = set(_leaf_paths(profiles["real"][1]))
    if sim_paths != real_paths:
        raise ValueError("sim and real profiles must have identical leaf paths")
