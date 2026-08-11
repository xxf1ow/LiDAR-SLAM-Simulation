import argparse
import copy
from ipaddress import ip_address
import math
import sys
import tempfile
from pathlib import Path, PureWindowsPath

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

_SUPPORTED_LIDAR = {
    "sim": ("gazebo", "gpu_lidar"),
    "real": ("vanjee", "vanjee_722"),
}
_POINT_TIME_CONTRACT = ("time", "seconds", "scan_start")
_NETWORK_KEYS = (
    "host_address",
    "device_address",
    "host_msop_port",
    "device_msop_port",
)

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


def _validate_bringup_selection(source, config):
    platform = config.get("platform")
    if platform not in ("sim", "real") or isinstance(platform, bool):
        raise ValueError(f"{source}: platform must be 'sim' or 'real'")

    selected = config.get("profiles")
    if not isinstance(selected, dict):
        raise ValueError(f"{source}: profiles must be a mapping")
    missing = sorted({"sim", "real"} - set(selected))
    extra = sorted(
        set(selected) - {"sim", "real"},
        key=lambda item: (type(item).__name__, repr(item)),
    )
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
        if (
            relative.is_absolute()
            or relative.anchor
            or relative.drive
            or PureWindowsPath(raw).anchor
        ):
            raise ValueError(f"{source}: profiles.{name} must be relative")
        resolved = (source.parent / relative).resolve()
        if not resolved.is_file():
            raise ValueError(f"{source}: profiles.{name} does not exist: {resolved}")
        paths[name] = resolved
    return platform, paths


def load_bringup_context(path):
    """Return (resolved_source, parsed_mapping, platform, profile_paths)."""
    source, config = _read_yaml_mapping(path, "bringup config")
    platform, paths = _validate_bringup_selection(source, config)
    return source, config, platform, paths


def load_bringup_selection(path):
    _, _, platform, paths = load_bringup_context(path)
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
        extra = sorted(
            set(value) - set(schema),
            key=lambda item: (type(item).__name__, repr(item)),
        )
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
    if isinstance(value, (int, float)):
        try:
            finite = math.isfinite(value)
        except OverflowError:
            finite = False
        if not finite:
            raise ValueError(f"{source}: {_field_name(path)} must be finite")


def _leaf_paths(value, prefix=()):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from _leaf_paths(child, prefix + (key,))
        return
    yield prefix


def _value_at(profile, path):
    value = profile
    for key in path:
        value = value[key]
    return value


def _validate_semantics(profile, source):
    positive = (
        ("robot", "body", "front_extent"),
        ("robot", "body", "rear_extent"),
        ("robot", "body", "left_extent"),
        ("robot", "body", "right_extent"),
        ("robot", "body", "height"),
        ("robot", "drive", "wheel_radius"),
        ("robot", "drive", "wheel_width"),
        ("robot", "drive", "wheel_separation"),
        ("sensors", "lidar", "scan_lines"),
        ("sensors", "lidar", "columns_per_scan"),
        ("sensors", "lidar", "scan_rate_hz"),
    )
    for path in positive:
        if _value_at(profile, path) <= 0:
            raise ValueError(f"{source}: {_field_name(path)} must be > 0")
    clearance_path = ("robot", "body", "ground_clearance")
    if _value_at(profile, clearance_path) < 0:
        raise ValueError(
            f"{source}: {_field_name(clearance_path)} must be >= 0"
        )


def _validate_sensor_semantics(platform, profile, source):
    hardware = profile["hardware"]["lidar"]
    actual = (hardware["backend"], hardware["model"])
    expected = _SUPPORTED_LIDAR[platform]
    if actual != expected:
        raise ValueError(
            f"{source}: {platform} hardware.lidar must be "
            f"{expected[0]}/{expected[1]}"
        )

    if platform == "sim":
        for key in _NETWORK_KEYS:
            if hardware[key] is not None:
                raise ValueError(
                    f"{source}: sim hardware.lidar.{key} must be null"
                )
    else:
        for key in ("host_address", "device_address"):
            try:
                parsed = ip_address(hardware[key])
            except (TypeError, ValueError) as exc:
                raise ValueError(
                    f"{source}: real hardware.lidar.{key} must be an IPv4 literal"
                ) from exc
            if parsed.version != 4:
                raise ValueError(
                    f"{source}: real hardware.lidar.{key} must be an IPv4 literal"
                )
        for key in ("host_msop_port", "device_msop_port"):
            value = hardware[key]
            if (
                isinstance(value, bool)
                or not isinstance(value, int)
                or not 1 <= value <= 65535
            ):
                raise ValueError(
                    f"{source}: real hardware.lidar.{key} must be in 1..65535"
                )

    lidar = profile["sensors"]["lidar"]
    imu = profile["sensors"]["imu"]
    if imu["rate_hz"] <= 0:
        raise ValueError(f"{source}: sensors.imu.rate_hz must be > 0")
    if lidar["min_range"] < 0:
        raise ValueError(f"{source}: sensors.lidar.min_range must be >= 0")
    if lidar["max_range"] <= lidar["min_range"]:
        raise ValueError(
            f"{source}: sensors.lidar.max_range must be greater than min_range"
        )
    if platform == "real" and not (
        0 <= lidar["horizontal_start_angle"] <= math.tau
        and 0 <= lidar["horizontal_end_angle"] <= math.tau
    ):
        raise ValueError(
            f"{source}: real sensors.lidar horizontal angles must be in [0, 2*pi]"
        )
    span = lidar["horizontal_end_angle"] - lidar["horizontal_start_angle"]
    if span <= 0 or span > math.tau:
        raise ValueError(
            f"{source}: sensors.lidar horizontal span must be in (0, 2*pi]"
        )
    point_time = (
        lidar["point_time_field"],
        lidar["point_time_unit"],
        lidar["point_time_reference"],
    )
    if point_time != _POINT_TIME_CONTRACT:
        raise ValueError(
            f"{source}: sensors.lidar point time must be time/seconds/scan_start"
        )


def validate_profile_pair(profiles):
    if set(profiles) != {"sim", "real"}:
        raise ValueError("profile pair must contain exactly sim and real")
    for name in ("sim", "real"):
        source, profile = profiles[name]
        _validate_node(profile, _PROFILE_SCHEMA, Path(source))
        _validate_semantics(profile, Path(source))
        _validate_sensor_semantics(name, profile, Path(source))
    sim_paths = set(_leaf_paths(profiles["sim"][1]))
    real_paths = set(_leaf_paths(profiles["real"][1]))
    if sim_paths != real_paths:
        raise ValueError("sim and real profiles must have identical leaf paths")


def _stable_float(value):
    return 0.0 if abs(value) < 1e-15 else float(value)


def _normalize_quaternion(values):
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        for value in values
    ):
        raise ValueError("derived transform quaternion must contain finite numbers")
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm <= 0.0:
        raise ValueError("derived transform quaternion norm must be positive")
    return tuple(_stable_float(value / norm) for value in values)


def _quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return _normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def _quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return _normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def _rotate_vector(rotation, vector):
    qx, qy, qz, qw = rotation
    matrix = (
        (1.0 - 2.0 * (qy*qy + qz*qz), 2.0 * (qx*qy - qz*qw),
         2.0 * (qx*qz + qy*qw)),
        (2.0 * (qx*qy + qz*qw), 1.0 - 2.0 * (qx*qx + qz*qz),
         2.0 * (qy*qz - qx*qw)),
        (2.0 * (qx*qz - qy*qw), 2.0 * (qy*qz + qx*qw),
         1.0 - 2.0 * (qx*qx + qy*qy)),
    )
    return tuple(
        _stable_float(sum(matrix[row][column] * vector[column]
                          for column in range(3)))
        for row in range(3)
    )


def _transform_from_mount(mount):
    return (
        (float(mount["x"]), float(mount["y"]), float(mount["z"])),
        _quaternion_from_rpy(mount["roll"], mount["pitch"], mount["yaw"]),
    )


def _invert_transform(transform):
    translation, rotation = transform
    qx, qy, qz, qw = rotation
    inverse_rotation = (-qx, -qy, -qz, qw)
    inverse_translation = _rotate_vector(
        inverse_rotation, tuple(-value for value in translation)
    )
    return inverse_translation, inverse_rotation


def _compose_transforms(left, right):
    left_translation, left_rotation = left
    right_translation, right_rotation = right
    rotated = _rotate_vector(left_rotation, right_translation)
    return (
        tuple(_stable_float(a + b) for a, b in zip(left_translation, rotated)),
        _quaternion_multiply(left_rotation, right_rotation),
    )


def _transform_report(transform):
    translation, rotation = transform
    return {
        "translation": list(translation),
        "rotation_xyzw": list(_normalize_quaternion(rotation)),
    }


def _mount_relative_to_base_link(mount, base_link_height):
    result = copy.deepcopy(mount)
    result["z"] = mount["z"] - base_link_height
    return result


def derive_effective_profile(platform, source_path, profile):
    body = profile["robot"]["body"]
    drive = profile["robot"]["drive"]
    mounts = profile["robot"]["mounts"]
    lidar = profile["sensors"]["lidar"]

    front = body["front_extent"]
    rear = body["rear_extent"]
    left = body["left_extent"]
    right = body["right_extent"]
    base_link_height = body["ground_clearance"] + body["height"] / 2.0

    base_from_lidar = _transform_from_mount(mounts["lidar"])
    base_from_imu = _transform_from_mount(mounts["imu"])
    imu_from_base = _invert_transform(base_from_imu)
    geometry = {
        "body": {
            "length": front + rear,
            "width": left + right,
            "height": body["height"],
            "center_x": (front - rear) / 2.0,
            "center_y": (left - right) / 2.0,
            "base_link_height": base_link_height,
        },
        "drive": copy.deepcopy(drive),
        "footprint": [
            [front, left],
            [front, -right],
            [-rear, -right],
            [-rear, left],
        ],
        "mounts_relative_to_base_link": {
            name: _mount_relative_to_base_link(mount, base_link_height)
            for name, mount in mounts.items()
        },
        "relative_transforms": {
            "imu_from_lidar": _transform_report(
                _compose_transforms(imu_from_base, base_from_lidar)
            ),
            "imu_from_base_footprint": _transform_report(imu_from_base),
        },
    }
    return {
        "platform": platform,
        "source_profile": str(Path(source_path).resolve()),
        "profile": copy.deepcopy(profile),
        "derived": {
            "use_sim_time": platform == "sim",
            "geometry": geometry,
            "sensor_contract": {
                "points_per_scan": (
                    lidar["scan_lines"] * lidar["columns_per_scan"]
                ),
                "scan_period": 1.0 / lidar["scan_rate_hz"],
            },
        },
    }


def compile_profile(bringup_config_path, output_dir=None):
    platform, paths = load_bringup_selection(bringup_config_path)
    profiles = {
        name: (path, load_profile(path))
        for name, path in paths.items()
    }
    validate_profile_pair(profiles)
    source, selected = profiles[platform]
    effective = derive_effective_profile(platform, source, selected)

    if output_dir is None:
        directory = Path(tempfile.mkdtemp(prefix="system_bringup-profile-"))
    else:
        directory = Path(output_dir).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
    output = directory / "effective_profile.generated.yaml"
    output.write_text(
        yaml.safe_dump(effective, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return output


class _ProfileCompilerArgumentParser(argparse.ArgumentParser):
    def error(self, message):
        self.exit(1, f"profile compilation failed: {message}\n")


def main(argv=None):
    parser = _ProfileCompilerArgumentParser(
        description="Validate and compile a selected platform Profile"
    )
    parser.add_argument(
        "--bringup-config",
        required=True,
        help="Path to source bringup.yaml",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Generated file directory; default is a private temporary directory",
    )
    try:
        args = parser.parse_args(argv)
    except SystemExit as exc:
        return 0 if exc.code == 0 else 1
    try:
        output = compile_profile(args.bringup_config, args.output_dir)
    except (OSError, ValueError) as exc:
        print(f"profile compilation failed: {exc}", file=sys.stderr)
        return 1
    print(output.resolve())
    return 0


if __name__ == "__main__":
    sys.exit(main())
