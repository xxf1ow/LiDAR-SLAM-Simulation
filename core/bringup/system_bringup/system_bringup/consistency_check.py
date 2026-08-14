"""Validate compiled runtime manifests and generated configuration artifacts."""
import tempfile
from pathlib import Path

try:
    import yaml
except ImportError:  # pyyaml is a runtime dependency; report it through failures.
    yaml = None

def _runtime_artifacts(platform, runtime_compiler):
    try:
        return runtime_compiler.runtime_manifest_artifacts(platform)
    except (TypeError, ValueError):
        output_filenames = {
            **runtime_compiler.COMMON_OUTPUT_FILENAMES,
            "effective_profile": runtime_compiler.EFFECTIVE_PROFILE_FILENAME,
        }
        return {
            f"{artifact_name}_path": artifact_name
            for artifact_name in output_filenames
        }


_MISSING = object()
_PATH_ERRORS = (OSError, TypeError, ValueError, RuntimeError)


def _nested_value(mapping, path):
    value = mapping
    for key in path:
        if not isinstance(value, dict) or key not in value:
            return _MISSING
        value = value[key]
    return value


def _normalize_path(value, label, failures, require_absolute=False):
    try:
        path = Path(value).expanduser()
        if require_absolute and not path.is_absolute():
            failures.append(f"{label} must be absolute: {value!r}")
            return None
        return path.resolve()
    except _PATH_ERRORS as exc:
        failures.append(f"{label} has invalid path value {value!r}: {exc}")
        return None


def _same_path(left, right, left_label, right_label, failures):
    left_path = _normalize_path(left, left_label, failures)
    right_path = _normalize_path(right, right_label, failures)
    if left_path is None or right_path is None:
        return False
    return left_path == right_path


def _load_runtime_artifacts(manifest, failures, runtime_compiler, artifacts):
    paths = {}
    loaded = {}
    try:
        output_filenames = runtime_compiler._output_filenames(
            manifest.get("platform")
        )
    except (TypeError, ValueError):
        output_filenames = {
            **runtime_compiler.COMMON_OUTPUT_FILENAMES,
            "effective_profile": runtime_compiler.EFFECTIVE_PROFILE_FILENAME,
        }
    temp_root = _normalize_path(
        tempfile.gettempdir(), "OS temporary directory", failures
    )

    for manifest_key, artifact_name in artifacts.items():
        raw_path = manifest.get(manifest_key, _MISSING)
        if raw_path is _MISSING:
            failures.append(f"manifest missing {manifest_key}")
            continue
        path = _normalize_path(
            raw_path,
            f"manifest {manifest_key}",
            failures,
            require_absolute=True,
        )
        if path is None:
            continue
        paths[manifest_key] = path
        if path.name != output_filenames[artifact_name]:
            failures.append(
                f"manifest {manifest_key} has unexpected artifact filename: {path}"
            )
        if not path.is_file():
            failures.append(f"manifest {manifest_key} file does not exist: {path}")
            continue
        if temp_root is not None:
            try:
                path.relative_to(temp_root)
            except ValueError:
                failures.append(
                    f"manifest {manifest_key} must be inside the OS temporary directory: {path}"
                )

    if paths:
        reference_dir = paths.get("effective_profile_path", next(iter(paths.values()))).parent
        if not reference_dir.name.startswith("system_bringup-runtime-"):
            failures.append(
                f"manifest runtime directory must start with system_bringup-runtime-: {reference_dir}"
            )
        for manifest_key, path in paths.items():
            if path.parent != reference_dir:
                failures.append(
                    f"manifest {manifest_key} must use the same runtime directory "
                    f"as effective_profile_path: {path.parent} != {reference_dir}"
                )

    for manifest_key, artifact_name in artifacts.items():
        path = paths.get(manifest_key)
        if path is None or not path.is_file():
            continue
        try:
            with path.open("r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream)
        except (OSError, RuntimeError, yaml.YAMLError) as exc:
            failures.append(f"cannot load manifest {manifest_key} {path}: {exc}")
            continue
        if not isinstance(data, dict):
            failures.append(f"manifest {manifest_key} root must be a mapping: {path}")
            continue
        loaded[artifact_name] = data

    return paths, loaded


def _load_runtime_template(
    source_path, artifact_name, template_filename, runtime_compiler, failures
):
    if source_path is None or not source_path.is_file():
        return None
    template_path = (
        source_path.parent
        / "templates"
        / template_filename
    )
    try:
        _, template = runtime_compiler._load_template(
            template_path, artifact_name
        )
    except (
        KeyError,
        OSError,
        RuntimeError,
        TypeError,
        ValueError,
        yaml.YAMLError,
    ) as exc:
        failures.append(
            f"cannot load source {artifact_name} template {template_path}: {exc}"
        )
        return None
    return template


_ACTIVE_RUNTIME_FILES = {
    "formal": "core/bringup/system_bringup/launch/bringup.launch.py",
    "slam": "core/bringup/system_bringup/launch/slam_stack.launch.py",
    "sim": "core/simulation/robot_gz_bringup/launch/robot_gz.launch.py",
    "real_chassis": "core/robot/robot_bringup/launch/real_chassis.launch.py",
    "real_robot": "core/robot/robot_bringup/launch/robot.launch.py",
    "navigation": "core/navigation/robot_navigation/launch/navigation.launch.py",
    "gicp_launch": (
        "core/localization/gicp_localization/launch/localization.launch.py"
    ),
    "vanjee_launch": (
        "core/robot/drivers/lidar_vanjee_722/vanjee_lidar_ros/launch/"
        "vanjee_lidar.launch.py"
    ),
    "cmd_gate": "core/robot/cmd_vel_gate/cmd_vel_gate/gate_node.py",
    "web_ui": "core/bringup/robot_web_ui/robot_web_ui/web_ui_node.py",
    "lidar_adapter": (
        "core/simulation/lidar_pointcloud_adapter/"
        "lidar_pointcloud_adapter/adapter_node.py"
    ),
    "profile_compiler": (
        "core/bringup/system_bringup/system_bringup/profile_compiler.py"
    ),
    "runtime_config_compiler": (
        "core/bringup/system_bringup/system_bringup/runtime_config_compiler.py"
    ),
    "consistency_check": (
        "core/bringup/system_bringup/system_bringup/consistency_check.py"
    ),
    "sensor_gate_logic": (
        "core/bringup/system_bringup/system_bringup/sensor_gate_logic.py"
    ),
    "sensor_gate_node": (
        "core/bringup/system_bringup/system_bringup/sensor_gate_node.py"
    ),
}


_INSTALLED_RUNTIME_SHARES = {
    "formal": ("system_bringup", "launch/bringup.launch.py"),
    "slam": ("system_bringup", "launch/slam_stack.launch.py"),
    "sim": ("robot_gz_bringup", "launch/robot_gz.launch.py"),
    "real_chassis": ("robot_bringup", "launch/real_chassis.launch.py"),
    "real_robot": ("robot_bringup", "launch/robot.launch.py"),
    "navigation": ("robot_navigation", "launch/navigation.launch.py"),
    "gicp_launch": ("gicp_localization", "launch/localization.launch.py"),
    "vanjee_launch": ("vanjee_lidar_ros", "launch/vanjee_lidar.launch.py"),
}
_INSTALLED_RUNTIME_MODULES = {
    "cmd_gate": "cmd_vel_gate.gate_node",
    "web_ui": "robot_web_ui.web_ui_node",
    "lidar_adapter": "lidar_pointcloud_adapter.adapter_node",
    "profile_compiler": "system_bringup.profile_compiler",
    "runtime_config_compiler": "system_bringup.runtime_config_compiler",
    "consistency_check": "system_bringup.consistency_check",
    "sensor_gate_logic": "system_bringup.sensor_gate_logic",
    "sensor_gate_node": "system_bringup.sensor_gate_node",
}


def _resolve_installed_runtime_paths(failures):
    """Resolve files ROS will actually load; imports stay lazy for portable tooling."""
    try:
        import importlib.util
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except (ImportError, ModuleNotFoundError) as exc:
        failures.append(
            "active installed runtime cannot be resolved outside a sourced ROS "
            "environment; package shares and node modules "
            "cmd_vel_gate.gate_node, robot_web_ui.web_ui_node, and "
            "lidar_pointcloud_adapter.adapter_node are required: "
            f"{exc}"
        )
        return {}

    paths = {}
    for label, (package, relative_path) in _INSTALLED_RUNTIME_SHARES.items():
        try:
            share = get_package_share_directory(package)
        except (PackageNotFoundError, OSError, RuntimeError, ValueError) as exc:
            failures.append(
                f"active installed runtime package {package} cannot be resolved: {exc}"
            )
            continue
        share_path = _normalize_path(
            share,
            f"active installed runtime package share {package}",
            failures,
        )
        if share_path is None:
            continue
        path = _normalize_path(
            share_path / relative_path,
            f"active installed runtime {package}/{relative_path}",
            failures,
        )
        if path is not None:
            paths[label] = path

    for label, module_name in _INSTALLED_RUNTIME_MODULES.items():
        try:
            spec = importlib.util.find_spec(module_name)
        except (ImportError, ModuleNotFoundError, AttributeError, ValueError) as exc:
            failures.append(f"active installed {module_name} cannot be resolved: {exc}")
            continue
        origin = None if spec is None else spec.origin
        path = _normalize_path(
            origin,
            f"active installed {module_name}",
            failures,
        )
        if path is not None:
            paths[label] = path
    return paths


def _validate_installed_freshness(repo_root, failures):
    """Ensure files ROS will load match the reviewed source bytes."""
    reviewed_paths = {}
    for label, relative_path in _ACTIVE_RUNTIME_FILES.items():
        path = _normalize_path(
            repo_root / relative_path,
            f"reviewed runtime source {relative_path}",
            failures,
        )
        if path is not None:
            reviewed_paths[label] = path

    active_paths = _resolve_installed_runtime_paths(failures)
    if set(active_paths) != set(_ACTIVE_RUNTIME_FILES):
        missing = sorted(set(_ACTIVE_RUNTIME_FILES) - set(active_paths))
        extra = sorted(set(active_paths) - set(_ACTIVE_RUNTIME_FILES))
        failures.append(
            "active installed runtime path set is incomplete: "
            f"missing={missing}, extra={extra}"
        )
        return

    for label, relative_path in _ACTIVE_RUNTIME_FILES.items():
        reviewed_path = reviewed_paths.get(label)
        active_path = active_paths[label]
        if reviewed_path is None:
            continue
        try:
            reviewed_bytes = reviewed_path.read_bytes()
            active_bytes = (
                reviewed_bytes
                if active_path == reviewed_path
                else active_path.read_bytes()
            )
        except (OSError, RuntimeError) as exc:
            failures.append(
                f"active runtime {relative_path} cannot be read from reviewed/installed "
                f"paths {reviewed_path}/{active_path}: {exc}"
            )
            continue
        if active_bytes != reviewed_bytes:
            failures.append(
                "active installed runtime differs from reviewed source; rebuild the "
                "workspace; run colcon build before launch: "
                f"{active_path} != {reviewed_path}"
            )


def run_runtime_consistency(repo_root, manifest):
    """Validate one compiled runtime manifest without rereading or regenerating it."""
    failures = []
    if yaml is None:
        return ["runtime consistency requires PyYAML"]
    from system_bringup import runtime_config_compiler as rcc

    if not isinstance(manifest, dict):
        return [f"runtime manifest must be a mapping; got {type(manifest).__name__}"]

    root = _normalize_path(repo_root, "repo_root", failures)
    if root is not None and not root.is_dir():
        failures.append(f"repo_root is not a directory: {root}")

    config = manifest.get("bringup_config")
    if not isinstance(config, dict):
        failures.append("manifest bringup_config must be a mapping")
        config = {}

    platform = manifest.get("platform")
    if platform not in ("sim", "real"):
        failures.append(f"manifest platform must be 'sim' or 'real'; got {platform!r}")
    else:
        unselected_sensor_path = (
            "vanjee_lidar_path" if platform == "sim" else "lidar_adapter_path"
        )
        if unselected_sensor_path in manifest:
            failures.append(
                f"manifest {unselected_sensor_path} is not valid for platform {platform}"
            )
    mode = manifest.get("mode")
    if not isinstance(mode, str) or mode not in rcc.SUPPORTED_MODES:
        failures.append(
            f"manifest mode must be 'mapping' or 'navigation'; got {mode!r}"
        )
    use_sim_time = manifest.get("use_sim_time")
    if not isinstance(use_sim_time, bool):
        failures.append(
            f"manifest use_sim_time must be a boolean; got {use_sim_time!r}"
        )

    if config.get("platform", _MISSING) != platform:
        failures.append(
            f"manifest platform {platform!r} != bringup_config.platform "
            f"{config.get('platform', _MISSING)!r}"
        )
    if config.get("mode", _MISSING) != mode:
        failures.append(
            f"manifest mode {mode!r} != bringup_config.mode "
            f"{config.get('mode', _MISSING)!r}"
        )

    source_path = manifest.get("bringup_config_path", _MISSING)
    if source_path is _MISSING:
        failures.append("manifest missing bringup_config_path")
        source_path = None
    else:
        source_path = _normalize_path(
            source_path,
            "manifest bringup_config_path",
            failures,
            require_absolute=True,
        )
        if source_path is not None:
            if not source_path.is_file():
                failures.append(
                    f"manifest bringup_config_path file does not exist: {source_path}"
                )
            if root is not None:
                try:
                    source_path.relative_to(root)
                except ValueError:
                    failures.append(
                        f"manifest bringup_config_path is outside repo_root: {source_path}"
                    )

    artifacts = _runtime_artifacts(platform, rcc)
    paths, loaded = _load_runtime_artifacts(
        manifest, failures, rcc, artifacts
    )
    source_templates = {}
    source_template_filenames = {}
    if platform in ("sim", "real"):
        source_template_filenames = rcc.runtime_template_filenames(platform)
        for name, filename in source_template_filenames.items():
            template = _load_runtime_template(
                source_path, name, filename, rcc, failures
            )
            if template is not None:
                source_templates[name] = template
    report = loaded.get("effective_profile")
    if report is not None:
        if report.get("platform", _MISSING) != platform:
            failures.append(
                f"effective report platform {report.get('platform', _MISSING)!r} "
                f"!= manifest platform {platform!r}"
            )
        report_clock = _nested_value(report, ("derived", "use_sim_time"))
        if report_clock is _MISSING or report_clock is not use_sim_time:
            failures.append(
                f"effective report derived.use_sim_time {report_clock!r} "
                f"!= manifest use_sim_time {use_sim_time!r}"
            )

        if source_path is not None and platform in ("sim", "real"):
            profiles = config.get("profiles")
            profile_ref = profiles.get(platform, _MISSING) if isinstance(profiles, dict) else _MISSING
            if not isinstance(profile_ref, str) or not profile_ref:
                failures.append(
                    f"manifest bringup_config profiles.{platform} must be a non-empty string"
                )
            else:
                expected_profile = _normalize_path(
                    source_path.parent / profile_ref,
                    f"manifest bringup_config profiles.{platform}",
                    failures,
                )
                report_profile = report.get("source_profile", _MISSING)
                if expected_profile is not None and (
                    report_profile is _MISSING
                    or not _same_path(
                        report_profile,
                        expected_profile,
                        "effective report source_profile",
                        f"manifest bringup_config profiles.{platform}",
                        failures,
                    )
                ):
                    failures.append(
                        f"effective report source_profile {report_profile!r} "
                        f"!= selected profile {expected_profile}"
                    )

        expected_backends = {
            "sim": {"chassis": "gazebo", "lidar": "gazebo"},
            "real": {"chassis": "can_8030d", "lidar": "vanjee"},
        }
        if isinstance(platform, str) and platform in expected_backends:
            for component, expected in expected_backends[platform].items():
                actual = _nested_value(
                    report, ("profile", "hardware", component, "backend")
                )
                if actual != expected:
                    failures.append(
                        f"effective report {component} backend {actual!r} "
                        f"!= expected {expected!r} for platform {platform}"
                    )

        generated_refs = report.get("generated_configs")
        if isinstance(generated_refs, dict):
            expected_generated_keys = {
                artifact_name
                for artifact_name in artifacts.values()
                if artifact_name != "effective_profile"
            }
            if set(generated_refs) != expected_generated_keys:
                failures.append(
                    "effective report generated_configs keys "
                    f"{sorted(map(repr, generated_refs))} != expected "
                    f"{sorted(expected_generated_keys)} for platform {platform}"
                )
        for manifest_key, artifact_name in artifacts.items():
            if artifact_name == "effective_profile":
                continue
            expected_path = paths.get(manifest_key)
            actual_path = (
                generated_refs.get(artifact_name, _MISSING)
                if isinstance(generated_refs, dict)
                else _MISSING
            )
            if expected_path is not None and not _same_path(
                actual_path,
                expected_path,
                f"effective report generated_configs.{artifact_name}",
                f"manifest {manifest_key}",
                failures,
            ):
                failures.append(
                    f"effective report generated_configs.{artifact_name} "
                    f"{actual_path!r} != manifest {manifest_key} {expected_path}"
                )

        try:
            expected_geometry = rcc._derive_robot_launch_arguments(report)
        except (KeyError, TypeError, ValueError) as exc:
            failures.append(f"effective report geometry is invalid: {exc}")
        else:
            actual_geometry = manifest.get("robot_launch_arguments")
            for key, expected in expected_geometry.items():
                actual = (
                    actual_geometry.get(key, _MISSING)
                    if isinstance(actual_geometry, dict)
                    else _MISSING
                )
                if actual != expected:
                    failures.append(
                        f"manifest robot_launch_arguments.{key} {actual!r} "
                        f"!= effective geometry {expected!r}"
                    )

        try:
            expected_bridge = rcc._derive_fast_lio_body_bridge_arguments(report)
        except (KeyError, TypeError, ValueError) as exc:
            failures.append(f"effective report FAST-LIO body bridge is invalid: {exc}")
        else:
            actual_bridge = manifest.get("fast_lio_body_bridge_arguments")
            for key, expected in expected_bridge.items():
                actual = (
                    actual_bridge.get(key, _MISSING)
                    if isinstance(actual_bridge, dict)
                    else _MISSING
                )
                if actual != expected:
                    failures.append(
                        f"manifest fast_lio_body_bridge_arguments.{key} "
                        f"{actual!r} != effective bridge {expected!r}"
                    )

    if (
        report is not None
        and all(name in loaded for name in ("controllers", "web_ui", "nav2"))
        and all(name in source_templates for name in ("controllers", "web_ui", "nav2"))
    ):
        try:
            rcc._validate_generated_configs(
                report,
                source_templates,
                loaded["controllers"],
                loaded["web_ui"],
                loaded["nav2"],
            )
        except (KeyError, TypeError, ValueError) as exc:
            failures.append(f"generated runtime config mismatch: {exc}")

    if (
        isinstance(platform, str)
        and platform in rcc.SENSOR_OUTPUT_FILENAMES
    ):
        sensor_names = set(source_template_filenames) - set(rcc.TEMPLATE_FILENAMES)
        if report is not None and all(
            name in loaded for name in sensor_names
        ) and all(name in source_templates for name in sensor_names):
            try:
                rcc._validate_sensor_generated_configs(
                    platform,
                    report,
                    {name: source_templates[name] for name in sensor_names},
                    {key: loaded[key] for key in sensor_names},
                )
            except (KeyError, TypeError, ValueError) as exc:
                failures.append(f"generated runtime config mismatch: {exc}")

    if (
        report is not None
        and "fast_lio" in loaded
        and "fast_lio" in source_templates
    ):
        try:
            rcc._validate_fast_lio_generated(
                report,
                source_templates["fast_lio"],
                loaded["fast_lio"],
            )
        except (KeyError, TypeError, ValueError) as exc:
            failures.append(f"generated fast_lio validation failed: {exc}")

    if report is not None and "gicp" in loaded and "gicp" in source_templates:
        try:
            rcc._validate_gicp_generated(
                report, source_templates["gicp"], loaded["gicp"]
            )
        except (KeyError, TypeError, ValueError) as exc:
            failures.append(f"generated gicp validation failed: {exc}")

    if report is not None and "lio_sam" in loaded and "lio_sam" in source_templates:
        try:
            rcc._validate_lio_sam_generated(
                report,
                config.get("map_artifacts"),
                source_templates["lio_sam"],
                loaded["lio_sam"],
            )
        except (KeyError, TypeError, ValueError) as exc:
            failures.append(f"generated lio_sam validation failed: {exc}")

    if isinstance(platform, str) and platform in rcc.SENSOR_OUTPUT_FILENAMES:
        protocol_names = {
            "controllers",
            "fast_lio",
            "gicp",
            *rcc.SENSOR_OUTPUT_FILENAMES[platform],
        }
        if all(name in loaded for name in protocol_names):
            try:
                rcc._validate_runtime_protocol(platform, loaded)
            except (KeyError, TypeError, ValueError) as exc:
                failures.append(f"generated runtime protocol mismatch: {exc}")

    if root is not None and root.is_dir():
        _validate_installed_freshness(root, failures)
    return failures
