"""跨模块魔法值一致性检查(纯 Python,无 ROS 依赖,本机/构建机皆可跑)。

解析已跟踪源文件；仿真检查既有默认值，真机从 bringup.yaml 单一源生成后检查。
- run(repo_root) -> list[str]   失败描述(空=全过)
- main()        -> int          打印失败、返回退出码(供启动闸门/CLI)
- find_repo_root()              从 __file__ 上溯定位仓库根(pytest 本机用)

契约类(帧/话题)不纳入。patch 文件只信任 '+'(项目改值)行。
"""
import argparse
import ast
from collections import Counter
import math
import os
from pathlib import Path
import re
import sys
import tempfile

try:
    import yaml
except ImportError:  # pyyaml 是硬依赖
    yaml = None

# ---- 已跟踪源文件(仓库相对路径) ----
F_MACRO = "core/robot/robot_description/urdf/robot_macro.urdf.xacro"
F_ROBOT_XACRO = "core/robot/robot_description/urdf/robot.urdf.xacro"
F_GAZEBO = "core/robot/robot_description/gazebo/robot.gazebo.xacro"
F_CONTROLLERS = "core/robot/robot_bringup/config/robot_controllers.yaml"
F_NAV_PARAMS = "core/navigation/robot_navigation/config/nav2_params.yaml"
F_NAV_PARAMS_REAL = "core/navigation/robot_navigation/config/nav2_params_real.yaml"
F_NAV_LAUNCH = "core/navigation/robot_navigation/launch/navigation.launch.py"
F_GZ_LAUNCH = "core/simulation/robot_gz_bringup/launch/robot_gz.launch.py"
F_FASTLIO_PATCH = "core/localization/fast-lio2.patch"
F_LIOSAM_PATCH = "core/mapping/lio-sam.patch"
F_VANJEE_PARAMS = (
    "core/robot/drivers/lidar_vanjee_722/"
    "vanjee_lidar_ros/config/vanjee_722.yaml"
)

FASTLIO_CONFIG = {
    "sim": "config/gazebo_velodyne.yaml",
    "real": "config/vanjee_722.yaml",
}
LIOSAM_CONFIG = {
    "sim": "config/params.yaml",
    "real": "config/params_real.yaml",
}
_MARKER = os.path.join("core", "bringup", "system_bringup")


# ---- 仓库根定位 ----
def find_repo_root(start=None):
    here = os.path.abspath(start or __file__)
    d = here if os.path.isdir(here) else os.path.dirname(here)
    while True:
        if os.path.isdir(os.path.join(d, _MARKER)) or os.path.isdir(os.path.join(d, ".git")):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            raise RuntimeError("找不到仓库根(向上未见 %s 或 .git);start=%s" % (_MARKER, start))
        d = parent


def load_bringup_config(repo_root=None):
    """读源码 bringup config(launch 运行时调,不经 install —— 改 config 不用 rebuild)。"""
    repo_root = repo_root or find_repo_root()
    cfg_path = os.path.join(repo_root, "core", "bringup", "system_bringup", "config", "bringup.yaml")
    with open(cfg_path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def require_runtime_config_file(path, component):
    """在启动外部 SLAM 节点前确认所选安装态配置实际存在。"""
    resolved = os.fspath(path)
    if not os.path.isfile(resolved):
        raise RuntimeError(
            "%s 运行时配置不存在: %s；请 apply 对应 patch 并 rebuild。"
            % (component, resolved)
        )
    return resolved


def derive_real_geometry(config):
    """从 bringup.yaml 的实测值派生各模块需要的真机几何。"""
    measured = config["real_geometry"]
    body = measured["body"]
    wheel = measured["drive_wheel"]
    lidar = measured["lidar"]

    body_length = float(body["length"])
    body_width = float(body["width"])
    body_height = float(body["height"])
    ground_clearance = float(body["ground_clearance"])
    base_link_height = ground_clearance + body_height / 2.0
    wheel_diameter = float(wheel["diameter"])
    wheel_width = float(wheel["width"])
    wheel_separation = float(wheel["separation"])
    lidar_x = float(lidar["x"])
    lidar_y = float(lidar["y"])
    lidar_z = float(lidar["z"])
    roll = float(lidar["roll"])
    pitch = float(lidar["pitch"])
    yaw = float(lidar["yaw"])

    values = {
        "body.length": body_length,
        "body.width": body_width,
        "body.height": body_height,
        "body.ground_clearance": ground_clearance,
        "drive_wheel.diameter": wheel_diameter,
        "drive_wheel.width": wheel_width,
        "drive_wheel.separation": wheel_separation,
        "lidar.x": lidar_x,
        "lidar.y": lidar_y,
        "lidar.z": lidar_z,
        "lidar.roll": roll,
        "lidar.pitch": pitch,
        "lidar.yaw": yaw,
    }
    for name, value in values.items():
        if not math.isfinite(value):
            raise ValueError("real_geometry.%s 必须是有限数。" % name)
    for name in (
        "body.length", "body.width", "body.height",
        "drive_wheel.diameter", "drive_wheel.width",
        "drive_wheel.separation", "lidar.z",
    ):
        if values[name] <= 0.0:
            raise ValueError("real_geometry.%s 必须大于 0。" % name)
    if ground_clearance < 0.0:
        raise ValueError("real_geometry.body.ground_clearance 不能小于 0。")
    if wheel_separation + wheel_width > body_width + 1e-12:
        raise ValueError("real_geometry 轮子外缘宽度超过 body.width。")
    if any(abs(angle) > 1e-12 for angle in (roll, pitch, yaw)):
        raise ValueError("real_geometry.lidar 当前仅支持零安装角；非零角需同时实现刚体逆变换。")

    return {
        "body": {
            "length": body_length,
            "width": body_width,
            "height": body_height,
            "base_link_height": base_link_height,
        },
        "drive_wheel": {
            "radius": wheel_diameter / 2.0,
            "width": wheel_width,
            "separation": wheel_separation,
        },
        "sensor": {
            "x": lidar_x,
            "y": lidar_y,
            "z": lidar_z - base_link_height,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
        },
        "body_to_base_footprint": {
            "x": -lidar_x,
            "y": -lidar_y,
            "z": -lidar_z,
            "roll": -roll,
            "pitch": -pitch,
            "yaw": -yaw,
        },
        "footprint": [
            [body_length / 2.0, body_width / 2.0],
            [body_length / 2.0, -body_width / 2.0],
            [-body_length / 2.0, -body_width / 2.0],
            [-body_length / 2.0, body_width / 2.0],
        ],
    }


def _read(repo_root, relpath):
    with open(os.path.join(repo_root, *relpath.split("/")), encoding="utf-8") as f:
        return f.read()


def _yaml(text):
    return yaml.safe_load(text)


_RUNTIME_ARTIFACTS = {
    "controllers_path": "controllers",
    "web_ui_path": "web_ui",
    "nav2_path": "nav2",
    "effective_profile_path": "effective_profile",
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


def _require_runtime_value(mapping, path, predicate, failures, expectation):
    value = _nested_value(mapping, path)
    dotted = ".".join(path)
    if value is _MISSING:
        failures.append(f"manifest bringup_config missing {dotted}")
    elif not predicate(value):
        failures.append(
            f"manifest bringup_config {dotted} must be {expectation}; got {value!r}"
        )


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


def _load_runtime_artifacts(manifest, failures, runtime_compiler):
    paths = {}
    loaded = {}
    temp_root = _normalize_path(
        tempfile.gettempdir(), "OS temporary directory", failures
    )

    for manifest_key, artifact_name in _RUNTIME_ARTIFACTS.items():
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
        if path.name != runtime_compiler.OUTPUT_FILENAMES[artifact_name]:
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

    for manifest_key, artifact_name in _RUNTIME_ARTIFACTS.items():
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


def _validate_unmigrated_runtime_config(config, platform, failures):
    nonempty_string = lambda value: isinstance(value, str) and bool(value.strip())
    nonnegative_number = lambda value: (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
        and value >= 0.0
    )
    _require_runtime_value(
        config,
        ("slam_stack", "settling"),
        nonnegative_number,
        failures,
        "a finite number >= 0",
    )
    for path in (
        ("slam_stack", platform, "lio_sam", "config"),
        ("slam_stack", platform, "fast_lio", "config"),
        ("slam_stack", platform, "gicp_localization", "config"),
        ("slam_stack", platform, "gicp_localization", "prior_map_path"),
        ("slam_stack", platform, "robot_navigation", "config"),
        ("slam_stack", platform, "robot_navigation", "map"),
    ):
        _require_runtime_value(
            config, path, nonempty_string, failures, "a non-empty string"
        )

    if platform == "sim":
        for key in ("gui", "rviz", "world", "spawn_x", "spawn_y", "spawn_z"):
            _require_runtime_value(
                config,
                ("robot_gz", key),
                nonempty_string,
                failures,
                "a non-empty string",
            )
    elif platform == "real":
        _require_runtime_value(
            config,
            ("robot_bringup", "use_mock_hardware"),
            lambda value: isinstance(value, bool),
            failures,
            "a boolean",
        )
        _require_runtime_value(
            config,
            ("vanjee_lidar", "config"),
            nonempty_string,
            failures,
            "a non-empty string",
        )


_ACTIVE_TOPOLOGY_FILES = {
    "formal": "core/bringup/system_bringup/launch/bringup.launch.py",
    "slam": "core/bringup/system_bringup/launch/slam_stack.launch.py",
    "sim": "core/simulation/robot_gz_bringup/launch/robot_gz.launch.py",
    "real_chassis": "core/robot/robot_bringup/launch/real_chassis.launch.py",
    "real_robot": "core/robot/robot_bringup/launch/robot.launch.py",
    "navigation": "core/navigation/robot_navigation/launch/navigation.launch.py",
    "cmd_gate": "core/robot/cmd_vel_gate/cmd_vel_gate/gate_node.py",
}
_ROBOT_ARGUMENTS = (
    "controllers_file",
    "base_length",
    "base_width",
    "base_height",
    "base_link_height",
    "wheel_radius",
    "wheel_width",
    "wheel_separation",
    "sensor_x",
    "sensor_y",
    "sensor_z",
    "sensor_roll",
    "sensor_pitch",
    "sensor_yaw",
    "use_sim_time",
)
_PROHIBITED_ACTIVE_LEGACY = {
    "derive_real_geometry",
    "build_real_runtime_configs",
    "write_real_runtime_configs",
    "real_geometry_launch_arguments",
    "load_bringup_config",
    "run",
}


def _active_source_trees(repo_root, failures):
    trees = {}
    rendered = {}
    strings = {}
    for label, relative_path in _ACTIVE_TOPOLOGY_FILES.items():
        path = _normalize_path(
            repo_root / relative_path,
            f"active topology source {relative_path}",
            failures,
        )
        if path is None:
            continue
        if not path.is_file():
            failures.append(f"active topology source file does not exist: {path}")
            continue
        try:
            source = path.read_text(encoding="utf-8")
            tree = ast.parse(source, filename=str(path))
        except (OSError, RuntimeError, SyntaxError, UnicodeError) as exc:
            failures.append(f"active topology source {relative_path} is invalid: {exc}")
            continue
        trees[label] = tree
        rendered[label] = ast.unparse(tree)
        strings[label] = Counter(
            node.value
            for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
        )
    return trees, rendered, strings


def _contract_failure(failures, category, details):
    failures.append(f"active topology {category} drift: {details}")


def _validate_active_topology(repo_root, failures):
    trees, rendered, strings = _active_source_trees(repo_root, failures)
    if set(trees) != set(_ACTIVE_TOPOLOGY_FILES):
        return

    formal = rendered["formal"]
    slam = rendered["slam"]
    sim = rendered["sim"]
    real_chassis = rendered["real_chassis"]
    real_robot = rendered["real_robot"]
    navigation = rendered["navigation"]
    cmd_gate = rendered["cmd_gate"]

    generated_contract = (
        "parameters=[str(manifest['web_ui_path'])]" in formal
        and "'nav2_params_file': str(manifest['nav2_path'])" in formal
        and formal.count("'controllers_file': str(manifest['controllers_path'])") == 2
        and formal.count("compile_runtime_configs(source_config)") == 1
        and formal.count("run_runtime_consistency(repo_root, manifest)") == 1
    )
    if not generated_contract:
        _contract_failure(
            failures,
            "generated runtime artifacts",
            "formal bringup must consume generated Web UI/Nav2 and one controller path "
            "for each backend after exactly one compile/check",
        )

    sim_missing = [
        name for name in _ROBOT_ARGUMENTS if strings["sim"][name] < 2
    ]
    sim_contract = (
        not sim_missing
        and "'gz_controllers_file:=', controllers_file" in sim
        and "base = _inc('robot_gz_bringup', 'launch/robot_gz.launch.py'" in formal
        and formal.count("'use_sim_time': use_sim, **geometry") >= 2
    )
    if not sim_contract:
        _contract_failure(
            failures,
            "simulation backend interface",
            f"controller/full geometry/use_sim_time forwarding is incomplete; missing={sim_missing}",
        )

    real_chassis_missing = [
        name for name in _ROBOT_ARGUMENTS if strings["real_chassis"][name] < 2
    ]
    real_robot_missing = [
        name for name in _ROBOT_ARGUMENTS if strings["real_robot"][name] < 2
    ]
    real_contract = (
        not real_chassis_missing
        and not real_robot_missing
        and "chassis = _inc('robot_bringup', 'launch/real_chassis.launch.py'" in formal
        and "parameters=[controllers_file, {'use_sim_time': use_sim_time}]"
        in real_robot
    )
    if not real_contract:
        _contract_failure(
            failures,
            "real backend interface",
            "controller/full geometry/use_sim_time forwarding is incomplete; "
            f"real_chassis missing={real_chassis_missing}, robot missing={real_robot_missing}",
        )

    routing_contract = (
        all(
            strings["cmd_gate"][topic] >= 1
            for topic in ("/cmd_vel_auto", "/cmd_vel_manual", "/cmd_vel")
        )
        and "'cmd_vel_output_topic': '/cmd_vel_auto'" in formal
        and "'input_topic': '/cmd_vel_nav'" in navigation
        and "'output_topic': cmd_vel_output_topic" in navigation
        and "('/base_controller/cmd_vel', '/cmd_vel')" in real_robot
    )
    if not routing_contract:
        _contract_failure(
            failures,
            "command gate routing",
            "/cmd_vel_auto and /cmd_vel_manual must be the only gate inputs and /cmd_vel its output",
        )

    weld_sources = "\n".join((formal, slam, navigation))
    weld_contract = (
        all(name not in weld_sources for name in ("weld_roll", "weld_pitch", "weld_yaw"))
        and all(
            strings[label][name] >= 1
            for label in ("formal", "slam", "navigation")
            for name in ("weld_qx", "weld_qy", "weld_qz", "weld_qw")
        )
        and "'--qx', weld_qx, '--qy', weld_qy, '--qz', weld_qz, '--qw', weld_qw"
        in navigation
    )
    if not weld_contract:
        _contract_failure(
            failures,
            "quaternion weld",
            "formal/shared/navigation weld must use qx/qy/qz/qw only",
        )

    readiness_contract = (
        "ready_gate(['/points_raw', '/joint_states'], 300.0" in formal
        and "ready_gate(['/Odometry', '/cloud_registered'], 60.0" in slam
        and "ready_gate(['/localization', '/base_controller/odom'], 60.0" in slam
        and "[gicp] + ready_gate(" in slam
        and "[nav2], use_sim_time=use_sim, settling=settling" in slam
    )
    if not readiness_contract:
        _contract_failure(
            failures,
            "readiness sequence",
            "sim, FAST-LIO, GICP and Nav2 readiness topics/timeouts/order changed",
        )

    frame_contract = (
        "'--frame-id', 'body', '--child-frame-id', 'base_footprint'" in navigation
        and "'frame_id': 'base_link'" in navigation
        and "'output_topic': '/points_raw', 'output_frame': 'velodyne'" in sim
        and "output.header.frame_id = 'base_link'" in cmd_gate
    )
    if not frame_contract:
        _contract_failure(
            failures,
            "critical frames",
            "body/base_footprint, base_link or velodyne contract changed",
        )

    sequencing_contract = (
        "return control_layer + flow_log + [base] + ready_gate(" in formal
        and "return control_layer + flow_log + [chassis, lidar] + "
        "_real_sensor_gate([slam_stack], use_sim_time)" in formal
    )
    if not sequencing_contract:
        _contract_failure(
            failures,
            "backend sequencing",
            "backend must start before its readiness gate and shared slam_stack",
        )

    prohibited = set()
    for node in ast.walk(trees["formal"]):
        if isinstance(node, ast.ImportFrom):
            prohibited.update(
                alias.name
                for alias in node.names
                if alias.name in _PROHIBITED_ACTIVE_LEGACY
            )
        elif isinstance(node, ast.Call):
            name = (
                node.func.id
                if isinstance(node.func, ast.Name)
                else node.func.attr
                if isinstance(node.func, ast.Attribute)
                else None
            )
            if name in _PROHIBITED_ACTIVE_LEGACY:
                prohibited.add(name)
    if prohibited:
        _contract_failure(
            failures,
            "prohibited active legacy",
            f"formal bringup imports/calls {sorted(prohibited)}",
        )


def _validate_report_metadata(report, expected_weld, runtime_compiler, failures):
    body_weld = {key: float(value) for key, value in expected_weld.items()}
    try:
        expected_report = runtime_compiler._build_runtime_report(report, body_weld)
    except (KeyError, TypeError, ValueError) as exc:
        failures.append(f"effective report metadata cannot be derived: {exc}")
        return

    expected_body = expected_report["compatibility"]["body_to_base_footprint"]
    for key in ("status", "assumption", "follow_up_section"):
        path = ("compatibility", "body_to_base_footprint", key)
        actual = _nested_value(report, path)
        expected = expected_body[key]
        if actual != expected:
            failures.append(
                f"effective report {'.'.join(path)} {actual!r} != {expected!r}"
            )

    expected_deferred = next(
        entry
        for entry in expected_report["deferred_compatibility"]
        if entry.get("component") == "nav2.behavior_server"
    )
    actual_entries = report.get("deferred_compatibility")
    actual_deferred = (
        next(
            (
                entry
                for entry in actual_entries
                if isinstance(entry, dict)
                and entry.get("component") == "nav2.behavior_server"
            ),
            None,
        )
        if isinstance(actual_entries, list)
        else None
    )
    prefix = "deferred_compatibility.nav2.behavior_server"
    if actual_deferred is None:
        failures.append(f"effective report missing {prefix}")
        return

    for path in (
        ("status",),
        ("template_values", "max_rotational_vel"),
        ("template_values", "min_rotational_vel"),
        ("template_values", "rotational_acc_lim"),
        ("profile_values", "max_angular_velocity"),
        ("profile_values", "max_angular_acceleration"),
        ("reason",),
    ):
        actual = _nested_value(actual_deferred, path)
        expected = _nested_value(expected_deferred, path)
        if actual != expected:
            failures.append(
                f"effective report {prefix}.{'.'.join(path)} "
                f"{actual!r} != {expected!r}"
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

    paths, loaded = _load_runtime_artifacts(manifest, failures, rcc)
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
        for manifest_key, artifact_name in _RUNTIME_ARTIFACTS.items():
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

        profile = report.get("profile")
        try:
            expected_weld = rcc._derive_compatibility_body_weld_arguments(profile)
        except (KeyError, TypeError, ValueError, ZeroDivisionError) as exc:
            failures.append(f"effective report compatibility weld is invalid: {exc}")
        else:
            actual_weld = manifest.get("compatibility_body_weld_arguments")
            for key, expected in expected_weld.items():
                actual = (
                    actual_weld.get(key, _MISSING)
                    if isinstance(actual_weld, dict)
                    else _MISSING
                )
                if actual != expected:
                    failures.append(
                        f"manifest compatibility_body_weld_arguments.{key} "
                        f"{actual!r} != effective weld {expected!r}"
                    )
                section = "translation" if key in ("x", "y", "z") else "rotation"
                report_value = _nested_value(
                    report,
                    ("compatibility", "body_to_base_footprint", section, key),
                )
                expected_report_value = float(expected)
                if (
                    isinstance(report_value, bool)
                    or not isinstance(report_value, (int, float))
                    or report_value != expected_report_value
                ):
                    failures.append(
                        "effective report "
                        f"compatibility.body_to_base_footprint.{section}.{key} "
                        f"{report_value!r} != derived/manifest weld "
                        f"{expected_report_value!r}/{actual!r}"
                    )
            _validate_report_metadata(report, expected_weld, rcc, failures)

    if all(name in loaded for name in _RUNTIME_ARTIFACTS.values()):
        try:
            rcc._validate_generated_configs(
                loaded["effective_profile"],
                loaded["controllers"],
                loaded["web_ui"],
                loaded["nav2"],
            )
        except (KeyError, TypeError, ValueError) as exc:
            failures.append(f"generated runtime config mismatch: {exc}")

    if platform in ("sim", "real"):
        _validate_unmigrated_runtime_config(config, platform, failures)
    if root is not None and root.is_dir():
        _validate_active_topology(root, failures)
    return failures


# ---- 解析器 ----
def _xacro_props(text):
    """字面量 xacro property -> float dict(跳过 ${...} 表达式 value)。"""
    out = {}
    for name, val in re.findall(r'<xacro:property\s+name="([^"]+)"\s+value="([^"]+)"\s*/>', text):
        try:
            out[name] = float(val)
        except ValueError:
            pass
    return out


def _xacro_args(text):
    """字面量 xacro arg default -> float dict。"""
    return {
        name: float(value)
        for name, value in re.findall(
            r'<xacro:arg\s+name="([^"]+)"\s+default="([-\d.]+)"\s*/>', text
        )
    }


def _xacro_joint_origin_xyz(text, joint_substr):
    """含 joint_substr 的 <joint> 块内第一个 <origin xyz="..."> 的 xyz 串。"""
    m = re.search(r'<joint[^>]*name="[^"]*' + re.escape(joint_substr) + r'[^"]*"[^>]*>(.*?)</joint>',
                  text, re.DOTALL)
    if not m:
        return None
    mo = re.search(r'<origin\s+xyz="([^"]+)"', m.group(1))
    return mo.group(1).strip() if mo else None


def _patch_file_section(text, relative_path):
    """从 unified diff 中取出指定文件的完整 diff 段。"""
    marker = "diff --git a/%s b/%s" % (relative_path, relative_path)
    start = text.find(marker)
    if start < 0:
        raise ValueError("patch 中找不到文件: %s" % relative_path)
    return text[start:].split("\ndiff --git ", 1)[0]


def _patch_added_file(text, relative_path):
    """从 unified diff 中重建指定新增文件；找不到或不是新增文件时明确失败。"""
    section = _patch_file_section(text, relative_path)
    if "--- /dev/null" not in section:
        raise ValueError("patch 目标不是新增文件: %s" % relative_path)
    return "\n".join(
        line[1:]
        for line in section.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )


def _patch_added_value(text, relative_path, key):
    """在指定文件 diff 的 '+' 行里读取 `key: value`。"""
    section = _patch_file_section(text, relative_path)
    for line in section.splitlines():
        if line.startswith("+++") or not line.startswith("+"):
            continue
        match = re.match(
            r"\s*" + re.escape(key) + r"\s*:\s*(.+?)\s*(#.*)?$",
            line[1:],
        )
        if match:
            return match.group(1).strip().strip('"')
    return None


def _gazebo_lidar(text):
    """gpu_lidar 传感器块 -> dict(h_samples,v_samples,update_rate,range_min)。"""
    blk = re.search(r'<sensor name="gpu_lidar".*?</sensor>', text, re.DOTALL).group(0)
    h = re.search(r'<horizontal>.*?<samples>(\d+)</samples>', blk, re.DOTALL).group(1)
    v = re.search(r'<vertical>.*?<samples>(\d+)</samples>', blk, re.DOTALL).group(1)
    rate = re.search(r'<update_rate>(\d+)</update_rate>', blk).group(1)
    rmin = re.search(r'<range>.*?<min>([\d.]+)</min>', blk, re.DOTALL).group(1)
    return {"h_samples": int(h), "v_samples": int(v),
            "update_rate": int(rate), "range_min": float(rmin)}


def _launch_floats(text, names):
    """从 launch py 文本取 `NAME = <float>` 字面量。"""
    out = {}
    for n in names:
        m = re.search(re.escape(n) + r'\s*=\s*([-\d.]+)', text)
        if m:
            out[n] = float(m.group(1))
    return out


def _adapter_scan_period(text):
    return float(re.search(r'"scan_period"\s*:\s*([\d.]+)', text).group(1))


def _parse_footprint(s):
    return [(float(x), float(y)) for x, y in ast.literal_eval(s)]


def _format_footprint(points):
    return "[ " + ", ".join("[%.3f, %.3f]" % (x, y) for x, y in points) + " ]"


def build_real_runtime_configs(repo_root, config):
    """以既有 YAML 为模板，只注入 bringup 中的真机几何派生值。"""
    geometry = derive_real_geometry(config)
    controllers = _yaml(_read(repo_root, F_CONTROLLERS))
    controller = controllers["base_controller"]["ros__parameters"]
    controller["wheel_radius"] = geometry["drive_wheel"]["radius"]
    controller["wheel_separation"] = geometry["drive_wheel"]["separation"]

    nav2 = _yaml(_read(repo_root, F_NAV_PARAMS_REAL))
    footprint = _format_footprint(geometry["footprint"])
    for scope in ("global_costmap", "local_costmap"):
        nav2[scope][scope]["ros__parameters"]["footprint"] = footprint

    return {
        "geometry": geometry,
        "controllers": controllers,
        "nav2": nav2,
    }


def real_geometry_launch_arguments(geometry):
    """把派生几何转换为 robot/navigation launch 的字符串参数。"""
    def as_text(value):
        return str(0.0 if abs(value) < 1e-12 else value)

    body = geometry["body"]
    wheel = geometry["drive_wheel"]
    sensor = geometry["sensor"]
    weld = geometry["body_to_base_footprint"]
    return {
        "robot": {
            "base_length": as_text(body["length"]),
            "base_width": as_text(body["width"]),
            "base_height": as_text(body["height"]),
            "base_link_height": as_text(body["base_link_height"]),
            "wheel_radius": as_text(wheel["radius"]),
            "wheel_width": as_text(wheel["width"]),
            "wheel_separation": as_text(wheel["separation"]),
            "sensor_x": as_text(sensor["x"]),
            "sensor_y": as_text(sensor["y"]),
            "sensor_z": as_text(sensor["z"]),
            "sensor_roll": as_text(sensor["roll"]),
            "sensor_pitch": as_text(sensor["pitch"]),
            "sensor_yaw": as_text(sensor["yaw"]),
        },
        "navigation": {
            "weld_x": as_text(weld["x"]),
            "weld_y": as_text(weld["y"]),
            "weld_z": as_text(weld["z"]),
            "weld_roll": as_text(weld["roll"]),
            "weld_pitch": as_text(weld["pitch"]),
            "weld_yaw": as_text(weld["yaw"]),
        },
    }


def write_real_runtime_configs(repo_root, config, output_dir=None):
    """把派生 YAML 写到临时目录；不改源码，也不改 install。"""
    runtime = build_real_runtime_configs(repo_root, config)
    if output_dir is None:
        directory = Path(tempfile.mkdtemp(prefix="system_bringup-"))
    else:
        directory = Path(output_dir)
        directory.mkdir(parents=True, exist_ok=True)
    paths = {
        "controllers": directory / "robot_controllers_real.generated.yaml",
        "nav2": directory / "nav2_params_real.generated.yaml",
    }
    for name, path in paths.items():
        path.write_text(
            yaml.safe_dump(runtime[name], sort_keys=False, allow_unicode=True),
            encoding="utf-8",
        )
    return paths


def check_geometry(repo_root, platform="sim"):
    """G1–G5:几何派生值在 xacro / controllers / nav2 / launch 间自洽。"""
    if platform not in ("sim", "real"):
        return ["未知 platform=%r(应为 sim|real)。" % platform]
    fails = []
    macro = _read(repo_root, F_MACRO)
    if platform == "real":
        runtime = build_real_runtime_configs(
            repo_root, load_bringup_config(repo_root)
        )
        geometry = runtime["geometry"]
        base_l = geometry["body"]["length"]
        base_w = geometry["body"]["width"]
        base_h = geometry["body"]["height"]
        wheel_r = geometry["drive_wheel"]["radius"]
        wheel_separation = geometry["drive_wheel"]["separation"]
        nav = runtime["nav2"]
        ctrl = runtime["controllers"]
    else:
        defaults = _xacro_args(_read(repo_root, F_ROBOT_XACRO))
        base_l = defaults["base_length"]
        base_w = defaults["base_width"]
        base_h = defaults["base_height"]
        wheel_r = defaults["wheel_radius"]
        wheel_separation = defaults["wheel_separation"]
        nav = _yaml(_read(repo_root, F_NAV_PARAMS))
        ctrl = _yaml(_read(repo_root, F_CONTROLLERS))
    cp = ctrl["base_controller"]["ros__parameters"]

    # G1 footprint(global + local 两处)半长/半宽 == 车体半长/半宽
    for scope in ("global_costmap", "local_costmap"):
        fp = nav[scope][scope]["ros__parameters"]["footprint"]
        pts = _parse_footprint(fp)
        hx = max(abs(x) for x, _ in pts)
        hy = max(abs(y) for _, y in pts)
        if abs(hx - base_l / 2) > 1e-6:
            fails.append("[G1] %s footprint 半长 %.3f != base_length/2 %.3f(源 xacro)。改 nav2_params.yaml 或核对 xacro。"
                         % (scope, hx, base_l / 2))
        if abs(hy - base_w / 2) > 1e-6:
            fails.append("[G1] %s footprint 半宽 %.3f != base_width/2 %.3f。" % (scope, hy, base_w / 2))

    # G2 controllers 轮参 == 当前 platform 的几何源
    if abs(cp["wheel_radius"] - wheel_r) > 1e-9:
        fails.append("[G2] wheel_radius 不一致: geometry=%.4f vs controllers=%.4f。"
                     % (wheel_r, cp["wheel_radius"]))
    if abs(cp["wheel_separation"] - wheel_separation) > 1e-9:
        fails.append("[G2] wheel_separation controllers=%.4f != geometry %.4f。" %
                     (cp["wheel_separation"], wheel_separation))

    # G3 仿真 launch 默认焊接继续与仿真 xacro 默认值一致；真机由同一份派生值下发。
    if platform == "sim":
        lidar_h = _xacro_props(macro)["lidar_height"]
        lc = _launch_floats(
            _read(repo_root, F_NAV_LAUNCH),
            ["_BASE_HEIGHT", "_WHEEL_RADIUS", "_LIDAR_HEIGHT"],
        )
        for cname, xval in [
            ("_BASE_HEIGHT", base_h),
            ("_WHEEL_RADIUS", wheel_r),
            ("_LIDAR_HEIGHT", lidar_h),
        ]:
            if cname not in lc:
                fails.append("[G3] navigation.launch.py 缺仿真几何常量 %s。" % cname)
            elif abs(lc[cname] - xval) > 1e-9:
                fails.append("[G3] navigation.launch.py %s=%.4f != 仿真 xacro %.4f。"
                             % (cname, lc[cname], xval))

    # G4 共位 -> 外参零
    vxyz = _xacro_joint_origin_xyz(macro, "velodyne_joint")
    ixyz = _xacro_joint_origin_xyz(macro, "imu_joint")
    if vxyz != ixyz:
        fails.append("[G4] velodyne_joint/imu_joint origin 不同(应共位): '%s' vs '%s'。" % (vxyz, ixyz))
    fastlio = _yaml(_patch_added_file(
        _read(repo_root, F_FASTLIO_PATCH),
        FASTLIO_CONFIG[platform],
    ))["/**"]["ros__parameters"]
    flm = fastlio["mapping"]
    if [float(v) for v in flm["extrinsic_T"]] != [0.0, 0.0, 0.0]:
        fails.append("[G4] fast-lio extrinsic_T 非零: %s(velodyne/imu 共位应为 [0,0,0])。" % flm["extrinsic_T"])
    if [float(v) for v in flm["extrinsic_R"]] != [1, 0, 0, 0, 1, 0, 0, 0, 1]:
        fails.append("[G4] fast-lio extrinsic_R 非单位阵: %s。" % flm["extrinsic_R"])

    # G5 nav2 限速 <= 底盘限速
    fpp = nav["controller_server"]["ros__parameters"]["FollowPath"]
    if fpp["vx_max"] > cp["linear.x.max_velocity"] + 1e-9:
        fails.append("[G5] nav2 vx_max %.2f > 底盘 linear.x.max_velocity %.2f。" % (fpp["vx_max"], cp["linear.x.max_velocity"]))
    if fpp["wz_max"] > cp["angular.z.max_velocity"] + 1e-9:
        fails.append("[G5] nav2 wz_max %.2f > 底盘 angular.z.max_velocity %.2f。" % (fpp["wz_max"], cp["angular.z.max_velocity"]))

    return fails


def check_lidar(repo_root, platform="sim"):
    """sim 检查 L1–L4，real 检查 R1–R9。"""
    if platform not in ("sim", "real"):
        return ["未知 platform=%r(应为 sim|real)。" % platform]
    fails = []
    fastlio = _yaml(_patch_added_file(
        _read(repo_root, F_FASTLIO_PATCH),
        FASTLIO_CONFIG[platform],
    ))["/**"]["ros__parameters"]

    if platform == "real":
        lio = _yaml(_patch_added_file(
            _read(repo_root, F_LIOSAM_PATCH),
            LIOSAM_CONFIG[platform],
        ))["/**"]["ros__parameters"]
        driver = _yaml(_read(repo_root, F_VANJEE_PARAMS))["vanjee_lidar"]["ros__parameters"]
        fl_pre = fastlio["preprocess"]
        fl_common = fastlio["common"]
        fl_lidar_type = int(fl_pre["lidar_type"])
        fl_scan_line = int(fl_pre["scan_line"])
        fl_scan_rate = int(fl_pre["scan_rate"])
        fl_timestamp_unit = int(fl_pre["timestamp_unit"])
        fl_blind = float(fl_pre["blind"])
        driver_min_distance = float(driver["min_distance"])
        lio_n_scan = int(lio["N_SCAN"])
        lio_horizon_scan = int(lio["Horizon_SCAN"])

        if not (driver["lidar_type"] == "vanjee_722" and fl_lidar_type == 2):
            fails.append("[R1] lidar_type 不一致: driver=%r, fast-lio=%r(应为 vanjee_722/2)。"
                         % (driver["lidar_type"], fl_lidar_type))
        if not (fl_scan_line == lio_n_scan == 32):
            fails.append("[R2] 线数不一致: fast-lio=%d, lio-sam=%d(应均为 32)。"
                         % (fl_scan_line, lio_n_scan))
        if fl_scan_rate != 10:
            fails.append("[R3] fast-lio scan_rate=%d(应为 10)。" % fl_scan_rate)
        if fl_timestamp_unit != 0:
            fails.append("[R4] fast-lio timestamp_unit=%d(应为 0)。" % fl_timestamp_unit)
        if not (fl_blind == 0.3 and fl_blind >= driver_min_distance):
            fails.append("[R5] fast-lio blind=%.2f(应为 0.30且不小于 driver min_distance=%.2f)。"
                         % (fl_blind, driver_min_distance))
        if not (driver["point_cloud_topic"] == fl_common["lid_topic"] == lio["pointCloudTopic"] == "/points_raw"):
            fails.append("[R6] 点云话题不一致: driver=%r, fast-lio=%r, lio-sam=%r(应均为 /points_raw)。"
                         % (driver["point_cloud_topic"], fl_common["lid_topic"], lio["pointCloudTopic"]))
        if not (driver["imu_topic"] == fl_common["imu_topic"] == lio["imuTopic"] == "/imu/data"):
            fails.append("[R7] IMU 话题不一致: driver=%r, fast-lio=%r, lio-sam=%r(应均为 /imu/data)。"
                         % (driver["imu_topic"], fl_common["imu_topic"], lio["imuTopic"]))
        if (driver["lidar_frame"], driver["imu_frame"]) != ("velodyne", "imu_link"):
            fails.append("[R8] driver frame 不一致: lidar=%r, imu=%r(应为 velodyne/imu_link)。"
                         % (driver["lidar_frame"], driver["imu_frame"]))
        if lio["lidarFrame"] != driver["lidar_frame"]:
            fails.append("[R8] lio-sam lidarFrame=%r != driver lidar_frame=%r。"
                         % (lio["lidarFrame"], driver["lidar_frame"]))
        if not (lio_horizon_scan == 1200 and lio["use_sim_time"] is False):
            fails.append("[R9] lio-sam Horizon_SCAN=%d/use_sim_time=%r(应为 1200/false)。"
                         % (lio_horizon_scan, lio["use_sim_time"]))
        return fails

    gz = _gazebo_lidar(_read(repo_root, F_GAZEBO))
    fl_pre = fastlio["preprocess"]
    n_scan = int(_patch_added_value(
        _read(repo_root, F_LIOSAM_PATCH),
        LIOSAM_CONFIG[platform],
        "N_SCAN",
    ))
    horizon = int(_patch_added_value(
        _read(repo_root, F_LIOSAM_PATCH),
        LIOSAM_CONFIG[platform],
        "Horizon_SCAN",
    ))
    adapter_rate = round(1.0 / _adapter_scan_period(_read(repo_root, F_GZ_LAUNCH)))

    # L1 线数
    if not (gz["v_samples"] == n_scan == fl_pre["scan_line"]):
        fails.append("[L1] 线数不一致: gazebo=%d, lio-sam N_SCAN=%d, fast-lio scan_line=%d。"
                     % (gz["v_samples"], n_scan, fl_pre["scan_line"]))
    # L2 水平
    if gz["h_samples"] != horizon:
        fails.append("[L2] 水平点数不一致: gazebo=%d, lio-sam Horizon_SCAN=%d。" % (gz["h_samples"], horizon))
    # L3 频率
    if not (gz["update_rate"] == fl_pre["scan_rate"] == adapter_rate):
        fails.append("[L3] 频率不一致: gazebo update_rate=%d, fast-lio scan_rate=%d, adapter 1/scan_period=%d。"
                     % (gz["update_rate"], fl_pre["scan_rate"], adapter_rate))
    # L4 近距(不等式:盲区 >= 传感器最小距)
    if fl_pre["blind"] < gz["range_min"] - 1e-9:
        fails.append("[L4] fast-lio blind %.2f < gazebo range.min %.2f(盲区应 >= 传感器最小距)。"
                     % (fl_pre["blind"], gz["range_min"]))
    return fails


def run(repo_root=None):
    """跑全部检查,返回失败描述列表(空=全过)。repo_root 为空则从 __file__ 上溯。"""
    if yaml is None:
        return ["缺少 pyyaml(pip install pyyaml / apt install python3-yaml)。"]
    if repo_root is None:
        repo_root = find_repo_root()
    platform = load_bringup_config(repo_root)["platform"]
    fails = []
    fails += check_geometry(repo_root, platform)
    fails += check_lidar(repo_root, platform)
    return fails


def main(argv=None):
    ap = argparse.ArgumentParser(description="跨模块魔法值一致性检查")
    ap.add_argument("--repo-root", default=None, help="仓库根(默认从 __file__ 上溯)")
    ns = ap.parse_args(argv)
    root = os.path.expanduser(ns.repo_root) if ns.repo_root else None
    fails = run(root)
    if fails:
        print("跨模块一致性检查 未通过:")
        for f in fails:
            print("  - " + f)
        return 1
    print("跨模块一致性检查 通过。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
