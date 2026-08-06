"""跨模块魔法值一致性检查(纯 Python,无 ROS 依赖,本机/构建机皆可跑)。

解析已跟踪源文件；仿真检查既有默认值，真机从 bringup.yaml 单一源生成后检查。
- run(repo_root) -> list[str]   失败描述(空=全过)
- main()        -> int          打印失败、返回退出码(供启动闸门/CLI)
- find_repo_root()              从 __file__ 上溯定位仓库根(pytest 本机用)

契约类(帧/话题)不纳入。patch 文件只信任 '+'(项目改值)行。
"""
import argparse
import ast
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
    "web_ui": "core/bringup/robot_web_ui/robot_web_ui/web_ui_node.py",
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


_INSTALLED_TOPOLOGY_SHARES = {
    "formal": ("system_bringup", "launch/bringup.launch.py"),
    "slam": ("system_bringup", "launch/slam_stack.launch.py"),
    "sim": ("robot_gz_bringup", "launch/robot_gz.launch.py"),
    "real_chassis": ("robot_bringup", "launch/real_chassis.launch.py"),
    "real_robot": ("robot_bringup", "launch/robot.launch.py"),
    "navigation": ("robot_navigation", "launch/navigation.launch.py"),
}
_INSTALLED_TOPOLOGY_MODULES = {
    "cmd_gate": "cmd_vel_gate.gate_node",
    "web_ui": "robot_web_ui.web_ui_node",
}
_GEOMETRY_ARGUMENTS = tuple(
    name
    for name in _ROBOT_ARGUMENTS
    if name not in ("controllers_file", "use_sim_time")
)
_UNKNOWN = object()


def _resolve_installed_topology_paths(failures):
    """Resolve files ROS will actually load; imports stay lazy for portable tooling."""
    try:
        import importlib.util
        from ament_index_python.packages import (
            PackageNotFoundError,
            get_package_share_directory,
        )
    except (ImportError, ModuleNotFoundError) as exc:
        failures.append(
            "active installed topology cannot be resolved outside a sourced ROS "
            "environment; package shares and node modules "
            "cmd_vel_gate.gate_node, robot_web_ui.web_ui_node are required: "
            f"{exc}"
        )
        return {}

    paths = {}
    for label, (package, relative_path) in _INSTALLED_TOPOLOGY_SHARES.items():
        try:
            share = get_package_share_directory(package)
        except (PackageNotFoundError, OSError, RuntimeError, ValueError) as exc:
            failures.append(
                f"active installed topology package {package} cannot be resolved: {exc}"
            )
            continue
        share_path = _normalize_path(
            share,
            f"active installed topology package share {package}",
            failures,
        )
        if share_path is None:
            continue
        path = _normalize_path(
            share_path / relative_path,
            f"active installed topology {package}/{relative_path}",
            failures,
        )
        if path is not None:
            paths[label] = path

    for label, module_name in _INSTALLED_TOPOLOGY_MODULES.items():
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


def _active_source_trees(repo_root, failures):
    reviewed_paths = {}
    for label, relative_path in _ACTIVE_TOPOLOGY_FILES.items():
        path = _normalize_path(
            repo_root / relative_path,
            f"reviewed topology source {relative_path}",
            failures,
        )
        if path is not None:
            reviewed_paths[label] = path

    active_paths = _resolve_installed_topology_paths(failures)
    if set(active_paths) != set(_ACTIVE_TOPOLOGY_FILES):
        missing = sorted(set(_ACTIVE_TOPOLOGY_FILES) - set(active_paths))
        extra = sorted(set(active_paths) - set(_ACTIVE_TOPOLOGY_FILES))
        failures.append(
            "active installed topology path set is incomplete: "
            f"missing={missing}, extra={extra}"
        )
        return {}

    trees = {}
    for label, relative_path in _ACTIVE_TOPOLOGY_FILES.items():
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
                f"active topology {relative_path} cannot be read from reviewed/installed "
                f"paths {reviewed_path}/{active_path}: {exc}"
            )
            continue
        if active_bytes != reviewed_bytes:
            failures.append(
                "active installed topology differs from reviewed source; rebuild the "
                "workspace (prefer --symlink-install) before launch: "
                f"{active_path} != {reviewed_path}"
            )
            continue
        try:
            active_source = active_bytes.decode("utf-8")
            trees[label] = ast.parse(active_source, filename=str(active_path))
        except (SyntaxError, UnicodeError) as exc:
            failures.append(f"active installed topology {active_path} is invalid: {exc}")
    return trees


def _walk_scope(scope):
    def walk(node):
        yield node
        if isinstance(node, ast.If) and isinstance(node.test, ast.Constant):
            branch = node.body if node.test.value else node.orelse
            for statement in branch:
                yield from walk(statement)
            return
        for child in ast.iter_child_nodes(node):
            if isinstance(
                child,
                (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef, ast.Lambda),
            ):
                continue
            yield from walk(child)

    for statement in scope.body:
        yield from walk(statement)


def _definition(tree, name, expected_type=ast.FunctionDef):
    return next(
        (
            node
            for node in tree.body
            if isinstance(node, expected_type) and node.name == name
        ),
        None,
    )


def _method(class_node, name):
    if class_node is None:
        return None
    return next(
        (
            node
            for node in class_node.body
            if isinstance(node, ast.FunctionDef) and node.name == name
        ),
        None,
    )


def _assignments(scope):
    values = {}
    if scope is None:
        return values
    for node in _walk_scope(scope):
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for target in targets:
                if isinstance(target, ast.Name):
                    values[target.id] = node.value
    return values


def _resolve_alias(node, assignments, trail=()):
    if isinstance(node, ast.Name) and node.id in assignments and node.id not in trail:
        return _resolve_alias(assignments[node.id], assignments, trail + (node.id,))
    return node


def _signature(node, assignments, trail=()):
    if node is None:
        return None
    if isinstance(node, ast.Name):
        if node.id in assignments and node.id not in trail:
            return _signature(assignments[node.id], assignments, trail + (node.id,))
        return ("name", node.id)
    if isinstance(node, ast.Constant):
        return ("constant", node.value)
    if isinstance(node, ast.Attribute):
        return ("attribute", _signature(node.value, assignments, trail), node.attr)
    if isinstance(node, ast.Subscript):
        return (
            "subscript",
            _signature(node.value, assignments, trail),
            _signature(node.slice, assignments, trail),
        )
    if isinstance(node, ast.Call):
        return (
            "call",
            _signature(node.func, assignments, trail),
            tuple(_signature(arg, assignments, trail) for arg in node.args),
            tuple(
                (keyword.arg, _signature(keyword.value, assignments, trail))
                for keyword in node.keywords
            ),
        )
    if isinstance(node, (ast.List, ast.Tuple, ast.Set)):
        return (
            type(node).__name__,
            tuple(_signature(item, assignments, trail) for item in node.elts),
        )
    if isinstance(node, ast.Dict):
        return (
            "dict",
            tuple(
                (
                    _signature(key, assignments, trail),
                    _signature(value, assignments, trail),
                )
                for key, value in zip(node.keys, node.values)
            ),
        )
    if isinstance(node, ast.BinOp):
        return (
            "binop",
            type(node.op).__name__,
            _signature(node.left, assignments, trail),
            _signature(node.right, assignments, trail),
        )
    return (type(node).__name__, ast.dump(node, include_attributes=False))


def _expression(source):
    return ast.parse(source, mode="eval").body


def _same_expression(node, expected_source, assignments):
    return _signature(node, assignments) == _signature(
        _expression(expected_source), assignments
    )


def _literal(node, assignments):
    node = _resolve_alias(node, assignments)
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, (ast.List, ast.Tuple)):
        values = [_literal(item, assignments) for item in node.elts]
        if any(value is _UNKNOWN for value in values):
            return _UNKNOWN
        return values if isinstance(node, ast.List) else tuple(values)
    return _UNKNOWN


def _sequence(node, assignments):
    node = _resolve_alias(node, assignments)
    return list(node.elts) if isinstance(node, (ast.List, ast.Tuple)) else None


def _dict_parts(node, assignments):
    node = _resolve_alias(node, assignments)
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "items"
        and not node.args
        and not node.keywords
    ):
        node = _resolve_alias(node.func.value, assignments)
    if not isinstance(node, ast.Dict):
        return None, None
    values = {}
    expansions = []
    for key, value in zip(node.keys, node.values):
        if key is None:
            expansions.append(value)
            continue
        literal_key = _literal(key, assignments)
        if not isinstance(literal_key, str) or literal_key in values:
            return None, None
        values[literal_key] = value
    return values, expansions


def _call_name(call):
    if not isinstance(call, ast.Call):
        return None
    func = call.func
    if isinstance(func, ast.Name):
        return func.id
    if isinstance(func, ast.Attribute):
        return func.attr
    return None


def _calls(scope, name=None):
    if scope is None:
        return []
    calls = [node for node in _walk_scope(scope) if isinstance(node, ast.Call)]
    return calls if name is None else [call for call in calls if _call_name(call) == name]


def _keyword(call, name):
    return next(
        (keyword.value for keyword in call.keywords if keyword.arg == name),
        None,
    )


def _node_identity(call, assignments):
    if _call_name(call) != "Node":
        return None
    return (
        _literal(_keyword(call, "package"), assignments),
        _literal(_keyword(call, "executable"), assignments),
    )


def _include_identity(call, assignments):
    if _call_name(call) != "_inc" or len(call.args) < 2:
        return None
    return (_literal(call.args[0], assignments), _literal(call.args[1], assignments))


def _launch_include_identity(call, assignments):
    if _call_name(call) != "IncludeLaunchDescription" or len(call.args) != 1:
        return None
    source = _resolve_alias(call.args[0], assignments)
    if _call_name(source) != "PythonLaunchDescriptionSource" or len(source.args) != 1:
        return None
    joined = _resolve_alias(source.args[0], assignments)
    if _call_name(joined) != "PathJoinSubstitution" or len(joined.args) != 1:
        return None
    parts = _sequence(joined.args[0], assignments)
    if parts is None or len(parts) < 2:
        return None
    package_call = _resolve_alias(parts[0], assignments)
    if _call_name(package_call) != "FindPackageShare" or len(package_call.args) != 1:
        return None
    package = _literal(package_call.args[0], assignments)
    relative_parts = [_literal(part, assignments) for part in parts[1:]]
    if not isinstance(package, str) or not all(
        isinstance(part, str) for part in relative_parts
    ):
        return None
    return package, "/".join(relative_parts)


def _platform_branch(function, platform):
    if function is None:
        return None
    for node in function.body:
        if not isinstance(node, ast.If) or not isinstance(node.test, ast.Compare):
            continue
        compare = node.test
        if (
            isinstance(compare.left, ast.Name)
            and compare.left.id == "platform"
            and len(compare.ops) == 1
            and isinstance(compare.ops[0], ast.Eq)
            and len(compare.comparators) == 1
            and isinstance(compare.comparators[0], ast.Constant)
            and compare.comparators[0].value == platform
        ):
            return node
    return None


def _concat_terms(node):
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return _concat_terms(node.left) + _concat_terms(node.right)
    return [node]


def _single_return(scope):
    returns = [node for node in _walk_scope(scope) if isinstance(node, ast.Return)]
    return returns[0] if len(returns) == 1 else None


def _flag_arguments(node, assignments):
    items = _sequence(node, assignments)
    if items is None:
        return None
    flags = {}
    index = 0
    while index < len(items):
        flag = _literal(items[index], assignments)
        if not isinstance(flag, str) or not flag.startswith("--") or index + 1 >= len(items):
            return None
        if flag in flags:
            return None
        flags[flag] = items[index + 1]
        index += 2
    return flags


def _declare_names(function, assignments):
    names = []
    for call in _calls(function, "DeclareLaunchArgument"):
        if call.args:
            names.append(_literal(call.args[0], assignments))
    return names


def _same_unique_members(actual, required):
    try:
        return len(actual) == len(required) and set(actual) == set(required)
    except (TypeError, ValueError):
        return False


def _dict_comprehension_launch_configs(node, required_names, assignments):
    node = _resolve_alias(node, assignments)
    if not isinstance(node, ast.DictComp) or len(node.generators) != 1:
        return False
    generator = node.generators[0]
    if generator.ifs or not isinstance(generator.target, ast.Name):
        return False
    name = generator.target.id
    values = _literal(generator.iter, assignments)
    value = node.value
    if (
        isinstance(value, ast.Call)
        and isinstance(value.func, ast.Attribute)
        and value.func.attr == "perform"
    ):
        value = value.func.value
    return (
        _same_unique_members(values, required_names)
        and isinstance(node.key, ast.Name)
        and node.key.id == name
        and isinstance(value, ast.Call)
        and _call_name(value) == "LaunchConfiguration"
        and len(value.args) == 1
        and isinstance(value.args[0], ast.Name)
        and value.args[0].id == name
    )


def _contract_failure(failures, category, details):
    failures.append(f"active topology {category} drift: {details}")


def _manifest_value(node, key, assignments, stringify=True):
    expression = f'manifest["{key}"]'
    if stringify:
        expression = f"str({expression})"
    return _same_expression(node, expression, assignments)


def _launch_configuration(node, name, assignments):
    return _same_expression(node, f'LaunchConfiguration("{name}")', assignments)


def _node_by_identity(function, identity, assignments):
    return [
        call
        for call in _calls(function, "Node")
        if _node_identity(call, assignments) == identity
    ]


def _include_by_identity(scope, identity, assignments):
    return [
        call
        for call in _calls(scope, "_inc")
        if _include_identity(call, assignments) == identity
    ]


def _parameter_dict(call, assignments):
    parameters = _sequence(_keyword(call, "parameters"), assignments)
    if parameters is None or len(parameters) != 1:
        return None
    values, expansions = _dict_parts(parameters[0], assignments)
    return values if values is not None and not expansions else None


def _command_argument_items(function, assignments):
    commands = _calls(function, "Command")
    if len(commands) != 1 or len(commands[0].args) != 1:
        return None
    return _sequence(commands[0].args[0], assignments)


def _forwarded_command_value(items, marker, assignments):
    if items is None:
        return None
    indexes = [
        index
        for index, item in enumerate(items[:-1])
        if _literal(item, assignments) == marker
    ]
    return items[indexes[0] + 1] if len(indexes) == 1 else None


def _remapping_pairs(call, assignments):
    value = _keyword(call, "remappings")
    if value is None:
        return []
    pairs = _literal(value, assignments)
    if not isinstance(pairs, list) or not all(
        isinstance(pair, tuple)
        and len(pair) == 2
        and all(isinstance(item, str) for item in pair)
        for pair in pairs
    ):
        return None
    return pairs


def _validate_formal_topology(tree, failures):
    function = _definition(tree, "_bringup")
    assignments = _assignments(function)
    sim_branch = _platform_branch(function, "sim")
    real_branch = _platform_branch(function, "real")
    includes = _calls(function, "_inc")
    identities = [_include_identity(call, assignments) for call in includes]
    required_includes = {
        ("system_bringup", "launch/slam_stack.launch.py"): 1,
        ("robot_gz_bringup", "launch/robot_gz.launch.py"): 1,
        ("robot_bringup", "launch/real_chassis.launch.py"): 1,
        ("vanjee_lidar_ros", "launch/vanjee_lidar.launch.py"): 1,
    }
    node_calls = _calls(function, "Node")
    node_ids = [_node_identity(call, assignments) for call in node_calls]
    cardinality_ok = (
        function is not None
        and sim_branch is not None
        and real_branch is not None
        and len(includes) == 4
        and not _calls(function, "IncludeLaunchDescription")
        and all(
            identities.count(identity) == count
            for identity, count in required_includes.items()
        )
        and node_ids.count(("cmd_vel_gate", "cmd_vel_gate")) == 1
        and node_ids.count(("robot_web_ui", "robot_web_ui")) == 1
        and len(_include_by_identity(
            sim_branch, ("robot_gz_bringup", "launch/robot_gz.launch.py"), assignments
        )) == 1
        and len(_include_by_identity(
            real_branch, ("robot_bringup", "launch/real_chassis.launch.py"), assignments
        )) == 1
        and len(_include_by_identity(
            real_branch, ("vanjee_lidar_ros", "launch/vanjee_lidar.launch.py"), assignments
        )) == 1
    )
    if not cardinality_ok:
        _contract_failure(
            failures,
            "topology cardinality",
            "formal bringup requires exactly one gate, Web UI, shared slam include, "
            "sim backend, real chassis and Vanjee backend with no duplicate layers",
        )

    compile_calls = _calls(function, "compile_runtime_configs")
    check_calls = _calls(function, "run_runtime_consistency")
    web_nodes = _node_by_identity(
        function, ("robot_web_ui", "robot_web_ui"), assignments
    )
    slam_calls = _include_by_identity(
        function, ("system_bringup", "launch/slam_stack.launch.py"), assignments
    )
    sim_calls = _include_by_identity(
        function, ("robot_gz_bringup", "launch/robot_gz.launch.py"), assignments
    )
    real_calls = _include_by_identity(
        function, ("robot_bringup", "launch/real_chassis.launch.py"), assignments
    )
    generated_ok = len(compile_calls) == len(check_calls) == 1
    generated_ok = generated_ok and (
        len(compile_calls[0].args) == 1
        and _same_expression(compile_calls[0].args[0], "source_config", assignments)
        and len(check_calls[0].args) == 2
        and _same_expression(check_calls[0].args[0], "repo_root", assignments)
        and _same_expression(check_calls[0].args[1], "manifest", assignments)
    )
    if len(web_nodes) == 1:
        parameters = _sequence(_keyword(web_nodes[0], "parameters"), assignments)
        generated_ok = generated_ok and parameters is not None and len(parameters) == 1
        generated_ok = generated_ok and _manifest_value(
            parameters[0], "web_ui_path", assignments
        )
    else:
        generated_ok = False

    slam_values, slam_expansions = (
        _dict_parts(slam_calls[0].args[2], assignments)
        if len(slam_calls) == 1 and len(slam_calls[0].args) >= 3
        else (None, None)
    )
    generated_ok = generated_ok and slam_values is not None and not slam_expansions
    generated_ok = generated_ok and _manifest_value(
        None if slam_values is None else slam_values.get("nav2_params_file"),
        "nav2_path",
        assignments,
    )
    for calls in (sim_calls, real_calls):
        values, expansions = (
            _dict_parts(calls[0].args[2], assignments)
            if len(calls) == 1 and len(calls[0].args) >= 3
            else (None, None)
        )
        generated_ok = generated_ok and values is not None
        generated_ok = generated_ok and _manifest_value(
            None if values is None else values.get("controllers_file"),
            "controllers_path",
            assignments,
        )
    if not generated_ok:
        _contract_failure(
            failures,
            "generated runtime artifacts",
            "formal bringup must data-flow generated Web UI/Nav2/controllers through "
            "exactly one compile and one runtime check",
        )

    expected_sim_keys = {
        "gui", "rviz", "world", "spawn_x", "spawn_y", "spawn_z",
        "controllers_file", "use_sim_time",
    }
    sim_values, sim_expansions = (
        _dict_parts(sim_calls[0].args[2], assignments)
        if len(sim_calls) == 1 and len(sim_calls[0].args) >= 3
        else (None, None)
    )
    sim_interface_ok = (
        sim_values is not None
        and set(sim_values) == expected_sim_keys
        and len(sim_expansions) == 1
        and _same_expression(sim_expansions[0], "geometry", assignments)
        and _same_expression(sim_values["use_sim_time"], "use_sim", assignments)
    )
    if not sim_interface_ok:
        _contract_failure(
            failures,
            "simulation backend interface",
            "formal sim include must forward exact controller/full geometry/use_sim_time interface",
        )

    expected_real_keys = {"gui", "controllers_file", "use_sim_time"}
    real_values, real_expansions = (
        _dict_parts(real_calls[0].args[2], assignments)
        if len(real_calls) == 1 and len(real_calls[0].args) >= 3
        else (None, None)
    )
    real_interface_ok = (
        real_values is not None
        and set(real_values) == expected_real_keys
        and len(real_expansions) == 1
        and _same_expression(real_expansions[0], "geometry", assignments)
        and _same_expression(real_values["use_sim_time"], "use_sim", assignments)
    )
    if not real_interface_ok:
        _contract_failure(
            failures,
            "real backend interface",
            "formal real include must forward exact "
            "controller/full geometry/use_sim_time interface",
        )

    weld_names = tuple(
        f"weld_{name}" for name in ("x", "y", "z", "qx", "qy", "qz", "qw")
    )
    weld_ok = slam_values is not None and all(
        name in slam_values
        and _same_expression(slam_values[name], f'weld["{name[5:]}"]', assignments)
        for name in weld_names
    )
    weld_ok = weld_ok and not any(
        name in (slam_values or {}) for name in ("weld_roll", "weld_pitch", "weld_yaw")
    )
    if not weld_ok:
        _contract_failure(
            failures, "quaternion weld", "formal slam interface must forward quaternion weld only"
        )

    routing_ok = (
        slam_values is not None
        and _literal(slam_values.get("cmd_vel_output_topic"), assignments)
        == "/cmd_vel_auto"
    )
    if not routing_ok:
        _contract_failure(
            failures, "command gate routing", "formal Nav2 output must route to /cmd_vel_auto"
        )

    sim_return = _single_return(sim_branch)
    sim_terms = [] if sim_return is None else _concat_terms(sim_return.value)
    sim_ready = _calls(sim_branch, "ready_gate")
    sim_sequence_ok = (
        len(sim_terms) == 4
        and len(sim_ready) == 1
        and _same_expression(sim_terms[2], "[base]", assignments)
        and sim_terms[3] is sim_ready[0]
        and len(sim_ready[0].args) >= 4
        and _literal(sim_ready[0].args[0], assignments)
        == ["/points_raw", "/joint_states"]
        and _literal(sim_ready[0].args[1], assignments) == 300.0
    )
    if not sim_sequence_ok:
        _contract_failure(
            failures,
            "backend sequencing",
            "simulation backend must precede its exact readiness gate and shared slam actions",
        )
        _contract_failure(
            failures,
            "readiness sequence",
            "simulation readiness requires /points_raw+/joint_states with timeout 300",
        )

    real_return = _single_return(real_branch)
    real_terms = [] if real_return is None else _concat_terms(real_return.value)
    real_gates = _calls(real_branch, "_real_sensor_gate")
    real_sequence_ok = (
        len(real_terms) == 4
        and len(real_gates) == 1
        and _same_expression(real_terms[2], "[chassis, lidar]", assignments)
        and real_terms[3] is real_gates[0]
        and len(real_gates[0].args) == 2
        and _same_expression(real_gates[0].args[0], "[slam_stack]", assignments)
        and _same_expression(real_gates[0].args[1], "use_sim_time", assignments)
    )
    if not real_sequence_ok:
        _contract_failure(
            failures,
            "backend sequencing",
            "real chassis+Vanjee must precede the sensor gate and shared slam stack",
        )

    prohibited = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            prohibited.update(
                alias.name for alias in node.names if alias.name in _PROHIBITED_ACTIVE_LEGACY
            )
    prohibited.update(
        _call_name(call)
        for call in _calls(function)
        if _call_name(call) in _PROHIBITED_ACTIVE_LEGACY
    )
    if prohibited:
        _contract_failure(
            failures,
            "prohibited active legacy",
            f"formal bringup imports/calls {sorted(prohibited)}",
        )


def _validate_sim_topology(tree, failures):
    function = _definition(tree, "generate_launch_description")
    assignments = _assignments(function)
    declarations = _declare_names(function, assignments)
    items = _command_argument_items(function, assignments)
    interface_ok = function is not None and all(
        declarations.count(name) == 1 for name in _ROBOT_ARGUMENTS
    )
    interface_ok = interface_ok and _same_expression(
        _forwarded_command_value(items, "gz_controllers_file:=", assignments),
        "controllers_file",
        assignments,
    )
    interface_ok = interface_ok and all(
        _launch_configuration(
            _forwarded_command_value(items, f"{name}:=", assignments),
            name,
            assignments,
        )
        for name in _GEOMETRY_ARGUMENTS
    )
    robot_description, expansions = _dict_parts(
        assignments.get("robot_description"), assignments
    )
    interface_ok = interface_ok and robot_description is not None and not expansions
    interface_ok = interface_ok and _same_expression(
        robot_description.get("use_sim_time") if robot_description else None,
        "use_sim_time",
        assignments,
    )
    if not interface_ok:
        _contract_failure(
            failures,
            "simulation backend interface",
            "installed Gazebo launch must declare and consume "
            "controller/full geometry/use_sim_time",
        )

    adapters = _node_by_identity(
        function, ("lidar_pointcloud_adapter", "adapter_node"), assignments
    )
    parameters = _parameter_dict(adapters[0], assignments) if len(adapters) == 1 else None
    if not (
        parameters is not None
        and _literal(parameters.get("output_topic"), assignments) == "/points_raw"
        and _literal(parameters.get("output_frame"), assignments) == "velodyne"
    ):
        _contract_failure(
            failures, "critical frames", "Gazebo lidar adapter must publish /points_raw in velodyne"
        )


def _validate_real_topology(chassis_tree, robot_tree, failures):
    chassis = _definition(chassis_tree, "generate_launch_description")
    chassis_assignments = _assignments(chassis)
    declarations = _declare_names(chassis, chassis_assignments)
    geometry_ok = all(declarations.count(name) == 1 for name in _ROBOT_ARGUMENTS)
    geometry_ok = geometry_ok and _dict_comprehension_launch_configs(
        chassis_assignments.get("geometry_arguments"),
        _ROBOT_ARGUMENTS,
        chassis_assignments,
    )
    robot_includes = _calls(chassis, "IncludeLaunchDescription")
    include_identities = [
        _launch_include_identity(call, chassis_assignments) for call in robot_includes
    ]
    geometry_ok = (
        geometry_ok
        and len(include_identities) == 2
        and include_identities.count(("can_driver", "can_driver_8030.launch.py")) == 1
        and include_identities.count(("robot_bringup", "launch/robot.launch.py")) == 1
    )
    robot_include = next(
        (
            call
            for call, identity in zip(robot_includes, include_identities)
            if identity == ("robot_bringup", "launch/robot.launch.py")
        ),
        None,
    )
    values, expansions = (
        _dict_parts(_keyword(robot_include, "launch_arguments"), chassis_assignments)
        if robot_include is not None
        else (None, None)
    )
    geometry_ok = (
        geometry_ok
        and values is not None
        and set(values) == {"gui", "use_mock_hardware"}
        and len(expansions) == 1
        and _same_expression(expansions[0], "geometry_arguments", chassis_assignments)
    )

    robot = _definition(robot_tree, "generate_launch_description")
    robot_assignments = _assignments(robot)
    robot_declarations = _declare_names(robot, robot_assignments)
    items = _command_argument_items(robot, robot_assignments)
    geometry_ok = geometry_ok and all(
        robot_declarations.count(name) == 1 for name in _ROBOT_ARGUMENTS
    )
    geometry_ok = geometry_ok and all(
        _launch_configuration(
            _forwarded_command_value(items, f"{name}:=", robot_assignments),
            name,
            robot_assignments,
        )
        for name in _GEOMETRY_ARGUMENTS
    )
    controls = _node_by_identity(
        robot, ("controller_manager", "ros2_control_node"), robot_assignments
    )
    parameters = (
        _sequence(_keyword(controls[0], "parameters"), robot_assignments)
        if len(controls) == 1
        else None
    )
    controller_clock = (
        _dict_parts(parameters[1], robot_assignments)[0]
        if parameters is not None and len(parameters) == 2
        else None
    )
    geometry_ok = (
        geometry_ok
        and parameters is not None
        and len(parameters) == 2
        and _same_expression(parameters[0], "controllers_file", robot_assignments)
        and controller_clock is not None
        and set(controller_clock) == {"use_sim_time"}
        and _same_expression(
            controller_clock["use_sim_time"], "use_sim_time", robot_assignments
        )
    )
    if not geometry_ok:
        _contract_failure(
            failures,
            "real backend interface",
            "installed real chassis/robot must declare and consume "
            "controller/full geometry/use_sim_time",
        )

    remappings = (
        _remapping_pairs(controls[0], robot_assignments)
        if len(controls) == 1
        else None
    )
    required_remappings = (
        ("~/robot_description", "/robot_description"),
        ("/base_controller/cmd_vel", "/cmd_vel"),
    )
    if not _same_unique_members(remappings, required_remappings):
        _contract_failure(
            failures,
            "command gate routing",
            "real controller must have only the allowed /base_controller/cmd_vel -> /cmd_vel remap",
        )


def _validate_slam_topology(tree, failures):
    function = _definition(tree, "_stack")
    assignments = _assignments(function)
    navigation_branch = next(
        (
            node
            for node in function.body
            if isinstance(node, ast.If)
            and isinstance(node.test, ast.Compare)
            and any(
                isinstance(item, ast.Constant) and item.value == "navigation"
                for item in node.test.comparators
            )
        ),
        None,
    ) if function is not None else None
    includes = _calls(navigation_branch, "_inc")
    identities = [_include_identity(call, assignments) for call in includes]
    nav_call = next(
        (call for call in includes if _include_identity(call, assignments)
         == ("robot_navigation", "launch/navigation.launch.py")),
        None,
    )
    values, expansions = (
        _dict_parts(nav_call.args[2], assignments)
        if nav_call is not None and len(nav_call.args) >= 3
        else (None, None)
    )
    weld_names = tuple(
        f"weld_{name}" for name in ("x", "y", "z", "qx", "qy", "qz", "qw")
    )
    weld_ok = (
        len(includes) == 3
        and identities.count(("fast_lio", "launch/mapping.launch.py")) == 1
        and identities.count(("gicp_localization", "launch/localization.launch.py")) == 1
        and identities.count(("robot_navigation", "launch/navigation.launch.py")) == 1
        and values is not None
        and len(expansions) == 1
        and _dict_comprehension_launch_configs(
            expansions[0], weld_names, assignments
        )
        and not any(name in values for name in ("weld_roll", "weld_pitch", "weld_yaw"))
    )
    if not weld_ok:
        _contract_failure(
            failures,
            "quaternion weld",
            "shared stack must forward one quaternion-only weld dictionary",
        )

    if not (
        values is not None
        and _same_expression(values.get("params_file"), "nav2_params", assignments)
    ):
        _contract_failure(
            failures,
            "generated runtime artifacts",
            "shared stack must forward the generated Nav2 parameters file to navigation",
        )

    routing_ok = values is not None and _same_expression(
        values.get("cmd_vel_output_topic"), "cmd_vel_output_topic", assignments
    )
    if not routing_ok:
        _contract_failure(
            failures,
            "command gate routing",
            "shared stack must forward the selected Nav2 output topic",
        )

    ready_calls = _calls(navigation_branch, "ready_gate")
    outer = next(
        (
            call for call in ready_calls
            if call.args and _literal(call.args[0], assignments)
            == ["/Odometry", "/cloud_registered"]
        ),
        None,
    )
    inner = next(
        (
            call for call in ready_calls
            if call.args and _literal(call.args[0], assignments)
            == ["/localization", "/base_controller/odom"]
        ),
        None,
    )
    sequence_ok = len(ready_calls) == 2 and outer is not None and inner is not None
    sequence_ok = sequence_ok and len(outer.args) >= 4 and len(inner.args) >= 4
    sequence_ok = sequence_ok and _literal(outer.args[1], assignments) == 60.0
    sequence_ok = sequence_ok and _literal(inner.args[1], assignments) == 60.0
    outer_actions = _concat_terms(outer.args[3]) if sequence_ok else []
    sequence_ok = (
        sequence_ok
        and len(outer_actions) == 2
        and _same_expression(outer_actions[0], "[gicp]", assignments)
        and outer_actions[1] is inner
        and _same_expression(inner.args[3], "[nav2]", assignments)
    )
    if not sequence_ok:
        _contract_failure(
            failures,
            "readiness sequence",
            "FAST-LIO and GICP readiness topics/timeouts/order must lead to GICP then Nav2",
        )


def _validate_navigation_topology(tree, failures):
    function = _definition(tree, "generate_launch_description")
    assignments = _assignments(function)
    outer_params_binding = assignments.get("params_file")
    outer_params_ok = _launch_configuration(
        outer_params_binding, "params_file", assignments
    )
    expected_params_signature = (
        _signature(outer_params_binding, assignments)
        if outer_params_ok
        else None
    )
    map_function = next(
        (
            node
            for node in (function.body if function is not None else ())
            if isinstance(node, ast.FunctionDef) and node.name == "_map_server"
        ),
        None,
    )
    map_assignments = dict(assignments)
    map_assignments.update(_assignments(map_function))
    map_nodes = _node_by_identity(
        map_function, ("nav2_map_server", "map_server"), map_assignments
    )
    map_parameters = (
        _sequence(_keyword(map_nodes[0], "parameters"), map_assignments)
        if len(map_nodes) == 1
        else None
    )
    map_override, map_expansions = (
        _dict_parts(map_parameters[1], map_assignments)
        if map_parameters is not None and len(map_parameters) == 2
        else (None, None)
    )
    map_params_ok = (
        outer_params_ok
        and map_parameters is not None
        and len(map_parameters) == 2
        and _signature(map_parameters[0], map_assignments)
        == expected_params_signature
        and map_override is not None
        and set(map_override) == {"yaml_filename"}
        and not map_expansions
        and _same_expression(
            map_override["yaml_filename"], "map_yaml", map_assignments
        )
    )
    if not map_params_ok:
        _contract_failure(
            failures,
            "generated runtime artifacts",
            "map_server must consume generated params_file plus only yaml_filename override",
        )
    controller = _node_by_identity(
        function, ("nav2_controller", "controller_server"), assignments
    )
    behavior = _node_by_identity(
        function, ("nav2_behaviors", "behavior_server"), assignments
    )
    stamper = _node_by_identity(
        function, ("robot_navigation", "twist_stamper"), assignments
    )
    nav2_consumers = controller + behavior
    for identity in (
        ("nav2_planner", "planner_server"),
        ("nav2_bt_navigator", "bt_navigator"),
    ):
        nav2_consumers += _node_by_identity(function, identity, assignments)
    nav2_params_ok = len(nav2_consumers) == 4 and all(
        (
            (parameters := _sequence(_keyword(call, "parameters"), assignments))
            is not None
            and len(parameters) == 1
            and _same_expression(parameters[0], "params_file", assignments)
        )
        for call in nav2_consumers
    )
    if not nav2_params_ok:
        _contract_failure(
            failures,
            "generated runtime artifacts",
            "navigation servers must consume the generated Nav2 parameters file",
        )
    routing_ok = len(controller) == len(behavior) == len(stamper) == 1
    routing_ok = routing_ok and all(
        _remapping_pairs(call, assignments) == [("cmd_vel", "/cmd_vel_nav")]
        for call in controller + behavior
    )
    stamper_params = _parameter_dict(stamper[0], assignments) if len(stamper) == 1 else None
    routing_ok = (
        routing_ok
        and stamper_params is not None
        and _literal(stamper_params.get("input_topic"), assignments) == "/cmd_vel_nav"
        and _same_expression(
            stamper_params.get("output_topic"), "cmd_vel_output_topic", assignments
        )
    )
    for call in _calls(function, "Node"):
        pairs = _remapping_pairs(call, assignments)
        if pairs is None or any(target == "/cmd_vel" for _, target in pairs):
            routing_ok = False
    if not routing_ok:
        _contract_failure(
            failures,
            "command gate routing",
            "Nav2 consumers must route only through /cmd_vel_nav and the "
            "configurable stamper output",
        )

    transforms = _node_by_identity(
        function, ("tf2_ros", "static_transform_publisher"), assignments
    )
    flags = (
        _flag_arguments(_keyword(transforms[0], "arguments"), assignments)
        if len(transforms) == 1
        else None
    )
    expected_flags = {
        "--x": "weld_x", "--y": "weld_y", "--z": "weld_z",
        "--qx": "weld_qx", "--qy": "weld_qy", "--qz": "weld_qz", "--qw": "weld_qw",
    }
    weld_ok = flags is not None and set(flags) == set(expected_flags) | {
        "--frame-id", "--child-frame-id"
    }
    weld_ok = weld_ok and all(
        _same_expression(flags[flag], name, assignments)
        for flag, name in expected_flags.items()
    )
    frame_ok = (
        weld_ok
        and _literal(flags["--frame-id"], assignments) == "body"
        and _literal(flags["--child-frame-id"], assignments) == "base_footprint"
        and stamper_params is not None
        and _literal(stamper_params.get("frame_id"), assignments) == "base_link"
    )
    declarations = _declare_names(function, assignments)
    weld_ok = weld_ok and all(
        declarations.count(name) == 1
        for name in ("weld_qx", "weld_qy", "weld_qz", "weld_qw")
    ) and not any(name in declarations for name in ("weld_roll", "weld_pitch", "weld_yaw"))
    if not weld_ok:
        _contract_failure(
            failures, "quaternion weld", "navigation static transform must use qx/qy/qz/qw only"
        )
    if not frame_ok:
        _contract_failure(
            failures,
            "critical frames",
            "navigation must weld body->base_footprint and stamp base_link",
        )


def _validate_cmd_gate_topology(tree, failures):
    class_node = _definition(tree, "CmdVelGate", ast.ClassDef)
    init = _method(class_node, "__init__")
    init_assignments = _assignments(init)
    subscriptions = [
        call
        for call in _calls(init, "create_subscription")
        if call.args and _signature(call.args[0], init_assignments) == ("name", "TwistStamped")
    ]
    subscription_topics = [
        _literal(call.args[1], init_assignments) if len(call.args) >= 2 else _UNKNOWN
        for call in subscriptions
    ]
    publishers = []
    if class_node is not None:
        for method in (node for node in class_node.body if isinstance(node, ast.FunctionDef)):
            method_assignments = _assignments(method)
            publishers.extend(
                (call, method_assignments)
                for call in _calls(method, "create_publisher")
                if call.args
                and _signature(call.args[0], method_assignments) == ("name", "TwistStamped")
            )
    routing_ok = sorted(subscription_topics) == ["/cmd_vel_auto", "/cmd_vel_manual"]
    routing_ok = routing_ok and len(publishers) == 1 and len(publishers[0][0].args) >= 2
    routing_ok = routing_ok and _literal(
        publishers[0][0].args[1], publishers[0][1]
    ) == "/cmd_vel"
    if not routing_ok:
        _contract_failure(
            failures,
            "command gate routing",
            "CmdVelGate requires exactly auto/manual TwistStamped subscriptions "
            "and one /cmd_vel publisher",
        )

    new_output = _method(class_node, "_new_output")
    frame_assignments = [
        node
        for node in _walk_scope(new_output)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Attribute)
            and target.attr == "frame_id"
            and isinstance(target.value, ast.Attribute)
            and target.value.attr == "header"
            for target in node.targets
        )
    ] if new_output is not None else []
    if not (
        len(frame_assignments) == 1
        and isinstance(frame_assignments[0].value, ast.Constant)
        and frame_assignments[0].value.value == "base_link"
    ):
        _contract_failure(
            failures, "critical frames", "CmdVelGate output header frame must be base_link"
        )


def _validate_web_ui_topology(tree, failures):
    class_node = _definition(tree, "WebUiNode", ast.ClassDef)
    publishers = []
    direct_remap = False
    if class_node is not None:
        for method in (
            node for node in class_node.body if isinstance(node, ast.FunctionDef)
        ):
            assignments = _assignments(method)
            publishers.extend(
                (call, assignments)
                for call in _calls(method, "create_publisher")
            )
            for call in _calls(method):
                pairs = _remapping_pairs(call, assignments)
                if pairs is None or any(target == "/cmd_vel" for _, target in pairs):
                    direct_remap = True

    twist_publishers = [
        (call, assignments)
        for call, assignments in publishers
        if call.args
        and _signature(call.args[0], assignments) == ("name", "TwistStamped")
    ]
    publisher_topics = [
        _literal(call.args[1], assignments)
        for call, assignments in publishers
        if len(call.args) >= 2
    ]
    routing_ok = (
        len(twist_publishers) == 1
        and len(twist_publishers[0][0].args) >= 2
        and _literal(
            twist_publishers[0][0].args[1], twist_publishers[0][1]
        )
        == "/cmd_vel_manual"
        and "/cmd_vel" not in publisher_topics
        and not direct_remap
    )
    if not routing_ok:
        _contract_failure(
            failures,
            "command gate routing",
            "WebUiNode requires exactly one TwistStamped /cmd_vel_manual publisher "
            "and no direct /cmd_vel publisher/remap",
        )


def _validate_active_topology(repo_root, failures):
    trees = _active_source_trees(repo_root, failures)
    if set(trees) != set(_ACTIVE_TOPOLOGY_FILES):
        return
    _validate_formal_topology(trees["formal"], failures)
    _validate_sim_topology(trees["sim"], failures)
    _validate_real_topology(trees["real_chassis"], trees["real_robot"], failures)
    _validate_slam_topology(trees["slam"], failures)
    _validate_navigation_topology(trees["navigation"], failures)
    _validate_cmd_gate_topology(trees["cmd_gate"], failures)
    _validate_web_ui_topology(trees["web_ui"], failures)


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
