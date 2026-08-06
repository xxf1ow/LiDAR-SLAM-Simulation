import builtins
import shutil
import tempfile
from pathlib import Path

import pytest
import yaml

from system_bringup import consistency_check as cc
from system_bringup import runtime_config_compiler as rcc


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
SOURCE_REPO_ROOT = PACKAGE_ROOT.parents[2]
ACTIVE_TOPOLOGY_FILES = {
    "formal": "core/bringup/system_bringup/launch/bringup.launch.py",
    "slam": "core/bringup/system_bringup/launch/slam_stack.launch.py",
    "sim": "core/simulation/robot_gz_bringup/launch/robot_gz.launch.py",
    "real_chassis": "core/robot/robot_bringup/launch/real_chassis.launch.py",
    "real_robot": "core/robot/robot_bringup/launch/robot.launch.py",
    "navigation": "core/navigation/robot_navigation/launch/navigation.launch.py",
    "cmd_gate": "core/robot/cmd_vel_gate/cmd_vel_gate/gate_node.py",
    "web_ui": "core/bringup/robot_web_ui/robot_web_ui/web_ui_node.py",
}


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _write_yaml(path, data):
    path.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def _mutate_yaml(path, dotted_path, value):
    data = _load_yaml(path)
    target = data
    for key in dotted_path[:-1]:
        target = target[key]
    target[dotted_path[-1]] = value
    _write_yaml(path, data)


def _delete_yaml(path, dotted_path):
    data = _load_yaml(path)
    target = data
    for key in dotted_path[:-1]:
        target = target[key]
    del target[dotted_path[-1]]
    _write_yaml(path, data)


def _replace_source(repo_root, relative_path, old, new):
    path = repo_root / relative_path
    source = path.read_text(encoding="utf-8")
    assert old in source
    path.write_text(source.replace(old, new), encoding="utf-8")


@pytest.fixture
def runtime_factory(tmp_path, monkeypatch):
    runtime_dirs = []
    active_paths = {}
    count = 0

    monkeypatch.setattr(
        cc,
        "_resolve_installed_topology_paths",
        lambda failures: {
            label: path
            for label, path in active_paths.items()
            if label in cc._ACTIVE_TOPOLOGY_FILES
        },
        raising=False,
    )

    def build(platform="sim", mode="navigation"):
        nonlocal count
        count += 1
        repo_root = tmp_path / f"repo-{count}"
        config_dir = repo_root / "core" / "bringup" / "system_bringup" / "config"
        shutil.copytree(CONFIG_DIR, config_dir)
        for label, relative_path in ACTIVE_TOPOLOGY_FILES.items():
            source = SOURCE_REPO_ROOT / relative_path
            destination = repo_root / relative_path
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
            active_paths[label] = destination
        config_path = config_dir / "bringup.yaml"
        config = _load_yaml(config_path)
        config["platform"] = platform
        config["mode"] = mode
        _write_yaml(config_path, config)

        runtime_dir = Path(tempfile.mkdtemp(prefix="system_bringup-runtime-"))
        runtime_dirs.append(runtime_dir)
        manifest = rcc.compile_runtime_configs(config_path, runtime_dir)
        return repo_root, manifest

    yield build

    for runtime_dir in runtime_dirs:
        shutil.rmtree(runtime_dir, ignore_errors=True)


@pytest.mark.parametrize(
    "platform,mode",
    [("sim", "navigation"), ("real", "mapping")],
)
def test_valid_manifest_is_read_once_without_source_reload_compile_or_write(
    runtime_factory, monkeypatch, platform, mode
):
    repo_root, manifest = runtime_factory(platform, mode)
    safe_load = cc.yaml.safe_load
    loads = []
    validations = []
    validate_generated = rcc._validate_generated_configs

    def counted_load(stream):
        loads.append(stream)
        return safe_load(stream)

    def counted_validation(*args):
        validations.append(args)
        return validate_generated(*args)

    def forbidden(*_args, **_kwargs):
        raise AssertionError("runtime consistency must be read-only and manifest-only")

    monkeypatch.setattr(cc.yaml, "safe_load", counted_load)
    monkeypatch.setattr(cc, "load_bringup_config", forbidden)
    monkeypatch.setattr(cc, "build_real_runtime_configs", forbidden)
    monkeypatch.setattr(cc, "write_real_runtime_configs", forbidden)
    monkeypatch.setattr(rcc, "compile_runtime_configs", forbidden)
    monkeypatch.setattr(rcc, "_validate_generated_configs", counted_validation)
    monkeypatch.setattr(Path, "write_text", forbidden)
    monkeypatch.setattr(Path, "write_bytes", forbidden)

    assert cc.run_runtime_consistency(repo_root, manifest) == []
    assert len(loads) == 4
    assert len(validations) == 1


def test_missing_artifact_reports_manifest_key_and_path(runtime_factory):
    repo_root, manifest = runtime_factory()
    missing = manifest["nav2_path"]
    missing.unlink()

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("nav2_path" in failure and str(missing) in failure for failure in failures)


def test_foreign_artifact_path_is_rejected(runtime_factory, tmp_path):
    repo_root, manifest = runtime_factory()
    foreign = tmp_path / "foreign-nav2.yaml"
    shutil.copyfile(manifest["nav2_path"], foreign)
    manifest["nav2_path"] = foreign

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("nav2_path" in failure and "runtime" in failure for failure in failures)


def test_artifact_paths_must_share_one_runtime_directory(runtime_factory):
    repo_root, manifest = runtime_factory()
    other_runtime = Path(tempfile.mkdtemp(prefix="system_bringup-runtime-"))
    try:
        other_web_ui = other_runtime / manifest["web_ui_path"].name
        shutil.copyfile(manifest["web_ui_path"], other_web_ui)
        manifest["web_ui_path"] = other_web_ui

        failures = cc.run_runtime_consistency(repo_root, manifest)

        assert any("web_ui_path" in failure and "same runtime directory" in failure for failure in failures)
    finally:
        shutil.rmtree(other_runtime, ignore_errors=True)


@pytest.mark.parametrize(
    "mutation,expected",
    [
        (lambda manifest: manifest.__setitem__("platform", "real"), "platform"),
        (lambda manifest: manifest.__setitem__("platform", []), "platform"),
        (lambda manifest: manifest.__setitem__("platform", {}), "platform"),
        (lambda manifest: manifest.__setitem__("mode", "mapping"), "mode"),
        (lambda manifest: manifest.__setitem__("mode", "unsupported"), "mode"),
        (lambda manifest: manifest.__setitem__("mode", []), "mode"),
        (lambda manifest: manifest.__setitem__("mode", {}), "mode"),
        (
            lambda manifest: manifest.__setitem__(
                "use_sim_time", not manifest["use_sim_time"]
            ),
            "use_sim_time",
        ),
    ],
)
def test_manifest_platform_mode_and_clock_mismatch_is_reported(
    runtime_factory, mutation, expected
):
    repo_root, manifest = runtime_factory()
    mutation(manifest)

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(expected in failure for failure in failures)
    if expected == "platform" and isinstance(manifest["platform"], (list, dict)):
        assert sum("platform" in failure for failure in failures) >= 3


@pytest.mark.parametrize(
    "platform,profile_path,value,expected",
    [
        ("sim", ("profile", "hardware", "chassis", "backend"), "can_8030d", "gazebo"),
        ("sim", ("profile", "hardware", "lidar", "backend"), "vanjee", "gazebo"),
        ("real", ("profile", "hardware", "chassis", "backend"), "gazebo", "can_8030d"),
        ("real", ("profile", "hardware", "lidar", "backend"), "gazebo", "vanjee"),
    ],
)
def test_backend_mismatch_is_reported(
    runtime_factory, platform, profile_path, value, expected
):
    repo_root, manifest = runtime_factory(platform)
    _mutate_yaml(manifest["effective_profile_path"], profile_path, value)

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("backend" in failure and expected in failure for failure in failures)


@pytest.mark.parametrize(
    "artifact_key,dotted_path,value,expected",
    [
        (
            "controllers_path",
            ("base_controller", "ros__parameters", "wheel_radius"),
            123.0,
            "controllers",
        ),
        (
            "web_ui_path",
            ("robot_web_ui", "ros__parameters", "max_linear_speed"),
            123.0,
            "web_ui",
        ),
        (
            "nav2_path",
            ("controller_server", "ros__parameters", "FollowPath", "vx_max"),
            123.0,
            "nav2",
        ),
        (
            "nav2_path",
            (
                "global_costmap",
                "global_costmap",
                "ros__parameters",
                "footprint",
            ),
            "[]",
            "footprint",
        ),
    ],
)
def test_generated_controller_web_ui_nav2_and_footprint_drift_is_reported(
    runtime_factory, artifact_key, dotted_path, value, expected
):
    repo_root, manifest = runtime_factory()
    _mutate_yaml(manifest[artifact_key], dotted_path, value)

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(expected in failure for failure in failures)


def test_manifest_geometry_drift_is_reported(runtime_factory):
    repo_root, manifest = runtime_factory()
    manifest["robot_launch_arguments"]["base_length"] = "99.0"

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("robot_launch_arguments.base_length" in failure for failure in failures)


def test_manifest_weld_drift_is_reported(runtime_factory):
    repo_root, manifest = runtime_factory()
    manifest["compatibility_body_weld_arguments"]["qw"] = "0.0"

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("compatibility_body_weld_arguments.qw" in failure for failure in failures)


@pytest.mark.parametrize(
    "section,key,value",
    [
        ("translation", "x", 99.0),
        ("rotation", "qw", 0.0),
    ],
)
def test_effective_report_compatibility_weld_drift_is_reported(
    runtime_factory, section, key, value
):
    repo_root, manifest = runtime_factory()
    _mutate_yaml(
        manifest["effective_profile_path"],
        ("compatibility", "body_to_base_footprint", section, key),
        value,
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        f"compatibility.body_to_base_footprint.{section}.{key}" in failure
        for failure in failures
    )


@pytest.mark.parametrize(
    "section,key",
    [("translation", "x"), ("rotation", "qw")],
)
def test_effective_report_compatibility_weld_missing_value_is_reported(
    runtime_factory, section, key
):
    repo_root, manifest = runtime_factory()
    _delete_yaml(
        manifest["effective_profile_path"],
        ("compatibility", "body_to_base_footprint", section, key),
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        f"compatibility.body_to_base_footprint.{section}.{key}" in failure
        for failure in failures
    )


@pytest.mark.parametrize(
    "dotted_path,value,expected",
    [
        (
            ("compatibility", "body_to_base_footprint", "status"),
            "permanent",
            "compatibility.body_to_base_footprint.status",
        ),
        (
            ("compatibility", "body_to_base_footprint", "assumption"),
            "different assumption",
            "compatibility.body_to_base_footprint.assumption",
        ),
        (
            ("compatibility", "body_to_base_footprint", "follow_up_section"),
            99,
            "compatibility.body_to_base_footprint.follow_up_section",
        ),
        (
            ("deferred_compatibility", 0, "component"),
            "different.component",
            "deferred_compatibility.nav2.behavior_server",
        ),
        (
            ("deferred_compatibility", 0, "status"),
            "done",
            "deferred_compatibility.nav2.behavior_server.status",
        ),
        (
            (
                "deferred_compatibility",
                0,
                "template_values",
                "max_rotational_vel",
            ),
            99.0,
            "deferred_compatibility.nav2.behavior_server.template_values.max_rotational_vel",
        ),
        (
            (
                "deferred_compatibility",
                0,
                "profile_values",
                "max_angular_velocity",
            ),
            99.0,
            "deferred_compatibility.nav2.behavior_server.profile_values.max_angular_velocity",
        ),
        (
            ("deferred_compatibility", 0, "reason"),
            "none",
            "deferred_compatibility.nav2.behavior_server.reason",
        ),
    ],
)
def test_effective_report_required_metadata_drift_is_reported(
    runtime_factory, dotted_path, value, expected
):
    repo_root, manifest = runtime_factory()
    _mutate_yaml(manifest["effective_profile_path"], dotted_path, value)

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(expected in failure for failure in failures)


@pytest.mark.parametrize(
    "dotted_path,expected",
    [
        (
            ("compatibility", "body_to_base_footprint", "assumption"),
            "compatibility.body_to_base_footprint.assumption",
        ),
        (
            ("deferred_compatibility", 0, "reason"),
            "deferred_compatibility.nav2.behavior_server.reason",
        ),
    ],
)
def test_effective_report_required_metadata_deletion_is_reported(
    runtime_factory, dotted_path, expected
):
    repo_root, manifest = runtime_factory()
    _delete_yaml(manifest["effective_profile_path"], dotted_path)

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(expected in failure for failure in failures)


def test_malformed_artifact_path_is_aggregated(runtime_factory):
    class BadPath:
        def __fspath__(self):
            raise RuntimeError("forced bad path")

    repo_root, manifest = runtime_factory()
    manifest["controllers_path"] = BadPath()

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "controllers_path" in failure and "forced bad path" in failure
        for failure in failures
    )


def test_forced_artifact_resolution_error_is_aggregated(
    runtime_factory, monkeypatch
):
    repo_root, manifest = runtime_factory()
    target = Path(manifest["nav2_path"])
    resolve = Path.resolve

    def forced_resolve(path, *args, **kwargs):
        if str(path) == str(target):
            raise RuntimeError("forced resolve failure")
        return resolve(path, *args, **kwargs)

    monkeypatch.setattr(Path, "resolve", forced_resolve)

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "nav2_path" in failure and "forced resolve failure" in failure
        for failure in failures
    )


def test_forced_same_path_resolution_error_is_aggregated(
    runtime_factory, monkeypatch
):
    repo_root, manifest = runtime_factory()
    bad_reference = "forced-relative-controller.yaml"
    _mutate_yaml(
        manifest["effective_profile_path"],
        ("generated_configs", "controllers"),
        bad_reference,
    )
    resolve = Path.resolve

    def forced_resolve(path, *args, **kwargs):
        if str(path) == bad_reference:
            raise RuntimeError("forced same-path failure")
        return resolve(path, *args, **kwargs)

    monkeypatch.setattr(Path, "resolve", forced_resolve)

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "generated_configs.controllers" in failure
        and "forced same-path failure" in failure
        for failure in failures
    )


@pytest.mark.parametrize(
    "relative_path,old,new,expected",
    [
        (
            "core/bringup/system_bringup/launch/bringup.launch.py",
            'manifest["web_ui_path"]',
            'manifest["bringup_config_path"]',
            "generated runtime artifacts",
        ),
        (
            "core/bringup/system_bringup/launch/bringup.launch.py",
            'manifest["nav2_path"]',
            'manifest["bringup_config_path"]',
            "generated runtime artifacts",
        ),
        (
            "core/bringup/system_bringup/launch/bringup.launch.py",
            'manifest["controllers_path"]',
            'manifest["bringup_config_path"]',
            "generated runtime artifacts",
        ),
        (
            "core/simulation/robot_gz_bringup/launch/robot_gz.launch.py",
            '"sensor_y"',
            '"sensor_side"',
            "simulation backend interface",
        ),
        (
            "core/robot/robot_bringup/launch/real_chassis.launch.py",
            '"wheel_width"',
            '"wheel_span"',
            "real backend interface",
        ),
        (
            "core/robot/cmd_vel_gate/cmd_vel_gate/gate_node.py",
            '"/cmd_vel_manual"',
            '"/cmd_vel_bypass"',
            "command gate routing",
        ),
        (
            "core/navigation/robot_navigation/launch/navigation.launch.py",
            "'--qw'",
            "'--yaw'",
            "quaternion weld",
        ),
        (
            "core/bringup/system_bringup/launch/slam_stack.launch.py",
            '"/cloud_registered"',
            '"/cloud_changed"',
            "readiness sequence",
        ),
        (
            "core/navigation/robot_navigation/launch/navigation.launch.py",
            "'base_footprint'",
            "'base_changed'",
            "critical frames",
        ),
        (
            "core/bringup/system_bringup/launch/bringup.launch.py",
            "+ [chassis, lidar]",
            "+ [lidar, chassis]",
            "backend sequencing",
        ),
    ],
)
def test_active_topology_contract_drift_is_reported(
    runtime_factory, relative_path, old, new, expected
):
    repo_root, manifest = runtime_factory()
    _replace_source(repo_root, relative_path, old, new)

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(expected in failure for failure in failures)


@pytest.mark.parametrize("legacy_name", ["derive_real_geometry", "run"])
def test_active_topology_rejects_prohibited_legacy_import(
    runtime_factory, legacy_name
):
    repo_root, manifest = runtime_factory()
    path = repo_root / "core/bringup/system_bringup/launch/bringup.launch.py"
    source = path.read_text(encoding="utf-8")
    path.write_text(
        f"from system_bringup.consistency_check import {legacy_name}\n"
        + source,
        encoding="utf-8",
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("prohibited active legacy" in failure for failure in failures)


def test_active_topology_rejects_source_install_mismatch(
    runtime_factory, tmp_path, monkeypatch
):
    repo_root, manifest = runtime_factory()
    installed_root = tmp_path / "installed"
    installed_paths = {}
    for label, relative_path in ACTIVE_TOPOLOGY_FILES.items():
        source = repo_root / relative_path
        destination = installed_root / relative_path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        installed_paths[label] = destination
    installed_paths["formal"].write_text(
        installed_paths["formal"].read_text(encoding="utf-8")
        + "\nSTALE_INSTALLED_COPY = True\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(
        cc,
        "_resolve_installed_topology_paths",
        lambda failures: {
            label: path
            for label, path in installed_paths.items()
            if label in cc._ACTIVE_TOPOLOGY_FILES
        },
        raising=False,
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "installed" in failure
        and "reviewed source" in failure
        and "rebuild" in failure
        for failure in failures
    )


def test_active_topology_source_install_comparison_is_byte_exact(
    runtime_factory, tmp_path, monkeypatch
):
    repo_root, manifest = runtime_factory()
    installed_root = tmp_path / "installed-newlines"
    installed_paths = {}
    for label, relative_path in ACTIVE_TOPOLOGY_FILES.items():
        source = repo_root / relative_path
        destination = installed_root / relative_path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        installed_paths[label] = destination
    formal = installed_paths["formal"]
    formal.write_bytes(formal.read_bytes().replace(b"\n", b"\r\n"))
    monkeypatch.setattr(
        cc,
        "_resolve_installed_topology_paths",
        lambda failures: installed_paths,
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "installed topology differs from reviewed source" in failure
        for failure in failures
    )


def test_installed_resolver_no_ros_diagnostic_names_both_node_modules(
    monkeypatch,
):
    real_import = builtins.__import__

    def reject_ament(name, *args, **kwargs):
        if name.startswith("ament_index_python"):
            raise ModuleNotFoundError("forced missing ament")
        return real_import(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", reject_ament)
    failures = []

    assert cc._resolve_installed_topology_paths(failures) == {}
    assert any(
        "cmd_vel_gate.gate_node" in failure
        and "robot_web_ui.web_ui_node" in failure
        for failure in failures
    )


def test_active_topology_rejects_web_ui_manual_publisher_bypass_with_dead_decoy(
    runtime_factory,
):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["web_ui"],
        '            "/cmd_vel_manual",\n',
        '            "/cmd_vel",\n',
    )
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["web_ui"],
        "        self._manual_publisher = self.create_publisher(\n",
        "        if False:\n"
        '            "/cmd_vel_manual"\n'
        "        self._manual_publisher = self.create_publisher(\n",
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("command gate routing" in failure for failure in failures)


def test_active_topology_rejects_additive_web_ui_direct_publisher(
    runtime_factory,
):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["web_ui"],
        "        mode_qos = QoSProfile(\n",
        "        self._bypass_publisher = self.create_publisher(\n"
        '            TwistStamped, "/cmd_vel", 1\n'
        "        )\n"
        "        mode_qos = QoSProfile(\n",
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("command gate routing" in failure for failure in failures)


@pytest.mark.parametrize(
    "node_source",
    [
        'Node(package="cmd_vel_gate", executable="cmd_vel_gate"),',
        'Node(package="robot_web_ui", executable="robot_web_ui", '
        'parameters=[str(manifest["web_ui_path"])]),',
    ],
)
def test_active_topology_rejects_duplicate_control_layer_nodes(
    runtime_factory, node_source
):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["formal"],
        "    control_layer = [\n",
        f"    control_layer = [\n        {node_source}\n",
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("topology cardinality" in failure for failure in failures)


def test_active_topology_rejects_duplicate_shared_slam_include(runtime_factory):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["formal"],
        '    if platform == "sim":\n',
        "    control_layer.append(\n"
        "        _inc(\"system_bringup\", \"launch/slam_stack.launch.py\", {})\n"
        "    )\n\n"
        '    if platform == "sim":\n',
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("topology cardinality" in failure for failure in failures)


def test_active_topology_rejects_direct_extra_include(runtime_factory):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["formal"],
        '    if platform == "sim":\n',
        "    control_layer.append(\n"
        "        IncludeLaunchDescription(\n"
        "            PythonLaunchDescriptionSource(\"duplicate.launch.py\")\n"
        "        )\n"
        "    )\n\n"
        '    if platform == "sim":\n',
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("topology cardinality" in failure for failure in failures)


@pytest.mark.parametrize(
    "branch_marker,duplicate_call",
    [
        (
            '    if platform == "sim":\n',
            '        control_layer.append(_inc("robot_gz_bringup", '
            '"launch/robot_gz.launch.py", {}))\n',
        ),
        (
            '    if platform == "real":\n',
            '        control_layer.append(_inc("robot_bringup", '
            '"launch/real_chassis.launch.py", {}))\n'
            '        control_layer.append(_inc("vanjee_lidar_ros", '
            '"launch/vanjee_lidar.launch.py", {}))\n',
        ),
    ],
)
def test_active_topology_rejects_duplicate_backend_includes(
    runtime_factory, branch_marker, duplicate_call
):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["formal"],
        branch_marker,
        branch_marker + duplicate_call,
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("topology cardinality" in failure for failure in failures)


def test_active_topology_rejects_additive_direct_cmd_vel_bypass(runtime_factory):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["navigation"],
        "remappings=[('cmd_vel', '/cmd_vel_nav')],",
        "remappings=[('cmd_vel', '/cmd_vel_nav'), "
        "('cmd_vel_unchecked', '/cmd_vel')],",
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("command gate routing" in failure for failure in failures)


def test_active_topology_ignores_dead_string_decoy(runtime_factory):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["formal"],
        'parameters=[str(manifest["web_ui_path"])],',
        'parameters=[str(manifest["bringup_config_path"])],',
    )
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["formal"],
        "    control_layer = [\n",
        "    if False:\n"
        "        \"parameters=[str(manifest['web_ui_path'])]\"\n"
        "    control_layer = [\n",
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("generated runtime artifacts" in failure for failure in failures)


@pytest.mark.parametrize(
    "label,old,new",
    [
        (
            "slam",
            '"params_file": nav2_params',
            '"params_file": fast_cfg',
        ),
        (
            "navigation",
            "parameters=[params_file],",
            "parameters=[default_params],",
        ),
    ],
)
def test_active_topology_rejects_generated_nav2_dataflow_drift(
    runtime_factory, label, old, new
):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES[label],
        old,
        new,
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("generated runtime artifacts" in failure for failure in failures)


@pytest.mark.parametrize(
    "replacement",
    [
        "parameters=[default_params, {'yaml_filename': map_yaml}],",
        "parameters=[{'yaml_filename': map_yaml}],",
        "parameters=[params_file, {'yaml_filename': map_yaml}, "
        "{'use_sim_time': use_sim_time}],",
    ],
)
def test_active_topology_rejects_map_server_parameter_drift(
    runtime_factory, replacement
):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["navigation"],
        "parameters=[params_file, {'yaml_filename': map_yaml}],",
        replacement,
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("generated runtime artifacts" in failure for failure in failures)


def test_active_topology_accepts_reordered_real_geometry_arguments(
    runtime_factory,
):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["real_chassis"],
        '            "controllers_file",\n'
        '            "base_length", "base_width", "base_height", "base_link_height",\n',
        '            "base_length",\n'
        '            "controllers_file", "base_width", "base_height", "base_link_height",\n',
    )

    assert cc.run_runtime_consistency(repo_root, manifest) == []


def test_active_topology_accepts_reordered_real_controller_remappings(
    runtime_factory,
):
    repo_root, manifest = runtime_factory()
    _replace_source(
        repo_root,
        ACTIVE_TOPOLOGY_FILES["real_robot"],
        '            ("~/robot_description", "/robot_description"),\n'
        '            ("/base_controller/cmd_vel", "/cmd_vel"),\n',
        '            ("/base_controller/cmd_vel", "/cmd_vel"),\n'
        '            ("~/robot_description", "/robot_description"),\n',
    )

    assert cc.run_runtime_consistency(repo_root, manifest) == []


def test_active_topology_accepts_equivalent_local_aliases_and_assigned_dict(
    runtime_factory,
):
    repo_root, manifest = runtime_factory()
    formal_path = ACTIVE_TOPOLOGY_FILES["formal"]
    _replace_source(
        repo_root,
        formal_path,
        "    control_layer = [\n",
        "    web_ui_parameters = [str(manifest[\"web_ui_path\"])]\n"
        "    controllers_path = str(manifest[\"controllers_path\"])\n"
        "    control_layer = [\n",
    )
    _replace_source(
        repo_root,
        formal_path,
        'parameters=[str(manifest["web_ui_path"])],',
        "parameters=web_ui_parameters,",
    )
    _replace_source(
        repo_root,
        formal_path,
        '"controllers_file": str(manifest["controllers_path"]),',
        '"controllers_file": controllers_path,',
    )
    _replace_source(
        repo_root,
        formal_path,
        "    slam_stack = _inc(\n"
        '        "system_bringup",\n'
        '        "launch/slam_stack.launch.py",\n'
        "        {\n",
        "    slam_arguments = {\n",
    )
    _replace_source(
        repo_root,
        formal_path,
        '            "weld_qw": weld["qw"],\n'
        "        },\n"
        "    )\n\n"
        '    if platform == "sim":\n',
        '        "weld_qw": weld["qw"],\n'
        "    }\n"
        "    slam_stack = _inc(\n"
        '        "system_bringup", "launch/slam_stack.launch.py", slam_arguments\n'
        "    )\n\n"
        '    if platform == "sim":\n',
    )

    assert cc.run_runtime_consistency(repo_root, manifest) == []


def test_report_generated_path_reference_drift_is_reported(runtime_factory):
    repo_root, manifest = runtime_factory()
    _mutate_yaml(
        manifest["effective_profile_path"],
        ("generated_configs", "controllers"),
        "/tmp/not-the-controller.yaml",
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("generated_configs.controllers" in failure for failure in failures)


def test_report_source_profile_must_match_selected_manifest_profile(runtime_factory):
    repo_root, manifest = runtime_factory()
    _mutate_yaml(
        manifest["effective_profile_path"],
        ("source_profile",),
        "/tmp/wrong-profile.yaml",
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("source_profile" in failure for failure in failures)


@pytest.mark.parametrize(
    "platform,dotted_path,expected",
    [
        ("sim", ("slam_stack", "sim", "fast_lio", "config"), "slam_stack.sim.fast_lio.config"),
        ("sim", ("robot_gz", "world"), "robot_gz.world"),
        ("real", ("slam_stack", "real", "lio_sam", "config"), "slam_stack.real.lio_sam.config"),
        ("real", ("robot_bringup", "use_mock_hardware"), "robot_bringup.use_mock_hardware"),
        ("real", ("vanjee_lidar", "config"), "vanjee_lidar.config"),
    ],
)
def test_unmigrated_downstream_config_shape_is_validated(
    runtime_factory, platform, dotted_path, expected
):
    repo_root, manifest = runtime_factory(platform)
    target = manifest["bringup_config"]
    for key in dotted_path[:-1]:
        target = target[key]
    del target[dotted_path[-1]]

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(expected in failure for failure in failures)
