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
ACTIVE_RUNTIME_FILES = {
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


@pytest.fixture
def runtime_factory(tmp_path, monkeypatch):
    runtime_dirs = []
    active_paths = {}
    count = 0

    monkeypatch.setattr(
        cc,
        "_resolve_installed_runtime_paths",
        lambda failures: {
            label: path
            for label, path in active_paths.items()
            if label in cc._ACTIVE_RUNTIME_FILES
        },
        raising=False,
    )

    def build(platform="sim", mode="navigation"):
        nonlocal count
        count += 1
        repo_root = tmp_path / f"repo-{count}"
        config_dir = repo_root / "core" / "bringup" / "system_bringup" / "config"
        shutil.copytree(CONFIG_DIR, config_dir)
        for label, relative_path in ACTIVE_RUNTIME_FILES.items():
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
    sensor_validations = []
    validate_generated = rcc._validate_generated_configs
    validate_sensors = rcc._validate_sensor_generated_configs

    def counted_load(stream):
        loads.append(stream)
        return safe_load(stream)

    def counted_validation(*args):
        validations.append(args)
        return validate_generated(*args)

    def counted_sensor_validation(*args):
        sensor_validations.append(args)
        return validate_sensors(*args)

    def forbidden(*_args, **_kwargs):
        raise AssertionError("runtime consistency must be read-only and manifest-only")

    monkeypatch.setattr(cc.yaml, "safe_load", counted_load)
    monkeypatch.setattr(cc, "load_bringup_config", forbidden)
    monkeypatch.setattr(cc, "build_real_runtime_configs", forbidden)
    monkeypatch.setattr(cc, "write_real_runtime_configs", forbidden)
    monkeypatch.setattr(rcc, "compile_runtime_configs", forbidden)
    monkeypatch.setattr(rcc, "_validate_generated_configs", counted_validation)
    monkeypatch.setattr(
        rcc, "_validate_sensor_generated_configs", counted_sensor_validation
    )
    monkeypatch.setattr(Path, "write_text", forbidden)
    monkeypatch.setattr(Path, "write_bytes", forbidden)

    assert cc.run_runtime_consistency(repo_root, manifest) == []
    assert len(loads) == len(cc._runtime_artifacts(platform))
    assert len(validations) == 1
    assert len(sensor_validations) == 1


def test_missing_artifact_reports_manifest_key_and_path(runtime_factory):
    repo_root, manifest = runtime_factory()
    missing = manifest["nav2_path"]
    missing.unlink()

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any("nav2_path" in failure and str(missing) in failure for failure in failures)


@pytest.mark.parametrize(
    "platform,present,absent",
    [
        ("sim", "lidar_adapter_path", "vanjee_lidar_path"),
        ("real", "vanjee_lidar_path", "lidar_adapter_path"),
    ],
)
def test_consistency_requires_only_selected_sensor_artifacts(
    runtime_factory, platform, present, absent
):
    repo_root, manifest = runtime_factory(platform=platform)

    assert present in manifest
    assert absent not in manifest
    assert cc.run_runtime_consistency(repo_root, manifest) == []

    missing = manifest[present]
    missing.unlink()
    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        present in failure and str(missing) in failure for failure in failures
    )


@pytest.mark.parametrize(
    "platform,foreign_key",
    [
        ("sim", "vanjee_lidar_path"),
        ("real", "lidar_adapter_path"),
    ],
)
def test_consistency_rejects_unselected_sensor_manifest_path(
    runtime_factory, platform, foreign_key
):
    repo_root, manifest = runtime_factory(platform)
    manifest[foreign_key] = manifest["sensor_gate_path"]

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        foreign_key in failure and platform in failure for failure in failures
    )


@pytest.mark.parametrize(
    "platform,foreign_artifact",
    [
        ("sim", "vanjee_lidar"),
        ("real", "lidar_adapter"),
    ],
)
def test_consistency_rejects_unselected_sensor_report_reference(
    runtime_factory, platform, foreign_artifact
):
    repo_root, manifest = runtime_factory(platform)
    _mutate_yaml(
        manifest["effective_profile_path"],
        ("generated_configs", foreign_artifact),
        str(manifest["sensor_gate_path"]),
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "generated_configs" in failure and foreign_artifact in failure
        for failure in failures
    )


def test_sensor_artifact_path_must_stay_in_os_runtime_directory(
    runtime_factory, tmp_path, monkeypatch
):
    repo_root, manifest = runtime_factory("sim")
    runtime_dir = manifest["effective_profile_path"].parent
    monkeypatch.setattr(cc.tempfile, "gettempdir", lambda: str(runtime_dir))
    foreign = tmp_path / manifest["lidar_adapter_path"].name
    shutil.copyfile(manifest["lidar_adapter_path"], foreign)
    manifest["lidar_adapter_path"] = foreign

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "lidar_adapter_path" in failure and "temporary directory" in failure
        for failure in failures
    )


def test_sensor_artifact_filename_is_fixed(runtime_factory):
    repo_root, manifest = runtime_factory("real")
    wrong = manifest["vanjee_lidar_path"].with_name("wrong.generated.yaml")
    shutil.copyfile(manifest["vanjee_lidar_path"], wrong)
    manifest["vanjee_lidar_path"] = wrong

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "vanjee_lidar_path" in failure and "unexpected artifact filename" in failure
        for failure in failures
    )


def test_sensor_artifact_must_share_effective_runtime_directory(runtime_factory):
    repo_root, manifest = runtime_factory("sim")
    other_runtime = Path(tempfile.mkdtemp(prefix="system_bringup-runtime-"))
    try:
        other_gate = other_runtime / manifest["sensor_gate_path"].name
        shutil.copyfile(manifest["sensor_gate_path"], other_gate)
        manifest["sensor_gate_path"] = other_gate

        failures = cc.run_runtime_consistency(repo_root, manifest)

        assert any(
            "sensor_gate_path" in failure and "same runtime directory" in failure
            for failure in failures
        )
    finally:
        shutil.rmtree(other_runtime, ignore_errors=True)


def test_sensor_report_path_reference_drift_is_reported(runtime_factory):
    repo_root, manifest = runtime_factory("sim")
    _mutate_yaml(
        manifest["effective_profile_path"],
        ("generated_configs", "lidar_adapter"),
        "/tmp/not-the-adapter.yaml",
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "generated_configs.lidar_adapter" in failure for failure in failures
    )


@pytest.mark.parametrize(
    "platform,manifest_key,dotted_path,value",
    [
        (
            "sim",
            "lidar_adapter_path",
            ("lidar_pointcloud_adapter", "ros__parameters", "output_topic"),
            "/wrong_points",
        ),
        (
            "real",
            "vanjee_lidar_path",
            ("vanjee_lidar", "ros__parameters", "lidar_frame"),
            "wrong_frame",
        ),
        (
            "real",
            "vanjee_lidar_path",
            ("vanjee_lidar", "ros__parameters", "point_cloud_topic"),
            "/wrong_points",
        ),
        (
            "sim",
            "sensor_gate_path",
            (
                "sensor_contract_gate",
                "ros__parameters",
                "expected_points_per_scan",
            ),
            1,
        ),
        (
            "real",
            "sensor_gate_path",
            ("sensor_contract_gate", "ros__parameters", "expected_imu_hz"),
            1.0,
        ),
        (
            "sim",
            "sensor_gate_path",
            (
                "sensor_contract_gate",
                "ros__parameters",
                "minimum_point_rate_ratio",
            ),
            0.0,
        ),
    ],
)
def test_sensor_generated_config_drift_is_reported(
    runtime_factory, platform, manifest_key, dotted_path, value
):
    repo_root, manifest = runtime_factory(platform)
    _mutate_yaml(manifest[manifest_key], dotted_path, value)

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "generated runtime config mismatch" in failure for failure in failures
    )


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


def test_installed_freshness_rejects_source_install_mismatch(
    runtime_factory, tmp_path, monkeypatch
):
    repo_root, manifest = runtime_factory()
    installed_root = tmp_path / "installed"
    installed_paths = {}
    for label, relative_path in ACTIVE_RUNTIME_FILES.items():
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
        "_resolve_installed_runtime_paths",
        lambda failures: {
            label: path
            for label, path in installed_paths.items()
            if label in cc._ACTIVE_RUNTIME_FILES
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


def test_installed_freshness_comparison_is_byte_exact(
    runtime_factory, tmp_path, monkeypatch
):
    repo_root, manifest = runtime_factory()
    installed_root = tmp_path / "installed-newlines"
    installed_paths = {}
    for label, relative_path in ACTIVE_RUNTIME_FILES.items():
        source = repo_root / relative_path
        destination = installed_root / relative_path
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, destination)
        installed_paths[label] = destination
    formal = installed_paths["formal"]
    formal.write_bytes(formal.read_bytes().replace(b"\n", b"\r\n"))
    monkeypatch.setattr(
        cc,
        "_resolve_installed_runtime_paths",
        lambda failures: installed_paths,
    )

    failures = cc.run_runtime_consistency(repo_root, manifest)

    assert any(
        "installed runtime differs from reviewed source" in failure
        for failure in failures
    )


def test_runtime_consistency_does_not_parse_topology(
    runtime_factory, monkeypatch
):
    repo_root, manifest = runtime_factory()

    def forbidden_topology_validation(*_args, **_kwargs):
        raise AssertionError("runtime consistency must not interpret topology")

    monkeypatch.setattr(
        cc,
        "_validate_active_topology",
        forbidden_topology_validation,
        raising=False,
    )

    assert cc.run_runtime_consistency(repo_root, manifest) == []


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

    assert cc._resolve_installed_runtime_paths(failures) == {}
    assert any(
        "cmd_vel_gate.gate_node" in failure
        and "robot_web_ui.web_ui_node" in failure
        for failure in failures
    )


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
