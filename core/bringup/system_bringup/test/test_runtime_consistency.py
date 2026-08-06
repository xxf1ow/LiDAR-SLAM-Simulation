import shutil
import tempfile
from pathlib import Path

import pytest
import yaml

from system_bringup import consistency_check as cc
from system_bringup import runtime_config_compiler as rcc


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"


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


@pytest.fixture
def runtime_factory(tmp_path):
    runtime_dirs = []
    count = 0

    def build(platform="sim", mode="navigation"):
        nonlocal count
        count += 1
        repo_root = tmp_path / f"repo-{count}"
        config_dir = repo_root / "core" / "bringup" / "system_bringup" / "config"
        shutil.copytree(CONFIG_DIR, config_dir)
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
