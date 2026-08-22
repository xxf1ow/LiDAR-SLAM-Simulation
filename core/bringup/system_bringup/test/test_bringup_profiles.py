import math
from pathlib import Path

import pytest
import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config" / "bringup.yaml"
PROFILE_DIR = CONFIG.parent / "profiles"


def _load():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def _leaf_paths(value, prefix=()):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from _leaf_paths(child, prefix + (key,))
        return
    yield prefix


def _load_profile(name):
    return yaml.safe_load(
        (PROFILE_DIR / name).read_text(encoding="utf-8")
    )


def test_runtime_selection_uses_supported_platform_and_mode_schema():
    cfg = _load()
    assert cfg["platform"] in {"sim", "real"}
    assert cfg["mode"] in {"mapping", "navigation"}


def test_runtime_selection_defaults_to_real_navigation():
    cfg = _load()
    assert cfg["platform"] == "real"
    assert cfg["mode"] == "navigation"


def test_bringup_has_only_runtime_selection_resources_and_orchestration():
    cfg = _load()
    assert set(cfg) == {
        "platform", "mode", "profiles", "map_artifacts", "robot_gz", "slam_stack"
    }
    assert set(cfg["slam_stack"]) == {"rviz", "settling"}
    assert cfg["slam_stack"]["rviz"] is False
    settling = cfg["slam_stack"]["settling"]
    assert not isinstance(settling, bool)
    assert isinstance(settling, (int, float))
    assert math.isfinite(settling) and settling >= 0.0
    assert set(cfg["map_artifacts"]) == {
        "lio_sam_work_dir", "prior_pcd", "nav2_map"
    }
    assert all(
        isinstance(value, str) and value.strip()
        for value in cfg["map_artifacts"].values()
    )


def test_bringup_selects_relative_sim_and_real_profiles():
    profiles = _load()["profiles"]
    assert set(profiles) == {"sim", "real"}
    resolved = []
    for value in profiles.values():
        path = Path(value)
        assert not path.is_absolute()
        resolved.append((CONFIG.parent / path).resolve())
    assert len(set(resolved)) == len(resolved)
    assert all(path.is_file() for path in resolved)


def test_profiles_are_complete_and_have_identical_leaf_paths():
    sim = _load_profile("sim.yaml")
    real = _load_profile("real.yaml")
    assert set(_leaf_paths(sim)) == set(_leaf_paths(real))


@pytest.mark.parametrize("name", ["sim.yaml", "real.yaml"])
def test_profiles_define_obstacle_geometry_facts(name):
    profile = _load_profile(name)
    vertical_fov = profile["sensors"]["lidar"]["vertical_fov_angle"]
    obstacle_height = profile["perception"]["obstacle_height"]
    clearing_range = profile["perception"]["clearing_range"]
    assert math.isfinite(vertical_fov) and 0.0 < vertical_fov <= math.pi
    assert set(obstacle_height) == {"min", "max"}
    assert all(math.isfinite(value) for value in obstacle_height.values())
    assert obstacle_height["min"] < obstacle_height["max"]
    assert obstacle_height["min"] == profile["robot"]["drive"]["wheel_radius"]
    assert obstacle_height["max"] == profile["robot"]["mounts"]["lidar"]["z"]
    assert clearing_range == {
        "min": 0.9 if name == "sim.yaml" else 0.3,
        "max": 20.0,
    }


@pytest.mark.parametrize("name", ["sim.yaml", "real.yaml"])
def test_profiles_define_complete_positive_motion_limits(name):
    motion = _load_profile(name)["motion"]
    assert set(motion) == {
        "max_linear_velocity",
        "max_angular_velocity",
        "max_linear_acceleration",
        "max_angular_acceleration",
    }
    assert all(
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(value)
        and value > 0.0
        for value in motion.values()
    )


@pytest.mark.parametrize("name", ["sim.yaml", "real.yaml"])
def test_every_profile_scalar_has_an_inline_comment(name):
    path = PROFILE_DIR / name
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or stripped.endswith(":"):
            continue
        assert " #" in line, f"{path.name}:{line_number} 缺少字段注释"


@pytest.mark.parametrize("name", ["sim.yaml", "real.yaml"])
def test_profiles_define_positive_geometry_and_sensor_capabilities(name):
    profile = _load_profile(name)
    for section in (profile["robot"]["body"], profile["robot"]["drive"]):
        assert all(
            not isinstance(value, bool)
            and isinstance(value, (int, float))
            and math.isfinite(value)
            and value > 0.0
            for value in section.values()
        )

    lidar = profile["sensors"]["lidar"]
    assert type(lidar["scan_lines"]) is int and lidar["scan_lines"] > 0
    assert type(lidar["columns_per_scan"]) is int
    assert lidar["columns_per_scan"] > 0
    assert lidar["scan_rate_hz"] > 0.0
    assert 0.0 <= lidar["min_range"] < lidar["max_range"]
    assert profile["sensors"]["imu"]["rate_hz"] > 0.0


def test_sim_and_real_profiles_keep_backend_and_network_schema_separate():
    sim_lidar = _load_profile("sim.yaml")["hardware"]["lidar"]
    real_lidar = _load_profile("real.yaml")["hardware"]["lidar"]

    assert sim_lidar["backend"] == "gazebo"
    assert all(
        sim_lidar[key] is None
        for key in (
            "host_address", "device_address", "host_msop_port",
            "device_msop_port",
        )
    )
    assert real_lidar["backend"] == "vanjee"
    assert all(
        isinstance(real_lidar[key], str) and real_lidar[key]
        for key in ("host_address", "device_address")
    )
    assert all(
        type(real_lidar[key]) is int and 1 <= real_lidar[key] <= 65535
        for key in ("host_msop_port", "device_msop_port")
    )
