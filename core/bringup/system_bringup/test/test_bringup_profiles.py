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


def test_default_platform_and_mode_remain_sim_navigation():
    cfg = _load()
    assert cfg["platform"] == "sim"
    assert cfg["mode"] == "navigation"


def test_sim_profile_preserves_existing_files_and_maps():
    cfg = _load()
    sim = cfg["slam_stack"]["sim"]
    assert cfg["map_artifacts"] == {
        "lio_sam_work_dir": "/result/loam/",
        "prior_pcd": "~/result/GlobalMap.pcd",
        "nav2_map": "~/result/factory_map.yaml",
    }
    assert sim["lio_sam"]["config"] == "params.yaml"
    assert sim["gicp_localization"] == {
        "config": "gicp_localization.yaml",
        "prior_map_path": "~/result/GlobalMap.pcd",
    }
    assert sim["robot_navigation"] == {
        "config": "nav2_params.yaml",
        "map": "~/result/factory_map.yaml",
    }
    for platform in ("sim", "real"):
        stack = cfg["slam_stack"][platform]
        assert stack["gicp_localization"]["prior_map_path"] == cfg[
            "map_artifacts"
        ]["prior_pcd"]
        assert stack["robot_navigation"]["map"] == cfg["map_artifacts"][
            "nav2_map"
        ]


def test_real_profile_selects_only_real_parameter_files():
    real = _load()["slam_stack"]["real"]
    assert real["lio_sam"]["config"] == "params_real.yaml"
    assert real["gicp_localization"]["config"] == "gicp_localization_real.yaml"
    assert real["robot_navigation"]["config"] == "nav2_params_real.yaml"
    assert real["gicp_localization"]["prior_map_path"] == "~/result/GlobalMap.pcd"
    assert real["robot_navigation"]["map"] == "~/result/factory_map.yaml"


def test_fast_lio_is_not_selected_by_platform_config():
    config = _load()
    for platform in ("sim", "real"):
        assert "fast_lio" not in config["slam_stack"][platform]


def test_real_driver_selects_vanjee_722_config():
    assert _load()["vanjee_lidar"]["config"] == "vanjee_722.yaml"


def test_real_geometry_keeps_all_measured_values_in_bringup():
    geometry = _load()["real_geometry"]

    assert geometry == {
        "body": {
            "length": 0.960,
            "width": 0.610,
            "height": 0.377,
            "ground_clearance": 0.143,
        },
        "drive_wheel": {
            "diameter": 0.205,
            "width": 0.101,
            "separation": 0.463,
        },
        "lidar": {
            "x": 0.443,
            "y": 0.0,
            "z": 0.905,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    }


def test_bringup_selects_relative_sim_and_real_profiles():
    assert _load()["profiles"] == {
        "sim": "profiles/sim.yaml",
        "real": "profiles/real.yaml",
    }


def test_profiles_are_complete_and_have_identical_leaf_paths():
    sim = _load_profile("sim.yaml")
    real = _load_profile("real.yaml")
    assert set(_leaf_paths(sim)) == set(_leaf_paths(real))


@pytest.mark.parametrize("name", ["sim.yaml", "real.yaml"])
def test_profiles_define_obstacle_geometry_facts(name):
    profile = _load_profile(name)
    assert profile["sensors"]["lidar"]["vertical_fov_angle"] == 0.523
    assert profile["perception"]["obstacle_height"] == {
        "min": -0.52,
        "max": 2.0,
    }


@pytest.mark.parametrize(
    ("name", "expected"),
    [
        (
            "sim.yaml",
            {
                "max_linear_velocity": 1.0,
                "max_angular_velocity": 1.8,
                "max_linear_acceleration": 1.0,
                "max_angular_acceleration": 1.0,
            },
        ),
        (
            "real.yaml",
            {
                "max_linear_velocity": 1.0,
                "max_angular_velocity": 0.4,
                "max_linear_acceleration": 1.0,
                "max_angular_acceleration": 0.3,
            },
        ),
    ],
)
def test_profiles_define_confirmed_motion_limits(name, expected):
    assert _load_profile(name)["motion"] == expected


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


def test_real_profile_preserves_confirmed_geometry_and_sensor_contract():
    real = _load_profile("real.yaml")
    assert real["robot"]["body"] == {
        "front_extent": 0.480,
        "rear_extent": 0.480,
        "left_extent": 0.305,
        "right_extent": 0.305,
        "height": 0.377,
        "ground_clearance": 0.143,
    }
    assert real["robot"]["drive"] == {
        "wheel_radius": 0.1025,
        "wheel_width": 0.101,
        "wheel_separation": 0.463,
    }
    assert real["sensors"]["lidar"]["scan_lines"] == 32
    assert real["sensors"]["lidar"]["columns_per_scan"] == 1200
    assert real["sensors"]["lidar"]["scan_rate_hz"] == 10.0
    assert real["sensors"]["imu"]["rate_hz"] == 200.0


def test_sim_profile_preserves_current_xacro_and_gazebo_facts():
    sim = _load_profile("sim.yaml")
    assert sim["robot"]["body"] == {
        "front_extent": 0.375,
        "rear_extent": 0.375,
        "left_extent": 0.275,
        "right_extent": 0.275,
        "height": 0.40,
        "ground_clearance": 0.12,
    }
    assert sim["robot"]["drive"] == {
        "wheel_radius": 0.12,
        "wheel_width": 0.06,
        "wheel_separation": 0.55,
    }
    assert sim["robot"]["mounts"]["lidar"]["z"] == 0.556
    assert sim["sensors"]["lidar"]["scan_lines"] == 16
    assert sim["sensors"]["lidar"]["columns_per_scan"] == 1800
    assert sim["sensors"]["lidar"]["min_range"] == 0.9
