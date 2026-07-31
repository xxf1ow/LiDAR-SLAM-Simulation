from pathlib import Path

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "config" / "bringup.yaml"


def _load():
    return yaml.safe_load(CONFIG.read_text(encoding="utf-8"))


def test_default_platform_and_mode_remain_sim_navigation():
    cfg = _load()
    assert cfg["platform"] == "sim"
    assert cfg["mode"] == "navigation"


def test_sim_profile_preserves_existing_files_and_maps():
    sim = _load()["slam_stack"]["sim"]
    assert sim["lio_sam"]["config"] == "params.yaml"
    assert sim["fast_lio"]["config"] == "gazebo_velodyne.yaml"
    assert sim["gicp_localization"] == {
        "config": "gicp_localization.yaml",
        "prior_map_path": "~/result/GlobalMap.pcd",
    }
    assert sim["robot_navigation"] == {
        "config": "nav2_params.yaml",
        "map": "~/result/factory_map.yaml",
    }


def test_real_profile_selects_only_real_parameter_files():
    real = _load()["slam_stack"]["real"]
    assert real["lio_sam"]["config"] == "params_real.yaml"
    assert real["fast_lio"]["config"] == "vanjee_722.yaml"
    assert real["gicp_localization"]["config"] == "gicp_localization_real.yaml"
    assert real["robot_navigation"]["config"] == "nav2_params_real.yaml"
    assert real["gicp_localization"]["prior_map_path"] == "~/result/GlobalMap.pcd"
    assert real["robot_navigation"]["map"] == "~/result/factory_map.yaml"


def test_real_driver_selects_vanjee_722_config():
    assert _load()["vanjee_lidar"]["config"] == "vanjee_722.yaml"
