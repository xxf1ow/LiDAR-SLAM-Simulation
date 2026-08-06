from pathlib import Path

import yaml

from system_bringup import consistency_check as cc


ROOT = Path(__file__).resolve().parents[4]


def _yaml_file(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _patch_yaml(patch_path, relative_path):
    text = patch_path.read_text(encoding="utf-8")
    return yaml.safe_load(cc._patch_added_file(text, relative_path))


def test_real_fast_lio_matches_vanjee_contract():
    fast = _patch_yaml(
        ROOT / "core/localization/fast-lio2.patch",
        "config/vanjee_722.yaml",
    )["/**"]["ros__parameters"]
    assert fast["common"] == {
        "lid_topic": "/points_raw",
        "imu_topic": "/imu/data",
        "time_sync_en": False,
        "time_offset_lidar_to_imu": 0.0,
    }
    assert fast["preprocess"] == {
        "lidar_type": 2,
        "scan_line": 32,
        "scan_rate": 10,
        "timestamp_unit": 0,
        "blind": 0.3,
    }
    assert fast["mapping"]["extrinsic_T"] == [0.0, 0.0, 0.0]
    assert fast["mapping"]["extrinsic_R"] == [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]
    assert fast["mapping"]["extrinsic_est_en"] is True


def test_real_gicp_only_switches_clock_domain():
    sim = _yaml_file(
        ROOT / "core/localization/gicp_localization/config/gicp_localization.yaml"
    )
    real = _yaml_file(
        ROOT / "core/localization/gicp_localization/config/gicp_localization_real.yaml"
    )
    sim_params = sim["gicp_localization"]["ros__parameters"]
    real_params = real["gicp_localization"]["ros__parameters"]
    assert sim_params["use_sim_time"] is True
    assert real_params["use_sim_time"] is False
    assert {k: v for k, v in real_params.items() if k != "use_sim_time"} == {
        k: v for k, v in sim_params.items() if k != "use_sim_time"
    }


def test_real_lio_sam_and_driver_contract_remain_aligned():
    driver = _yaml_file(
        ROOT / "core/robot/drivers/lidar_vanjee_722/"
        "vanjee_lidar_ros/config/vanjee_722.yaml"
    )["vanjee_lidar"]["ros__parameters"]
    lio = _patch_yaml(
        ROOT / "core/mapping/lio-sam.patch",
        "config/params_real.yaml",
    )["/**"]["ros__parameters"]
    assert driver["point_cloud_topic"] == lio["pointCloudTopic"] == "/points_raw"
    assert driver["imu_topic"] == lio["imuTopic"] == "/imu/data"
    assert driver["lidar_frame"] == lio["lidarFrame"] == "velodyne"
    assert driver["imu_frame"] == "imu_link"
    assert lio["use_sim_time"] is False
    assert (lio["N_SCAN"], lio["Horizon_SCAN"]) == (32, 1200)
