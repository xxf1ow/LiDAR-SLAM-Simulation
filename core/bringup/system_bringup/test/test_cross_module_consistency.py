"""跨模块一致性:对真实仓库源文件跑(纯解析,无 ROS,本机 pytest 可跑)。"""
import os

import pytest

from system_bringup import consistency_check as cc


def _root():
    return cc.find_repo_root(__file__)


def test_repo_root_found():
    assert os.path.isdir(os.path.join(_root(), "core", "bringup", "system_bringup"))


def test_xacro_props_reads_literals_skips_expressions():
    props = cc._xacro_props(cc._read(_root(), cc.F_MACRO))
    assert props["wheel_radius"] == 0.12
    assert props["base_width"] == 0.55
    assert props["base_height"] == 0.40
    assert "sensor_z" not in props  # ${...} 表达式被跳过


def test_xacro_joint_origin_colocated():
    macro = cc._read(_root(), cc.F_MACRO)
    assert cc._xacro_joint_origin_xyz(macro, "velodyne_joint") == \
        cc._xacro_joint_origin_xyz(macro, "imu_joint")


def test_fastlio_sim_patch_yaml_reconstructed_by_path():
    patch = cc._read(_root(), cc.F_FASTLIO_PATCH)
    params = cc._yaml(
        cc._patch_added_file(patch, "config/gazebo_velodyne.yaml")
    )["/**"]["ros__parameters"]
    assert params["preprocess"] == {
        "lidar_type": 2,
        "scan_line": 16,
        "scan_rate": 10,
        "timestamp_unit": 2,
        "blind": 1.0,
    }


def test_liosam_sim_patch_values_are_scoped_by_path():
    patch = cc._read(_root(), cc.F_LIOSAM_PATCH)
    assert cc._patch_added_value(
        patch, "config/params.yaml", "N_SCAN"
    ) == "16"
    assert cc._patch_added_value(
        patch, "config/params.yaml", "Horizon_SCAN"
    ) == "1800"
    assert cc._patch_added_value(
        patch, "config/params.yaml", "lidarFrame"
    ) == "velodyne"


def test_gazebo_lidar_block():
    gz = cc._gazebo_lidar(cc._read(_root(), cc.F_GAZEBO))
    assert gz["v_samples"] == 16
    assert gz["h_samples"] == 1800
    assert gz["update_rate"] == 10
    assert gz["range_min"] == 0.9


def test_adapter_scan_period():
    assert cc._adapter_scan_period(cc._read(_root(), cc.F_GZ_LAUNCH)) == 0.1


def test_geometry_consistent():
    fails = cc.check_geometry(_root())
    assert fails == [], "几何不一致:\n" + "\n".join(fails)


def test_lidar_consistent():
    fails = cc.check_lidar(_root())
    assert fails == [], "雷达不一致:\n" + "\n".join(fails)


def test_sim_lidar_consistent_without_reading_real_profile():
    assert cc.check_lidar(_root(), "sim") == []


def test_real_lidar_consistent_without_reading_gazebo_geometry():
    assert cc.check_lidar(_root(), "real") == []


def test_sim_geometry_consistent():
    assert cc.check_geometry(_root(), "sim") == []


def test_real_geometry_consistent():
    assert cc.check_geometry(_root(), "real") == []


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_run_checks_only_selected_platform(monkeypatch, platform):
    calls = []
    monkeypatch.setattr(
        cc,
        "load_bringup_config",
        lambda repo_root: {"platform": platform},
    )
    monkeypatch.setattr(
        cc,
        "check_geometry",
        lambda repo_root, selected: calls.append(("geometry", selected)) or [],
    )
    monkeypatch.setattr(
        cc,
        "check_lidar",
        lambda repo_root, selected: calls.append(("lidar", selected)) or [],
    )
    assert cc.run(_root()) == []
    assert calls == [("geometry", platform), ("lidar", platform)]


def test_run_all_consistent():
    assert cc.run(_root()) == []
