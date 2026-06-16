"""跨模块一致性:对真实仓库源文件跑(纯解析,无 ROS,本机 pytest 可跑)。"""
import os

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


def test_patch_added_value_reads_plus_lines():
    liosam = cc._read(_root(), cc.F_LIOSAM_PATCH)
    assert cc._patch_added_value(liosam, "N_SCAN") == "16"
    assert cc._patch_added_value(liosam, "Horizon_SCAN") == "1800"
    assert cc._patch_added_value(liosam, "lidarFrame") == "velodyne"


def test_fastlio_patch_yaml_reconstructed():
    pre = cc._yaml(cc._patch_added_text(cc._read(_root(), cc.F_FASTLIO_PATCH)))["/**"]["ros__parameters"]["preprocess"]
    assert pre["scan_line"] == 16
    assert pre["scan_rate"] == 10
    assert pre["blind"] == 1.0


def test_gazebo_lidar_block():
    gz = cc._gazebo_lidar(cc._read(_root(), cc.F_GAZEBO))
    assert gz["v_samples"] == 16
    assert gz["h_samples"] == 1800
    assert gz["update_rate"] == 10
    assert gz["range_min"] == 0.9


def test_adapter_scan_period():
    assert cc._adapter_scan_period(cc._read(_root(), cc.F_GZ_LAUNCH)) == 0.1
