"""跨模块一致性:对真实仓库源文件跑(纯解析,无 ROS,本机 pytest 可跑)。"""
import os

import pytest

from system_bringup import consistency_check as cc


def _root():
    return cc.find_repo_root(__file__)


def _guarded_read(monkeypatch, forbidden):
    original_read = cc._read
    seen = []

    def guarded_read(repo_root, relpath):
        seen.append(relpath)
        if relpath in forbidden:
            raise AssertionError("不应读取跨 platform 源: %s" % relpath)
        return original_read(repo_root, relpath)

    monkeypatch.setattr(cc, "_read", guarded_read)
    return seen


def _guarded_added_file(monkeypatch, forbidden):
    original_added_file = cc._patch_added_file
    seen = []

    def guarded_added_file(text, relative_path):
        seen.append(relative_path)
        if relative_path in forbidden:
            raise AssertionError("不应读取跨 platform patch 目标: %s" % relative_path)
        return original_added_file(text, relative_path)

    monkeypatch.setattr(cc, "_patch_added_file", guarded_added_file)
    return seen


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


def test_fastlio_patch_uses_sensor_data_qos_for_imu_subscription():
    patch = cc._read(_root(), cc.F_FASTLIO_PATCH)
    section = cc._patch_file_section(patch, "src/laserMapping.cpp")
    assert (
        "-        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>"
        "(imu_topic, 10, imu_cbk);"
    ) in section
    assert (
        "+        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>"
        "(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);"
    ) in section


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


def test_patch_added_file_rejects_modified_sim_liosam_params():
    with pytest.raises(ValueError, match="不是新增文件"):
        cc._patch_added_file(
            cc._read(_root(), cc.F_LIOSAM_PATCH),
            cc.LIOSAM_CONFIG["sim"],
        )


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


def test_sim_lidar_reads_only_sim_sources(monkeypatch):
    reads = _guarded_read(monkeypatch, {cc.F_NAV_PARAMS_REAL, cc.F_VANJEE_PARAMS})
    added_files = _guarded_added_file(monkeypatch, {cc.FASTLIO_CONFIG["real"]})

    assert cc.check_lidar(_root(), "sim") == []
    assert set(reads) == {cc.F_FASTLIO_PATCH, cc.F_LIOSAM_PATCH, cc.F_GAZEBO, cc.F_GZ_LAUNCH}
    assert added_files == [cc.FASTLIO_CONFIG["sim"]]


def test_real_lidar_consistent_without_reading_gazebo_geometry():
    assert cc.check_lidar(_root(), "real") == []


def test_real_lidar_reads_only_real_sources(monkeypatch):
    reads = _guarded_read(monkeypatch, {cc.F_GAZEBO, cc.F_GZ_LAUNCH, cc.F_NAV_PARAMS})
    added_files = _guarded_added_file(
        monkeypatch,
        {cc.FASTLIO_CONFIG["sim"], cc.LIOSAM_CONFIG["sim"]},
    )

    assert cc.check_lidar(_root(), "real") == []
    assert set(reads) == {cc.F_FASTLIO_PATCH, cc.F_LIOSAM_PATCH, cc.F_VANJEE_PARAMS}
    assert added_files == [cc.FASTLIO_CONFIG["real"], cc.LIOSAM_CONFIG["real"]]


def test_sim_geometry_consistent():
    assert cc.check_geometry(_root(), "sim") == []


def test_sim_geometry_reads_only_sim_sources(monkeypatch):
    reads = _guarded_read(monkeypatch, {cc.F_NAV_PARAMS_REAL, cc.F_VANJEE_PARAMS})
    added_files = _guarded_added_file(monkeypatch, {cc.FASTLIO_CONFIG["real"]})

    assert cc.check_geometry(_root(), "sim") == []
    assert set(reads) == {
        cc.F_MACRO,
        cc.F_NAV_PARAMS,
        cc.F_CONTROLLERS,
        cc.F_NAV_LAUNCH,
        cc.F_FASTLIO_PATCH,
    }
    assert added_files == [cc.FASTLIO_CONFIG["sim"]]


def test_real_geometry_consistent():
    assert cc.check_geometry(_root(), "real") == []


def test_real_geometry_reads_only_real_sources(monkeypatch):
    reads = _guarded_read(monkeypatch, {cc.F_GAZEBO, cc.F_GZ_LAUNCH, cc.F_NAV_PARAMS})
    added_files = _guarded_added_file(monkeypatch, {cc.FASTLIO_CONFIG["sim"]})

    assert cc.check_geometry(_root(), "real") == []
    assert set(reads) == {
        cc.F_MACRO,
        cc.F_NAV_PARAMS_REAL,
        cc.F_CONTROLLERS,
        cc.F_NAV_LAUNCH,
        cc.F_FASTLIO_PATCH,
    }
    assert added_files == [cc.FASTLIO_CONFIG["real"]]


@pytest.mark.parametrize("checker", [cc.check_geometry, cc.check_lidar])
def test_unknown_platform_fails_explicitly(checker):
    assert checker(_root(), "unsupported") == ["未知 platform='unsupported'(应为 sim|real)。"]


def test_real_lidar_uses_added_files_not_added_values(monkeypatch):
    added_files = _guarded_added_file(monkeypatch, set())
    monkeypatch.setattr(
        cc,
        "_patch_added_value",
        lambda *args: pytest.fail("real LIO-SAM 参数必须从新增文件重建"),
    )

    assert cc.check_lidar(_root(), "real") == []
    assert added_files == [cc.FASTLIO_CONFIG["real"], cc.LIOSAM_CONFIG["real"]]


def test_sim_lidar_uses_added_values_for_modified_liosam_params(monkeypatch):
    original_added_value = cc._patch_added_value
    added_values = []

    def guarded_added_value(text, relative_path, key):
        added_values.append((relative_path, key))
        return original_added_value(text, relative_path, key)

    monkeypatch.setattr(cc, "_patch_added_value", guarded_added_value)
    added_files = _guarded_added_file(monkeypatch, {cc.LIOSAM_CONFIG["sim"]})

    assert cc.check_lidar(_root(), "sim") == []
    assert added_values == [
        (cc.LIOSAM_CONFIG["sim"], "N_SCAN"),
        (cc.LIOSAM_CONFIG["sim"], "Horizon_SCAN"),
    ]
    assert added_files == [cc.FASTLIO_CONFIG["sim"]]


def test_real_lidar_accepts_quoted_numeric_yaml_values(monkeypatch):
    original_yaml = cc._yaml

    def quoted_numeric_yaml(text):
        value = original_yaml(text)
        if "/**" in value:
            params = value["/**"]["ros__parameters"]
            if "preprocess" in params:
                params["preprocess"].update({
                    "lidar_type": "2",
                    "scan_line": "32",
                    "scan_rate": "10",
                    "timestamp_unit": "0",
                    "blind": "0.3",
                })
            else:
                params["N_SCAN"] = "32"
                params["Horizon_SCAN"] = "1200"
        else:
            value["vanjee_lidar"]["ros__parameters"]["min_distance"] = "0.05"
        return value

    monkeypatch.setattr(cc, "_yaml", quoted_numeric_yaml)

    assert cc.check_lidar(_root(), "real") == []


def test_real_lidar_reports_quoted_invalid_scan_rate_without_type_error(monkeypatch):
    original_yaml = cc._yaml

    def quoted_invalid_rate_yaml(text):
        value = original_yaml(text)
        if "/**" in value and "preprocess" in value["/**"]["ros__parameters"]:
            value["/**"]["ros__parameters"]["preprocess"]["scan_rate"] = "11"
        return value

    monkeypatch.setattr(cc, "_yaml", quoted_invalid_rate_yaml)

    assert cc.check_lidar(_root(), "real") == ["[R3] fast-lio scan_rate=11(应为 10)。"]


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
