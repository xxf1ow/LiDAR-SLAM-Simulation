"""跨模块一致性:对真实仓库源文件跑(纯解析,无 ROS,本机 pytest 可跑)。"""
import os
from pathlib import Path
import subprocess

import pytest

from system_bringup import consistency_check as cc
from system_bringup import runtime_config_compiler as rcc


def _root():
    return cc.find_repo_root(__file__)


@pytest.fixture
def sim_adapter_scan_period(tmp_path):
    config = Path(_root()) / "core/bringup/system_bringup/config/bringup.yaml"
    manifest = rcc.compile_runtime_configs(config, tmp_path / "runtime")
    adapter = cc._yaml(
        Path(manifest["lidar_adapter_path"]).read_text(encoding="utf-8")
    )
    return adapter["lidar_pointcloud_adapter"]["ros__parameters"]["scan_period"]


def test_repo_root_found():
    assert os.path.isdir(os.path.join(_root(), "core", "bringup", "system_bringup"))


def test_fast_lio_patch_contains_only_the_imu_qos_source_change():
    patch = cc._read(_root(), cc.F_FASTLIO_PATCH)
    headers = [
        line for line in patch.splitlines() if line.startswith("diff --git ")
    ]
    assert headers == [
        "diff --git a/src/laserMapping.cpp b/src/laserMapping.cpp"
    ]
    assert "config/gazebo_velodyne.yaml" not in patch
    assert "config/vanjee_722.yaml" not in patch
    section = cc._patch_file_section(patch, "src/laserMapping.cpp")
    hunks = [line for line in section.splitlines() if line.startswith("@@ ")]
    assert hunks == ["@@ -926,7 +926,7 @@ public:"]
    removed = [
        line
        for line in section.splitlines()
        if line.startswith("-") and not line.startswith("---")
    ]
    added = [
        line
        for line in section.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    ]
    assert removed == [
        "-        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>"
        "(imu_topic, 10, imu_cbk);"
    ]
    assert added == [
        "+        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>"
        "(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);"
    ]


def test_fast_lio_patch_passes_git_apply_check_against_pinned_context(tmp_path):
    source = tmp_path / "src/laserMapping.cpp"
    source.parent.mkdir(parents=True)
    source.write_text(
        "// pinned-equivalent filler\n" * 925
        + "        {\n"
        + "            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);\n"
        + "        }\n"
        + "        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);\n"
        + "        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(\"/cloud_registered\", 20);\n"
        + "        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(\"/cloud_registered_body\", 20);\n"
        + "        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(\"/cloud_effected\", 20);\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            "git",
            "apply",
            "--check",
            "--no-index",
            "--verbose",
            str(Path(_root()) / cc.F_FASTLIO_PATCH),
        ],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ, "GIT_CEILING_DIRECTORIES": str(tmp_path.parent)},
    )

    assert result.returncode == 0, result.stderr


def test_sim_imu_topic_is_atomically_fixed_across_active_sources():
    root = _root()
    liosam_patch = cc._read(root, cc.F_LIOSAM_PATCH)
    assert cc._patch_added_value(
        liosam_patch, "config/params.yaml", "imuTopic"
    ) == "/imu/data"

    bridge = cc._yaml(
        cc._read(root, "core/simulation/robot_gz_bringup/config/bridge.yaml")
    )
    imu_bridge = next(item for item in bridge if item["gz_topic_name"] == "/imu")
    assert imu_bridge["ros_topic_name"] == "/imu/data"


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


def test_adapter_scan_period(sim_adapter_scan_period):
    assert sim_adapter_scan_period == 0.1


def test_legacy_checker_does_not_parse_generated_adapter_config_from_launch():
    assert not hasattr(cc, "_adapter_scan_period")


def test_retired_legacy_source_interfaces_are_absent():
    assert not hasattr(cc, "FASTLIO_CONFIG")
    assert not hasattr(cc, "F_NAV_LAUNCH")
    assert not hasattr(cc, "_launch_floats")
