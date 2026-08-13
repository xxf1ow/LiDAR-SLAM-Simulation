"""Cross-module source contracts that do not belong to runtime validation."""
import os
from pathlib import Path
import subprocess

import pytest
import yaml

from system_bringup import consistency_check as cc
from system_bringup import runtime_config_compiler as rcc


ROOT = Path(__file__).resolve().parents[4]
FAST_LIO_PATCH = ROOT / "core/localization/fast-lio2.patch"


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _patch_file_section(text, relative_path):
    marker = f"diff --git a/{relative_path} b/{relative_path}"
    start = text.find(marker)
    if start < 0:
        raise ValueError(f"patch missing file: {relative_path}")
    return text[start:].split("\ndiff --git ", 1)[0]


@pytest.fixture
def sim_adapter_scan_period(tmp_path):
    config = ROOT / "core/bringup/system_bringup/config/bringup.yaml"
    manifest = rcc.compile_runtime_configs(config, tmp_path / "runtime")
    adapter = _load_yaml(Path(manifest["lidar_adapter_path"]))
    return adapter["lidar_pointcloud_adapter"]["ros__parameters"]["scan_period"]


def test_repository_fixture_resolves_from_test_location():
    assert (ROOT / "core/bringup/system_bringup").is_dir()


def test_fast_lio_patch_contains_only_the_imu_qos_source_change():
    patch = FAST_LIO_PATCH.read_text(encoding="utf-8")
    headers = [
        line for line in patch.splitlines() if line.startswith("diff --git ")
    ]
    assert headers == [
        "diff --git a/src/laserMapping.cpp b/src/laserMapping.cpp"
    ]
    assert "config/gazebo_velodyne.yaml" not in patch
    assert "config/vanjee_722.yaml" not in patch
    section = _patch_file_section(patch, "src/laserMapping.cpp")
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
            str(FAST_LIO_PATCH),
        ],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ, "GIT_CEILING_DIRECTORIES": str(tmp_path.parent)},
    )

    assert result.returncode == 0, result.stderr


def test_adapter_scan_period(sim_adapter_scan_period):
    assert sim_adapter_scan_period == 0.1


def test_legacy_checker_does_not_parse_generated_adapter_config_from_launch():
    assert not hasattr(cc, "_adapter_scan_period")


def test_retired_legacy_source_interfaces_are_absent():
    assert not hasattr(cc, "FASTLIO_CONFIG")
    assert not hasattr(cc, "F_NAV_LAUNCH")
    assert not hasattr(cc, "_launch_floats")
