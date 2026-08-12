"""跨模块一致性:对真实仓库源文件跑(纯解析,无 ROS,本机 pytest 可跑)。"""
import copy
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


def test_runtime_config_file_must_exist():
    existing = Path(__file__)

    assert cc.require_runtime_config_file(existing, "FAST-LIO") == str(existing)

    missing = existing.with_name("missing.yaml")
    with pytest.raises(RuntimeError, match="FAST-LIO.*missing.yaml.*apply.*rebuild"):
        cc.require_runtime_config_file(missing, "FAST-LIO")


def test_xacro_args_keep_existing_sim_geometry_defaults():
    defaults = cc._xacro_args(cc._read(_root(), cc.F_ROBOT_XACRO))
    assert defaults["wheel_radius"] == 0.12
    assert defaults["wheel_separation"] == 0.55
    assert defaults["base_width"] == 0.55
    assert defaults["base_height"] == 0.40
    assert defaults["lidar_z"] == 0.236
    assert defaults["imu_z"] == 0.236


def test_xacro_joint_origins_consume_independent_mount_arguments():
    macro = cc._read(_root(), cc.F_MACRO)
    assert cc._xacro_joint_origin_xyz(macro, "velodyne_joint") == \
        "${lidar_x} ${lidar_y} ${lidar_z}"
    assert cc._xacro_joint_origin_xyz(macro, "imu_joint") == \
        "${imu_x} ${imu_y} ${imu_z}"


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
    assert hunks == ["@@ -925,4 +925,4 @@"]
    assert "(imu_topic, 10, imu_cbk);" in section
    assert "(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);" in section


def test_fast_lio_patch_passes_git_apply_check_against_pinned_context(tmp_path):
    source = tmp_path / "src/laserMapping.cpp"
    source.parent.mkdir(parents=True)
    source.write_text(
        "// pinned-equivalent filler\n" * 924
        + "        {\n"
        + "            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);\n"
        + "        }\n"
        + "        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            "git",
            "apply",
            "--check",
            "--no-index",
            str(Path(_root()) / cc.F_FASTLIO_PATCH),
        ],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
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


def test_gazebo_lidar_block():
    gz = cc._gazebo_lidar(
        cc._read(_root(), cc.F_GAZEBO),
        cc._xacro_args(cc._read(_root(), cc.F_ROBOT_XACRO)),
    )
    assert gz["v_samples"] == 16
    assert gz["h_samples"] == 1800
    assert gz["update_rate"] == 10
    assert gz["range_min"] == 0.9


def test_adapter_scan_period(sim_adapter_scan_period):
    assert sim_adapter_scan_period == 0.1


def test_legacy_checker_does_not_parse_generated_adapter_config_from_launch():
    assert not hasattr(cc, "_adapter_scan_period")


def test_legacy_source_checks_do_not_reconstruct_fast_lio_patch_yaml(monkeypatch):
    seen = []
    original = cc._patch_added_file

    def counted(text, relative_path):
        seen.append(relative_path)
        return original(text, relative_path)

    monkeypatch.setattr(cc, "_patch_added_file", counted)
    assert cc.check_geometry(_root(), "sim") == []
    assert cc.check_lidar(_root(), "sim") == []
    assert cc.check_geometry(_root(), "real") == []
    assert cc.check_lidar(_root(), "real") == []
    assert not any("fast" in path.lower() for path in seen)


def test_retired_legacy_source_interfaces_are_absent():
    assert not hasattr(cc, "FASTLIO_CONFIG")
    assert not hasattr(cc, "F_NAV_LAUNCH")
    assert not hasattr(cc, "_launch_floats")


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
    added_files = _guarded_added_file(monkeypatch, {cc.LIOSAM_CONFIG["real"]})

    assert cc.check_lidar(_root(), "sim") == []
    assert set(reads) == {
        cc.F_LIOSAM_PATCH,
        cc.F_GAZEBO,
        cc.F_ROBOT_XACRO,
    }
    assert added_files == []


def test_real_lidar_consistent_without_reading_gazebo_geometry():
    assert cc.check_lidar(_root(), "real") == []


def test_real_lidar_reads_only_real_sources(monkeypatch):
    reads = _guarded_read(monkeypatch, {cc.F_GAZEBO, cc.F_GZ_LAUNCH, cc.F_NAV_PARAMS})
    added_files = _guarded_added_file(
        monkeypatch,
        {cc.LIOSAM_CONFIG["sim"]},
    )

    assert cc.check_lidar(_root(), "real") == []
    assert set(reads) == {cc.F_LIOSAM_PATCH, cc.F_VANJEE_PARAMS}
    assert added_files == [cc.LIOSAM_CONFIG["real"]]


def test_sim_geometry_consistent():
    assert cc.check_geometry(_root(), "sim") == []


def test_sim_geometry_reads_only_sim_sources(monkeypatch):
    reads = _guarded_read(monkeypatch, {cc.F_NAV_PARAMS_REAL, cc.F_VANJEE_PARAMS})
    added_files = _guarded_added_file(monkeypatch, set())

    assert cc.check_geometry(_root(), "sim") == []
    assert set(reads) == {
        cc.F_ROBOT_XACRO,
        cc.F_NAV_PARAMS,
        cc.F_CONTROLLERS,
    }
    assert added_files == []


def test_real_geometry_consistent():
    assert cc.check_geometry(_root(), "real") == []


def test_real_geometry_derives_runtime_values_from_bringup_only():
    values = cc.derive_real_geometry(cc.load_bringup_config(_root()))

    assert values["body"] == {
        "length": 0.960,
        "width": 0.610,
        "height": 0.377,
        "base_link_height": 0.3315,
    }
    assert values["drive_wheel"] == {
        "radius": 0.1025,
        "width": 0.101,
        "separation": 0.463,
    }
    assert values["sensor"] == {
        "x": 0.443,
        "y": 0.0,
        "z": 0.5735,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    assert values["body_to_base_footprint"] == {
        "x": -0.443,
        "y": 0.0,
        "z": -0.905,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }
    assert values["footprint"] == [
        [0.480, 0.305],
        [0.480, -0.305],
        [-0.480, -0.305],
        [-0.480, 0.305],
    ]


def test_real_geometry_rejects_nonphysical_wheel_dimensions():
    config = copy.deepcopy(cc.load_bringup_config(_root()))
    config["real_geometry"]["drive_wheel"]["diameter"] = -0.205

    with pytest.raises(ValueError, match="drive_wheel.diameter"):
        cc.derive_real_geometry(config)


def test_real_geometry_rejects_wheels_outside_body_width():
    config = copy.deepcopy(cc.load_bringup_config(_root()))
    config["real_geometry"]["drive_wheel"]["separation"] = 0.60

    with pytest.raises(ValueError, match="轮子外缘宽度"):
        cc.derive_real_geometry(config)


@pytest.mark.parametrize(
    "path,value,expected",
    [
        (("body", "length"), 0.0, "body.length"),
        (("body", "width"), -0.610, "body.width"),
        (("body", "height"), 0.0, "body.height"),
        (("body", "ground_clearance"), -0.001, "body.ground_clearance"),
        (("drive_wheel", "width"), 0.0, "drive_wheel.width"),
        (("drive_wheel", "separation"), 0.0, "drive_wheel.separation"),
        (("lidar", "z"), 0.0, "lidar.z"),
        (("body", "length"), float("nan"), "有限数"),
        (("lidar", "x"), float("inf"), "有限数"),
    ],
)
def test_real_geometry_rejects_invalid_measurements(path, value, expected):
    config = copy.deepcopy(cc.load_bringup_config(_root()))
    config["real_geometry"][path[0]][path[1]] = value

    with pytest.raises(ValueError, match=expected):
        cc.derive_real_geometry(config)


def test_real_runtime_configs_are_generated_from_measured_geometry():
    runtime = cc.build_real_runtime_configs(
        _root(), cc.load_bringup_config(_root())
    )

    controller = runtime["controllers"]["base_controller"]["ros__parameters"]
    assert controller["wheel_radius"] == 0.1025
    assert controller["wheel_separation"] == 0.463

    footprint = "[ [0.480, 0.305], [0.480, -0.305], [-0.480, -0.305], [-0.480, 0.305] ]"
    nav = runtime["nav2"]
    assert nav["global_costmap"]["global_costmap"]["ros__parameters"]["footprint"] == footprint
    assert nav["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"] == footprint

    # 非几何真机调参仍来自原模板，不被生成过程重写。
    assert nav["local_costmap"]["local_costmap"]["ros__parameters"]["width"] == 6
    assert controller["publish_rate"] == 50.0


def test_real_launch_arguments_are_derived_without_repeating_measurements():
    geometry = cc.derive_real_geometry(cc.load_bringup_config(_root()))
    arguments = cc.real_geometry_launch_arguments(geometry)

    assert arguments["robot"] == {
        "base_length": "0.96",
        "base_width": "0.61",
        "base_height": "0.377",
        "base_link_height": "0.3315",
        "wheel_radius": "0.1025",
        "wheel_width": "0.101",
        "wheel_separation": "0.463",
        "sensor_x": "0.443",
        "sensor_y": "0.0",
        "sensor_z": "0.5735",
        "sensor_roll": "0.0",
        "sensor_pitch": "0.0",
        "sensor_yaw": "0.0",
    }
    assert arguments["navigation"] == {
        "weld_x": "-0.443",
        "weld_y": "0.0",
        "weld_z": "-0.905",
        "weld_roll": "0.0",
        "weld_pitch": "0.0",
        "weld_yaw": "0.0",
    }


def test_real_runtime_configs_are_written_outside_the_repository(tmp_path):
    paths = cc.write_real_runtime_configs(
        _root(), cc.load_bringup_config(_root()), tmp_path
    )

    assert set(paths) == {"controllers", "nav2"}
    assert all(path.parent == tmp_path for path in paths.values())
    assert cc._yaml(paths["controllers"].read_text(encoding="utf-8"))[
        "base_controller"
    ]["ros__parameters"]["wheel_radius"] == 0.1025
    assert cc._yaml(paths["nav2"].read_text(encoding="utf-8"))[
        "global_costmap"
    ]["global_costmap"]["ros__parameters"]["footprint"].startswith(
        "[ [0.480, 0.305]"
    )


def test_default_runtime_output_uses_a_private_unique_directory(
    tmp_path, monkeypatch
):
    calls = []

    def fake_mkdtemp(prefix):
        calls.append(prefix)
        private = tmp_path / "system_bringup-private"
        private.mkdir()
        return str(private)

    monkeypatch.setattr(cc.tempfile, "mkdtemp", fake_mkdtemp)

    paths = cc.write_real_runtime_configs(
        _root(), cc.load_bringup_config(_root())
    )

    assert calls == ["system_bringup-"]
    assert {path.parent for path in paths.values()} == {
        tmp_path / "system_bringup-private"
    }


def test_real_geometry_reads_only_real_sources(monkeypatch):
    reads = _guarded_read(monkeypatch, {cc.F_GAZEBO, cc.F_GZ_LAUNCH, cc.F_NAV_PARAMS})
    added_files = _guarded_added_file(monkeypatch, set())

    assert cc.check_geometry(_root(), "real") == []
    assert set(reads) == {
        cc.F_NAV_PARAMS_REAL,
        cc.F_CONTROLLERS,
    }
    assert added_files == []


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
    assert added_files == [cc.LIOSAM_CONFIG["real"]]


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
    assert added_files == []


def test_real_lidar_accepts_quoted_liosam_numeric_values(monkeypatch):
    original_yaml = cc._yaml

    def quoted_numeric_yaml(text):
        value = original_yaml(text)
        if "/**" in value:
            params = value["/**"]["ros__parameters"]
            params["N_SCAN"] = "32"
            params["Horizon_SCAN"] = "1200"
        return value

    monkeypatch.setattr(cc, "_yaml", quoted_numeric_yaml)

    assert cc.check_lidar(_root(), "real") == []


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
