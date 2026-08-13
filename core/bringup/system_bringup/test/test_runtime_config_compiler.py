import shutil
from copy import deepcopy
import json
import math
from math import degrees
from pathlib import Path
import subprocess
import sys
import tempfile

import pytest
import yaml

from system_bringup import profile_compiler as pc
from system_bringup import runtime_config_compiler as rcc


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = PACKAGE_ROOT / "config"
TEMPLATE_DIR = CONFIG_DIR / "templates"
SENSOR_TEMPLATE_NAMES = {
    "lidar_adapter": "lidar_adapter.yaml",
    "vanjee_lidar": "vanjee_lidar.yaml",
    "sensor_gate": "sensor_gate.yaml",
}
SIM_FOOTPRINT = [
    [0.375, 0.275],
    [0.375, -0.275],
    [-0.375, -0.275],
    [-0.375, 0.275],
]
REAL_FOOTPRINT = [
    [0.48, 0.305],
    [0.48, -0.305],
    [-0.48, -0.305],
    [-0.48, 0.305],
]
ROBOT_LAUNCH_ARGUMENT_KEYS = {
    "base_length",
    "base_width",
    "base_height",
    "base_link_height",
    "wheel_radius",
    "wheel_width",
    "wheel_separation",
    "lidar_x",
    "lidar_y",
    "lidar_z",
    "lidar_roll",
    "lidar_pitch",
    "lidar_yaw",
    "imu_x",
    "imu_y",
    "imu_z",
    "imu_roll",
    "imu_pitch",
    "imu_yaw",
    "lidar_scan_lines",
    "lidar_columns_per_scan",
    "lidar_scan_rate_hz",
    "lidar_min_range",
    "lidar_max_range",
    "lidar_horizontal_start_angle",
    "lidar_horizontal_end_angle",
    "imu_rate_hz",
}


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _set_config_path(runtime_tree, path, value):
    config = _load_yaml(runtime_tree.config)
    target = config
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    runtime_tree.config.write_text(
        yaml.safe_dump(config, sort_keys=False), encoding="utf-8"
    )


class _RuntimeTree:
    def __init__(self, config):
        self.config = config

    def set_bringup_value(self, key, value):
        config = _load_yaml(self.config)
        config[key] = value
        self.config.write_text(
            yaml.safe_dump(config, sort_keys=False), encoding="utf-8"
        )

    def set_profile_value(self, platform, path, value):
        profile_path = self.config.parent / "profiles" / f"{platform}.yaml"
        profile = _load_yaml(profile_path)
        target = profile
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value
        profile_path.write_text(
            yaml.safe_dump(profile, sort_keys=False), encoding="utf-8"
        )

    def mutate_template(self, label, path, mutation):
        template_path = (
            self.config.parent
            / "templates"
            / rcc.TEMPLATE_FILENAMES[label]
        )
        template = _load_yaml(template_path)
        _mutate_path(template, path, mutation)
        template_path.write_text(
            yaml.safe_dump(template, sort_keys=False), encoding="utf-8"
        )


@pytest.fixture
def runtime_tree(tmp_path):
    config_dir = tmp_path / "config"
    shutil.copytree(CONFIG_DIR / "profiles", config_dir / "profiles")
    shutil.copytree(CONFIG_DIR / "templates", config_dir / "templates")
    config = config_dir / "bringup.yaml"
    config.write_text(
        yaml.safe_dump(
            {
                "platform": "sim",
                "mode": "navigation",
                "profiles": {
                    "sim": "profiles/sim.yaml",
                    "real": "profiles/real.yaml",
                },
                "map_artifacts": {
                    "lio_sam_work_dir": "/result/loam/",
                    "prior_pcd": "~/result/GlobalMap.pcd",
                    "nav2_map": "~/result/factory_map.yaml",
                },
                "slam_stack": {"settling": 20.0},
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    return _RuntimeTree(config)


def test_runtime_inputs_preserve_literal_map_artifacts(runtime_tree):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    assert inputs["map_artifacts"] == {
        "lio_sam_work_dir": "/result/loam/",
        "prior_pcd": "~/result/GlobalMap.pcd",
        "nav2_map": "~/result/factory_map.yaml",
    }


def test_runtime_inputs_need_no_retired_platform_selectors(runtime_tree):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    assert inputs["map_artifacts"] == {
        "lio_sam_work_dir": "/result/loam/",
        "prior_pcd": "~/result/GlobalMap.pcd",
        "nav2_map": "~/result/factory_map.yaml",
    }


@pytest.mark.parametrize(
    "value",
    [
        "",
        "/",
        "result/loam/",
        "/result/loam",
        "/result//loam/",
        "/result/./",
        "/result/../",
        "/result/~/",
    ],
)
def test_lio_sam_work_dir_rejects_unsafe_literal_paths(runtime_tree, value):
    _set_config_path(
        runtime_tree, ("map_artifacts", "lio_sam_work_dir"), value
    )

    with pytest.raises(ValueError, match="map_artifacts.lio_sam_work_dir"):
        rcc._load_runtime_inputs(runtime_tree.config)


@pytest.mark.parametrize("key,value", [("prior_pcd", ""), ("nav2_map", "  ")])
def test_map_file_artifacts_must_be_non_empty_strings(runtime_tree, key, value):
    _set_config_path(runtime_tree, ("map_artifacts", key), value)

    with pytest.raises(ValueError, match=rf"map_artifacts\.{key}"):
        rcc._load_runtime_inputs(runtime_tree.config)


def test_missing_map_artifacts_does_not_fall_back_to_slam_stack(runtime_tree):
    config = _load_yaml(runtime_tree.config)
    del config["map_artifacts"]
    runtime_tree.config.write_text(
        yaml.safe_dump(config, sort_keys=False), encoding="utf-8"
    )

    with pytest.raises(ValueError, match="map_artifacts must contain exactly"):
        rcc._load_runtime_inputs(runtime_tree.config)


@pytest.mark.parametrize("mode", ["mapping", "navigation"])
def test_runtime_accepts_only_supported_modes(runtime_tree, mode):
    runtime_tree.set_bringup_value("mode", mode)

    assert rcc._load_runtime_inputs(runtime_tree.config)["mode"] == mode


@pytest.mark.parametrize("mode", [None, True, 1, "", "localization"])
def test_runtime_rejects_invalid_mode(runtime_tree, mode):
    runtime_tree.set_bringup_value("mode", mode)

    with pytest.raises(ValueError, match="mode"):
        rcc._load_runtime_inputs(runtime_tree.config)


@pytest.mark.parametrize("mode", [[], {}], ids=["list", "mapping"])
def test_public_runtime_compile_rejects_non_string_mode_as_value_error(
    runtime_tree, tmp_path, mode
):
    runtime_tree.set_bringup_value("mode", mode)

    with pytest.raises(
        ValueError,
        match="bringup config mode must be 'mapping' or 'navigation'",
    ):
        rcc.compile_runtime_configs(runtime_tree.config, tmp_path / "output")


@pytest.mark.parametrize("profile_name", ["sim", "real"])
@pytest.mark.parametrize("key", rcc.MOTION_KEYS)
@pytest.mark.parametrize(
    "value",
    [
        None,
        True,
        False,
        0,
        0.0,
        -0.1,
        float("inf"),
        float("-inf"),
        float("nan"),
    ],
)
def test_runtime_rejects_invalid_motion_in_either_profile(
    runtime_tree, profile_name, key, value
):
    runtime_tree.set_profile_value(profile_name, ("motion", key), value)

    with pytest.raises(ValueError, match=rf"{profile_name}.*motion\.{key}"):
        rcc._load_runtime_inputs(runtime_tree.config)


def test_runtime_reads_bringup_yaml_exactly_once(runtime_tree, monkeypatch):
    calls = []
    original = pc._read_yaml_mapping

    def counted(path, label):
        if label == "bringup config":
            calls.append(Path(path))
        return original(path, label)

    monkeypatch.setattr(pc, "_read_yaml_mapping", counted)

    rcc._load_runtime_inputs(runtime_tree.config)

    assert calls == [runtime_tree.config]


@pytest.mark.parametrize("name", rcc.TEMPLATE_FILENAMES)
def test_runtime_rejects_missing_template(runtime_tree, name):
    path = runtime_tree.config.parent / "templates" / rcc.TEMPLATE_FILENAMES[name]
    path.unlink()

    with pytest.raises(ValueError, match="template file does not exist"):
        rcc._load_template(path, name)


@pytest.mark.parametrize(
    "name,text",
    [
        ("controllers", ""),
        ("web_ui", "- item\n"),
        ("nav2", "not-a-mapping\n"),
    ],
)
def test_runtime_rejects_non_mapping_template(runtime_tree, name, text):
    path = runtime_tree.config.parent / "templates" / rcc.TEMPLATE_FILENAMES[name]
    path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match="template root must be a mapping"):
        rcc._load_template(path, name)


@pytest.mark.parametrize("name", rcc.TEMPLATE_FILENAMES)
def test_runtime_inputs_reject_missing_source_template(runtime_tree, name):
    path = runtime_tree.config.parent / "templates" / rcc.TEMPLATE_FILENAMES[name]
    path.unlink()

    with pytest.raises(ValueError, match="template file does not exist"):
        rcc._load_runtime_inputs(runtime_tree.config)


@pytest.mark.parametrize(
    "name,text",
    [
        ("controllers", ""),
        ("web_ui", "- item\n"),
        ("nav2", "not-a-mapping\n"),
    ],
)
def test_runtime_inputs_reject_non_mapping_source_template(
    runtime_tree, name, text
):
    path = runtime_tree.config.parent / "templates" / rcc.TEMPLATE_FILENAMES[name]
    path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match="template root must be a mapping"):
        rcc._load_runtime_inputs(runtime_tree.config)


def test_runtime_inputs_return_validated_template_mappings(runtime_tree):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    assert inputs["templates"] == {
        name: _load_yaml(
            runtime_tree.config.parent / "templates" / filename
        )
        for name, filename in rcc.TEMPLATE_FILENAMES.items()
    }


def test_shared_templates_are_complete_mappings():
    for name in (
        "robot_controllers.yaml",
        "robot_web_ui.yaml",
        "nav2.yaml",
        "fast_lio.yaml",
    ):
        data = _load_yaml(TEMPLATE_DIR / name)
        assert isinstance(data, dict)
        assert data


def test_gicp_template_is_complete_confirmed_real_baseline():
    params = _load_yaml(TEMPLATE_DIR / "gicp.yaml")["gicp_localization"][
        "ros__parameters"
    ]
    assert params == {
        "map_frame": "map",
        "odom_frame": "camera_init",
        "base_frame": "body",
        "cloud_topic": "/cloud_registered",
        "odom_topic": "/Odometry",
        "prior_map_path": "",
        "map_voxel_size": 0.4,
        "scan_voxel_size": 0.1,
        "localization_freq": 0.5,
        "tf_pub_freq": 50.0,
        "gicp_max_corr_dist": 1.0,
        "gicp_num_neighbors": 20,
        "gicp_num_threads": 4,
        "gicp_max_iterations": 20,
        "fitness_threshold": 0.9,
        "min_scan_points": 100,
        "initial_pose": [0.0] * 6,
        "use_sim_time": False,
    }


def test_retired_package_algorithm_configs_are_absent():
    core = PACKAGE_ROOT.parents[1]
    retired = [
        core / "localization/gicp_localization/config/gicp_localization.yaml",
        core / "localization/gicp_localization/config/gicp_localization_real.yaml",
        core / "navigation/robot_navigation/config/nav2_params.yaml",
        core / "navigation/robot_navigation/config/nav2_params_real.yaml",
    ]
    assert all(not path.exists() for path in retired)


def test_lio_sam_template_is_complete_confirmed_real_baseline():
    data = _load_yaml(TEMPLATE_DIR / "lio_sam.yaml")
    assert set(data) == {"/**"}
    params = data["/**"]["ros__parameters"]
    assert set(params) == {
        "use_sim_time", "pointCloudTopic", "imuTopic", "odomTopic", "gpsTopic",
        "lidarFrame", "baselinkFrame", "odometryFrame", "mapFrame",
        "useImuHeadingInitialization", "useGpsElevation", "gpsCovThreshold",
        "poseCovThreshold", "savePCD", "savePCDDirectory", "sensor", "N_SCAN",
        "Horizon_SCAN", "downsampleRate", "lidarMinRange", "lidarMaxRange",
        "imuAccNoise", "imuGyrNoise", "imuAccBiasN", "imuGyrBiasN", "imuGravity",
        "imuRPYWeight", "extrinsicTrans", "extrinsicRot", "extrinsicRPY",
        "edgeThreshold", "surfThreshold", "edgeFeatureMinValidNum",
        "surfFeatureMinValidNum", "odometrySurfLeafSize", "mappingCornerLeafSize",
        "mappingSurfLeafSize", "z_tollerance", "rotation_tollerance",
        "numberOfCores", "mappingProcessInterval",
        "surroundingkeyframeAddingDistThreshold",
        "surroundingkeyframeAddingAngleThreshold", "surroundingKeyframeDensity",
        "surroundingKeyframeSearchRadius", "loopClosureEnableFlag",
        "loopClosureFrequency", "surroundingKeyframeSize",
        "historyKeyframeSearchRadius", "historyKeyframeSearchTimeDiff",
        "historyKeyframeSearchNum", "historyKeyframeFitnessScore",
        "globalMapVisualizationSearchRadius", "globalMapVisualizationPoseDensity",
        "globalMapVisualizationLeafSize",
    }
    assert params == {
        "use_sim_time": False,
        "pointCloudTopic": "/points_raw",
        "imuTopic": "/imu/data",
        "odomTopic": "odometry/imu",
        "gpsTopic": "odometry/gpsz",
        "lidarFrame": "velodyne",
        "baselinkFrame": "base_footprint",
        "odometryFrame": "odom",
        "mapFrame": "map",
        "useImuHeadingInitialization": False,
        "useGpsElevation": False,
        "gpsCovThreshold": 2.0,
        "poseCovThreshold": 25.0,
        "savePCD": True,
        "savePCDDirectory": "/result/loam/",
        "sensor": "velodyne",
        "N_SCAN": 32,
        "Horizon_SCAN": 1200,
        "downsampleRate": 1,
        "lidarMinRange": 0.3,
        "lidarMaxRange": 40.0,
        "imuAccNoise": 3.9939570888238808e-03,
        "imuGyrNoise": 1.5636343949698187e-03,
        "imuAccBiasN": 6.4356659353532566e-05,
        "imuGyrBiasN": 3.5640318696367613e-05,
        "imuGravity": 9.80511,
        "imuRPYWeight": 0.01,
        "extrinsicTrans": [0.0, 0.0, 0.0],
        "extrinsicRot": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "extrinsicRPY": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        "edgeThreshold": 0.1,
        "surfThreshold": 0.1,
        "edgeFeatureMinValidNum": 5,
        "surfFeatureMinValidNum": 30,
        "odometrySurfLeafSize": 0.2,
        "mappingCornerLeafSize": 0.1,
        "mappingSurfLeafSize": 0.2,
        "z_tollerance": 1000.0,
        "rotation_tollerance": 1000.0,
        "numberOfCores": 3,
        "mappingProcessInterval": 0.0,
        "surroundingkeyframeAddingDistThreshold": 1.0,
        "surroundingkeyframeAddingAngleThreshold": 0.2,
        "surroundingKeyframeDensity": 2.0,
        "surroundingKeyframeSearchRadius": 50.0,
        "loopClosureEnableFlag": False,
        "loopClosureFrequency": 1.0,
        "surroundingKeyframeSize": 25,
        "historyKeyframeSearchRadius": 15.0,
        "historyKeyframeSearchTimeDiff": 30.0,
        "historyKeyframeSearchNum": 25,
        "historyKeyframeFitnessScore": 0.3,
        "globalMapVisualizationSearchRadius": 1000.0,
        "globalMapVisualizationPoseDensity": 10.0,
        "globalMapVisualizationLeafSize": 1.0,
    }


def test_fast_lio_template_is_complete_and_preserves_confirmed_policy():
    params = _load_yaml(TEMPLATE_DIR / "fast_lio.yaml")["/**"]["ros__parameters"]
    assert set(params) == {
        "feature_extract_enable", "point_filter_num", "max_iteration",
        "filter_size_corner", "filter_size_surf", "filter_size_map",
        "cube_side_length", "runtime_pos_log_enable", "map_file_path",
        "common", "preprocess", "mapping", "publish", "pcd_save",
    }
    assert params["common"] == {
        "lid_topic": "/points_raw", "imu_topic": "/imu/data",
        "time_sync_en": False, "time_offset_lidar_to_imu": 0.0,
    }
    assert params["preprocess"] == {
        "lidar_type": 2, "scan_line": 16, "scan_rate": 10,
        "timestamp_unit": 0, "blind": 0.3,
    }
    assert params["mapping"] == {
        "acc_cov": 0.1, "gyr_cov": 0.1, "b_acc_cov": 0.0001,
        "b_gyr_cov": 0.0001, "fov_degree": 360.0, "det_range": 100.0,
        "extrinsic_est_en": True, "extrinsic_T": [0.0, 0.0, 0.0],
        "extrinsic_R": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    }
    assert params["publish"] == {
        "path_en": False, "effect_map_en": False, "map_en": False,
        "scan_publish_en": True, "dense_publish_en": True,
        "scan_bodyframe_pub_en": True,
    }
    assert params["pcd_save"] == {"pcd_save_en": False, "interval": -1}
    assert params["feature_extract_enable"] is False
    assert params["point_filter_num"] == 4
    assert params["max_iteration"] == 3
    assert params["filter_size_corner"] == 0.5
    assert params["filter_size_surf"] == 0.5
    assert params["filter_size_map"] == 0.5
    assert params["cube_side_length"] == 1000.0
    assert params["runtime_pos_log_enable"] is False
    assert params["map_file_path"] == ""


def test_controller_template_contains_all_owned_target_leaves():
    source = _load_yaml(
        PACKAGE_ROOT.parents[1]
        / "robot/robot_bringup/config/robot_controllers.yaml"
    )
    expected = deepcopy(source)
    expected["controller_manager"]["ros__parameters"]["use_sim_time"] = True
    expected["base_controller"]["ros__parameters"]["use_sim_time"] = True
    expected["base_controller"]["ros__parameters"][
        "linear.x.min_acceleration"
    ] = -1.0

    assert _load_yaml(TEMPLATE_DIR / "robot_controllers.yaml") == expected


def test_web_ui_template_is_a_complete_native_parameter_file():
    assert _load_yaml(TEMPLATE_DIR / "robot_web_ui.yaml") == {
        "robot_web_ui": {
            "ros__parameters": {
                "use_sim_time": True,
                "max_linear_speed": 1.5,
                "max_angular_speed": 2.0,
                "host": "0.0.0.0",
                "port": 8080,
            }
        }
    }


def test_nav2_template_is_complete_native_parameter_file():
    template = _load_yaml(TEMPLATE_DIR / "nav2.yaml")
    assert set(template) == {
        "map_server",
        "planner_server",
        "controller_server",
        "global_costmap",
        "local_costmap",
        "behavior_server",
        "bt_navigator",
    }
    follow_path = template["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow_path["vx_min"] == -0.1
    assert follow_path["wz_std"] == 0.2
    behavior = template["behavior_server"]["ros__parameters"]
    assert behavior["max_rotational_vel"] == 0.2
    assert behavior["min_rotational_vel"] == 0.1
    assert behavior["rotational_acc_lim"] == 0.2


def test_sensor_templates_are_complete_native_parameter_files():
    adapter = _load_yaml(
        TEMPLATE_DIR / SENSOR_TEMPLATE_NAMES["lidar_adapter"]
    )
    assert adapter["lidar_pointcloud_adapter"]["ros__parameters"] == {
        "use_sim_time": True,
        "input_topic": "/lidar/points",
        "output_topic": "/points_raw",
        "output_frame": "velodyne",
        "scan_period": 0.1,
    }

    vanjee = _load_yaml(TEMPLATE_DIR / SENSOR_TEMPLATE_NAMES["vanjee_lidar"])
    vanjee_params = vanjee["vanjee_lidar"]["ros__parameters"]
    assert vanjee_params["point_cloud_topic"] == "/points_raw"
    assert vanjee_params["imu_topic"] == "/imu/data"

    gate = _load_yaml(TEMPLATE_DIR / SENSOR_TEMPLATE_NAMES["sensor_gate"])
    gate_params = gate["sensor_contract_gate"]["ros__parameters"]
    assert gate_params["minimum_point_rate_ratio"] == 0.8
    assert gate_params["minimum_imu_rate_ratio"] == 0.75
    assert gate_params["max_stamp_age"] == 0.5
    assert gate_params["rate_window"] == 2.0
    assert gate_params["stable_duration"] == 2.0
    assert gate_params["timeout"] == 300.0


@pytest.mark.parametrize(
    "platform,expected_keys",
    [
        ("sim", {"lidar_adapter", "sensor_gate"}),
        ("real", {"vanjee_lidar", "sensor_gate"}),
    ],
)
def test_sensor_renderer_returns_only_selected_platform_configs(
    runtime_tree, platform, expected_keys
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    generated = rcc._render_sensor_configs(inputs)

    assert set(generated) == expected_keys
    assert set(inputs["sensor_templates"]) == expected_keys


def test_sim_sensor_renderer_maps_only_scan_period_clock_and_fixed_interface(
    runtime_tree,
):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    generated = rcc._render_sensor_configs(inputs)

    params = generated["lidar_adapter"]["lidar_pointcloud_adapter"][
        "ros__parameters"
    ]
    assert params == {
        "use_sim_time": True,
        "input_topic": "/lidar/points",
        "output_topic": "/points_raw",
        "output_frame": "velodyne",
        "scan_period": 0.1,
    }


def test_real_sensor_renderer_maps_vanjee_profile_values(runtime_tree):
    runtime_tree.set_bringup_value("platform", "real")
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    generated = rcc._render_sensor_configs(inputs)

    params = generated["vanjee_lidar"]["vanjee_lidar"]["ros__parameters"]
    assert params["lidar_type"] == "vanjee_722"
    assert params["host_address"] == "192.168.2.88"
    assert params["lidar_address"] == "192.168.2.86"
    assert params["host_msop_port"] == 3001
    assert params["lidar_msop_port"] == 3333
    assert params["start_angle"] == pytest.approx(0.0)
    assert params["end_angle"] == pytest.approx(360.0)
    assert params["min_distance"] == 0.05
    assert params["max_distance"] == 70.0
    assert params["lidar_frame"] == "velodyne"
    assert params["imu_frame"] == "imu_link"
    assert params["point_cloud_topic"] == "/points_raw"
    assert params["imu_topic"] == "/imu/data"


@pytest.mark.parametrize(
    "platform,expected_points,expected_clock",
    [("sim", 28800, True), ("real", 38400, False)],
)
def test_sensor_gate_renderer_maps_profile_facts_and_preserves_thresholds(
    runtime_tree, platform, expected_points, expected_clock
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    generated = rcc._render_sensor_configs(inputs)

    params = generated["sensor_gate"]["sensor_contract_gate"][
        "ros__parameters"
    ]
    assert params == {
        "use_sim_time": expected_clock,
        "expected_points_per_scan": expected_points,
        "expected_point_hz": 10.0,
        "expected_imu_hz": 200.0,
        "minimum_point_rate_ratio": 0.8,
        "minimum_imu_rate_ratio": 0.75,
        "max_stamp_age": 0.5,
        "rate_window": 2.0,
        "stable_duration": 2.0,
        "timeout": 300.0,
    }


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_sensor_renderer_normalizes_ros_double_parameters(
    runtime_tree, platform
):
    runtime_tree.set_bringup_value("platform", platform)
    runtime_tree.set_profile_value(
        platform, ("sensors", "lidar", "scan_rate_hz"), 10
    )
    runtime_tree.set_profile_value(
        platform, ("sensors", "imu", "rate_hz"), 200
    )
    if platform == "real":
        runtime_tree.set_profile_value(
            platform, ("sensors", "lidar", "min_range"), 1
        )
        runtime_tree.set_profile_value(
            platform, ("sensors", "lidar", "max_range"), 70
        )

    generated = rcc._render_sensor_configs(
        rcc._load_runtime_inputs(runtime_tree.config)
    )

    gate = generated["sensor_gate"]["sensor_contract_gate"]["ros__parameters"]
    assert type(gate["expected_point_hz"]) is float
    assert type(gate["expected_imu_hz"]) is float
    if platform == "sim":
        adapter = generated["lidar_adapter"]["lidar_pointcloud_adapter"][
            "ros__parameters"
        ]
        assert type(adapter["scan_period"]) is float
    else:
        vanjee = generated["vanjee_lidar"]["vanjee_lidar"]["ros__parameters"]
        for key in ("start_angle", "end_angle", "min_distance", "max_distance"):
            assert type(vanjee[key]) is float


@pytest.mark.parametrize(
    "platform,config_name,path,value",
    [
        (
            "sim",
            "lidar_adapter",
            ("lidar_pointcloud_adapter", "ros__parameters", "output_topic"),
            "/wrong_points",
        ),
        (
            "real",
            "vanjee_lidar",
            ("vanjee_lidar", "ros__parameters", "lidar_frame"),
            "wrong_lidar",
        ),
        (
            "real",
            "vanjee_lidar",
            ("vanjee_lidar", "ros__parameters", "imu_topic"),
            "/wrong_imu",
        ),
        (
            "sim",
            "lidar_adapter",
            ("lidar_pointcloud_adapter", "ros__parameters", "use_sim_time"),
            1,
        ),
        (
            "sim",
            "sensor_gate",
            ("sensor_contract_gate", "ros__parameters", "expected_points_per_scan"),
            1,
        ),
        (
            "sim",
            "sensor_gate",
            ("sensor_contract_gate", "ros__parameters", "minimum_point_rate_ratio"),
            0.0,
        ),
        (
            "real",
            "sensor_gate",
            ("sensor_contract_gate", "ros__parameters", "minimum_imu_rate_ratio"),
            1.1,
        ),
        (
            "real",
            "sensor_gate",
            ("sensor_contract_gate", "ros__parameters", "minimum_imu_rate_ratio"),
            1,
        ),
        (
            "real",
            "sensor_gate",
            ("sensor_contract_gate", "ros__parameters", "rate_window"),
            0.0,
        ),
    ],
)
def test_sensor_generated_validator_rejects_contract_drift(
    runtime_tree, platform, config_name, path, value
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    generated = rcc._render_sensor_configs(inputs)
    node = generated[config_name]
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value

    with pytest.raises(ValueError, match="generated"):
        rcc._validate_sensor_generated_configs(
            platform,
            inputs["effective"],
            generated,
        )


def test_sensor_renderer_does_not_mutate_loaded_templates(runtime_tree):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    original = deepcopy(inputs["sensor_templates"])

    rcc._render_sensor_configs(inputs)

    assert inputs["sensor_templates"] == original


def _rendered(runtime_tree, platform, mode="navigation"):
    runtime_tree.set_bringup_value("platform", platform)
    runtime_tree.set_bringup_value("mode", mode)
    return rcc._render_runtime_configs(rcc._load_runtime_inputs(runtime_tree.config))


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_renderer_maps_profile_motion_and_geometry_to_all_modules(
    runtime_tree, platform
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    rendered = rcc._render_runtime_configs(inputs)
    effective = inputs["effective"]
    geometry = effective["derived"]["geometry"]
    motion = effective["profile"]["motion"]
    controllers = rendered["controllers"]
    base = controllers["base_controller"]["ros__parameters"]
    web_ui = rendered["web_ui"]["robot_web_ui"]["ros__parameters"]
    follow_path = rendered["nav2"]["controller_server"]["ros__parameters"][
        "FollowPath"
    ]

    assert base["wheel_radius"] == geometry["drive"]["wheel_radius"]
    assert base["wheel_separation"] == geometry["drive"]["wheel_separation"]
    assert "wheel_width" not in base
    assert base["linear.x.max_velocity"] == motion["max_linear_velocity"]
    assert base["linear.x.min_velocity"] == -motion["max_linear_velocity"]
    assert base["linear.x.max_acceleration"] == motion["max_linear_acceleration"]
    assert base["linear.x.min_acceleration"] == -motion["max_linear_acceleration"]
    assert base["angular.z.max_velocity"] == motion["max_angular_velocity"]
    assert base["angular.z.min_velocity"] == -motion["max_angular_velocity"]
    assert base["angular.z.max_acceleration"] == motion["max_angular_acceleration"]
    assert base["angular.z.min_acceleration"] == -motion["max_angular_acceleration"]
    assert base["linear.x.has_acceleration_limits"] is True
    assert base["angular.z.has_acceleration_limits"] is True
    assert base["linear.x.has_jerk_limits"] is False
    assert base["angular.z.has_jerk_limits"] is False

    assert web_ui["max_linear_speed"] == motion["max_linear_velocity"]
    assert web_ui["max_angular_speed"] == motion["max_angular_velocity"]
    assert web_ui["host"] == "0.0.0.0"
    assert web_ui["port"] == 8080

    assert follow_path["vx_max"] == motion["max_linear_velocity"]
    assert follow_path["wz_max"] == motion["max_angular_velocity"]
    assert follow_path["vx_min"] == -0.1
    assert {"ax_max", "ax_min", "az_max"}.isdisjoint(follow_path)
    assert follow_path["vx_std"] == 0.2
    assert follow_path["wz_std"] == 0.2
    behavior = rendered["nav2"]["behavior_server"]["ros__parameters"]
    assert behavior["max_rotational_vel"] == 0.2
    assert behavior["min_rotational_vel"] == 0.1
    assert behavior["rotational_acc_lim"] == 0.2
    assert rendered["nav2"]["controller_server"]["ros__parameters"][
        "controller_frequency"
    ] == 10.0
    assert rendered["nav2"]["behavior_server"]["ros__parameters"][
        "spin"
    ] == {"plugin": "nav2_behaviors/Spin"}
    assert rendered["nav2"]["behavior_server"]["ros__parameters"][
        "backup"
    ] == {"plugin": "nav2_behaviors/BackUp"}
    assert rendered["nav2"]["controller_server"]["ros__parameters"][
        "FollowPath"
    ]["ConstraintCritic"] == {"enabled": True, "cost_power": 1, "cost_weight": 4.0}

    footprints = [
        rendered["nav2"]["global_costmap"]["global_costmap"]["ros__parameters"][
            "footprint"
        ],
        rendered["nav2"]["local_costmap"]["local_costmap"]["ros__parameters"][
            "footprint"
        ],
    ]
    assert all(isinstance(value, str) for value in footprints)
    assert footprints[0] == footprints[1]
    assert json.loads(footprints[0]) == geometry["footprint"]

    expected_time = platform == "sim"
    for mapping, paths in (
        (controllers, rcc.CONTROLLER_TIME_PATHS),
        (rendered["web_ui"], rcc.WEB_UI_TIME_PATHS),
        (rendered["nav2"], rcc.NAV2_TIME_PATHS),
    ):
        for path in paths:
            node = mapping
            for key in path:
                node = node[key]
            assert node is expected_time


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_renderer_does_not_depend_on_mode(runtime_tree, platform):
    assert _rendered(runtime_tree, platform, "mapping") == _rendered(
        runtime_tree, platform, "navigation"
    )


def test_renderer_does_not_mutate_loaded_templates(runtime_tree):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    original_templates = deepcopy(inputs["templates"])

    rcc._render_runtime_configs(inputs)

    assert inputs["templates"] == original_templates


@pytest.mark.parametrize(
    "profile_path,value,expected",
    [
        (("motion", "max_linear_velocity"), 0.09, "vx_min"),
        (("motion", "max_angular_velocity"), 0.19, "max_rotational_vel"),
        (("motion", "max_angular_velocity"), 0.09, "min_rotational_vel"),
        (("motion", "max_angular_acceleration"), 0.19, "rotational_acc_lim"),
    ],
)
def test_nav2_fixed_behavior_must_fit_profile_capability(
    runtime_tree, profile_path, value, expected
):
    runtime_tree.set_profile_value("real", profile_path, value)
    runtime_tree.set_bringup_value("platform", "real")
    with pytest.raises(ValueError, match=expected):
        inputs = rcc._load_runtime_inputs(runtime_tree.config)
        rcc._render_runtime_configs(inputs)


@pytest.mark.parametrize(
    "platform,scan_line",
    [("sim", 16), ("real", 32)],
)
def test_fast_lio_renderer_maps_platform_scan_and_time_contract(
    runtime_tree, platform, scan_line
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    params = rcc._render_fast_lio(
        inputs["templates"]["fast_lio"], inputs["effective"]
    )["/**"]["ros__parameters"]
    assert params["preprocess"] == {
        "lidar_type": 2, "scan_line": scan_line, "scan_rate": 10,
        "timestamp_unit": 0, "blind": 0.3,
    }
    assert params["mapping"]["extrinsic_T"] == [0.0, 0.0, 0.0]
    assert params["mapping"]["extrinsic_R"] == [
        1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
    ]
    assert "use_sim_time" not in params
    assert "min_range" not in params and "max_range" not in params
    assert params["mapping"]["det_range"] == 100.0


def test_fast_lio_renderer_changes_only_the_five_whitelisted_leaves(runtime_tree):
    runtime_tree.set_bringup_value("platform", "real")
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    source = inputs["templates"]["fast_lio"]
    before = deepcopy(source)
    rendered = rcc._render_fast_lio(source, inputs["effective"])
    expected = deepcopy(before)
    expected_params = expected["/**"]["ros__parameters"]
    actual_params = rendered["/**"]["ros__parameters"]
    for section, key in (
        ("preprocess", "scan_line"),
        ("preprocess", "scan_rate"),
        ("preprocess", "timestamp_unit"),
        ("mapping", "extrinsic_T"),
        ("mapping", "extrinsic_R"),
    ):
        expected_params[section][key] = actual_params[section][key]
    assert rendered == expected
    assert source == before


FAST_LIO_RENDER_PATHS = (
    ("/**", "ros__parameters", "preprocess", "scan_line"),
    ("/**", "ros__parameters", "preprocess", "scan_rate"),
    ("/**", "ros__parameters", "preprocess", "timestamp_unit"),
    ("/**", "ros__parameters", "mapping", "extrinsic_T"),
    ("/**", "ros__parameters", "mapping", "extrinsic_R"),
)
GICP_RENDER_PATHS = (
    ("gicp_localization", "ros__parameters", "use_sim_time"),
)
LIO_SAM_RENDER_PATHS = (
    ("/**", "ros__parameters", "use_sim_time"),
    ("/**", "ros__parameters", "N_SCAN"),
    ("/**", "ros__parameters", "Horizon_SCAN"),
    ("/**", "ros__parameters", "savePCDDirectory"),
    ("/**", "ros__parameters", "extrinsicTrans"),
    ("/**", "ros__parameters", "extrinsicRot"),
)


@pytest.mark.parametrize("path", FAST_LIO_RENDER_PATHS)
@pytest.mark.parametrize("mutation", ["missing_parent", "missing_leaf", "wrong_type"])
def test_fast_lio_renderer_rejects_template_target_drift(
    runtime_tree, path, mutation
):
    runtime_tree.mutate_template("fast_lio", path, mutation)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    with pytest.raises(ValueError, match="fast_lio template"):
        rcc._render_fast_lio(inputs["templates"]["fast_lio"], inputs["effective"])


def _set_nested(mapping, path, value):
    node = mapping
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value


@pytest.mark.parametrize(
    "path,value,error",
    [
        (("profile", "sensors", "lidar", "scan_rate_hz"), 10.5, "integer"),
        (("profile", "sensors", "lidar", "point_time_unit"),
         "milliseconds", "seconds"),
        (("derived", "geometry", "relative_transforms", "imu_from_lidar",
          "translation"), [True, 0.0, 0.0], "finite"),
        (("derived", "geometry", "relative_transforms", "imu_from_lidar",
          "translation"), [float("nan"), 0.0, 0.0], "finite"),
        (("derived", "geometry", "relative_transforms", "imu_from_lidar",
          "rotation_xyzw"), [0.0, 0.0, 0.0, 2.0], "normalized"),
    ],
)
def test_fast_lio_renderer_rejects_invalid_effective_values(
    runtime_tree, path, value, error
):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    _set_nested(inputs["effective"], path, value)
    with pytest.raises(ValueError, match=error):
        rcc._render_fast_lio(inputs["templates"]["fast_lio"], inputs["effective"])


@pytest.mark.parametrize("value", [True, 16.0, 0, -1])
def test_fast_lio_renderer_rejects_non_positive_strict_integer_scan_lines(
    runtime_tree, value
):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    _set_nested(
        inputs["effective"], ("profile", "sensors", "lidar", "scan_lines"), value
    )

    with pytest.raises(ValueError, match="scan_lines must be a positive integer"):
        rcc._render_fast_lio(inputs["templates"]["fast_lio"], inputs["effective"])


def test_fast_lio_renderer_converts_xyzw_to_row_major_rotation(runtime_tree):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    relative = inputs["effective"]["derived"]["geometry"]["relative_transforms"][
        "imu_from_lidar"
    ]
    relative["translation"] = [1.0, 2.0, 3.0]
    relative["rotation_xyzw"] = [
        0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)
    ]
    params = rcc._render_fast_lio(
        inputs["templates"]["fast_lio"], inputs["effective"]
    )["/**"]["ros__parameters"]
    assert params["mapping"]["extrinsic_T"] == [1.0, 2.0, 3.0]
    assert params["mapping"]["extrinsic_R"] == pytest.approx(
        [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0], abs=1e-12
    )


@pytest.mark.parametrize(
    "matrix,error",
    [
        ([2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], "orthonormal"),
        ([-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0], "determinant"),
    ],
)
def test_fast_lio_validator_rejects_invalid_rotation_matrix(
    runtime_tree, matrix, error
):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    rendered = rcc._render_fast_lio(
        inputs["templates"]["fast_lio"], inputs["effective"]
    )
    rendered["/**"]["ros__parameters"]["mapping"]["extrinsic_R"] = matrix
    with pytest.raises(ValueError, match=error):
        rcc._validate_fast_lio_generated(inputs["effective"], rendered)


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_gicp_renderer_changes_only_the_whitelisted_clock_leaf(
    runtime_tree, platform
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    source = inputs["templates"]["gicp"]
    before = deepcopy(source)

    rendered = rcc._render_gicp(source, inputs["effective"])
    expected = deepcopy(before)
    expected["gicp_localization"]["ros__parameters"]["use_sim_time"] = (
        inputs["effective"]["derived"]["use_sim_time"]
    )

    assert rendered == expected
    assert source == before


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_lio_sam_renderer_changes_only_the_six_whitelisted_leaves(
    runtime_tree, platform
):
    runtime_tree.set_bringup_value("platform", platform)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    source = inputs["templates"]["lio_sam"]
    before = deepcopy(source)

    rendered = rcc._render_lio_sam(
        source, inputs["effective"], inputs["map_artifacts"]
    )
    expected = deepcopy(before)
    for path in LIO_SAM_RENDER_PATHS:
        expected_node = expected
        actual_node = rendered
        for key in path[:-1]:
            expected_node = expected_node[key]
            actual_node = actual_node[key]
        expected_node[path[-1]] = actual_node[path[-1]]

    assert rendered == expected
    assert source == before


@pytest.mark.parametrize("path", GICP_RENDER_PATHS)
@pytest.mark.parametrize("mutation", ["missing_parent", "missing_leaf", "wrong_type"])
def test_gicp_renderer_rejects_template_target_drift(runtime_tree, path, mutation):
    runtime_tree.mutate_template("gicp", path, mutation)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    with pytest.raises(ValueError, match="gicp template:.*template target"):
        rcc._render_gicp(inputs["templates"]["gicp"], inputs["effective"])


@pytest.mark.parametrize("path", LIO_SAM_RENDER_PATHS)
@pytest.mark.parametrize("mutation", ["missing_parent", "missing_leaf", "wrong_type"])
def test_lio_sam_renderer_rejects_template_target_drift(
    runtime_tree, path, mutation
):
    runtime_tree.mutate_template("lio_sam", path, mutation)
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    with pytest.raises(ValueError, match="lio_sam template:.*template target"):
        rcc._render_lio_sam(
            inputs["templates"]["lio_sam"],
            inputs["effective"],
            inputs["map_artifacts"],
        )


def test_lio_sam_renderer_uses_lidar_to_imu_translation_and_inverse_rotation(
    runtime_tree
):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    relative = inputs["effective"]["derived"]["geometry"][
        "relative_transforms"
    ]["imu_from_lidar"]
    relative["translation"] = [1.25, -2.5, 3.75]
    relative["rotation_xyzw"] = [
        0.2392983377447303,
        0.189307857412,
        0.03813457647485015,
        0.9515485246437885,
    ]

    params = rcc._render_lio_sam(
        inputs["templates"]["lio_sam"],
        inputs["effective"],
        inputs["map_artifacts"],
    )["/**"]["ros__parameters"]

    assert params["extrinsicTrans"] == [1.25, -2.5, 3.75]
    assert params["extrinsicRot"] == pytest.approx([
        0.9254165783983234, 0.16317591116653482, -0.34202014332566866,
        0.01802831123629728, 0.8825641192593856, 0.4698463103929541,
        0.37852230636979245, -0.44096961052988237, 0.8137976813493737,
    ], abs=1e-12)
    assert params["extrinsicRPY"] == [
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0,
    ]

    rotation = [params["extrinsicRot"][index:index + 3] for index in range(0, 9, 3)]
    for row in range(3):
        for column in range(3):
            dot = sum(rotation[row][index] * rotation[column][index]
                      for index in range(3))
            assert dot == pytest.approx(1.0 if row == column else 0.0, abs=1e-12)
    determinant = (
        rotation[0][0] * (
            rotation[1][1] * rotation[2][2] - rotation[1][2] * rotation[2][1]
        )
        - rotation[0][1] * (
            rotation[1][0] * rotation[2][2] - rotation[1][2] * rotation[2][0]
        )
        + rotation[0][2] * (
            rotation[1][0] * rotation[2][1] - rotation[1][1] * rotation[2][0]
        )
    )
    assert determinant == pytest.approx(1.0, abs=1e-12)


@pytest.mark.parametrize(
    "mutate",
    [
        lambda rendered: rendered["controllers"]["base_controller"][
            "ros__parameters"
        ].__setitem__("wheel_width", 0.06),
        lambda rendered: rendered["web_ui"]["robot_web_ui"]["ros__parameters"].__setitem__(
            "max_linear_speed", 99.0
        ),
        lambda rendered: rendered["nav2"]["controller_server"]["ros__parameters"][
            "FollowPath"
        ].__setitem__("ax_max", 1.0),
    ],
)
def test_generated_config_validator_rejects_cross_module_drift(
    runtime_tree, mutate
):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    rendered = rcc._render_runtime_configs(inputs)
    mutate(rendered)

    with pytest.raises(ValueError):
        rcc._validate_generated_configs(
            inputs["effective"],
            rendered["controllers"],
            rendered["web_ui"],
            rendered["nav2"],
        )


CONTROLLER_RENDER_PATHS = (
    ("base_controller", "ros__parameters", "wheel_radius"),
    ("base_controller", "ros__parameters", "wheel_separation"),
    ("base_controller", "ros__parameters", "linear.x.max_velocity"),
    ("base_controller", "ros__parameters", "linear.x.min_velocity"),
    ("base_controller", "ros__parameters", "linear.x.max_acceleration"),
    ("base_controller", "ros__parameters", "linear.x.min_acceleration"),
    ("base_controller", "ros__parameters", "angular.z.max_velocity"),
    ("base_controller", "ros__parameters", "angular.z.min_velocity"),
    ("base_controller", "ros__parameters", "angular.z.max_acceleration"),
    ("base_controller", "ros__parameters", "angular.z.min_acceleration"),
) + rcc.CONTROLLER_TIME_PATHS
WEB_UI_RENDER_PATHS = (
    ("robot_web_ui", "ros__parameters", "max_linear_speed"),
    ("robot_web_ui", "ros__parameters", "max_angular_speed"),
) + rcc.WEB_UI_TIME_PATHS
NAV2_RENDER_PATHS = (
    ("controller_server", "ros__parameters", "FollowPath", "vx_max"),
    ("controller_server", "ros__parameters", "FollowPath", "wz_max"),
    ("global_costmap", "global_costmap", "ros__parameters", "footprint"),
    ("local_costmap", "local_costmap", "ros__parameters", "footprint"),
)
NAV2_STVL_ROOTS = (
    ("global_costmap", "global_costmap", "ros__parameters", "stvl_layer"),
    ("local_costmap", "local_costmap", "ros__parameters", "stvl_layer"),
)
NAV2_STVL_RENDER_PATHS = tuple(
    root + suffix
    for root in NAV2_STVL_ROOTS
    for suffix in (
        ("pointcloud_mark", "min_obstacle_height"),
        ("pointcloud_mark", "max_obstacle_height"),
        ("pointcloud_clear", "min_z"),
        ("pointcloud_clear", "max_z"),
        ("pointcloud_clear", "vertical_fov_angle"),
        ("pointcloud_clear", "horizontal_fov_angle"),
    )
)
NAV2_RENDER_PATHS += NAV2_STVL_RENDER_PATHS + rcc.NAV2_TIME_PATHS
NAV2_FIXED_BEHAVIOR_PATHS = (
    ("controller_server", "ros__parameters", "FollowPath", "vx_min"),
    ("controller_server", "ros__parameters", "FollowPath", "wz_std"),
    ("behavior_server", "ros__parameters", "max_rotational_vel"),
    ("behavior_server", "ros__parameters", "min_rotational_vel"),
    ("behavior_server", "ros__parameters", "rotational_acc_lim"),
)


def test_nav2_renderer_changes_only_whitelisted_leaves_and_injects_stvl_profile(
    runtime_tree,
):
    runtime_tree.set_profile_value(
        "real", ("sensors", "lidar", "vertical_fov_angle"), 0.6
    )
    runtime_tree.set_profile_value(
        "real", ("perception", "obstacle_height", "min"), -0.4
    )
    runtime_tree.set_profile_value(
        "real", ("perception", "obstacle_height", "max"), 1.8
    )
    runtime_tree.set_bringup_value("platform", "real")
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    source = inputs["templates"]["nav2"]
    before = deepcopy(source)

    rendered = rcc._render_nav2(source, inputs["effective"])
    expected = deepcopy(before)
    for path in NAV2_RENDER_PATHS:
        _set_nested(expected, path, _path_value(rendered, path))

    assert rendered == expected
    assert source == before

    effective = inputs["effective"]
    min_height = effective["profile"]["perception"]["obstacle_height"]["min"]
    max_height = effective["profile"]["perception"]["obstacle_height"]["max"]
    lidar = effective["profile"]["sensors"]["lidar"]
    horizontal_fov = (
        lidar["horizontal_end_angle"] - lidar["horizontal_start_angle"]
    )
    for root in NAV2_STVL_ROOTS:
        assert _path_value(
            rendered, root + ("pointcloud_mark", "min_obstacle_height")
        ) == min_height
        assert _path_value(
            rendered, root + ("pointcloud_mark", "max_obstacle_height")
        ) == max_height
        assert _path_value(
            rendered, root + ("pointcloud_clear", "min_z")
        ) == min_height
        assert _path_value(
            rendered, root + ("pointcloud_clear", "max_z")
        ) == max_height
        assert _path_value(
            rendered, root + ("pointcloud_clear", "vertical_fov_angle")
        ) == lidar["vertical_fov_angle"]
        assert _path_value(
            rendered, root + ("pointcloud_clear", "horizontal_fov_angle")
        ) == horizontal_fov


@pytest.mark.parametrize(
    "path", NAV2_STVL_RENDER_PATHS + NAV2_FIXED_BEHAVIOR_PATHS
)
def test_generated_config_validator_rejects_nav2_stvl_and_fixed_behavior_drift(
    runtime_tree, path
):
    inputs = rcc._load_runtime_inputs(runtime_tree.config)
    rendered = rcc._render_runtime_configs(inputs)
    _set_nested(rendered["nav2"], path, 99.0)

    with pytest.raises(ValueError, match="generated nav2 mismatch"):
        rcc._validate_generated_configs(
            inputs["effective"],
            rendered["controllers"],
            rendered["web_ui"],
            rendered["nav2"],
        )


def _mutate_path(mapping, path, mutation):
    node = mapping
    for key in path[:-1]:
        node = node[key]
    if mutation == "missing_leaf":
        del node[path[-1]]
    elif mutation == "wrong_type":
        node[path[-1]] = "wrong" if isinstance(node[path[-1]], bool) else True
    else:
        parent = path[-2]
        grandparent = mapping
        for key in path[:-2]:
            grandparent = grandparent[key]
        del grandparent[parent]


@pytest.mark.parametrize(
    ("label", "path"),
    [("controllers", path) for path in CONTROLLER_RENDER_PATHS]
    + [("web_ui", path) for path in WEB_UI_RENDER_PATHS]
    + [("nav2", path) for path in NAV2_RENDER_PATHS],
)
@pytest.mark.parametrize("mutation", ["missing_parent", "missing_leaf", "wrong_type"])
def test_public_runtime_compile_rejects_source_template_target_drift(
    runtime_tree, tmp_path, label, path, mutation
):
    runtime_tree.mutate_template(label, path, mutation)
    output = tmp_path / "output"
    expected_error = rf"{label} template"
    if mutation != "missing_parent":
        expected_error += rf".*{'.'.join(path)}"

    with pytest.raises(ValueError, match=expected_error):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert not output.exists()


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_robot_launch_arguments_project_independent_effective_mounts_and_sensor_facts(
    runtime_tree, tmp_path, platform
):
    runtime_tree.set_bringup_value("platform", platform)
    for sensor, values in {
        "lidar": {
            "x": 0.31,
            "y": -0.21,
            "z": 0.91,
            "roll": 0.17,
            "pitch": -0.23,
            "yaw": 0.39,
        },
        "imu": {
            "x": -0.13,
            "y": 0.27,
            "z": 0.64,
            "roll": -0.11,
            "pitch": 0.29,
            "yaw": -0.37,
        },
    }.items():
        for key, value in values.items():
            runtime_tree.set_profile_value(
                platform, ("robot", "mounts", sensor, key), value
            )

    manifest = rcc.compile_runtime_configs(runtime_tree.config, tmp_path / platform)
    effective = _load_yaml(manifest["effective_profile_path"])
    arguments = manifest["robot_launch_arguments"]
    geometry = effective["derived"]["geometry"]
    mounts = geometry["mounts_relative_to_base_link"]

    assert set(arguments) == ROBOT_LAUNCH_ARGUMENT_KEYS
    assert "sensor_x" not in arguments
    assert {
        "base_length": arguments["base_length"],
        "base_width": arguments["base_width"],
        "base_height": arguments["base_height"],
        "base_link_height": arguments["base_link_height"],
    } == {
        "base_length": str(geometry["body"]["length"]),
        "base_width": str(geometry["body"]["width"]),
        "base_height": str(geometry["body"]["height"]),
        "base_link_height": str(geometry["body"]["base_link_height"]),
    }
    assert {
        "wheel_radius": arguments["wheel_radius"],
        "wheel_width": arguments["wheel_width"],
        "wheel_separation": arguments["wheel_separation"],
    } == {
        "wheel_radius": str(geometry["drive"]["wheel_radius"]),
        "wheel_width": str(geometry["drive"]["wheel_width"]),
        "wheel_separation": str(geometry["drive"]["wheel_separation"]),
    }
    for sensor in ("lidar", "imu"):
        for axis in ("x", "y", "z", "roll", "pitch", "yaw"):
            assert arguments[f"{sensor}_{axis}"] == str(mounts[sensor][axis])
    assert arguments["lidar_x"] == str(mounts["lidar"]["x"])
    assert arguments["imu_x"] == str(mounts["imu"]["x"])
    assert arguments["lidar_scan_lines"] == str(
        effective["profile"]["sensors"]["lidar"]["scan_lines"]
    )
    assert arguments["lidar_columns_per_scan"] == str(
        effective["profile"]["sensors"]["lidar"]["columns_per_scan"]
    )
    assert arguments["lidar_scan_rate_hz"] == str(
        effective["profile"]["sensors"]["lidar"]["scan_rate_hz"]
    )
    assert arguments["lidar_min_range"] == str(
        effective["profile"]["sensors"]["lidar"]["min_range"]
    )
    assert arguments["lidar_max_range"] == str(
        effective["profile"]["sensors"]["lidar"]["max_range"]
    )
    assert arguments["lidar_horizontal_start_angle"] == str(
        effective["profile"]["sensors"]["lidar"]["horizontal_start_angle"]
    )
    assert arguments["lidar_horizontal_end_angle"] == str(
        effective["profile"]["sensors"]["lidar"]["horizontal_end_angle"]
    )
    assert arguments["imu_rate_hz"] == str(
        effective["profile"]["sensors"]["imu"]["rate_hz"]
    )

    assert "wheel_width" not in _rendered(runtime_tree, platform)["controllers"][
        "base_controller"
    ]["ros__parameters"]


def test_runtime_manifest_has_only_the_permanent_fast_lio_bridge(runtime_tree, tmp_path):
    manifest = rcc.compile_runtime_configs(runtime_tree.config, tmp_path / "out")
    report = _load_yaml(manifest["effective_profile_path"])

    assert set(manifest["fast_lio_body_bridge_arguments"]) == {
        "x", "y", "z", "qx", "qy", "qz", "qw",
    }
    assert "compatibility_body_weld_arguments" not in manifest
    assert "compatibility" not in report
    assert "deferred_compatibility" not in report
    assert "deferred_to_section_9" not in yaml.safe_dump(report)


def test_runtime_compiler_exports_no_temporary_weld_helpers():
    assert not hasattr(rcc, "_derive_compatibility_body_weld_transform")
    assert not hasattr(rcc, "_derive_compatibility_body_weld_arguments")


def test_runtime_report_is_an_exact_copy_without_deferred_compatibility(runtime_tree):
    runtime_tree.set_bringup_value("platform", "real")
    inputs = rcc._load_runtime_inputs(runtime_tree.config)

    report = rcc._build_runtime_report(inputs["effective"])

    assert report == deepcopy(inputs["effective"])
    assert report is not inputs["effective"]
    assert "deferred_compatibility" not in report
    assert "deferred_to_section_9" not in yaml.safe_dump(report)


def _temporary_files(output):
    return sorted(output.glob(".*.tmp"))


def test_recursive_typed_tree_equality_checks_shape_container_and_leaf_types():
    expected = {"mapping": {"enabled": False, "values": [1, 2.0]}}

    assert rcc._same_typed_tree(deepcopy(expected), expected)
    assert not rcc._same_typed_tree([], {})
    assert not rcc._same_typed_tree({"mapping": {}}, expected)
    assert not rcc._same_typed_tree(
        {"mapping": {"enabled": False, "values": [1]}}, expected
    )
    assert not rcc._same_typed_tree(
        {"mapping": {"enabled": 0, "values": [1, 2]}}, expected
    )


def _yaml_schema(value):
    if isinstance(value, dict):
        return {key: _yaml_schema(child) for key, child in value.items()}
    if isinstance(value, list):
        return [_yaml_schema(child) for child in value]
    return type(value)


def _profile_owned_runtime_values(manifest):
    controllers = _load_yaml(manifest["controllers_path"])
    base = controllers["base_controller"]["ros__parameters"]
    web_ui_config = _load_yaml(manifest["web_ui_path"])
    web_ui = web_ui_config["robot_web_ui"]["ros__parameters"]
    nav2 = _load_yaml(manifest["nav2_path"])
    follow_path = nav2["controller_server"]["ros__parameters"]["FollowPath"]
    footprints = [
        nav2["global_costmap"]["global_costmap"]["ros__parameters"][
            "footprint"
        ],
        nav2["local_costmap"]["local_costmap"]["ros__parameters"][
            "footprint"
        ],
    ]
    return {
        "controllers": {
            "wheel_radius": base["wheel_radius"],
            "wheel_separation": base["wheel_separation"],
            "max_linear_velocity": base["linear.x.max_velocity"],
            "min_linear_velocity": base["linear.x.min_velocity"],
            "max_linear_acceleration": base["linear.x.max_acceleration"],
            "min_linear_acceleration": base["linear.x.min_acceleration"],
            "max_angular_velocity": base["angular.z.max_velocity"],
            "min_angular_velocity": base["angular.z.min_velocity"],
            "max_angular_acceleration": base["angular.z.max_acceleration"],
            "min_angular_acceleration": base["angular.z.min_acceleration"],
        },
        "web_ui": {
            "max_linear_speed": web_ui["max_linear_speed"],
            "max_angular_speed": web_ui["max_angular_speed"],
        },
        "nav2": {
            "vx_max": follow_path["vx_max"],
            "wz_max": follow_path["wz_max"],
            "footprints": [json.loads(value) for value in footprints],
        },
        "use_sim_time": {
            "controllers": [
                _path_value(controllers, path)
                for path in rcc.CONTROLLER_TIME_PATHS
            ],
            "web_ui": [
                _path_value(web_ui_config, path)
                for path in rcc.WEB_UI_TIME_PATHS
            ],
            "nav2": [_path_value(nav2, path) for path in rcc.NAV2_TIME_PATHS],
        },
    }


def _path_value(mapping, path):
    for key in path:
        mapping = mapping[key]
    return mapping


def test_sim_and_real_public_compiles_remain_schema_and_value_isolated(
    runtime_tree, tmp_path
):
    manifests = {platform: {} for platform in ("sim", "real")}
    generated = {platform: {} for platform in ("sim", "real")}
    for platform in ("sim", "real"):
        for mode in ("mapping", "navigation"):
            runtime_tree.set_bringup_value("platform", platform)
            runtime_tree.set_bringup_value("mode", mode)
            manifests[platform][mode] = rcc.compile_runtime_configs(
                runtime_tree.config, tmp_path / platform / mode
            )
            generated[platform][mode] = {
                name: _load_yaml(manifests[platform][mode][f"{name}_path"])
                for name in (
                    "controllers",
                    "web_ui",
                    "nav2",
                    "effective_profile",
                )
            }

        for name in ("controllers", "web_ui", "nav2"):
            assert generated[platform]["mapping"][name] == generated[platform][
                "navigation"
            ][name]

    for name in ("controllers", "web_ui", "nav2"):
        assert _yaml_schema(generated["sim"]["navigation"][name]) == _yaml_schema(
            generated["real"]["navigation"][name]
        )

    assert _profile_owned_runtime_values(manifests["sim"]["navigation"]) == {
        "controllers": {
            "wheel_radius": 0.12,
            "wheel_separation": 0.55,
            "max_linear_velocity": 1.0,
            "min_linear_velocity": -1.0,
            "max_linear_acceleration": 1.0,
            "min_linear_acceleration": -1.0,
            "max_angular_velocity": 1.8,
            "min_angular_velocity": -1.8,
            "max_angular_acceleration": 1.0,
            "min_angular_acceleration": -1.0,
        },
        "web_ui": {"max_linear_speed": 1.0, "max_angular_speed": 1.8},
        "nav2": {
            "vx_max": 1.0,
            "wz_max": 1.8,
            "footprints": [SIM_FOOTPRINT, SIM_FOOTPRINT],
        },
        "use_sim_time": {
            "controllers": [True, True],
            "web_ui": [True],
            "nav2": [True] * len(rcc.NAV2_TIME_PATHS),
        },
    }
    assert _profile_owned_runtime_values(manifests["real"]["navigation"]) == {
        "controllers": {
            "wheel_radius": 0.1025,
            "wheel_separation": 0.463,
            "max_linear_velocity": 1.0,
            "min_linear_velocity": -1.0,
            "max_linear_acceleration": 1.0,
            "min_linear_acceleration": -1.0,
            "max_angular_velocity": 0.4,
            "min_angular_velocity": -0.4,
            "max_angular_acceleration": 0.3,
            "min_angular_acceleration": -0.3,
        },
        "web_ui": {"max_linear_speed": 1.0, "max_angular_speed": 0.4},
        "nav2": {
            "vx_max": 1.0,
            "wz_max": 0.4,
            "footprints": [REAL_FOOTPRINT, REAL_FOOTPRINT],
        },
        "use_sim_time": {
            "controllers": [False, False],
            "web_ui": [False],
            "nav2": [False] * len(rcc.NAV2_TIME_PATHS),
        },
    }

    legacy_report_keys = {"platform", "source_profile", "profile", "derived"}
    runtime_only_keys = {"generated_configs"}
    for platform_values in generated.values():
        for values in platform_values.values():
            assert set(values["effective_profile"]) == (
                legacy_report_keys | runtime_only_keys
            )

    profile_path = pc.compile_profile(runtime_tree.config, tmp_path / "profile")
    profile_report = _load_yaml(profile_path)
    assert set(profile_report) == legacy_report_keys
    assert runtime_only_keys.isdisjoint(profile_report)


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_compile_runtime_configs_writes_owned_files_and_stable_manifest(
    runtime_tree, tmp_path, platform, monkeypatch
):
    runtime_tree.set_bringup_value("platform", platform)
    output = tmp_path / f"{platform}-output"
    loaded = []
    original = rcc._load_runtime_inputs

    def capture_inputs(path):
        inputs = original(path)
        loaded.append(inputs)
        return inputs

    monkeypatch.setattr(rcc, "_load_runtime_inputs", capture_inputs)

    manifest = rcc.compile_runtime_configs(runtime_tree.config, output)

    assert len(loaded) == 1
    output_filenames = rcc._output_filenames(platform)
    paths = {key: output / filename for key, filename in output_filenames.items()}
    assert set(output.iterdir()) == set(paths.values())
    assert set(manifest) == {
        "bringup_config_path",
        "bringup_config",
        "platform",
        "mode",
        "use_sim_time",
        "effective_profile_path",
        "controllers_path",
        "web_ui_path",
        "nav2_path",
        "fast_lio_path",
        "lio_sam_path",
        "gicp_path",
        *(f"{key}_path" for key in rcc.SENSOR_OUTPUT_FILENAMES[platform]),
        "robot_launch_arguments",
        "fast_lio_body_bridge_arguments",
    }
    assert manifest["bringup_config_path"] == runtime_tree.config.resolve()
    assert manifest["bringup_config"] == loaded[0]["config"]
    assert manifest["bringup_config"] is not loaded[0]["config"]
    assert (
        manifest["bringup_config"]["profiles"]
        is not loaded[0]["config"]["profiles"]
    )
    source_config = runtime_tree.config.read_bytes()
    selected_profile = loaded[0]["config"]["profiles"][platform]
    manifest["bringup_config"]["profiles"][platform] = "mutated"
    assert loaded[0]["config"]["profiles"][platform] == selected_profile
    assert runtime_tree.config.read_bytes() == source_config
    assert manifest["platform"] == platform
    assert manifest["mode"] == "navigation"
    assert manifest["use_sim_time"] is (platform == "sim")
    assert manifest["effective_profile_path"] == paths["effective_profile"]
    assert manifest["controllers_path"] == paths["controllers"]
    assert manifest["web_ui_path"] == paths["web_ui"]
    assert manifest["nav2_path"] == paths["nav2"]
    assert manifest["fast_lio_path"] == paths["fast_lio"]
    assert manifest["fast_lio_path"].name == "fast_lio.generated.yaml"
    assert manifest["lio_sam_path"].name == "lio_sam.generated.yaml"
    assert manifest["gicp_path"].name == "gicp.generated.yaml"
    assert manifest["robot_launch_arguments"] == rcc._derive_robot_launch_arguments(
        loaded[0]["effective"]
    )
    assert manifest["fast_lio_body_bridge_arguments"] == {
        key: str(value)
        for key, value in zip(
            ("x", "y", "z", "qx", "qy", "qz", "qw"),
            [
                *loaded[0]["effective"]["derived"]["geometry"][
                    "relative_transforms"
                ]["imu_from_base_footprint"]["translation"],
                *loaded[0]["effective"]["derived"]["geometry"][
                    "relative_transforms"
                ]["imu_from_base_footprint"]["rotation_xyzw"],
            ],
        )
    }
    assert "compatibility_body_weld_arguments" not in manifest

    report = _load_yaml(paths["effective_profile"])
    assert report["generated_configs"] == {
        key: str(paths[key])
        for key in output_filenames
        if key != "effective_profile"
    }
    assert set(report["generated_configs"]) == (
        set(rcc.COMMON_OUTPUT_FILENAMES)
        | set(rcc.SENSOR_OUTPUT_FILENAMES[platform])
    )
    assert _temporary_files(output) == []


@pytest.mark.parametrize("platform", ["sim", "real"])
@pytest.mark.parametrize("mode", ["mapping", "navigation"])
def test_every_platform_mode_generates_the_same_complete_common_set(
    runtime_tree, tmp_path, platform, mode
):
    runtime_tree.set_bringup_value("platform", platform)
    runtime_tree.set_bringup_value("mode", mode)
    manifest = rcc.compile_runtime_configs(
        runtime_tree.config, tmp_path / platform / mode
    )
    assert {
        manifest[f"{key}_path"].name for key in rcc.COMMON_OUTPUT_FILENAMES
    } == set(rcc.COMMON_OUTPUT_FILENAMES.values())


@pytest.mark.parametrize(
    "platform,mode,scan_line,bridge",
    [
        (
            "sim",
            "mapping",
            16,
            {
                "x": "0.0",
                "y": "0.0",
                "z": "-0.556",
                "qx": "0.0",
                "qy": "0.0",
                "qz": "0.0",
                "qw": "1.0",
            },
        ),
        (
            "sim",
            "navigation",
            16,
            {
                "x": "0.0",
                "y": "0.0",
                "z": "-0.556",
                "qx": "0.0",
                "qy": "0.0",
                "qz": "0.0",
                "qw": "1.0",
            },
        ),
        (
            "real",
            "mapping",
            32,
            {
                "x": "-0.443",
                "y": "0.0",
                "z": "-0.905",
                "qx": "0.0",
                "qy": "0.0",
                "qz": "0.0",
                "qw": "1.0",
            },
        ),
        (
            "real",
            "navigation",
            32,
            {
                "x": "-0.443",
                "y": "0.0",
                "z": "-0.905",
                "qx": "0.0",
                "qy": "0.0",
                "qz": "0.0",
                "qw": "1.0",
            },
        ),
    ],
)
def test_runtime_writes_fast_lio_for_every_platform_and_mode(
    runtime_tree, tmp_path, platform, mode, scan_line, bridge
):
    runtime_tree.set_bringup_value("platform", platform)
    runtime_tree.set_bringup_value("mode", mode)
    manifest = rcc.compile_runtime_configs(runtime_tree.config, tmp_path / "out")
    params = _load_yaml(manifest["fast_lio_path"])["/**"]["ros__parameters"]
    assert params["preprocess"]["scan_line"] == scan_line
    assert params["preprocess"]["scan_rate"] == 10
    assert params["preprocess"]["timestamp_unit"] == 0
    assert params["mapping"]["extrinsic_T"] == [0.0, 0.0, 0.0]
    assert params["mapping"]["extrinsic_R"] == [
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
    assert manifest["fast_lio_body_bridge_arguments"] == bridge
    report = _load_yaml(manifest["effective_profile_path"])
    assert report["generated_configs"]["fast_lio"] == str(
        manifest["fast_lio_path"]
    )


@pytest.mark.parametrize(
    "platform,manifest_sensor_keys,absent_key",
    [
        (
            "sim",
            {"lidar_adapter_path", "sensor_gate_path"},
            "vanjee_lidar_path",
        ),
        (
            "real",
            {"vanjee_lidar_path", "sensor_gate_path"},
            "lidar_adapter_path",
        ),
    ],
)
def test_runtime_compile_writes_only_selected_sensor_artifacts(
    runtime_tree,
    tmp_path,
    platform,
    manifest_sensor_keys,
    absent_key,
):
    runtime_tree.set_bringup_value("platform", platform)
    output = tmp_path / platform

    manifest = rcc.compile_runtime_configs(runtime_tree.config, output)

    expected_outputs = rcc._output_filenames(platform)
    assert {path.name for path in output.iterdir()} == set(
        expected_outputs.values()
    )
    assert manifest_sensor_keys <= set(manifest)
    assert absent_key not in manifest
    for key in manifest_sensor_keys:
        assert manifest[key].is_absolute()
        assert manifest[key].parent == output.resolve()

    report = _load_yaml(manifest["effective_profile_path"])
    expected_refs = {
        key: str(output.resolve() / filename)
        for key, filename in expected_outputs.items()
        if key != "effective_profile"
    }
    assert report["generated_configs"] == expected_refs
    assert set(manifest["robot_launch_arguments"]) == ROBOT_LAUNCH_ARGUMENT_KEYS


def test_sim_and_real_sensor_generation_stays_platform_isolated(
    runtime_tree, tmp_path
):
    core_dir = PACKAGE_ROOT.parents[1]
    protected = {
        *(TEMPLATE_DIR / filename for filename in rcc.TEMPLATE_FILENAMES.values()),
        *(
            TEMPLATE_DIR / filename
            for filenames in rcc.SENSOR_TEMPLATE_FILENAMES.values()
            for filename in filenames.values()
        ),
        core_dir
        / "robot/drivers/lidar_vanjee_722/vanjee_lidar_ros/config/vanjee_722.yaml",
        core_dir / "simulation/robot_gz_bringup/config/bridge.yaml",
        PACKAGE_ROOT / "launch/bringup.launch.py",
    }
    before = {path: path.read_bytes() for path in protected}

    sim_manifest = rcc.compile_runtime_configs(
        runtime_tree.config, tmp_path / "sim"
    )
    runtime_tree.set_bringup_value("platform", "real")
    real_manifest = rcc.compile_runtime_configs(
        runtime_tree.config, tmp_path / "real"
    )

    sim_outputs = {
        path.name: _load_yaml(path) for path in (tmp_path / "sim").iterdir()
    }
    real_outputs = {
        path.name: _load_yaml(path) for path in (tmp_path / "real").iterdir()
    }
    sim_report = sim_outputs[rcc.EFFECTIVE_PROFILE_FILENAME]
    real_report = real_outputs[rcc.EFFECTIVE_PROFILE_FILENAME]
    assert rcc.SENSOR_OUTPUT_FILENAMES["real"]["vanjee_lidar"] not in sim_outputs
    assert "vanjee_lidar_path" not in sim_manifest
    assert "vanjee_lidar" not in sim_report["generated_configs"]
    assert rcc.SENSOR_OUTPUT_FILENAMES["sim"]["lidar_adapter"] not in real_outputs
    assert "lidar_adapter_path" not in real_manifest
    assert "lidar_adapter" not in real_report["generated_configs"]

    def scalar_values(value):
        if isinstance(value, dict):
            for child in value.values():
                yield from scalar_values(child)
        elif isinstance(value, list):
            for child in value:
                yield from scalar_values(child)
        else:
            yield value

    real_hardware = real_report["profile"]["hardware"]["lidar"]
    sim_values = {
        value for data in sim_outputs.values() for value in scalar_values(data)
    }
    assert real_hardware["host_address"] not in sim_values
    assert real_hardware["device_address"] not in sim_values

    real_lidar = real_report["profile"]["sensors"]["lidar"]
    vanjee = real_outputs[
        rcc.SENSOR_OUTPUT_FILENAMES["real"]["vanjee_lidar"]
    ]["vanjee_lidar"]["ros__parameters"]
    assert vanjee["host_address"] == real_hardware["host_address"]
    assert vanjee["lidar_address"] == real_hardware["device_address"]
    assert vanjee["host_msop_port"] == real_hardware["host_msop_port"]
    assert vanjee["lidar_msop_port"] == real_hardware["device_msop_port"]
    assert vanjee["min_distance"] == real_lidar["min_range"]
    assert vanjee["max_distance"] == real_lidar["max_range"]
    assert vanjee["start_angle"] == degrees(
        real_lidar["horizontal_start_angle"]
    )
    assert vanjee["end_angle"] == degrees(real_lidar["horizontal_end_angle"])

    for manifest, expected_points in (
        (sim_manifest, 16 * 1800),
        (real_manifest, 32 * 1200),
    ):
        gate = _load_yaml(manifest["sensor_gate_path"])[
            "sensor_contract_gate"
        ]["ros__parameters"]
        assert gate["expected_points_per_scan"] == expected_points
        assert "expected_height" not in gate
        assert "expected_width" not in gate

    adapter = sim_outputs[
        rcc.SENSOR_OUTPUT_FILENAMES["sim"]["lidar_adapter"]
    ]["lidar_pointcloud_adapter"]["ros__parameters"]
    assert set(adapter) == {
        "use_sim_time",
        "input_topic",
        "output_topic",
        "output_frame",
        "scan_period",
    }
    assert {path: path.read_bytes() for path in protected} == before


def test_compile_runtime_configs_uses_integrated_renderer(
    runtime_tree, tmp_path, monkeypatch
):
    original = rcc._render_runtime_configs
    calls = []

    def render_with_marker(inputs):
        calls.append(inputs)
        generated = original(inputs)
        generated["web_ui"]["robot_web_ui"]["ros__parameters"][
            "host"
        ] = "renderer-used"
        return generated

    monkeypatch.setattr(rcc, "_render_runtime_configs", render_with_marker)

    manifest = rcc.compile_runtime_configs(
        runtime_tree.config, tmp_path / "output"
    )

    assert len(calls) == 1
    assert _load_yaml(manifest["web_ui_path"])["robot_web_ui"][
        "ros__parameters"
    ]["host"] == "renderer-used"


def test_compile_runtime_configs_uses_unique_private_temp_directories(runtime_tree):
    first = rcc.compile_runtime_configs(runtime_tree.config)
    second = rcc.compile_runtime_configs(runtime_tree.config)
    first_dir = first["effective_profile_path"].parent
    second_dir = second["effective_profile_path"].parent

    assert first_dir != second_dir
    for output in (first_dir, second_dir):
        assert output.name.startswith("system_bringup-runtime-")
        assert output.is_absolute()
        assert output.parent == Path(tempfile.gettempdir()).resolve()
        assert set(path.name for path in output.iterdir()) == set(
            rcc._output_filenames("sim").values()
        )


def test_explicit_runtime_output_preserves_unowned_files_across_compiles(
    runtime_tree, tmp_path
):
    output = tmp_path / "output"
    output.mkdir()
    keep = output / "keep.txt"
    keep.write_text("keep exactly", encoding="utf-8")

    first = rcc.compile_runtime_configs(runtime_tree.config, output)
    first_report = first["effective_profile_path"].read_bytes()
    runtime_tree.set_bringup_value("platform", "real")
    second = rcc.compile_runtime_configs(runtime_tree.config, output)

    assert first["effective_profile_path"] == second["effective_profile_path"]
    assert second["effective_profile_path"].read_bytes() != first_report
    assert keep.read_text(encoding="utf-8") == "keep exactly"
    assert set(path.name for path in output.iterdir()) == {
        "keep.txt",
        *rcc._output_filenames("sim").values(),
        *rcc._output_filenames("real").values(),
    }
    assert _temporary_files(output) == []


def test_runtime_compilation_does_not_modify_source_or_formal_files(
    runtime_tree, tmp_path
):
    core_dir = PACKAGE_ROOT.parents[1]
    protected = [
        *(TEMPLATE_DIR / filename for filename in rcc.TEMPLATE_FILENAMES.values()),
        *(
            TEMPLATE_DIR / filename
            for filenames in rcc.SENSOR_TEMPLATE_FILENAMES.values()
            for filename in filenames.values()
        ),
        *(
            runtime_tree.config.parent / "templates" / filename
            for filename in rcc.TEMPLATE_FILENAMES.values()
        ),
        *(
            runtime_tree.config.parent / "templates" / filename
            for filenames in rcc.SENSOR_TEMPLATE_FILENAMES.values()
            for filename in filenames.values()
        ),
        core_dir / "robot/robot_bringup/config/robot_controllers.yaml",
        PACKAGE_ROOT / "launch/bringup.launch.py",
    ]
    before = {path: path.read_bytes() for path in protected}

    rcc.compile_runtime_configs(runtime_tree.config, tmp_path / "output")

    assert {path: path.read_bytes() for path in protected} == before


def test_runtime_resources_resolve_from_config_for_absolute_and_relative_paths(
    runtime_tree, tmp_path, monkeypatch
):
    monkeypatch.chdir(tmp_path)
    absolute = rcc.compile_runtime_configs(
        runtime_tree.config.resolve(), tmp_path / "absolute-output"
    )
    relative_config = runtime_tree.config.relative_to(tmp_path)
    relative = rcc.compile_runtime_configs(
        relative_config, tmp_path / "relative-output"
    )

    assert _load_yaml(absolute["controllers_path"]) == _load_yaml(
        relative["controllers_path"]
    )
    assert absolute["bringup_config_path"] == relative["bringup_config_path"]


def test_in_memory_validation_failure_creates_no_completion_marker(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"

    def fail_validation(*args):
        raise ValueError("in-memory drift")

    monkeypatch.setattr(rcc, "_validate_generated_configs", fail_validation)

    with pytest.raises(ValueError, match="in-memory drift"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert not (output / rcc.EFFECTIVE_PROFILE_FILENAME).exists()
    assert not output.exists()


def test_sensor_in_memory_validation_failure_creates_no_output_directory(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"

    def fail_validation(*args):
        raise ValueError("sensor in-memory drift")

    monkeypatch.setattr(
        rcc, "_validate_sensor_generated_configs", fail_validation
    )

    with pytest.raises(ValueError, match="sensor in-memory drift"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert not output.exists()


def test_staging_write_failure_cleans_every_temporary_file(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    original = rcc.yaml.safe_dump
    calls = 0

    def fail_fourth_dump(*args, **kwargs):
        nonlocal calls
        calls += 1
        if calls == 4:
            raise OSError("staging write failed")
        return original(*args, **kwargs)

    monkeypatch.setattr(rcc.yaml, "safe_dump", fail_fourth_dump)

    with pytest.raises(OSError, match="staging write failed"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert not (output / rcc.EFFECTIVE_PROFILE_FILENAME).exists()
    assert _temporary_files(output) == []
    assert list(output.iterdir()) == []


def test_staged_reload_validation_failure_precedes_all_replacements(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    rcc.compile_runtime_configs(runtime_tree.config, output)
    before = {path: path.read_bytes() for path in output.iterdir()}
    runtime_tree.set_bringup_value("platform", "real")
    original = rcc._validate_generated_configs
    validations = 0

    def fail_second_validation(*args):
        nonlocal validations
        validations += 1
        if validations == 2:
            raise ValueError("staged drift")
        return original(*args)

    monkeypatch.setattr(rcc, "_validate_generated_configs", fail_second_validation)

    with pytest.raises(ValueError, match="staged drift"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert validations == 2
    assert {path: path.read_bytes() for path in output.iterdir()} == before
    assert _temporary_files(output) == []


def test_sensor_staged_reload_failure_precedes_all_replacements(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    original_load = rcc._load_staged_yaml
    original_replace = rcc.os.replace
    replaced = []

    def fail_sensor_load(path, label):
        if label == "lidar_adapter":
            raise ValueError("sensor staged drift")
        return original_load(path, label)

    def record_replace(source, destination):
        replaced.append(Path(destination).name)
        return original_replace(source, destination)

    monkeypatch.setattr(rcc, "_load_staged_yaml", fail_sensor_load)
    monkeypatch.setattr(rcc.os, "replace", record_replace)

    with pytest.raises(ValueError, match="sensor staged drift"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert replaced == []
    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


def test_fast_lio_staged_reload_failure_precedes_all_replacements(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    original_load = rcc._load_staged_yaml
    original_replace = rcc.os.replace
    replaced = []

    def corrupt_fast_lio_load(path, label):
        loaded = original_load(path, label)
        if label == "fast_lio":
            loaded["/**"]["ros__parameters"]["preprocess"]["scan_line"] = 0
        return loaded

    def record_replace(source, destination):
        replaced.append(Path(destination).name)
        return original_replace(source, destination)

    monkeypatch.setattr(rcc, "_load_staged_yaml", corrupt_fast_lio_load)
    monkeypatch.setattr(rcc.os, "replace", record_replace)

    with pytest.raises(ValueError, match="generated fast_lio mismatch"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert replaced == []
    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


def test_gicp_staged_reload_equal_value_wrong_type_precedes_all_replacements(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    runtime_tree.set_bringup_value("platform", "real")
    original_load = rcc._load_staged_yaml
    original_replace = rcc.os.replace
    replaced = []

    def corrupt_gicp_load(path, label):
        loaded = original_load(path, label)
        if label == "gicp":
            loaded["gicp_localization"]["ros__parameters"][
                "use_sim_time"
            ] = 0
        return loaded

    def record_replace(source, destination):
        replaced.append(Path(destination).name)
        return original_replace(source, destination)

    monkeypatch.setattr(rcc, "_load_staged_yaml", corrupt_gicp_load)
    monkeypatch.setattr(rcc.os, "replace", record_replace)

    with pytest.raises(
        ValueError, match="generated gicp does not match template plus overrides"
    ):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert replaced == []
    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


@pytest.mark.parametrize(
    "corrupt",
    [
        lambda params: params.__setitem__("N_SCAN", float(params["N_SCAN"])),
        lambda params: params["extrinsicRot"].__setitem__(
            0, int(params["extrinsicRot"][0])
        ),
    ],
    ids=["integer-as-equal-float", "double-array-element-as-equal-int"],
)
def test_lio_sam_staged_reload_equal_value_wrong_types_precede_all_replacements(
    runtime_tree, tmp_path, monkeypatch, corrupt
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    original_load = rcc._load_staged_yaml
    original_replace = rcc.os.replace
    replaced = []

    def corrupt_lio_sam_load(path, label):
        loaded = original_load(path, label)
        if label == "lio_sam":
            corrupt(loaded["/**"]["ros__parameters"])
        return loaded

    def record_replace(source, destination):
        replaced.append(Path(destination).name)
        return original_replace(source, destination)

    monkeypatch.setattr(rcc, "_load_staged_yaml", corrupt_lio_sam_load)
    monkeypatch.setattr(rcc.os, "replace", record_replace)

    with pytest.raises(
        ValueError, match="generated lio_sam does not match template plus overrides"
    ):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert replaced == []
    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


def test_staged_report_reference_drift_precedes_all_replacements_and_return(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    runtime_tree.set_profile_value(
        "sim", ("motion", "max_angular_velocity"), 1.7
    )
    original_load = rcc._load_staged_yaml
    original_replace = rcc.os.replace
    replaced = []
    returned = []

    def corrupt_report_load(path, label):
        loaded = original_load(path, label)
        if label == "effective_profile":
            loaded["source_profile"] = "/tmp/wrong-profile.yaml"
        return loaded

    def record_replace(source, destination):
        replaced.append(Path(destination).name)
        return original_replace(source, destination)

    def compile_and_record():
        returned.append(
            rcc.compile_runtime_configs(runtime_tree.config, output)
        )

    monkeypatch.setattr(rcc, "_load_staged_yaml", corrupt_report_load)
    monkeypatch.setattr(rcc.os, "replace", record_replace)

    with pytest.raises(
        ValueError, match="staged effective_profile does not match in-memory report"
    ):
        compile_and_record()

    assert replaced == []
    assert returned == []
    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


def test_staged_report_equal_value_wrong_type_precedes_replacements_and_return(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    runtime_tree.set_profile_value(
        "sim", ("motion", "max_angular_velocity"), 1.7
    )
    original_load = rcc._load_staged_yaml
    original_replace = rcc.os.replace
    replaced = []
    returned = []

    def corrupt_report_load(path, label):
        loaded = original_load(path, label)
        if label == "effective_profile":
            motion = loaded["profile"]["motion"]
            motion["max_linear_velocity"] = int(
                motion["max_linear_velocity"]
            )
        return loaded

    def record_replace(source, destination):
        replaced.append(Path(destination).name)
        return original_replace(source, destination)

    def compile_and_record():
        returned.append(
            rcc.compile_runtime_configs(runtime_tree.config, output)
        )

    monkeypatch.setattr(rcc, "_load_staged_yaml", corrupt_report_load)
    monkeypatch.setattr(rcc.os, "replace", record_replace)

    with pytest.raises(
        ValueError, match="staged effective_profile does not match in-memory report"
    ):
        compile_and_record()

    assert replaced == []
    assert returned == []
    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


def test_mid_replace_failure_does_not_update_existing_completion_marker(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    runtime_tree.set_bringup_value("platform", "real")
    original = rcc.os.replace
    replaced = []

    def fail_second_replace(source, destination):
        replaced.append(Path(destination).name)
        if len(replaced) == 2:
            raise OSError("replace interrupted")
        return original(source, destination)

    monkeypatch.setattr(rcc.os, "replace", fail_second_replace)

    with pytest.raises(OSError, match="replace interrupted"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert replaced == [
        rcc.COMMON_OUTPUT_FILENAMES["controllers"],
        rcc.COMMON_OUTPUT_FILENAMES["web_ui"],
    ]
    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


def test_sensor_replace_failure_does_not_update_existing_completion_marker(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    original = rcc.os.replace

    def fail_sensor_replace(source, destination):
        if Path(destination).name == rcc.SENSOR_OUTPUT_FILENAMES["sim"][
            "lidar_adapter"
        ]:
            raise OSError("sensor replace interrupted")
        return original(source, destination)

    monkeypatch.setattr(rcc.os, "replace", fail_sensor_replace)

    with pytest.raises(OSError, match="sensor replace interrupted"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


def test_effective_report_is_replaced_last_before_manifest_is_returned(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    original = rcc.os.replace
    replaced = []

    def record_replace(source, destination):
        replaced.append(Path(destination).name)
        return original(source, destination)

    monkeypatch.setattr(rcc.os, "replace", record_replace)

    manifest = rcc.compile_runtime_configs(runtime_tree.config, output)

    assert replaced == list(rcc._output_filenames("sim").values())
    assert manifest["effective_profile_path"].exists()
    assert _temporary_files(output) == []


def test_effective_report_replace_failure_keeps_previous_completion_marker(
    runtime_tree, tmp_path, monkeypatch
):
    output = tmp_path / "output"
    previous = rcc.compile_runtime_configs(runtime_tree.config, output)
    marker = previous["effective_profile_path"]
    marker_before = marker.read_bytes()
    runtime_tree.set_profile_value(
        "sim", ("motion", "max_linear_velocity"), 0.7
    )
    original = rcc.os.replace

    def fail_report_replace(source, destination):
        if Path(destination).name == rcc.EFFECTIVE_PROFILE_FILENAME:
            raise OSError("report replace interrupted")
        return original(source, destination)

    monkeypatch.setattr(rcc.os, "replace", fail_report_replace)

    with pytest.raises(OSError, match="report replace interrupted"):
        rcc.compile_runtime_configs(runtime_tree.config, output)

    assert marker.read_bytes() == marker_before
    assert _temporary_files(output) == []


def test_runtime_cli_prints_one_absolute_report_path(runtime_tree, tmp_path, capsys):
    output = tmp_path / "output"

    result = rcc.main(
        [
            "--bringup-config",
            str(runtime_tree.config),
            "--output-dir",
            str(output),
        ]
    )

    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert captured.out == (
        f"{(output / rcc.EFFECTIVE_PROFILE_FILENAME).resolve()}\n"
    )


def test_runtime_compiler_module_entrypoint_generates_configs(
    runtime_tree, tmp_path
):
    output = tmp_path / "module-output"

    result = subprocess.run(
        [
            sys.executable,
            "-m",
            "system_bringup.runtime_config_compiler",
            "--bringup-config",
            str(runtime_tree.config.resolve()),
            "--output-dir",
            str(output.resolve()),
        ],
        cwd=PACKAGE_ROOT,
        capture_output=True,
        text=True,
        check=False,
    )

    assert result.returncode == 0
    assert result.stderr == ""
    assert result.stdout == f"{(output / rcc.EFFECTIVE_PROFILE_FILENAME).resolve()}\n"
    assert {path.name for path in output.iterdir()} == set(
        rcc._output_filenames("sim").values()
    )


def test_runtime_cli_help_remains_standard(capsys):
    with pytest.raises(SystemExit) as exc_info:
        rcc.main(["--help"])

    captured = capsys.readouterr()
    assert exc_info.value.code == 0
    assert captured.err == ""
    assert captured.out.startswith("usage:")
    assert "--bringup-config" in captured.out
    assert "--output-dir" in captured.out


def test_runtime_cli_reports_one_actionable_error_without_traceback(
    runtime_tree, capsys
):
    runtime_tree.set_bringup_value("platform", "invalid")

    with pytest.raises(SystemExit) as exc_info:
        rcc.main(["--bringup-config", str(runtime_tree.config)])

    captured = capsys.readouterr()
    assert exc_info.value.code == 2
    assert captured.out == ""
    assert captured.err == (
        "compile_runtime_configs: "
        f"{runtime_tree.config.resolve()}: platform must be 'sim' or 'real'\n"
    )
    assert "Traceback" not in captured.err


@pytest.mark.parametrize("mode", [[], {}], ids=["list", "mapping"])
def test_runtime_cli_reports_non_string_mode_as_one_actionable_line(
    runtime_tree, mode, capsys
):
    runtime_tree.set_bringup_value("mode", mode)

    with pytest.raises(SystemExit) as exc_info:
        rcc.main(["--bringup-config", str(runtime_tree.config)])

    captured = capsys.readouterr()
    assert exc_info.value.code == 2
    assert captured.out == ""
    assert captured.err == (
        "compile_runtime_configs: "
        "bringup config mode must be 'mapping' or 'navigation'\n"
    )
    assert "Traceback" not in captured.err


def test_runtime_cli_missing_required_argument_is_one_line(capsys):
    with pytest.raises(SystemExit) as exc_info:
        rcc.main([])

    captured = capsys.readouterr()
    assert exc_info.value.code == 2
    assert captured.out == ""
    assert captured.err == (
        "compile_runtime_configs: the following arguments are required: "
        "--bringup-config\n"
    )
    assert "Traceback" not in captured.err


def test_runtime_cli_unknown_argument_is_one_line(runtime_tree, capsys):
    with pytest.raises(SystemExit) as exc_info:
        rcc.main(
            [
                "--bringup-config",
                str(runtime_tree.config),
                "--unexpected",
            ]
        )

    captured = capsys.readouterr()
    assert exc_info.value.code == 2
    assert captured.out == ""
    assert captured.err == (
        "compile_runtime_configs: unrecognized arguments: --unexpected\n"
    )
    assert "Traceback" not in captured.err


def test_formal_bringup_imports_only_the_integrated_runtime_compiler():
    launch_source = (PACKAGE_ROOT / "launch/bringup.launch.py").read_text(
        encoding="utf-8"
    )

    for name in (
        "profile_compiler",
        "compile_profile",
    ):
        assert name not in launch_source
    assert launch_source.count("compile_runtime_configs(") == 1
    assert (
        "from system_bringup.runtime_config_compiler import "
        "compile_runtime_configs"
    ) in launch_source
