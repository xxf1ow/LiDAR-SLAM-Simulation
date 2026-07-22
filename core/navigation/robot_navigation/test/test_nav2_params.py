"""nav2_params.yaml 关键 frame/源/拓扑校验(本机 pyyaml 可跑)。"""
import os

import yaml

HERE = os.path.dirname(__file__)
PARAMS = os.path.join(HERE, "..", "config", "nav2_params.yaml")


def _load():
    with open(PARAMS, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_global_costmap_frame_is_map():
    p = _load()
    assert p["global_costmap"]["global_costmap"]["ros__parameters"]["global_frame"] == "map"


def test_local_costmap_frame_is_camera_init():
    p = _load()
    assert p["local_costmap"]["local_costmap"]["ros__parameters"]["global_frame"] == "camera_init"


def test_behavior_server_frame_is_camera_init():
    p = _load()
    assert p["behavior_server"]["ros__parameters"]["global_frame"] == "camera_init"


def test_local_costmap_has_stvl_layer():
    p = _load()
    plugins = p["local_costmap"]["local_costmap"]["ros__parameters"]["plugins"]
    assert plugins == ["stvl_layer", "inflation_layer"]


def test_local_stvl_plugin_and_combination():
    p = _load()
    stvl = p["local_costmap"]["local_costmap"]["ros__parameters"]["stvl_layer"]
    assert stvl["plugin"] == "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
    assert stvl["combination_method"] == 1


def test_local_stvl_sources_are_cloud_registered():
    p = _load()
    stvl = p["local_costmap"]["local_costmap"]["ros__parameters"]["stvl_layer"]
    assert stvl["pointcloud_mark"]["topic"] == "/cloud_registered"
    assert stvl["pointcloud_clear"]["topic"] == "/cloud_registered"
    assert stvl["pointcloud_mark"]["sensor_frame"] == "body"


def test_odom_topic_is_base_controller_odom():
    p = _load()
    assert p["controller_server"]["ros__parameters"]["odom_topic"] == "/base_controller/odom"
    assert p["bt_navigator"]["ros__parameters"]["odom_topic"] == "/base_controller/odom"


def test_planner_is_smac_hybrid():
    p = _load()
    gb = p["planner_server"]["ros__parameters"]["GridBased"]
    assert gb["plugin"] == "nav2_smac_planner/SmacPlannerHybrid"


def test_controller_is_mppi():
    p = _load()
    fp = p["controller_server"]["ros__parameters"]["FollowPath"]
    assert fp["plugin"] == "nav2_mppi_controller::MPPIController"


def test_global_costmap_has_stvl_layer():
    p = _load()
    plugins = p["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]
    assert plugins == ["static_layer", "stvl_layer", "inflation_layer"]


def test_global_stvl_plugin_and_combination():
    p = _load()
    stvl = p["global_costmap"]["global_costmap"]["ros__parameters"]["stvl_layer"]
    assert stvl["plugin"] == "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
    assert stvl["combination_method"] == 1


def test_global_stvl_sources_are_cloud_registered():
    p = _load()
    stvl = p["global_costmap"]["global_costmap"]["ros__parameters"]["stvl_layer"]
    assert stvl["pointcloud_mark"]["topic"] == "/cloud_registered"
    assert stvl["pointcloud_clear"]["topic"] == "/cloud_registered"
