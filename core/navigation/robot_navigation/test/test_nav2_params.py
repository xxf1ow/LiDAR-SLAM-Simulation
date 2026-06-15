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


def test_local_voxel_origin_z_is_negative_one():
    p = _load()
    vox = p["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]
    assert vox["origin_z"] == -1.0


def test_local_obstacle_source_is_cloud_registered():
    p = _load()
    pc = p["local_costmap"]["local_costmap"]["ros__parameters"]["voxel_layer"]["pointcloud"]
    assert pc["topic"] == "/cloud_registered"
    assert pc["sensor_frame"] == "body"


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
