"""nav2_navigation.yaml 结构校验（编辑机可跑；防手滑写错节点/帧/插件结构）。

阶段二无 ROS 集成测试（本机无 ROS）；本测试只断言三块 params 的关键结构与
本项目特定适配值（odom_topic=/odom、robot_base_frame=base_footprint、
behavior_server.global_frame=camera_init）正确，对应 spec §5/§11。
"""
import os

import yaml

CFG = os.path.join(os.path.dirname(__file__), "..", "config", "nav2_navigation.yaml")


def _load():
    with open(CFG, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def test_has_three_nodes():
    cfg = _load()
    assert set(["bt_navigator", "behavior_server", "waypoint_follower"]).issubset(cfg)


def test_bt_navigator_frames_and_odom():
    p = _load()["bt_navigator"]["ros__parameters"]
    assert p["global_frame"] == "map"
    assert p["robot_base_frame"] == "base_footprint"
    assert p["odom_topic"] == "/odom"
    libs = p["plugin_lib_names"]
    assert "nav2_navigate_to_pose_action_bt_node" in libs
    assert "nav2_navigate_through_poses_action_bt_node" in libs


def test_behavior_server_frame_and_plugins():
    p = _load()["behavior_server"]["ros__parameters"]
    assert p["global_frame"] == "camera_init"
    assert p["robot_base_frame"] == "base_footprint"
    assert p["behavior_plugins"] == ["spin", "backup", "drive_on_heading", "wait"]
    assert p["spin"]["plugin"] == "nav2_behaviors/Spin"


def test_waypoint_follower_executor():
    p = _load()["waypoint_follower"]["ros__parameters"]
    assert p["waypoint_task_executor_plugin"] == "wait_at_waypoint"
    assert p["wait_at_waypoint"]["plugin"] == "nav2_waypoint_follower::WaitAtWaypoint"
