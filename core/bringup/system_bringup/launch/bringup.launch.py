"""全栈启动。config/bringup.yaml 选 platform(sim/real) + mode(nav/map),改 config 不 rebuild。

唯一入口:ros2 launch system_bringup bringup.launch.py
流程:① 一致性闸门(失败即中止)② 底层(platform=sim: robot_gz; platform=real: 骨架 TODO)
③ ready_gate 等 lidar(+ sim controller)④ slam_stack(mode=navigation: fast_lio→gicp→nav2;
mode=mapping: lio_sam)。
切 platform/mode 改 config/bringup.yaml 顶层两行即可,不用 rebuild。
use_sim_time 从 platform 推断(sim=true, real=false),不在 config。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from system_bringup import consistency_check
from system_bringup.ready_gate import ready_gate


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _bringup(context, *args, **kwargs):
    repo_root = consistency_check.find_repo_root()
    cfg = consistency_check.load_bringup_config(repo_root)
    platform = cfg["platform"]          # sim | real
    mode = cfg["mode"]                  # navigation | mapping
    use_sim = "true" if platform == "sim" else "false"
    stack_cfg = cfg["slam_stack"]
    flow = lambda m: LogInfo(msg="======== [system_bringup] %s" % m)

    failures = consistency_check.run(repo_root)
    if failures:
        raise RuntimeError("一致性闸门未通过(已中止,未启动任何节点):\n" + "\n".join(failures))

    slam_stack = _inc(
        "system_bringup", "launch/slam_stack.launch.py",
        {"mode": mode, "use_sim_time": use_sim,
         "fast_lio_config": stack_cfg["fast_lio"]["config"],
         "prior_map_path": stack_cfg["gicp_localization"]["prior_map_path"],
         "nav_map": stack_cfg["robot_navigation"]["map"],
         "settling": str(stack_cfg["settling"])})

    if platform == "sim":
        gz = cfg["robot_gz"]
        base = _inc("robot_gz_bringup", "launch/robot_gz.launch.py",
                    {"gui": gz.get("gui", "true"), "rviz": gz.get("rviz", "false"),
                     "world": gz.get("world", "factory.sdf"),
                     "spawn_x": gz.get("spawn_x", "4.0"),
                     "spawn_y": gz.get("spawn_y", "0.0"),
                     "spawn_z": gz.get("spawn_z", "0.05")})
        flow_log = [
            flow("一致性闸门通过 | platform=sim | mode=%s | config=源码 bringup.yaml(改它不 rebuild)" % mode),
            flow("① 起 robot_gz 仿真底层 → ready_gate 等 /points_raw+/joint_states + settling %ss 后起 slam_stack" % stack_cfg["settling"]),
        ]
        return flow_log + [base] + ready_gate(
            ["/points_raw", "/joint_states"], 300.0, "robot_gz(lidar+controller)",
            [slam_stack], settling=stack_cfg["settling"])

    if platform == "real":
        actions = [
            flow("一致性闸门通过 | platform=real | mode=%s | config=源码 bringup.yaml(改它不 rebuild)" % mode),
            flow("① 真机底层未实现(骨架);上真机时在此 include robot_bringup(真实硬件)+ velodyne/imu 驱动"),
        ]
        # TODO(真机): 取消下行注释并补真机底层 ——
        #   robot_bringup robot.launch.py use_mock_hardware:=false + 真实 velodyne/imu 驱动节点。
        # actions.append(_inc("robot_bringup", "launch/robot.launch.py",
        #     {"use_mock_hardware": str(cfg["robot_bringup"].get("use_mock_hardware", False))}))
        actions += ready_gate("/points_raw", 300.0, "真机 velodyne→/points_raw",
                              [slam_stack], settling=stack_cfg["settling"])
        return actions

    raise RuntimeError("未知 platform='%s'(应为 sim|real)" % platform)


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_bringup)])
