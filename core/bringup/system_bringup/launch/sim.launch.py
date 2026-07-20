"""仿真全栈启动(Gz Harmonic)。可变参数在 config/bringup.yaml(改它不用 rebuild)。

不接收命令行参数:流程 = ① 一致性闸门(失败即中止)② robot_gz 仿真底层
③ 错峰起 slam_stack(mode=navigation 默认:fast_lio→gicp→nav2;mode=mapping:lio_sam)。
切 mode 改 config/bringup.yaml 的 mode 字段即可,不用 rebuild system_bringup。
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
    sim = cfg.get("sim", {})
    flow = lambda m: LogInfo(msg="======== [system_bringup] %s" % m)

    failures = consistency_check.run(repo_root)
    if failures:
        raise RuntimeError("一致性闸门未通过(已中止,未启动任何节点):\n" + "\n".join(failures))

    robot_gz = _inc("robot_gz_bringup", "launch/robot_gz.launch.py",
                    {"gui": sim.get("gui", "true"), "rviz": sim.get("rviz", "false"),
                     "world": sim.get("world", "factory.sdf"),
                     "spawn_x": sim.get("spawn_x", "4.0"),
                     "spawn_y": sim.get("spawn_y", "0.0"),
                     "spawn_z": sim.get("spawn_z", "0.05")})
    slam_stack = _inc(
        "system_bringup", "launch/slam_stack.launch.py",
        {"mode": cfg["mode"], "use_sim_time": "true",
         "fast_lio_config": cfg["fast_lio_config"], "prior_map_path": cfg["prior_map_path"],
         "nav_map": cfg["nav_map"], "settling": str(cfg["settling"])})
    # 等 lidar(/points_raw) + controller(/joint_states) 都就绪,再 settling 秒后起上层栈
    return [
        flow("一致性闸门通过 | MODE=%s | config=源码 bringup.yaml(改它不 rebuild)" % cfg["mode"]),
        flow("① 起 robot_gz 仿真底层 → ready_gate 等 /points_raw+/joint_states + settling %ss 后起 slam_stack" % cfg["settling"]),
        robot_gz,
    ] + ready_gate(["/points_raw", "/joint_states"], 300.0,
                   "robot_gz(lidar+controller)", [slam_stack], settling=cfg["settling"])


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_bringup)])
