"""仿真全栈启动(Gz Harmonic)。

不接收命令行参数:所有下游转发参数在下方"配置"常量块,改参数=改本文件。
流程:① 一致性闸门(失败即中止、不拉任何节点)② robot_gz 仿真底层
③ 错峰起 slam_stack(mode=navigation 默认:fast_lio→gicp→nav2;mode=mapping:lio_sam)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from system_bringup import consistency_check
from system_bringup.ready_gate import ready_gate

# ===== 配置:改这里,不用命令行 =====
MODE = "navigation"              # "navigation" | "mapping"
GUI = "true"
RVIZ = "false"
WORLD = "factory.sdf"
SPAWN_X, SPAWN_Y, SPAWN_Z = "4.0", "0.0", "0.05"
FAST_LIO_CONFIG = "gazebo_velodyne.yaml"
PRIOR_MAP_PATH = "~/result/GlobalMap.pcd"
NAV_MAP = "~/result/factory_map.yaml"
SETTLING = 20.0                   # 就绪(话题出现)后再等的稳定秒数:给 controller 激活/gicp 收敛余量
# ====================================


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _bringup(context, *args, **kwargs):
    failures = consistency_check.run(consistency_check.find_repo_root())  # 源码根自动检测(上溯 core/bringup/system_bringup 或 .git)
    if failures:
        raise RuntimeError("一致性闸门未通过(已中止,未启动任何节点):\n" + "\n".join(failures))

    robot_gz = _inc("robot_gz_bringup", "launch/robot_gz.launch.py",
                    {"gui": GUI, "rviz": RVIZ, "world": WORLD,
                     "spawn_x": SPAWN_X, "spawn_y": SPAWN_Y, "spawn_z": SPAWN_Z})
    slam_stack = _inc(
        "system_bringup", "launch/slam_stack.launch.py",
        {"mode": MODE, "use_sim_time": "true",
         "fast_lio_config": FAST_LIO_CONFIG, "prior_map_path": PRIOR_MAP_PATH,
         "nav_map": NAV_MAP, "settling": str(SETTLING)})
    # 等 lidar(/points_raw) + controller(/joint_states) 都就绪,再 settling 秒后起上层栈
    return [robot_gz] + ready_gate(["/points_raw", "/joint_states"], 300.0,
                                   "robot_gz(lidar+controller)", [slam_stack], settling=SETTLING)


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_bringup)])
