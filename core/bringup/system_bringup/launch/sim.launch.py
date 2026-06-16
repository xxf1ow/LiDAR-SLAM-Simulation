"""仿真全栈启动(Gz Harmonic)。

不接收命令行参数:所有下游转发参数在下方"配置"常量块,改参数=改本文件。
流程:① 一致性闸门(失败即中止、不拉任何节点)② robot_gz 仿真底层
③ 错峰起 slam_stack(mode=navigation 默认:fast_lio→gicp→nav2;mode=mapping:lio_sam)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from system_bringup import consistency_check

# ===== 配置:改这里,不用命令行 =====
MODE = "navigation"              # "navigation" | "mapping"
GUI = "true"
RVIZ = "false"
WORLD = "factory.sdf"
SPAWN_X, SPAWN_Y, SPAWN_Z = "4.0", "0.0", "0.05"
REPO_ROOT = "~/LiDAR-SLAM-Simulation"          # 一致性检查读源树用
FAST_LIO_CONFIG = "gazebo_velodyne.yaml"
PRIOR_MAP_PATH = "~/result/GlobalMap.pcd"
NAV_MAP = "~/result/factory_map.yaml"
DELAY_UPPER = 20.0               # 仿真底层起好到拉上层的间隔
DELAY_GICP = 8.0                 # fast_lio 到 gicp
DELAY_NAV = 12.0                 # gicp 到 nav2
# ====================================


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _bringup(context, *args, **kwargs):
    failures = consistency_check.run(os.path.expanduser(REPO_ROOT))
    if failures:
        raise RuntimeError("一致性闸门未通过(已中止,未启动任何节点):\n" + "\n".join(failures))

    robot_gz = _inc("robot_gz_bringup", "launch/robot_gz.launch.py",
                    {"gui": GUI, "rviz": RVIZ, "world": WORLD,
                     "spawn_x": SPAWN_X, "spawn_y": SPAWN_Y, "spawn_z": SPAWN_Z})
    upper = TimerAction(period=DELAY_UPPER, actions=[_inc(
        "system_bringup", "launch/slam_stack.launch.py",
        {"mode": MODE, "use_sim_time": "true",
         "fast_lio_config": FAST_LIO_CONFIG, "prior_map_path": PRIOR_MAP_PATH,
         "nav_map": NAV_MAP, "delay_gicp": str(DELAY_GICP), "delay_nav": str(DELAY_NAV)})])
    return [robot_gz, upper]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_bringup)])
