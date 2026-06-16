"""真机全栈启动(本期仅骨架,未实现底层驱动)。

与 sim.launch.py 同构、同样头部常量块、无命令行参数:
① 一致性闸门 → ② 真机底层(robot_bringup 真实硬件 + 真实 velodyne/imu 驱动)
→ ③ slam_stack(mode)。本期真机驱动包尚不存在,② 留 TODO;上真机时填底层 include
即可,闸门与上层栈已接好。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from system_bringup import consistency_check

# ===== 配置:改这里,不用命令行 =====
MODE = "navigation"              # "navigation" | "mapping"
REPO_ROOT = "~/LiDAR-SLAM-Simulation"
FAST_LIO_CONFIG = "gazebo_velodyne.yaml"   # 真机另备 real 配置时改此
PRIOR_MAP_PATH = "~/result/GlobalMap.pcd"
NAV_MAP = "~/result/factory_map.yaml"
DELAY_UPPER = 5.0
DELAY_GICP = 8.0
DELAY_NAV = 12.0
# ====================================


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _bringup(context, *args, **kwargs):
    failures = consistency_check.run(os.path.expanduser(REPO_ROOT))
    if failures:
        raise RuntimeError("一致性闸门未通过(已中止):\n" + "\n".join(failures))

    actions = [LogInfo(msg="[real.launch] 真机底层未实现(骨架);仅演示闸门 + 上层栈接线。"
                           "上真机时在此 include robot_bringup(真实硬件)+ velodyne/imu 驱动。")]
    # TODO(真机): 取消下行注释并补真机底层 ——
    #   robot_bringup robot.launch.py use_mock_hardware:=false + 真实 velodyne/imu 驱动节点。
    # actions.append(_inc("robot_bringup", "launch/robot.launch.py", {"use_mock_hardware": "false"}))

    actions.append(TimerAction(period=DELAY_UPPER, actions=[_inc(
        "system_bringup", "launch/slam_stack.launch.py",
        {"mode": MODE, "use_sim_time": "false",
         "fast_lio_config": FAST_LIO_CONFIG, "prior_map_path": PRIOR_MAP_PATH,
         "nav_map": NAV_MAP, "delay_gicp": str(DELAY_GICP), "delay_nav": str(DELAY_NAV)})]))
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_bringup)])
