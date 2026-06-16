"""共享上层 SLAM/定位/导航栈(被 sim/real 内部 include,不直接用命令行调)。

mode=navigation:fast_lio 里程计 → (错峰) gicp 定位 → (错峰) nav2。
mode=mapping   :lio_sam 建图(与导航互斥,产先验图)。
参数由父级 launch 透传(launch_arguments),本文件用 DeclareLaunchArgument 接收。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _stack(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    use_sim = LaunchConfiguration("use_sim_time").perform(context)
    fast_cfg = LaunchConfiguration("fast_lio_config").perform(context)
    prior = LaunchConfiguration("prior_map_path").perform(context)
    nav_map = LaunchConfiguration("nav_map").perform(context)
    d_gicp = float(LaunchConfiguration("delay_gicp").perform(context))
    d_nav = float(LaunchConfiguration("delay_nav").perform(context))

    if mode == "mapping":
        return [_inc("lio_sam", "launch/run.launch.py")]
    if mode == "navigation":
        return [
            _inc("fast_lio", "launch/mapping.launch.py",
                 {"config_file": fast_cfg, "use_sim_time": use_sim, "rviz": "false"}),
            TimerAction(period=d_gicp, actions=[
                _inc("gicp_localization", "launch/localization.launch.py",
                     {"prior_map_path": prior})]),
            TimerAction(period=d_gicp + d_nav, actions=[
                _inc("robot_navigation", "launch/navigation.launch.py",
                     {"map": nav_map, "use_rviz": "true"})]),
        ]
    raise RuntimeError("未知 mode='%s'(应为 navigation|mapping)" % mode)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="navigation"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("fast_lio_config", default_value="gazebo_velodyne.yaml"),
        DeclareLaunchArgument("prior_map_path", default_value="~/result/GlobalMap.pcd"),
        DeclareLaunchArgument("nav_map", default_value="~/result/factory_map.yaml"),
        DeclareLaunchArgument("delay_gicp", default_value="8.0"),
        DeclareLaunchArgument("delay_nav", default_value="12.0"),
        OpaqueFunction(function=_stack),
    ])
