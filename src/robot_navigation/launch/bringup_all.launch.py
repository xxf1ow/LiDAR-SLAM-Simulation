#!/usr/bin/env python3
"""nav2 阶段二一键启动：单终端拉起全栈(sim + FAST-LIO + GICP + nav2)。

TimerAction 错峰，留建图/定位预热时间；一个 Ctrl-C 全清。
排查问题时仍可手动分四终端跑各子 launch(本文件只是把它们串起来)。

参数转发要点(踩过的坑)：
  - fast_lio 的 config_file 传【裸名】(它内部会自己 PathJoinSubstitution 拼 config 目录)；
  - stage2 的 params_file / nav_params_file 都传【绝对路径】(用 get_package_share_directory)，
    且两个都要转发——params_file 给 costmap/planner/controller，nav_params_file 给
    behavior/bt/waypoint(漏传则 behavior_server 吃默认 global_frame=odom)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_pkg = get_package_share_directory('robot_navigation')
    sim_pkg = get_package_share_directory('robot_gazebo')
    lio_pkg = get_package_share_directory('fast_lio')
    gicp_pkg = get_package_share_directory('gicp_localization')

    prior_map_path = LaunchConfiguration('prior_map_path')
    map_yaml = LaunchConfiguration('map')
    lio_config_file = LaunchConfiguration('lio_config_file')   # 裸名，传给 fast_lio
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')           # 绝对路径，costmap/planner/controller
    nav_params_file = LaunchConfiguration('nav_params_file')   # 绝对路径，behavior/bt/waypoint
    delay_lio = LaunchConfiguration('delay_lio')
    delay_gicp = LaunchConfiguration('delay_gicp')
    delay_nav = LaunchConfiguration('delay_nav')

    def inc(pkg, rel, **launch_args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', rel)),
            launch_arguments=launch_args.items(),
        )

    sim = inc(sim_pkg, 'robot_sim.launch.py',
              use_sim_time='true', use_teleop='false')
    lio = inc(lio_pkg, 'mapping.launch.py',
              config_file=lio_config_file, use_sim_time='true', rviz='false')
    gicp = inc(gicp_pkg, 'localization.launch.py',
               prior_map_path=prior_map_path)
    nav = inc(nav_pkg, 'stage2_navigation.launch.py',
              map=map_yaml, use_rviz=use_rviz,
              params_file=params_file, nav_params_file=nav_params_file)

    return LaunchDescription([
        DeclareLaunchArgument('prior_map_path', default_value='~/result/GlobalMap.pcd'),
        DeclareLaunchArgument('map', default_value='~/result/map.yaml'),
        DeclareLaunchArgument('lio_config_file', default_value='gazebo_velodyne.yaml'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_pkg, 'config', 'nav2_costmaps.yaml')),
        DeclareLaunchArgument(
            'nav_params_file',
            default_value=os.path.join(nav_pkg, 'config', 'nav2_navigation.yaml')),
        DeclareLaunchArgument('delay_lio', default_value='20.0'),
        DeclareLaunchArgument('delay_gicp', default_value='8.0'),
        DeclareLaunchArgument('delay_nav', default_value='12.0'),
        sim,
        TimerAction(period=delay_lio, actions=[lio]),
        TimerAction(period=delay_gicp, actions=[gicp]),
        TimerAction(period=delay_nav, actions=[nav]),
    ])
