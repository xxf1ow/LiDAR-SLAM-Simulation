#!/usr/bin/env python3
"""nav2 阶段二一键启动：单终端拉起全栈(sim + FAST-LIO + GICP + nav2)。

TimerAction 错峰，留建图/定位预热时间；一个 Ctrl-C 全清。
排查问题时仍可手动分四终端跑各子 launch(本文件只是把它们串起来)。

子 launch 参数适配：
  - robot_sim: use_teleop:=false(nav2 独占 /cmd_vel)、use_sim_time:=true
  - fast_lio:  config_file:=gazebo_velodyne.yaml use_sim_time:=true rviz:=false(只留 stage2 RViz)
  - gicp:      prior_map_path:=<arg>
  - stage2:    map:=<arg>、use_rviz:=<arg>、params_file:=<arg>
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
    config_file = LaunchConfiguration('config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
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
              config_file=config_file, use_sim_time='true', rviz='false')
    gicp = inc(gicp_pkg, 'localization.launch.py',
               prior_map_path=prior_map_path)
    nav = inc(nav_pkg, 'stage2_navigation.launch.py',
              map=map_yaml, use_rviz=use_rviz, params_file=params_file)

    return LaunchDescription([
        DeclareLaunchArgument('prior_map_path', default_value='~/result/GlobalMap.pcd'),
        DeclareLaunchArgument('map', description='2D 占据栅格 .yaml 路径'),
        DeclareLaunchArgument('config_file', default_value='gazebo_velodyne.yaml'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_pkg, 'config', 'nav2_costmaps.yaml')),
        DeclareLaunchArgument('delay_lio', default_value='5.0'),
        DeclareLaunchArgument('delay_gicp', default_value='8.0'),
        DeclareLaunchArgument('delay_nav', default_value='12.0'),

        sim,
        TimerAction(period=delay_lio, actions=[lio]),
        TimerAction(period=delay_gicp, actions=[gicp]),
        TimerAction(period=delay_nav, actions=[nav]),
    ])
