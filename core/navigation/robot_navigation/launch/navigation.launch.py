#!/usr/bin/env python3
"""5e Nav2 最小打通 bringup。

依赖现有栈已运行(终端 1-4)：robot_gz 仿真 + fast_lio 里程计 + gicp_localization 定位 + 已 /initialpose 锁定。
本 launch 起：map_server + planner/controller/behavior/bt_navigator + twist_stamper
以及 lifecycle_manager(autostart) + 可选 RViz。

拓扑(spec §4.8)：controller/behavior 的 cmd_vel remap 到 /cmd_vel_nav(Twist)，
twist_stamper 补戳后发到可配置输出话题(TwistStamped)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_navigation')
    default_rviz = os.path.join(pkg, 'config', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    cmd_vel_output_topic = LaunchConfiguration('cmd_vel_output_topic')

    lifecycle_nodes = [
        'map_server', 'planner_server', 'controller_server',
        'behavior_server', 'bt_navigator',
    ]

    # map 路径用 expanduser(nav2 map_io 不展开 ~)；map_server 的 yaml_filename 由此覆盖。
    def _map_server(context, *args, **kwargs):
        map_yaml = os.path.abspath(os.path.expanduser(
            LaunchConfiguration('map').perform(context)))
        return [Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[params_file, {'yaml_filename': map_yaml}],
        )]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time'),
        DeclareLaunchArgument('map'),
        DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='/cmd_vel'),

        # 1) map_server(yaml_filename 经 expanduser)
        OpaqueFunction(function=_map_server),

        # 2) planner_server(托管 global_costmap + Smac Hybrid-A*)
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[params_file],
        ),

        # 3) controller_server(托管 local_costmap + MPPI)；cmd_vel -> /cmd_vel_nav(Twist)
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen', parameters=[params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),

        # 4) behavior_server(恢复)；cmd_vel -> /cmd_vel_nav(Twist)
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen', parameters=[params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),

        # 5) bt_navigator(大脑)
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen', parameters=[params_file],
        ),

        # 6) twist_stamper：/cmd_vel_nav(Twist) -> 配置的 TwistStamped 输出话题
        Node(
            package='robot_navigation', executable='twist_stamper', name='twist_stamper',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/cmd_vel_nav',
                'output_topic': cmd_vel_output_topic,
                'frame_id': 'base_link',
            }],
        ),

        # 7) lifecycle_manager：autostart 五节点 configure->activate(bt_navigator 最后)
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),

        # 8) 可选 RViz(发 Nav2 Goal 的界面)
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', default_rviz],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
