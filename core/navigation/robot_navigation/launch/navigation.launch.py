#!/usr/bin/env python3
"""5e Nav2 最小打通 bringup。

依赖现有栈已运行(终端 1-4)：robot_gz 仿真 + fast_lio 里程计 + gicp_localization 定位 + 已 /initialpose 锁定。
本 launch 起：静态焊接 TF(body->base_footprint) + map_server + planner/controller/behavior/bt_navigator
+ twist_stamper + lifecycle_manager(autostart) + 可选 RViz。

拓扑(spec §4.8)：controller/behavior 的 cmd_vel remap 到 /cmd_vel_nav(Twist)，
twist_stamper 补戳成 /cmd_vel(TwistStamped) 给 diff_drive。
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
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(pkg, 'config', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    weld_z = LaunchConfiguration('weld_z')

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
            parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
        )]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser('~/result/factory_map.yaml'),
            description='2D 占据栅格 .yaml(pcd_to_occupancy 生成)'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # 焊接：parent=body -> child=base_footprint，单位旋转，z=-0.556=-(base_height/2+wheel_radius+sensor_z)。
        # 2D 导航对 z 符号不敏感(只影响 RViz 竖直位置)；验收 tf2_echo map base_footprint 应 z≈0。
        DeclareLaunchArgument('weld_z', default_value='-0.556'),

        # 1) TF 焊接：body(FAST-LIO) -> base_footprint(URDF 根)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='body_to_base_footprint', output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', weld_z,
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'body', '--child-frame-id', 'base_footprint',
            ],
        ),

        # 2) map_server(yaml_filename 经 expanduser)
        OpaqueFunction(function=_map_server),

        # 3) planner_server(托管 global_costmap + Smac Hybrid-A*)
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # 4) controller_server(托管 local_costmap + MPPI)；cmd_vel -> /cmd_vel_nav(Twist)
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),

        # 5) behavior_server(恢复)；cmd_vel -> /cmd_vel_nav(Twist)
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),

        # 6) bt_navigator(大脑)
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # 7) twist_stamper：/cmd_vel_nav(Twist) -> /cmd_vel(TwistStamped) 给 diff_drive
        Node(
            package='robot_navigation', executable='twist_stamper', name='twist_stamper',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'input_topic': '/cmd_vel_nav',
                'output_topic': '/cmd_vel',
                'frame_id': 'base_link',
            }],
        ),

        # 8) lifecycle_manager：autostart 五节点 configure->activate(bt_navigator 最后)
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),

        # 9) 可选 RViz(发 Nav2 Goal 的界面)
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', default_rviz],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
