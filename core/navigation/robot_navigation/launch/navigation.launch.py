#!/usr/bin/env python3
"""5e Nav2 最小打通 bringup。

依赖现有栈已运行(终端 1-4)：robot_gz 仿真 + fast_lio 里程计 + gicp_localization 定位 + 已 /initialpose 锁定。
本 launch 起：静态焊接 TF(body->base_footprint) + map_server + planner/controller/behavior/bt_navigator
+ twist_stamper + lifecycle_manager(autostart) + 可选 RViz。

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

# 仿真默认焊接；真机由 system_bringup/bringup.yaml 的 real_geometry 覆盖。
_BASE_HEIGHT = 0.40
_WHEEL_RADIUS = 0.12
_LIDAR_HEIGHT = 0.072
_SENSOR_Z = _BASE_HEIGHT / 2 + _LIDAR_HEIGHT / 2              # 0.236
_WELD_Z = -((_BASE_HEIGHT / 2 + _WHEEL_RADIUS) + _SENSOR_Z)  # -0.556


def generate_launch_description():
    pkg = get_package_share_directory('robot_navigation')
    default_params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(pkg, 'config', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    weld_x = LaunchConfiguration('weld_x')
    weld_y = LaunchConfiguration('weld_y')
    weld_z = LaunchConfiguration('weld_z')
    weld_qx = LaunchConfiguration('weld_qx')
    weld_qy = LaunchConfiguration('weld_qy')
    weld_qz = LaunchConfiguration('weld_qz')
    weld_qw = LaunchConfiguration('weld_qw')
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
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'map', default_value=os.path.expanduser('~/result/factory_map.yaml'),
            description='2D 占据栅格 .yaml(pcd_to_occupancy 生成)'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('cmd_vel_output_topic', default_value='/cmd_vel'),
        # 焊接：parent=body(雷达/IMU) -> child=base_footprint。
        # 默认值保持既有仿真；正式 system_bringup 显式传入 manifest 四元数。
        DeclareLaunchArgument('weld_x', default_value='0.0'),
        DeclareLaunchArgument('weld_y', default_value='0.0'),
        DeclareLaunchArgument('weld_z', default_value=f'{_WELD_Z:.4f}'),
        DeclareLaunchArgument('weld_qx', default_value='0.0'),
        DeclareLaunchArgument('weld_qy', default_value='0.0'),
        DeclareLaunchArgument('weld_qz', default_value='0.0'),
        DeclareLaunchArgument('weld_qw', default_value='1.0'),

        # 1) TF 焊接：body(FAST-LIO) -> base_footprint(URDF 根)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='body_to_base_footprint', output='screen',
            arguments=[
                '--x', weld_x, '--y', weld_y, '--z', weld_z,
                '--qx', weld_qx, '--qy', weld_qy, '--qz', weld_qz, '--qw', weld_qw,
                '--frame-id', 'body', '--child-frame-id', 'base_footprint',
            ],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # 2) map_server(yaml_filename 经 expanduser)
        OpaqueFunction(function=_map_server),

        # 3) planner_server(托管 global_costmap + Smac Hybrid-A*)
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[params_file],
        ),

        # 4) controller_server(托管 local_costmap + MPPI)；cmd_vel -> /cmd_vel_nav(Twist)
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen', parameters=[params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),

        # 5) behavior_server(恢复)；cmd_vel -> /cmd_vel_nav(Twist)
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen', parameters=[params_file],
            remappings=[('cmd_vel', '/cmd_vel_nav')],
        ),

        # 6) bt_navigator(大脑)
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen', parameters=[params_file],
        ),

        # 7) twist_stamper：/cmd_vel_nav(Twist) -> 配置的 TwistStamped 输出话题
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
