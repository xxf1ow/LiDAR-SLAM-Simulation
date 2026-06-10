#!/usr/bin/env python3
"""nav2 阶段一：把两座 TF 岛焊成单树 + 起 map_server / planner_server / controller_server。

costmap 由 planner_server(global_costmap) 与 controller_server(local_costmap) 托管
(nav2 官方架构；standalone nav2_costmap_2d 会撞名 /costmap/costmap、不可用)。
不起 bt_navigator/behavior、不发目标点 —— 两个服务器空转、不发 cmd_vel、机器人不动。
依赖完整现有栈已运行(robot_sim + fast_lio + gicp_localization)，否则 TF 有断边。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('robot_navigation')
    params_file = os.path.join(pkg, 'config', 'nav2_costmaps.yaml')
    rviz_file = os.path.join(pkg, 'config', 'stage1.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    use_rviz = LaunchConfiguration('use_rviz')

    # body->base_footprint 静态焊接：URDF 推算暂定值，build 机 tf2_echo 核正。
    tf_x = LaunchConfiguration('tf_x')
    tf_y = LaunchConfiguration('tf_y')
    tf_z = LaunchConfiguration('tf_z')
    tf_roll = LaunchConfiguration('tf_roll')
    tf_pitch = LaunchConfiguration('tf_pitch')
    tf_yaw = LaunchConfiguration('tf_yaw')

    # costmap 由服务器托管：global_costmap 在 planner_server、local_costmap 在 controller_server
    # (源码：planner_server.cpp/controller_server.cpp 各 new Costmap2DROS("global_costmap"/"local_costmap"))。
    # lifecycle_manager 管这三个【服务器】，costmap 作为子节点随服务器转生命周期。
    lifecycle_nodes = ['map_server', 'planner_server', 'controller_server']

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', description='2D 占据栅格 .yaml 路径(pcd_to_occupancy 生成)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('tf_x', default_value='0.0'),
        DeclareLaunchArgument('tf_y', default_value='0.0'),
        DeclareLaunchArgument('tf_z', default_value='-0.297322'),
        DeclareLaunchArgument('tf_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_pitch', default_value='3.14159274'),
        DeclareLaunchArgument('tf_yaw', default_value='0.0'),

        # 1) TF 焊接：body(FAST-LIO) -> base_footprint(URDF 根)
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='body_to_base_footprint', output='screen',
            arguments=[
                '--x', tf_x, '--y', tf_y, '--z', tf_z,
                '--roll', tf_roll, '--pitch', tf_pitch, '--yaw', tf_yaw,
                '--frame-id', 'body', '--child-frame-id', 'base_footprint',
            ],
        ),

        # 2) map_server：发 2D 先验图(latched)
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
        ),

        # 3) planner_server：托管 global_costmap(map 系，static 先验图 + 膨胀)
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # 4) controller_server：托管 local_costmap(camera_init 系，3D voxel 点云层 + 膨胀)
        Node(
            package='nav2_controller', executable='controller_server', name='controller_server',
            output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # 5) lifecycle_manager：autostart 把上面三个服务器 configure->activate
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_costmaps', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),

        # 6) 可选 RViz
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
