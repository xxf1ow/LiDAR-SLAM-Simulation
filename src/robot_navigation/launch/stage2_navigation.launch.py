#!/usr/bin/env python3
"""nav2 阶段二：完整自主导航。

在阶段一(TF 焊接 + map_server + planner/controller 托管 costmap)之上，新增
behavior_server(恢复行为) + bt_navigator(行为树大脑) + waypoint_follower(多点停靠)，
由 lifecycle_manager autostart 全部激活。机器人可从 RViz 发目标自主导航。

odom 速度源：bt_navigator/controller_server 的 odom_topic=/odom(diff_drive 真实 twist)；
pose 走 TF(map→base_footprint)。详见 spec §5。
依赖完整现有栈已运行(robot_sim use_teleop:=false + fast_lio + gicp_localization)。
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
    default_params = os.path.join(pkg, 'config', 'nav2_costmaps.yaml')
    default_nav_params = os.path.join(pkg, 'config', 'nav2_navigation.yaml')
    rviz_file = os.path.join(pkg, 'config', 'stage2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    params_file = LaunchConfiguration('params_file')
    nav_params_file = LaunchConfiguration('nav_params_file')

    tf_x = LaunchConfiguration('tf_x')
    tf_y = LaunchConfiguration('tf_y')
    tf_z = LaunchConfiguration('tf_z')
    tf_roll = LaunchConfiguration('tf_roll')
    tf_pitch = LaunchConfiguration('tf_pitch')
    tf_yaw = LaunchConfiguration('tf_yaw')

    # bt_navigator/waypoint_follower 必须在三个服务器激活之后(它们要调服务器)。
    lifecycle_nodes = [
        'map_server', 'planner_server', 'controller_server',
        'behavior_server', 'bt_navigator', 'waypoint_follower',
    ]

    def _map_server_and_planner_controller(context, *args, **kwargs):
        # 使用绝对路径处理 map_yaml 和 params_path, 确保能正确加载到文件
        map_yaml = os.path.abspath(os.path.expanduser(
            LaunchConfiguration('map').perform(context)))
        params_path = os.path.abspath(os.path.expanduser(
            LaunchConfiguration('params_file').perform(context)))
        return [
            # 1) map_server：发 2D 先验图(latched)
            Node(
                package='nav2_map_server', executable='map_server', name='map_server',
                output='screen',
                parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
            ),
            # 2) planner_server：托管 global_costmap
            Node(
                package='nav2_planner', executable='planner_server', name='planner_server',
                output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}],
            ),

            # 3) controller_server：托管 local_costmap + 跑 DWB，发 /cmd_vel
            Node(
                package='nav2_controller', executable='controller_server', name='controller_server',
                output='screen', parameters=[params_file, {'use_sim_time': use_sim_time}],
            )
        ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', description='2D 占据栅格 .yaml 路径(pcd_to_occupancy 生成)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='costmap/planner/controller 参数(默认 voxel_layer 版；切 STVL 传 nav2_costmaps_stvl.yaml)'),
        DeclareLaunchArgument(
            'nav_params_file', default_value=default_nav_params,
            description='behavior/bt/waypoint 导航参数 yaml'),
        DeclareLaunchArgument('tf_x', default_value='0.0'),
        DeclareLaunchArgument('tf_y', default_value='0.0'),
        # 焊接旋转=单位：FAST-LIO 的 body 已重力对齐(Z 上)，base_footprint≈body。
        # (早期 pitch=π 是误把 body 当成物理倒装 IMU 帧；运行时 /localization 证明 body 是正的。)
        DeclareLaunchArgument('tf_z', default_value='0.297322'),
        DeclareLaunchArgument('tf_roll', default_value='0.0'),
        DeclareLaunchArgument('tf_pitch', default_value='0.0'),
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
        OpaqueFunction(function=_map_server_and_planner_controller),

        # 3) behavior_server：恢复行为(spin/backup/drive_on_heading/wait)
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen', parameters=[nav_params_file, {'use_sim_time': use_sim_time}],
        ),

        # 4) bt_navigator：行为树大脑(单点 + 穿点)
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen', parameters=[nav_params_file, {'use_sim_time': use_sim_time}],
        ),

        # 5) waypoint_follower：逐点停靠巡航
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
            output='screen', parameters=[nav_params_file, {'use_sim_time': use_sim_time}],
        ),

        # 6) lifecycle_manager：autostart 六节点 configure->activate
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': lifecycle_nodes,
            }],
        ),

        # 7) 可选 RViz(带 Nav2 面板)
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
