#!/usr/bin/env python3
"""一键动态障碍：按 obstacles.yaml 渲染 SDF → 逐个 spawn → 启动编排节点。

前置：colcon build --packages-select sim_obstacles && source install/setup.bash
（本文件顶层 import sim_obstacles 模块，未 build+source 会报 ModuleNotFoundError）

用法（Gazebo 已在跑，第五个终端）：
    ros2 launch sim_obstacles dynamic_obstacles.launch.py
    # 换清单：config_file:=/abs/path/to/obstacles.yaml
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from sim_obstacles.config_loader import load_obstacles
from sim_obstacles.sdf_template import render_sdf

BOX_HALF_SIZE = '0.4'   # 立方体中心离地高（边长 0.8m），z=0.4 才坐在地面上


def _spawn_and_drive(context, *args, **kwargs):
    pkg = get_package_share_directory('sim_obstacles')
    config_path = os.path.abspath(os.path.expanduser(
        LaunchConfiguration('config_file').perform(context)))
    obstacles = load_obstacles(config_path)   # 非法配置 launch 期即报错

    with open(os.path.join(pkg, 'models', 'obstacle.sdf.in'),
              'r', encoding='utf-8') as f:
        template = f.read()

    tmpdir = tempfile.mkdtemp(prefix='sim_obstacles_')
    actions = []
    for ob in obstacles:
        sdf_path = os.path.join(tmpdir, f"{ob['name']}.sdf")
        with open(sdf_path, 'w', encoding='utf-8') as f:
            f.write(render_sdf(template, ob['name']))
        actions.append(Node(
            package='gazebo_ros', executable='spawn_entity.py',
            name=f"spawn_{ob['name']}", output='screen',
            arguments=[
                '-entity', ob['name'], '-file', sdf_path,
                '-x', str(ob['x']), '-y', str(ob['y']),
                '-z', BOX_HALF_SIZE, '-Y', str(ob['yaw']),
            ],
        ))

    actions.append(Node(
        package='sim_obstacles', executable='obstacle_driver',
        name='obstacle_driver', output='screen',
        parameters=[{'config_file': config_path, 'use_sim_time': True}],
    ))
    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('sim_obstacles'), 'config', 'obstacles.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file', default_value=default_config,
            description='障碍清单 yaml（默认包内 8 障碍；密度/速度在该文件调）'),
        OpaqueFunction(function=_spawn_and_drive),
    ])
