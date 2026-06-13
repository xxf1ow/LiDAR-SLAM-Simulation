"""起 Gz spike 世界 + ros_gz_bridge + ring/time 适配节点。

前置(build 机):colcon build --packages-select lidar_pointcloud_adapter &&
source install/setup.bash;且已装 ros_gz_sim / ros_gz_bridge(Harmonic)。
用法:ros2 launch <本文件绝对路径>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    world = os.path.abspath(os.path.join(here, '..', 'worlds', 'spike_lidar.sdf'))
    bridge_cfg = os.path.abspath(os.path.join(here, '..', 'config', 'bridge_spike.yaml'))

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('ros_gz_sim'),
            '/launch/gz_sim.launch.py']),
        launch_arguments=[('gz_args', ['-r -v 3 ', world])],
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='spike_bridge', output='screen',
        parameters=[{'config_file': bridge_cfg}],
    )

    adapter = Node(
        package='lidar_pointcloud_adapter', executable='adapter_node',
        name='lidar_pointcloud_adapter', output='screen',
        parameters=[{
            'input_topic': '/gz_lidar/points',
            'output_topic': '/points_raw',
            'output_frame': 'velodyne',
            'scan_period': 0.1,
            'use_sim_time': True,
        }],
    )

    return LaunchDescription([gz, bridge, adapter])
