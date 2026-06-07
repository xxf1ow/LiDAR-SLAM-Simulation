import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gicp_localization')
    default_cfg = os.path.join(pkg, 'config', 'gicp_localization.yaml')

    config_file = LaunchConfiguration('config_file')
    prior_map_path = LaunchConfiguration('prior_map_path')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_cfg),
        DeclareLaunchArgument('prior_map_path', default_value='~/result/GlobalMap.pcd'),
        Node(
            package='gicp_localization',
            executable='gicp_localization_node',
            name='gicp_localization',
            output='screen',
            parameters=[config_file, {'prior_map_path': prior_map_path}],
        ),
    ])
