from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    prior_map_path = LaunchConfiguration('prior_map_path')

    return LaunchDescription([
        DeclareLaunchArgument('config_file'),
        DeclareLaunchArgument('prior_map_path'),
        Node(
            package='gicp_localization',
            executable='gicp_localization_node',
            name='gicp_localization',
            output='screen',
            parameters=[config_file, {'prior_map_path': prior_map_path}],
        ),
    ])
