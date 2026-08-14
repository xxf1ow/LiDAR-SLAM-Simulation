from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
            ),
            Node(
                package="vanjee_lidar_ros",
                executable="vanjee_lidar_node",
                name="vanjee_lidar",
                output="screen",
                parameters=[config_file],
            ),
        ]
    )
