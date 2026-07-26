"""ZL-8030D vendor driver plus the formal real ros2_control chassis."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")

    vendor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("can_driver"), "can_driver_8030.launch.py"]
            )
        ),
        launch_arguments={
            "auto_enable_on_start": "false",
            "config_file": PathJoinSubstitution(
                [FindPackageShare("can_driver"), "can_driver_params.yaml"]
            ),
        }.items(),
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("robot_bringup"), "launch", "robot.launch.py"]
            )
        ),
        launch_arguments={
            "gui": gui,
            "use_mock_hardware": "false",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            vendor,
            robot,
        ]
    )
