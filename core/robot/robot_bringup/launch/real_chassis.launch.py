"""ZL-8030D vendor driver plus the formal real ros2_control chassis."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gui = LaunchConfiguration("gui")
    geometry_arguments = {
        name: LaunchConfiguration(name)
        for name in (
            "controllers_file",
            "base_length", "base_width", "base_height", "base_link_height",
            "wheel_radius", "wheel_width", "wheel_separation",
            "sensor_x", "sensor_y", "sensor_z",
            "sensor_roll", "sensor_pitch", "sensor_yaw",
        )
    }

    vendor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("can_driver"), "can_driver_8030.launch.py"]
            )
        ),
        launch_arguments={
            "auto_enable_on_start": "false",
            "log_level": "warn",
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
            **geometry_arguments,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("controllers_file"),
            DeclareLaunchArgument("base_length"),
            DeclareLaunchArgument("base_width"),
            DeclareLaunchArgument("base_height"),
            DeclareLaunchArgument("base_link_height"),
            DeclareLaunchArgument("wheel_radius"),
            DeclareLaunchArgument("wheel_width"),
            DeclareLaunchArgument("wheel_separation"),
            DeclareLaunchArgument("sensor_x"),
            DeclareLaunchArgument("sensor_y"),
            DeclareLaunchArgument("sensor_z"),
            DeclareLaunchArgument("sensor_roll"),
            DeclareLaunchArgument("sensor_pitch"),
            DeclareLaunchArgument("sensor_yaw"),
            vendor,
            robot,
        ]
    )
