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
            "lidar_x", "lidar_y", "lidar_z",
            "lidar_roll", "lidar_pitch", "lidar_yaw",
            "imu_x", "imu_y", "imu_z",
            "imu_roll", "imu_pitch", "imu_yaw",
            "lidar_scan_lines", "lidar_columns_per_scan", "lidar_scan_rate_hz",
            "lidar_min_range", "lidar_max_range",
            "lidar_horizontal_start_angle", "lidar_horizontal_end_angle",
            "imu_rate_hz",
            "use_sim_time",
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
            DeclareLaunchArgument("lidar_x"),
            DeclareLaunchArgument("lidar_y"),
            DeclareLaunchArgument("lidar_z"),
            DeclareLaunchArgument("lidar_roll"),
            DeclareLaunchArgument("lidar_pitch"),
            DeclareLaunchArgument("lidar_yaw"),
            DeclareLaunchArgument("imu_x"),
            DeclareLaunchArgument("imu_y"),
            DeclareLaunchArgument("imu_z"),
            DeclareLaunchArgument("imu_roll"),
            DeclareLaunchArgument("imu_pitch"),
            DeclareLaunchArgument("imu_yaw"),
            DeclareLaunchArgument("lidar_scan_lines"),
            DeclareLaunchArgument("lidar_columns_per_scan"),
            DeclareLaunchArgument("lidar_scan_rate_hz"),
            DeclareLaunchArgument("lidar_min_range"),
            DeclareLaunchArgument("lidar_max_range"),
            DeclareLaunchArgument("lidar_horizontal_start_angle"),
            DeclareLaunchArgument("lidar_horizontal_end_angle"),
            DeclareLaunchArgument("imu_rate_hz"),
            DeclareLaunchArgument("use_sim_time"),
            vendor,
            robot,
        ]
    )
