"""mock / 真机硬件 bringup：ros2_control_node + robot_state_publisher + 控制器 spawner + RViz。
Gz 仿真用另一个 launch(robot_gz_bringup)。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true",
                              description="自动启动 RViz2。"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true",
                              description="true=mock 硬件镜像命令到状态；false=真机 robot_hardware 插件。"),
        DeclareLaunchArgument("prefix", default_value="",
                              description="link/joint 名前缀。"),
        DeclareLaunchArgument("use_sim_time"),
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
    ]
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    prefix = LaunchConfiguration("prefix")
    controllers_file = LaunchConfiguration("controllers_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]),
        " ", "use_gazebo:=false",
        " ", "use_mock_hardware:=", use_mock_hardware,
        " ", "prefix:=", prefix,
        " ", "base_length:=", LaunchConfiguration("base_length"),
        " ", "base_width:=", LaunchConfiguration("base_width"),
        " ", "base_height:=", LaunchConfiguration("base_height"),
        " ", "base_link_height:=", LaunchConfiguration("base_link_height"),
        " ", "wheel_radius:=", LaunchConfiguration("wheel_radius"),
        " ", "wheel_width:=", LaunchConfiguration("wheel_width"),
        " ", "wheel_separation:=", LaunchConfiguration("wheel_separation"),
        " ", "lidar_x:=", LaunchConfiguration("lidar_x"),
        " ", "lidar_y:=", LaunchConfiguration("lidar_y"),
        " ", "lidar_z:=", LaunchConfiguration("lidar_z"),
        " ", "lidar_roll:=", LaunchConfiguration("lidar_roll"),
        " ", "lidar_pitch:=", LaunchConfiguration("lidar_pitch"),
        " ", "lidar_yaw:=", LaunchConfiguration("lidar_yaw"),
        " ", "imu_x:=", LaunchConfiguration("imu_x"),
        " ", "imu_y:=", LaunchConfiguration("imu_y"),
        " ", "imu_z:=", LaunchConfiguration("imu_z"),
        " ", "imu_roll:=", LaunchConfiguration("imu_roll"),
        " ", "imu_pitch:=", LaunchConfiguration("imu_pitch"),
        " ", "imu_yaw:=", LaunchConfiguration("imu_yaw"),
        " ", "lidar_scan_lines:=", LaunchConfiguration("lidar_scan_lines"),
        " ", "lidar_columns_per_scan:=", LaunchConfiguration("lidar_columns_per_scan"),
        " ", "lidar_scan_rate_hz:=", LaunchConfiguration("lidar_scan_rate_hz"),
        " ", "lidar_min_range:=", LaunchConfiguration("lidar_min_range"),
        " ", "lidar_max_range:=", LaunchConfiguration("lidar_max_range"),
        " ", "lidar_horizontal_start_angle:=", LaunchConfiguration("lidar_horizontal_start_angle"),
        " ", "lidar_horizontal_end_angle:=", LaunchConfiguration("lidar_horizontal_end_angle"),
        " ", "imu_rate_hz:=", LaunchConfiguration("imu_rate_hz"),
    ])
    robot_description = {
        "robot_description": robot_description_content,
        "use_sim_time": use_sim_time,
    }

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "rviz", "robot.rviz"])

    control_node = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[controllers_file, {"use_sim_time": use_sim_time}], output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
            ("/base_controller/cmd_vel", "/cmd_vel"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="both", parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}], condition=IfCondition(gui),
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    robot_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["base_controller", "--controller-manager", "/controller_manager"],
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner, on_exit=[rviz_node]))
    delay_base_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner, on_exit=[robot_controller_spawner]))

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_rviz,
        delay_base_controller,
    ]
    return LaunchDescription(declared_arguments + nodes)
