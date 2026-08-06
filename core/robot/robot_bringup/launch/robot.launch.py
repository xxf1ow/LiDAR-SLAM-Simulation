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
    default_controllers = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "robot_controllers.yaml"])
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true",
                              description="自动启动 RViz2。"),
        DeclareLaunchArgument("use_mock_hardware", default_value="true",
                              description="true=mock 硬件镜像命令到状态；false=真机 robot_hardware 插件。"),
        DeclareLaunchArgument("prefix", default_value="",
                              description="link/joint 名前缀。"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("controllers_file", default_value=default_controllers),
        DeclareLaunchArgument("base_length", default_value="0.75"),
        DeclareLaunchArgument("base_width", default_value="0.55"),
        DeclareLaunchArgument("base_height", default_value="0.40"),
        DeclareLaunchArgument("base_link_height", default_value="0.32"),
        DeclareLaunchArgument("wheel_radius", default_value="0.12"),
        DeclareLaunchArgument("wheel_width", default_value="0.06"),
        DeclareLaunchArgument("wheel_separation", default_value="0.55"),
        DeclareLaunchArgument("sensor_x", default_value="0.0"),
        DeclareLaunchArgument("sensor_y", default_value="0.0"),
        DeclareLaunchArgument("sensor_z", default_value="0.236"),
        DeclareLaunchArgument("sensor_roll", default_value="0.0"),
        DeclareLaunchArgument("sensor_pitch", default_value="0.0"),
        DeclareLaunchArgument("sensor_yaw", default_value="0.0"),
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
        " ", "sensor_x:=", LaunchConfiguration("sensor_x"),
        " ", "sensor_y:=", LaunchConfiguration("sensor_y"),
        " ", "sensor_z:=", LaunchConfiguration("sensor_z"),
        " ", "sensor_roll:=", LaunchConfiguration("sensor_roll"),
        " ", "sensor_pitch:=", LaunchConfiguration("sensor_pitch"),
        " ", "sensor_yaw:=", LaunchConfiguration("sensor_yaw"),
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
