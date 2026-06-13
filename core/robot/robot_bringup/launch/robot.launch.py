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
    ]
    gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    prefix = LaunchConfiguration("prefix")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]),
        " ", "use_gazebo:=false",
        " ", "use_mock_hardware:=", use_mock_hardware,
        " ", "prefix:=", prefix,
    ])
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "robot_controllers.yaml"])
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "rviz", "robot.rviz"])

    control_node = Node(
        package="controller_manager", executable="ros2_control_node",
        parameters=[robot_controllers], output="both",
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
        arguments=["-d", rviz_config_file], condition=IfCondition(gui),
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
