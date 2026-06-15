"""仅可视化机器人模型：robot_state_publisher + joint_state_publisher_gui + RViz。无 ros2_control。"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true",
                              description="启动 joint_state_publisher_gui 与 RViz。"),
        DeclareLaunchArgument("prefix", default_value="",
                              description="link/joint 名前缀。"),
    ]
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]),
        " ", "prefix:=", prefix,
    ])
    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "rviz", "robot.rviz"])

    robot_state_publisher_node = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="both", parameters=[robot_description],
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config_file], condition=IfCondition(gui),
    )

    return LaunchDescription(
        declared_arguments
        + [robot_state_publisher_node, joint_state_publisher_gui_node, rviz_node]
    )
