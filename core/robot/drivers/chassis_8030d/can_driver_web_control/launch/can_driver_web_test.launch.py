from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    driver_node = Node(
        package="can_driver",
        executable="can_driver_8030",
        name="can_driver_8030",
        # The vendor binary prints every speed command to stdout. Keep that
        # high-rate diagnostic output in the ROS log instead of flooding the
        # interactive terminal.
        output="log",
        parameters=[{"auto_enable_on_start": False}],
    )
    web_node = Node(
        package="can_driver_web_control",
        executable="can_driver_web_control",
        name="can_driver_web_control",
        output="screen",
    )
    stop_all_if_web_exits = RegisterEventHandler(
        OnProcessExit(
            target_action=web_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="can_driver_web_control exited")
                )
            ],
        )
    )
    return LaunchDescription([driver_node, web_node, stop_all_if_web_exits])
