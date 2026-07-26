import os
import sys
import time
import unittest

import launch_testing
import launch_testing.markers
import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory
from controller_manager.test_utils import check_controllers_running
from geometry_msgs.msg import TwistStamped
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray


@pytest.mark.rostest
def generate_test_description():
    fake = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(os.path.dirname(__file__), "fake_8030d_node.py"),
        ],
        output="screen",
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "robot.launch.py",
            )
        ),
        launch_arguments={
            "gui": "false",
            "use_mock_hardware": "false",
        }.items(),
    )
    return LaunchDescription([fake, robot, ReadyToTest()])


class TestRealHardwareChain(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_real_hardware_chain")
        self.motor = None
        self.joint_state = None
        self.odom = None
        self.motor_sub = self.node.create_subscription(
            Int16MultiArray,
            "/motor_speed",
            lambda msg: setattr(self, "motor", list(msg.data)),
            10,
        )
        self.joint_sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            lambda msg: setattr(self, "joint_state", msg),
            10,
        )
        self.odom_sub = self.node.create_subscription(
            Odometry,
            "/base_controller/odom",
            lambda msg: setattr(self, "odom", msg),
            10,
        )
        self.cmd_pub = self.node.create_publisher(
            TwistStamped, "/cmd_vel", 10
        )

    def tearDown(self):
        self.node.destroy_node()

    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return
        self.fail("condition was not met before timeout")

    def publish_forward_until_motor_moves(self):
        check_controllers_running(
            self.node, ["base_controller", "joint_state_broadcaster"]
        )
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and self.motor != [10, 10]:
            message = TwistStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.twist.linear.x = 0.12
            self.cmd_pub.publish(message)
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def test_controllers_activate(self):
        # Activation requires fresh feedback, and the fake publishes feedback only
        # after consuming /driver=1, so active controllers prove the enable handshake.
        check_controllers_running(
            self.node, ["base_controller", "joint_state_broadcaster"]
        )

    def test_cmd_vel_reaches_vendor_and_measured_feedback_reaches_odom(self):
        self.publish_forward_until_motor_moves()
        self.assertEqual(self.motor, [10, 10])

        def measured_state_visible():
            if self.joint_state is None or self.odom is None:
                return False
            velocities = dict(zip(self.joint_state.name, self.joint_state.velocity))
            return (
                velocities.get("left_wheel_joint", 0.0) > 1.0
                and velocities.get("right_wheel_joint", 0.0) > 1.0
                and self.odom.twist.twist.linear.x > 0.12
            )

        self.spin_until(measured_state_visible)
        velocities = dict(
            zip(self.joint_state.name, self.joint_state.velocity)
        )
        self.assertAlmostEqual(
            velocities["left_wheel_joint"], 1.0471975512, places=3
        )
        self.assertAlmostEqual(
            velocities["right_wheel_joint"], 1.0471975512, places=3
        )
        self.assertAlmostEqual(
            self.odom.twist.twist.linear.x, 0.1256637061, places=2
        )

    def test_cmd_vel_timeout_returns_motor_command_to_zero(self):
        self.publish_forward_until_motor_moves()
        self.spin_until(lambda: self.motor == [0, 0], timeout=2.0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
