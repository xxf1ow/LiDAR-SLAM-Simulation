import math
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

import launch_testing
import launch_testing.markers
import pytest
import rclpy
import yaml
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


CONTROLLERS_TEMPLATE = (
    Path(__file__).resolve().parents[3]
    / "bringup/system_bringup/config/templates/robot_controllers.yaml"
)
TEMPORARY_CONTROLLER_FILES = []
SYNTHETIC_HARDWARE_CHAIN_FIXTURE = {
    "robot_launch_arguments": {
        "base_length": 0.83,
        "base_width": 0.47,
        "base_height": 0.31,
        "base_link_height": 0.29,
        "wheel_radius": 0.137,
        "wheel_width": 0.052,
        "wheel_separation": 0.49,
        "lidar_x": 0.21,
        "lidar_y": -0.08,
        "lidar_z": 0.67,
        "lidar_roll": 0.11,
        "lidar_pitch": -0.07,
        "lidar_yaw": 0.19,
        "imu_x": -0.16,
        "imu_y": 0.09,
        "imu_z": 0.44,
        "imu_roll": -0.05,
        "imu_pitch": 0.08,
        "imu_yaw": -0.13,
        "lidar_scan_lines": 19,
        "lidar_columns_per_scan": 1337,
        "lidar_scan_rate_hz": 12.5,
        "lidar_min_range": 0.42,
        "lidar_max_range": 57.0,
        "lidar_horizontal_start_angle": -2.4,
        "lidar_horizontal_end_angle": 2.7,
        "imu_rate_hz": 173.0,
    },
    "motor_rpm": 11,
    "odom_tolerance": 0.002,
}


def _synthetic_robot_launch_arguments():
    return {
        name: str(value)
        for name, value in SYNTHETIC_HARDWARE_CHAIN_FIXTURE[
            "robot_launch_arguments"
        ].items()
    }


def _expected_motor_command():
    rpm = SYNTHETIC_HARDWARE_CHAIN_FIXTURE["motor_rpm"]
    return [-rpm, -rpm]


def _expected_wheel_velocity():
    rpm = SYNTHETIC_HARDWARE_CHAIN_FIXTURE["motor_rpm"]
    return rpm * 2.0 * math.pi / 60.0


def _expected_odom_linear_x():
    wheel_radius = SYNTHETIC_HARDWARE_CHAIN_FIXTURE[
        "robot_launch_arguments"
    ]["wheel_radius"]
    return wheel_radius * _expected_wheel_velocity()


def _real_controllers_file():
    controllers = yaml.safe_load(CONTROLLERS_TEMPLATE.read_text(encoding="utf-8"))
    for section in ("controller_manager", "base_controller"):
        controllers[section]["ros__parameters"]["use_sim_time"] = False
    fixture_arguments = SYNTHETIC_HARDWARE_CHAIN_FIXTURE[
        "robot_launch_arguments"
    ]
    controller_parameters = controllers["base_controller"]["ros__parameters"]
    controller_parameters["wheel_radius"] = fixture_arguments["wheel_radius"]
    controller_parameters["wheel_separation"] = fixture_arguments[
        "wheel_separation"
    ]
    with tempfile.NamedTemporaryFile(
        delete=False, suffix=".yaml", mode="w", encoding="utf-8"
    ) as stream:
        yaml.safe_dump(controllers, stream, sort_keys=False)
        path = Path(stream.name)
    TEMPORARY_CONTROLLER_FILES.append(path)
    return str(path)


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
            "controllers_file": _real_controllers_file(),
            "use_sim_time": "false",
            **_synthetic_robot_launch_arguments(),
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
        expected_motor = _expected_motor_command()
        while time.monotonic() < deadline and self.motor != expected_motor:
            message = TwistStamped()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.twist.linear.x = _expected_odom_linear_x()
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
        self.assertEqual(self.motor, _expected_motor_command())
        expected_wheel_velocity = _expected_wheel_velocity()
        expected_odom_linear_x = _expected_odom_linear_x()
        odom_tolerance = SYNTHETIC_HARDWARE_CHAIN_FIXTURE["odom_tolerance"]

        def measured_state_visible():
            if self.joint_state is None or self.odom is None:
                return False
            velocities = dict(zip(self.joint_state.name, self.joint_state.velocity))
            return (
                velocities.get("left_wheel_joint", 0.0) > 0.0
                and velocities.get("right_wheel_joint", 0.0) > 0.0
                and abs(
                    self.odom.twist.twist.linear.x
                    - expected_odom_linear_x
                )
                <= odom_tolerance
            )

        self.spin_until(measured_state_visible)
        velocities = dict(
            zip(self.joint_state.name, self.joint_state.velocity)
        )
        self.assertAlmostEqual(
            velocities["left_wheel_joint"], expected_wheel_velocity, places=3
        )
        self.assertAlmostEqual(
            velocities["right_wheel_joint"], expected_wheel_velocity, places=3
        )
        self.assertAlmostEqual(
            self.odom.twist.twist.linear.x,
            expected_odom_linear_x,
            delta=odom_tolerance,
        )

    def test_cmd_vel_timeout_returns_motor_command_to_zero(self):
        self.publish_forward_until_motor_moves()
        self.spin_until(lambda: self.motor == [0, 0], timeout=2.0)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_temporary_controller_files_are_removed(self):
        for path in TEMPORARY_CONTROLLER_FILES:
            path.unlink()
            self.assertFalse(path.exists())

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
