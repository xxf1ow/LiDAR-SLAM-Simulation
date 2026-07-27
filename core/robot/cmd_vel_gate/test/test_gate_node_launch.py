import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    gate = launch_ros.actions.Node(
        package="cmd_vel_gate",
        executable="cmd_vel_gate",
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [
                gate,
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"gate": gate},
    )


class TestCmdVelGate(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self._node = rclpy.create_node("cmd_vel_gate_integration_test")
        self._outputs = []
        self._output_subscription = self._node.create_subscription(
            TwistStamped,
            "/cmd_vel",
            self._outputs.append,
            1,
        )
        self._automatic_publisher = self._node.create_publisher(
            TwistStamped,
            "/cmd_vel_auto",
            1,
        )
        self._manual_publisher = self._node.create_publisher(
            TwistStamped,
            "/cmd_vel_manual",
            1,
        )
        self._manual_client = self._node.create_client(
            Trigger,
            "/cmd_vel_gate/takeover_manual",
        )
        self._automatic_client = self._node.create_client(
            Trigger,
            "/cmd_vel_gate/resume_automatic",
        )

        self.assertTrue(self._manual_client.wait_for_service(timeout_sec=5.0))
        self.assertTrue(
            self._automatic_client.wait_for_service(timeout_sec=5.0)
        )
        self._modes = []
        mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_subscription = self._node.create_subscription(
            String,
            "/cmd_vel_gate/mode",
            lambda message: self._modes.append(message.data),
            mode_qos,
        )
        self.assertTrue(
            self._wait_until(
                lambda: (
                    self._automatic_publisher.get_subscription_count() > 0
                    and self._manual_publisher.get_subscription_count() > 0
                ),
                timeout=5.0,
            )
        )
        self.assertIsNotNone(self._wait_for_value(0.0, timeout=2.0))
        self.assertTrue(
            self._wait_until(
                lambda: "automatic" in self._modes,
                timeout=2.0,
            )
        )
        self._outputs.clear()

    def tearDown(self):
        self._node.destroy_node()

    def _wait_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            rclpy.spin_once(self._node, timeout_sec=0.05)
        return predicate()

    def _wait_for_value(self, value, timeout):
        match = None

        def find_match():
            nonlocal match
            for message in self._outputs:
                if message.twist.linear.x == value:
                    match = message
                    return True
            return False

        if self._wait_until(find_match, timeout):
            return match
        return None

    def _message(self, value):
        message = TwistStamped()
        message.twist.linear.x = value
        return message

    def _publish_and_wait(self, publisher, value):
        self._outputs.clear()
        publisher.publish(self._message(value))
        output = self._wait_for_value(value, timeout=2.0)
        self.assertIsNotNone(output)
        self.assertEqual(output.header.frame_id, "base_link")

    def _assert_value_absent(self, publisher, value):
        self._outputs.clear()
        publisher.publish(self._message(value))
        self.assertFalse(
            self._wait_until(
                lambda: any(
                    message.twist.linear.x == value
                    for message in self._outputs
                ),
                timeout=0.25,
            )
        )

    def _switch(self, client, expected_message):
        self._outputs.clear()
        future = client.call_async(Trigger.Request())
        self.assertTrue(
            self._wait_until(lambda: future.done(), timeout=2.0)
        )
        response = future.result()
        self.assertTrue(response.success)
        self.assertEqual(response.message, expected_message)
        self.assertIsNotNone(self._wait_for_value(0.0, timeout=1.0))
        self.assertTrue(
            self._wait_until(
                lambda: self._modes[-1:] == [expected_message],
                timeout=1.0,
            )
        )

    def test_source_selection_and_timeout(self):
        self._publish_and_wait(self._automatic_publisher, 1.0)
        self._assert_value_absent(self._manual_publisher, 2.0)

        self._switch(self._manual_client, "manual")
        self._publish_and_wait(self._manual_publisher, 3.0)
        self._assert_value_absent(self._automatic_publisher, 4.0)

        self._switch(self._automatic_client, "automatic")
        self._publish_and_wait(self._automatic_publisher, 5.0)

        self._outputs.clear()
        self.assertFalse(
            self._wait_until(
                lambda: any(
                    message.twist.linear.x == 0.0
                    for message in self._outputs
                ),
                timeout=0.25,
            )
        )
        self.assertTrue(
            self._wait_until(
                lambda: len(self._outputs) >= 2,
                timeout=1.0,
            )
        )
        self.assertTrue(
            all(message.twist.linear.x == 0.0 for message in self._outputs)
        )
        zero_stamps = {
            (message.header.stamp.sec, message.header.stamp.nanosec)
            for message in self._outputs
        }
        self.assertGreaterEqual(len(zero_stamps), 2)
        self.assertNotIn("stopped", self._modes)
