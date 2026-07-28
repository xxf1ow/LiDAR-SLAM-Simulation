#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int8


class Fake8030D(Node):
    def __init__(self):
        super().__init__("fake_8030d")
        self.enabled = False
        self.command = [0, 0]
        self.driver_sub = self.create_subscription(
            Int8, "/driver", self.on_driver, 10
        )
        self.motor_sub = self.create_subscription(
            Int16MultiArray, "/motor_speed", self.on_motor, 10
        )
        self.feedback_pub = self.create_publisher(
            Int16MultiArray, "/current_speed", 10
        )
        self.feedback_timer = self.create_timer(0.05, self.publish_feedback)

    def on_driver(self, message):
        self.enabled = message.data == 1

    def on_motor(self, message):
        if len(message.data) >= 2:
            self.command = [int(message.data[0]), int(message.data[1])]

    def publish_feedback(self):
        if not self.enabled:
            return
        right_rpm, left_rpm = self.command
        message = Int16MultiArray()
        message.data = [-left_rpm * 10, right_rpm * 10]
        self.feedback_pub.publish(message)


def main():
    rclpy.init()
    node = Fake8030D()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError as exc:
        # humble shutdown 期偶发 "Unable to convert call argument to Python object"
        # (context 关闭时,半成品回调反序列化失败)。此时 context 已不在,属良性退出,
        # 忽略以免 exit 1 被 test_exit_codes 断言成失败;正常期仍抛出,不掩盖真异常。
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
