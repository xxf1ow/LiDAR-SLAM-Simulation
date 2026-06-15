#!/usr/bin/env python3
"""twist_stamper：把 Nav2 的无戳 Twist 补成底盘要的 TwistStamped。

拓扑(见 spec §4.8)：Nav2 的 controller/behavior 把 cmd_vel remap 到 /cmd_vel_nav(Twist)，
本节点订 /cmd_vel_nav → 补 header(stamp=now, frame_id) → 发 /cmd_vel(TwistStamped) → diff_drive。
diff_drive_controller 保持 use_stamped_vel=true，不动底盘。
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


def to_twist_stamped(twist, frame_id, stamp):
    """纯转换：Twist + frame_id + stamp(builtin_interfaces/Time) -> TwistStamped。"""
    out = TwistStamped()
    out.header.stamp = stamp
    out.header.frame_id = frame_id
    out.twist = twist
    return out


class TwistStamper(Node):
    def __init__(self):
        super().__init__('twist_stamper')
        self.declare_parameter('input_topic', '/cmd_vel_nav')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('frame_id', 'base_link')
        inp = self.get_parameter('input_topic').value
        out = self.get_parameter('output_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._pub = self.create_publisher(TwistStamped, out, 10)
        self._sub = self.create_subscription(Twist, inp, self._cb, 10)
        self.get_logger().info(
            "twist_stamper: %s(Twist) -> %s(TwistStamped) frame=%s"
            % (inp, out, self._frame_id))

    def _cb(self, msg):
        self._pub.publish(
            to_twist_stamped(msg, self._frame_id, self.get_clock().now().to_msg()))


def main(args=None):
    rclpy.init(args=args)
    node = TwistStamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
