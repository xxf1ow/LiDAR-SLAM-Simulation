"""twist_stamper 纯转换逻辑单测（需 geometry_msgs，构建机/装了 ROS 的机器跑）。"""
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist

from robot_navigation.twist_stamper import to_twist_stamped


def _twist(vx, wz):
    t = Twist()
    t.linear.x = vx
    t.angular.z = wz
    return t


def test_copies_twist_values():
    out = to_twist_stamped(_twist(0.5, -0.3), frame_id="base_link", stamp=Time())
    assert out.twist.linear.x == 0.5
    assert out.twist.angular.z == -0.3


def test_sets_frame_id():
    out = to_twist_stamped(_twist(0.0, 0.0), frame_id="base_link", stamp=Time())
    assert out.header.frame_id == "base_link"


def test_sets_stamp():
    stamp = Time(sec=12, nanosec=34)
    out = to_twist_stamped(_twist(0.0, 0.0), frame_id="base_link", stamp=stamp)
    assert out.header.stamp.sec == 12
    assert out.header.stamp.nanosec == 34
