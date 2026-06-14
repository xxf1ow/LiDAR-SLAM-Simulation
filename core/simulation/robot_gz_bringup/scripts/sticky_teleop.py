#!/usr/bin/env python3
"""Sticky 键盘遥控:按一次设定速度,松手照样走,直到你按下一个键改它(非"按住才动")。

与 teleop_twist_keyboard 的区别:本节点把当前速度存在状态里,用 ~20Hz 定时器**持续重发**
TwistStamped 到 /cmd_vel——既满足"按一下持续生效",又压过 diff_drive_controller 的
cmd_vel_timeout(0.5s 收不到新指令就停)。按键只是增量修改当前速度,不需要一直按。

建图用法(单开一个终端,先 `cd core && source install/setup.bash`):
  ros2 run robot_gz_bringup sticky_teleop.py --ros-args -p use_sim_time:=true

  i / ,    线速 +/-(前进更快 / 后退;每按一次进一步)
  j / l    角速 +/-(左转 / 右转)
  k / 空格 立即停车(线速、角速都归零)
  s        回正(角速归零,保留线速,适合直行微调后摆正)
  q        退出(先发一次停车再退,并恢复终端)
仅 Linux(用 termios 原始键盘读取);构建机即 Ubuntu,符合本项目运行环境。
"""
import sys
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class StickyTeleop(Node):
    def __init__(self):
        super().__init__("sticky_teleop")
        # 话题/帧:launch 已把 /base_controller/cmd_vel remap 成 /cmd_vel
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate", 20.0)   # Hz,需 > 1/cmd_vel_timeout=2Hz,留足余量
        self.declare_parameter("stamped", True)          # diff_drive_controller(Humble 默认)要 TwistStamped
        self.declare_parameter("lin_step", 0.1)          # m/s 每次按键增量
        self.declare_parameter("ang_step", 0.2)          # rad/s 每次按键增量
        self.declare_parameter("max_lin", 1.0)           # 钳位(控制器上限 1.5,留余量、慢速建图更稳)
        self.declare_parameter("max_ang", 1.5)           # 钳位(控制器上限 2.0)

        self.topic = self.get_parameter("cmd_vel_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.stamped = self.get_parameter("stamped").value
        self.lin_step = self.get_parameter("lin_step").value
        self.ang_step = self.get_parameter("ang_step").value
        self.max_lin = self.get_parameter("max_lin").value
        self.max_ang = self.get_parameter("max_ang").value
        rate = self.get_parameter("publish_rate").value

        msg_type = TwistStamped if self.stamped else Twist
        self.pub = self.create_publisher(msg_type, self.topic, 10)

        self.lin = 0.0   # 当前线速(sticky)
        self.ang = 0.0   # 当前角速(sticky)
        self.timer = self.create_timer(1.0 / rate, self._on_timer)

        # 进入原始键盘模式(cbreak:逐字符、无回显、无需回车)
        self._stdin_fd = sys.stdin.fileno()
        self._old_term = termios.tcgetattr(self._stdin_fd)
        tty.setcbreak(self._stdin_fd)
        self._print_help()
        self._print_state()

    def _print_help(self):
        self.get_logger().info(
            "sticky teleop: i/,=前后  j/l=左右转  k或空格=停  s=回正  q=退出  "
            "(按一下持续生效,不用一直按)")

    def _print_state(self):
        # \r 原地刷新当前速度,避免刷屏
        sys.stdout.write(f"\r当前指令  lin={self.lin:+.2f} m/s  ang={self.ang:+.2f} rad/s     ")
        sys.stdout.flush()

    def _read_key(self):
        """非阻塞读一个字符;无输入返回 ''。"""
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return sys.stdin.read(1)
        return ""

    def _clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def _apply_key(self, key):
        """按键 → 修改 sticky 速度。返回 False 表示请求退出。"""
        if key in ("i", "I"):
            self.lin = self._clamp(self.lin + self.lin_step, -self.max_lin, self.max_lin)
        elif key in (",",):
            self.lin = self._clamp(self.lin - self.lin_step, -self.max_lin, self.max_lin)
        elif key in ("j", "J"):
            self.ang = self._clamp(self.ang + self.ang_step, -self.max_ang, self.max_ang)
        elif key in ("l", "L"):
            self.ang = self._clamp(self.ang - self.ang_step, -self.max_ang, self.max_ang)
        elif key in ("k", "K", " "):
            self.lin = 0.0
            self.ang = 0.0
        elif key in ("s", "S"):
            self.ang = 0.0
        elif key in ("q", "Q", "\x03"):   # q 或 Ctrl-C
            return False
        return True

    def _publish(self):
        if self.stamped:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.twist.linear.x = float(self.lin)
            msg.twist.angular.z = float(self.ang)
        else:
            msg = Twist()
            msg.linear.x = float(self.lin)
            msg.angular.z = float(self.ang)
        self.pub.publish(msg)

    def _on_timer(self):
        key = self._read_key()
        if key:
            if not self._apply_key(key):
                self.lin = 0.0
                self.ang = 0.0
                self._publish()              # 退出前先发一次停车
                self.restore_terminal()
                rclpy.shutdown()
                return
            self._print_state()
        self._publish()                      # 每个 tick 持续重发当前 sticky 速度

    def restore_terminal(self):
        termios.tcsetattr(self._stdin_fd, termios.TCSADRAIN, self._old_term)
        sys.stdout.write("\n")
        sys.stdout.flush()


def main():
    rclpy.init()
    node = StickyTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
