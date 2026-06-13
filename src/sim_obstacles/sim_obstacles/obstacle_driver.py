"""动态障碍编排节点：按清单给每个障碍发往复 cmd_vel（spec §3）。

10 Hz 对每个障碍发布 /<name>/cmd_vel（geometry_msgs/Twist，body 系 linear.x）。
障碍以 yaw 朝向出生，planar_move 每步重置角速度 → 朝向近似锁定，
故只发 linear.x 即沿出生朝向往复。时间开环，无位置闭环（spec 关键决策）。
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from sim_obstacles.config_loader import load_obstacles
from sim_obstacles.oscillation import velocity_at

TICK_PERIOD_S = 0.1   # 10 Hz


class ObstacleDriver(Node):
    def __init__(self):
        super().__init__('obstacle_driver')
        self.declare_parameter('config_file', '')
        path = self.get_parameter('config_file').value
        if not path:
            raise RuntimeError('必须提供 config_file 参数（obstacles.yaml 路径）')
        self._obstacles = load_obstacles(path)   # 非法配置在此 fail fast
        self._pubs = {
            ob['name']: self.create_publisher(Twist, f"/{ob['name']}/cmd_vel", 10)
            for ob in self._obstacles
        }
        self._start_s = None   # 首个非零时钟才起算（sim time 下 /clock 可能晚到）
        self.create_timer(TICK_PERIOD_S, self._tick)
        self.get_logger().info(f'驱动 {len(self._obstacles)} 个动态障碍: '
                               + ', '.join(self._pubs))

    def _tick(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        if now_s == 0.0:
            return
        if self._start_s is None:
            self._start_s = now_s
        t = now_s - self._start_s
        for ob in self._obstacles:
            msg = Twist()
            msg.linear.x = velocity_at(t, ob['speed'], ob['period'], ob['phase'])
            self._pubs[ob['name']].publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ObstacleDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
