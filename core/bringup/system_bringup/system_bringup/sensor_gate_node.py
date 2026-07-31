import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from system_bringup.sensor_gate_logic import SensorGateState


class SensorGateNode(Node):
    def __init__(self):
        super().__init__("real_sensor_ready_gate")
        self.state = SensorGateState()
        self.timeout = float(self.declare_parameter("timeout", 300.0).value)
        self.started_at = time.monotonic()
        self.last_report_at = self.started_at
        self.exit_code = 1
        self.finished = False
        self.create_subscription(
            PointCloud2, "/points_raw", self._on_points, qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, "/imu/data", self._on_imu, qos_profile_sensor_data
        )
        self.timer = self.create_timer(0.1, self._check_status)

    def _on_points(self, msg):
        self.state.observe_point(
            received=time.monotonic(),
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            now_ros=self.get_clock().now().nanoseconds * 1e-9,
            frame_id=msg.header.frame_id,
            height=msg.height,
            width=msg.width,
            fields=tuple(field.name for field in msg.fields),
        )

    def _on_imu(self, msg):
        self.state.observe_imu(
            received=time.monotonic(),
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            now_ros=self.get_clock().now().nanoseconds * 1e-9,
            frame_id=msg.header.frame_id,
        )

    def _check_status(self):
        if self.finished:
            return
        now = time.monotonic()
        ready, reason = self.state.status(now)
        if ready:
            self._finish(
                0,
                "real sensor contract ready "
                f"(point={self.state.point_rate.hz(now):.1f} Hz, "
                f"imu={self.state.imu_rate.hz(now):.1f} Hz)",
            )
        elif now - self.started_at >= self.timeout:
            self._finish(1, f"real sensor contract timed out: {reason}")
        elif now - self.last_report_at >= 5.0:
            self.get_logger().info(f"real sensor contract waiting: {reason}")
            self.last_report_at = now

    def _finish(self, exit_code, message):
        self.finished = True
        self.exit_code = exit_code
        self.timer.cancel()
        self.get_logger().info(message)
        if rclpy.ok():
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = SensorGateNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code
