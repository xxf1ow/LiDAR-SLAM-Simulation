import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2

from system_bringup.sensor_gate_logic import SensorGateState


class SensorGateNode(Node):
    def __init__(self):
        super().__init__("sensor_contract_gate")
        expected_points_per_scan = int(
            self.declare_parameter(
                "expected_points_per_scan", Parameter.Type.INTEGER
            ).value
        )
        expected_point_hz = float(
            self.declare_parameter(
                "expected_point_hz", Parameter.Type.DOUBLE
            ).value
        )
        expected_imu_hz = float(
            self.declare_parameter("expected_imu_hz", Parameter.Type.DOUBLE).value
        )
        minimum_point_rate_ratio = float(
            self.declare_parameter(
                "minimum_point_rate_ratio", Parameter.Type.DOUBLE
            ).value
        )
        minimum_imu_rate_ratio = float(
            self.declare_parameter(
                "minimum_imu_rate_ratio", Parameter.Type.DOUBLE
            ).value
        )
        max_stamp_age = float(
            self.declare_parameter("max_stamp_age", Parameter.Type.DOUBLE).value
        )
        rate_window = float(
            self.declare_parameter("rate_window", Parameter.Type.DOUBLE).value
        )
        stable_duration = float(
            self.declare_parameter(
                "stable_duration", Parameter.Type.DOUBLE
            ).value
        )
        self.timeout = float(
            self.declare_parameter("timeout", Parameter.Type.DOUBLE).value
        )
        self.state = SensorGateState(
            expected_points_per_scan=expected_points_per_scan,
            minimum_point_hz=expected_point_hz * minimum_point_rate_ratio,
            minimum_imu_hz=expected_imu_hz * minimum_imu_rate_ratio,
            max_stamp_age=max_stamp_age,
            rate_window=rate_window,
            stable_duration=stable_duration,
        )
        self.started_at = self._now_seconds()
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
        now = self._now_seconds()
        self.state.observe_point(
            received=now,
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            now_ros=now,
            frame_id=msg.header.frame_id,
            height=msg.height,
            width=msg.width,
            fields=tuple(field.name for field in msg.fields),
        )

    def _on_imu(self, msg):
        now = self._now_seconds()
        self.state.observe_imu(
            received=now,
            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            now_ros=now,
            frame_id=msg.header.frame_id,
        )

    def _check_status(self):
        if self.finished:
            return
        now = self._now_seconds()
        ready, reason = self.state.status(now)
        if ready:
            self._finish(
                0,
                "sensor contract ready "
                f"(point={self.state.point_rate.hz(now):.1f} Hz, "
                f"imu={self.state.imu_rate.hz(now):.1f} Hz)",
            )
        elif now - self.started_at >= self.timeout:
            self._finish(1, f"sensor contract timed out: {reason}")
        elif now - self.last_report_at >= 5.0:
            self.get_logger().info(f"sensor contract waiting: {reason}")
            self.last_report_at = now

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _finish(self, exit_code, message):
        self.finished = True
        self.exit_code = exit_code
        self.timer.cancel()
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = SensorGateNode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code
