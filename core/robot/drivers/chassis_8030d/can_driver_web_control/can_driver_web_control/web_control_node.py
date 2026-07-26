from __future__ import annotations

import threading
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Int8

from .control_state import ControlError, ControlState
from .http_server import create_server


class WebControlNode(Node):
    def __init__(self) -> None:
        super().__init__("can_driver_web_control")
        self._destroy_started = False
        self._state = ControlState()
        self._http_server = None
        self._http_thread = None
        self._motor_publisher = None
        self._driver_publisher = None
        try:
            html_path = (
                Path(get_package_share_directory("can_driver_web_control"))
                / "web"
                / "index.html"
            )
            self._http_server = create_server(self._state, html_path)
            self._motor_publisher = self.create_publisher(
                Int16MultiArray, "/motor_speed", 10
            )
            self._driver_publisher = self.create_publisher(Int8, "/driver", 10)
            self._feedback_subscription = self.create_subscription(
                Int16MultiArray,
                "/current_speed",
                self._feedback_callback,
                10,
            )
            self._motor_timer = self.create_timer(0.05, self._publish_motor)
            self._driver_timer = self.create_timer(0.5, self._publish_driver)
            self._http_thread = threading.Thread(
                target=self._http_server.serve_forever,
                name="can-driver-http",
                daemon=True,
            )
            self._http_thread.start()
            self.get_logger().info(
                "Web control listening on http://0.0.0.0:8080"
            )
        except BaseException:
            try:
                self.destroy_node()
            except BaseException:
                pass
            raise

    def _publish_motor(self) -> None:
        self._state.set_driver_connected(
            self._motor_publisher.get_subscription_count() > 0
        )
        right_rpm, left_rpm = self._state.safe_command()
        message = Int16MultiArray()
        message.data = [right_rpm, left_rpm]
        self._motor_publisher.publish(message)

    def _publish_driver(self) -> None:
        status = self._state.snapshot()
        message = Int8()
        message.data = 1 if status["enabled"] else 0
        self._driver_publisher.publish(message)

    def _feedback_callback(self, message: Int16MultiArray) -> None:
        try:
            self._state.update_feedback(message.data)
        except ControlError as exc:
            self.get_logger().warning(str(exc))

    def destroy_node(self) -> None:
        if self._destroy_started:
            return
        self._destroy_started = True
        try:
            try:
                self._state.set_enabled(False)
            finally:
                try:
                    if self._motor_publisher is not None:
                        motor_message = Int16MultiArray()
                        motor_message.data = [0, 0]
                        self._motor_publisher.publish(motor_message)
                finally:
                    if self._driver_publisher is not None:
                        driver_message = Int8()
                        driver_message.data = 0
                        self._driver_publisher.publish(driver_message)
        finally:
            try:
                if (
                    self._http_server is not None
                    and self._http_thread is not None
                    and self._http_thread.is_alive()
                ):
                    self._http_server.shutdown()
            finally:
                try:
                    if self._http_server is not None:
                        self._http_server.server_close()
                finally:
                    try:
                        if self._http_thread is not None:
                            self._http_thread.join(timeout=1.0)
                    finally:
                        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = WebControlNode()
        rclpy.spin(node)
    except OSError as exc:
        if node is not None:
            node.get_logger().fatal(f"HTTP server failed: {exc}")
        else:
            print(f"HTTP server failed: {exc}")
        raise SystemExit(1) from exc
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
