from __future__ import annotations

import threading
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .http_server import (
    ActionConflict,
    ActionPending,
    ActionUnavailable,
    create_server,
)
from .manual_command import command_values


class WebUiNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_web_ui")
        self._destroy_started = False
        self._http_server = None
        self._http_thread = None

        max_linear = self.declare_parameter(
            "max_linear_speed", 1.5
        ).value
        max_angular = self.declare_parameter(
            "max_angular_speed", 2.0
        ).value
        host = self.declare_parameter("host", "0.0.0.0").value
        port = self.declare_parameter("port", 8080).value
        self._max_linear = float(max_linear)
        self._max_angular = float(max_angular)
        self._gate_mode = None

        self._manual_publisher = self.create_publisher(
            TwistStamped,
            "/cmd_vel_manual",
            10,
        )
        mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._mode_subscription = self.create_subscription(
            String,
            "/cmd_vel_gate/mode",
            self._gate_mode_callback,
            mode_qos,
        )
        self._takeover_client = self.create_client(
            Trigger,
            "/cmd_vel_gate/takeover_manual",
        )
        self._resume_client = self.create_client(
            Trigger,
            "/cmd_vel_gate/resume_automatic",
        )

        try:
            html_path = (
                Path(get_package_share_directory("robot_web_ui"))
                / "web"
                / "index.html"
            )
            self._http_server = create_server(
                self,
                html_path,
                host=str(host),
                port=int(port),
            )
            self._http_thread = threading.Thread(
                target=self._http_server.serve_forever,
                name="robot-web-ui-http",
                daemon=True,
            )
            self._http_thread.start()
            self.get_logger().info(
                f"Robot Web UI listening on http://{host}:{port}"
            )
        except BaseException:
            try:
                self.destroy_node()
            except BaseException:
                pass
            raise

    def manual_command(
        self,
        direction: str,
        speed_percent: float,
    ) -> str | None:
        linear, angular = command_values(
            direction,
            speed_percent,
            self._max_linear,
            self._max_angular,
        )
        if (
            (linear != 0.0 or angular != 0.0)
            and self._gate_mode != "manual"
        ):
            raise ActionConflict(
                "manual control is not active",
                self._gate_mode,
            )
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "base_link"
        message.twist.linear.x = linear
        message.twist.angular.z = angular
        self._manual_publisher.publish(message)
        return self._gate_mode

    def _gate_mode_callback(self, message: String) -> None:
        if message.data in {"manual", "automatic"}:
            self._gate_mode = message.data

    def takeover_manual(self) -> str:
        return self._call_mode_service(
            self._takeover_client,
            "manual takeover",
            "manual",
        )

    def resume_automatic(self) -> str:
        return self._call_mode_service(
            self._resume_client,
            "automatic resume",
            "automatic",
        )

    def _unconfirmed(self, target: str, message: str) -> str:
        if self._gate_mode == target:
            return target
        raise ActionPending(message, self._gate_mode)

    def _call_mode_service(
        self,
        client,
        action_name: str,
        target: str,
    ) -> str:
        if not client.service_is_ready():
            raise ActionUnavailable(f"{action_name} service unavailable")

        finished = threading.Event()
        result = {}
        future = client.call_async(Trigger.Request())

        def store_result(completed_future) -> None:
            try:
                result["response"] = completed_future.result()
            except BaseException as exc:
                result["error"] = exc
            finally:
                finished.set()

        future.add_done_callback(store_result)
        if not finished.wait(timeout=1.0):
            return self._unconfirmed(
                target,
                f"{action_name} service timed out",
            )
        if "error" in result:
            return self._unconfirmed(
                target,
                f"{action_name} service failed: {result['error']}",
            )

        response = result["response"]
        if not response.success:
            raise ActionUnavailable(
                response.message or f"{action_name} request rejected"
            )
        self._gate_mode = target
        return target

    def destroy_node(self) -> None:
        if self._destroy_started:
            return
        self._destroy_started = True
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
        node = WebUiNode()
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
