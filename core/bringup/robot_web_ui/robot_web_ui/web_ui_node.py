from __future__ import annotations

import threading
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
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
from .map_snapshot import BinarySnapshot, load_nav2_pgm


ODOM_TIMEOUT = 0.5


class WebUiNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_web_ui")
        self._destroy_started = False
        self._http_server = None
        self._http_thread = None

        max_linear = self.declare_parameter(
            "max_linear_speed", Parameter.Type.DOUBLE
        ).value
        max_angular = self.declare_parameter(
            "max_angular_speed", Parameter.Type.DOUBLE
        ).value
        host = self.declare_parameter("host", Parameter.Type.STRING).value
        port = self.declare_parameter("port", Parameter.Type.INTEGER).value
        self._max_linear = float(max_linear)
        self._max_angular = float(max_angular)
        self._gate_mode = None
        self._odom_feedback = None
        map_yaml_path = self.declare_parameter(
            "map_yaml_path", Parameter.Type.STRING
        ).value
        self._static_map = None
        self._map_error = None
        self._global_costmap = None
        self._local_costmap = None
        self._path_snapshot = None
        self._localization_pose = None
        try:
            self._static_map = load_nav2_pgm(Path(map_yaml_path))
        except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
            self._map_error = f"{type(exc).__name__}: {exc}"

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
        self._odom_subscription = self.create_subscription(
            Odometry,
            "/base_controller/odom",
            self._odom_callback,
            10,
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

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _odom_callback(self, message: Odometry) -> None:
        self._odom_feedback = (
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
            self._now_seconds(),
        )

    def motion_status(self) -> dict[str, object]:
        feedback = self._odom_feedback
        if (
            feedback is None
            or self._now_seconds() - feedback[2] >= ODOM_TIMEOUT
        ):
            return {
                "linear_x": None,
                "angular_z": None,
                "feedback_fresh": False,
            }
        return {
            "linear_x": feedback[0],
            "angular_z": feedback[1],
            "feedback_fresh": True,
        }

    def navigation_state(self) -> dict[str, object]:
        static = self._static_map
        return {
            "map_error": self._map_error,
            "localized": self._localization_pose is not None,
            "layers": {
                "static": None if static is None else {
                    **static.info.as_dict(),
                    "revision": static.binary.revision,
                    "etag": static.binary.etag,
                },
                "global_costmap": None,
                "local_costmap": None,
                "path": None,
            },
        }

    def navigation_asset(self, name: str) -> BinarySnapshot | None:
        if name == "static":
            return None if self._static_map is None else self._static_map.binary
        if name in {"global_costmap", "local_costmap", "path"}:
            return None
        raise KeyError(name)

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
        if response is None:
            raise ActionUnavailable(
                f"{action_name} service returned no response"
            )
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
