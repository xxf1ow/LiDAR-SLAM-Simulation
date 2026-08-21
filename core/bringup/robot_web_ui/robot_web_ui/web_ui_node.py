from __future__ import annotations

import math
import threading
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .http_server import (
    ActionConflict,
    ActionPending,
    ActionUnavailable,
    create_server,
)
from .manual_command import command_values
from .map_snapshot import (
    BinarySnapshot,
    GridInfo,
    load_nav2_pgm,
    update_grid_snapshot,
    update_path_snapshot,
)
from .navigation_request import parse_navigation_pose, yaw_quaternion


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
        self._path_snapshot = None
        self._localization_pose = None
        self._localization_error = None
        self._path_error = None
        self._local_layer = (None, None, None, None)
        self._goal_lock = threading.Lock()
        self._goal_status = "idle"
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
        initial_pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            initial_pose_qos,
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
        visualization_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._localization_subscription = self.create_subscription(
            Odometry,
            "/localization",
            self._localization_callback,
            visualization_qos,
        )
        self._plan_subscription = self.create_subscription(
            NavPath,
            "/plan",
            self._plan_callback,
            visualization_qos,
        )
        self._global_costmap_subscription = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self._global_costmap_callback,
            costmap_qos,
        )
        self._local_costmap_subscription = self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self._local_costmap_callback,
            costmap_qos,
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
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

    def publish_initial_pose(self, payload: dict[str, object]) -> None:
        static = self._static_map
        if static is None:
            raise ActionUnavailable("static map unavailable")
        pose = parse_navigation_pose(
            payload,
            static.info,
            static.binary.revision,
        )
        if self._initial_pose_publisher.get_subscription_count() <= 0:
            raise ActionUnavailable("initial pose subscriber unavailable")
        with self._goal_lock:
            goal_status = self._goal_status
        if goal_status in {"sending", "navigating", "canceling"}:
            raise ActionConflict("navigation goal is active", self._gate_mode)

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.position.z = 0.0
        message.pose.pose.orientation.x = 0.0
        message.pose.pose.orientation.y = 0.0
        message.pose.pose.orientation.z, message.pose.pose.orientation.w = (
            yaw_quaternion(pose.yaw)
        )
        message.pose.covariance = [0.0] * 36
        self._initial_pose_publisher.publish(message)

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _odom_callback(self, message: Odometry) -> None:
        self._odom_feedback = (
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
            self._now_seconds(),
        )

    @staticmethod
    def _yaw(rotation) -> float:
        return math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )

    @staticmethod
    def _finite(*values: float) -> bool:
        return all(math.isfinite(value) for value in values)

    def _localization_callback(self, message: Odometry) -> None:
        try:
            if message.header.frame_id != "map":
                raise ValueError("expected map frame")
            position = message.pose.pose.position
            yaw = self._yaw(message.pose.pose.orientation)
            x, y = float(position.x), float(position.y)
            if not self._finite(x, y, yaw):
                raise ValueError("expected finite pose")
        except (AttributeError, TypeError, ValueError):
            self._localization_error = "expected map pose"
            return
        self._localization_pose = (x, y, yaw)
        self._localization_error = None

    def _grid_fields(self, message: OccupancyGrid) -> tuple[GridInfo, bytes]:
        width = message.info.width
        height = message.info.height
        resolution = float(message.info.resolution)
        if (
            isinstance(width, bool)
            or isinstance(height, bool)
            or not isinstance(width, int)
            or not isinstance(height, int)
            or width <= 0
            or height <= 0
            or resolution <= 0
        ):
            raise ValueError("invalid grid dimensions")
        origin = message.info.origin
        x, y = float(origin.position.x), float(origin.position.y)
        yaw = self._yaw(origin.orientation)
        if not self._finite(resolution, x, y, yaw):
            raise ValueError("invalid grid geometry")
        frame_id = message.header.frame_id
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("invalid grid frame")
        data = []
        for value in message.data:
            if value == -1:
                data.append(255)
            elif isinstance(value, int) and not isinstance(value, bool) and 0 <= value <= 100:
                data.append(value)
            else:
                raise ValueError("invalid grid value")
        if len(data) != width * height:
            raise ValueError("invalid grid data length")
        return GridInfo(width, height, resolution, x, y, yaw, frame_id), bytes(data)

    def _global_costmap_callback(self, message: OccupancyGrid) -> None:
        try:
            info, data = self._grid_fields(message)
        except (AttributeError, TypeError, ValueError):
            return
        if info.frame_id != "map":
            return
        self._global_costmap = update_grid_snapshot(
            self._global_costmap, info, data
        )

    def _local_costmap_callback(self, message: OccupancyGrid) -> None:
        try:
            info, data = self._grid_fields(message)
            candidate = update_grid_snapshot(
                self._local_layer[0], info, data
            )
            transform = self._tf_buffer.lookup_transform(
                "map", info.frame_id, Time()
            )
            translation = transform.transform.translation
            yaw = self._yaw(transform.transform.rotation)
            x, y = float(translation.x), float(translation.y)
            if not self._finite(x, y, yaw):
                raise ValueError("invalid map transform")
        except TransformException as exc:
            self._local_layer = (
                self._local_layer[0],
                None,
                False,
                str(exc),
            )
            return
        except (AttributeError, TypeError, ValueError):
            return
        self._local_layer = (candidate, (x, y, yaw), True, None)

    def _plan_callback(self, message: NavPath) -> None:
        try:
            if message.header.frame_id != "map":
                raise ValueError("expected map frame")
            points = []
            for pose_stamped in message.poses:
                position = pose_stamped.pose.position
                x, y = float(position.x), float(position.y)
                if not self._finite(x, y):
                    raise ValueError("expected finite points")
                points.append((x, y))
        except (AttributeError, TypeError, ValueError):
            self._path_error = "expected map path"
            return
        self._path_snapshot = update_path_snapshot(
            self._path_snapshot, "map", points
        )
        self._path_error = None

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
        with self._goal_lock:
            goal_status = self._goal_status
        initial_pose_ready = (
            static is not None
            and self._initial_pose_publisher.get_subscription_count() > 0
            and goal_status not in {"sending", "navigating", "canceling"}
        )
        localization = self._localization_pose
        global_costmap = self._grid_state(self._global_costmap)
        (
            local_snapshot,
            local_map_from_source,
            local_transform_available,
            local_transform_error,
        ) = self._local_layer
        local_costmap = self._grid_state(local_snapshot)
        if local_costmap is None and local_transform_error is not None:
            local_costmap = {}
        if local_costmap is not None:
            local_costmap["map_from_source"] = (
                None
                if local_map_from_source is None
                else list(local_map_from_source)
            )
            local_costmap["transform_available"] = local_transform_available
            local_costmap["transform_error"] = local_transform_error
        path = self._path_snapshot
        return {
            "map_error": self._map_error,
            "localized": localization is not None,
            "localization": None if localization is None else {
                "frame_id": "map",
                "x": localization[0],
                "y": localization[1],
                "yaw": localization[2],
            },
            "localization_error": self._localization_error,
            "path_error": self._path_error,
            "gate_mode": self._gate_mode,
            "motion": self.motion_status(),
            "navigation": {
                "initial_pose_ready": initial_pose_ready,
                "action_server_ready": False,
                "goal_status": goal_status,
                "cancel_available": False,
                "distance_remaining": None,
                "message": None,
            },
            "layers": {
                "static": None if static is None else {
                    **static.info.as_dict(),
                    "revision": static.binary.revision,
                    "etag": static.binary.etag,
                },
                "global_costmap": global_costmap,
                "local_costmap": local_costmap,
                "path": None if path is None else {
                    "frame_id": path.frame_id,
                    "revision": path.binary.revision,
                    "etag": path.binary.etag,
                },
            },
        }

    @staticmethod
    def _grid_state(snapshot) -> dict[str, object] | None:
        if snapshot is None:
            return None
        return {
            **snapshot.info.as_dict(),
            "revision": snapshot.binary.revision,
            "etag": snapshot.binary.etag,
        }

    def navigation_asset(self, name: str) -> BinarySnapshot | None:
        if name == "static":
            return None if self._static_map is None else self._static_map.binary
        if name == "global_costmap":
            return None if self._global_costmap is None else self._global_costmap.binary
        if name == "local_costmap":
            snapshot = self._local_layer[0]
            return None if snapshot is None else snapshot.binary
        if name == "path":
            return None if self._path_snapshot is None else self._path_snapshot.binary
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
