import importlib
import math
import sys
import threading
import types

import pytest
import yaml

from robot_web_ui.map_snapshot import (
    BinarySnapshot,
    GridInfo,
    GridSnapshot,
    update_path_snapshot,
)


SYNTHETIC_MAX_LINEAR_SPEED = 1.7
SYNTHETIC_MAX_ANGULAR_SPEED = 2.3


@pytest.fixture
def node_module(monkeypatch):
    class FakeNode:
        def __init__(self, *_args):
            self.declared_parameters = {}
            self.publishers = []
            self.subscriptions = []
            self.destroyed = False

        def declare_parameter(self, name, _parameter_type):
            values = {
                "max_linear_speed": SYNTHETIC_MAX_LINEAR_SPEED,
                "max_angular_speed": SYNTHETIC_MAX_ANGULAR_SPEED,
                "host": "127.0.0.1",
                "port": 0,
                "map_yaml_path": "map.yaml",
            }
            value = values[name]
            self.declared_parameters[name] = value
            return types.SimpleNamespace(value=value)

        def create_publisher(self, *_args):
            publisher = FakePublisher()
            self.publishers.append(publisher)
            return publisher

        def create_subscription(self, *args):
            self.subscriptions.append(args)
            return object()

        def create_client(self, *_args):
            return object()

        def get_clock(self):
            return types.SimpleNamespace(
                now=lambda: types.SimpleNamespace(
                    to_msg=lambda: object(), nanoseconds=0
                )
            )

        def get_logger(self):
            return types.SimpleNamespace(info=lambda _message: None)

        def destroy_node(self):
            self.base_destroy_calls = (
                getattr(self, "base_destroy_calls", 0) + 1
            )

    class FakeParameter:
        class Type:
            DOUBLE = object()
            STRING = object()
            INTEGER = object()

    class FakeTwistStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.twist = types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0),
                angular=types.SimpleNamespace(z=0.0),
            )

    class FakeTrigger:
        class Request:
            pass

    class FakeString:
        def __init__(self, data=""):
            self.data = data

    class FakeOdometry:
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id="")
            self.pose = types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0
                    ),
                )
            )
            self.twist = types.SimpleNamespace(
                twist=types.SimpleNamespace(
                    linear=types.SimpleNamespace(x=0.0),
                    angular=types.SimpleNamespace(z=0.0),
                )
            )

    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_parameter = types.ModuleType("rclpy.parameter")
    rclpy_parameter.Parameter = FakeParameter
    rclpy_time = types.ModuleType("rclpy.time")
    rclpy_time.Time = lambda: object()
    rclpy_qos = types.ModuleType("rclpy.qos")
    class FakeQoSProfile:
        def __init__(self, **kwargs):
            self.settings = kwargs

    rclpy_qos.QoSProfile = FakeQoSProfile
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=object())
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE=object())
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(
        TRANSIENT_LOCAL=object(), VOLATILE=object()
    )
    ament = types.ModuleType("ament_index_python")
    ament_packages = types.ModuleType("ament_index_python.packages")
    ament_packages.get_package_share_directory = lambda _name: ""
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.TwistStamped = FakeTwistStamped
    nav_msgs = types.ModuleType("nav_msgs")
    nav_msgs_msg = types.ModuleType("nav_msgs.msg")
    nav_msgs_msg.Odometry = FakeOdometry
    class FakeOccupancyGrid:
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id="")
            self.info = types.SimpleNamespace(
                width=0,
                height=0,
                resolution=0.0,
                origin=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0
                    ),
                ),
            )
            self.data = []

    nav_msgs_msg.OccupancyGrid = FakeOccupancyGrid

    class FakePath:
        def __init__(self):
            self.header = types.SimpleNamespace(frame_id="")
            self.poses = []

    nav_msgs_msg.Path = FakePath
    std_srvs = types.ModuleType("std_srvs")
    std_srvs_srv = types.ModuleType("std_srvs.srv")
    std_srvs_srv.Trigger = FakeTrigger
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = FakeString
    tf2_ros = types.ModuleType("tf2_ros")
    class FakeTransformException(Exception):
        pass

    class FakeBuffer:
        def __init__(self):
            self.transform = None
            self.error = None
            self.lookups = []

        def lookup_transform(self, *args):
            self.lookups.append(args)
            if self.error is not None:
                raise self.error
            return self.transform

    class FakeTransformListener:
        def __init__(self, buffer, node):
            self.buffer = buffer
            self.node = node

    tf2_ros.Buffer = FakeBuffer
    tf2_ros.TransformListener = FakeTransformListener
    tf2_ros.TransformException = FakeTransformException

    modules = {
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.parameter": rclpy_parameter,
        "rclpy.time": rclpy_time,
        "rclpy.qos": rclpy_qos,
        "ament_index_python": ament,
        "ament_index_python.packages": ament_packages,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "std_srvs": std_srvs,
        "std_srvs.srv": std_srvs_srv,
        "tf2_ros": tf2_ros,
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)

    module_name = "robot_web_ui.web_ui_node"
    sys.modules.pop(module_name, None)
    module = importlib.import_module(module_name)
    class FakeServer:
        def serve_forever(self):
            return None

        def shutdown(self):
            return None

        def server_close(self):
            return None

    monkeypatch.setattr(
        module, "create_server", lambda *args, **kwargs: FakeServer()
    )
    module.Buffer = FakeBuffer
    module.TransformException = FakeTransformException
    module.OccupancyGrid = FakeOccupancyGrid
    module.FakePath = FakePath
    yield module
    sys.modules.pop(module_name, None)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeFuture:
    def __init__(
        self,
        *,
        response=None,
        error=None,
        complete=True,
        delay=0.0,
    ):
        self.response = response
        self.error = error
        self.complete = complete
        self.delay = delay

    def add_done_callback(self, callback):
        if not self.complete:
            return
        if self.delay:
            timer = threading.Timer(self.delay, callback, args=(self,))
            timer.start()
            return
        callback(self)

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response


class FakeClient:
    def __init__(self, future, *, ready=True):
        self.future = future
        self.ready = ready
        self.requests = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.requests.append(request)
        return self.future


def response(success=True, message=""):
    return types.SimpleNamespace(success=success, message=message)


def bare_node(module):
    return object.__new__(module.WebUiNode)


def _static_snapshot():
    info = GridInfo(2, 1, 0.05, -1.0, 2.0, 0.25, "map")
    binary = BinarySnapshot(
        3, '"etag-static"', "application/octet-stream", b"\x00\x64", b"gzip"
    )
    return GridSnapshot(info, binary)


def occupancy_grid(module, *, frame_id="map", data=(0, 100)):
    grid = module.OccupancyGrid()
    grid.header.frame_id = frame_id
    grid.info.width = 2
    grid.info.height = 1
    grid.info.resolution = 0.4
    grid.info.origin.position.x = -3.0
    grid.info.origin.position.y = 1.5
    grid.info.origin.orientation.z = 0.0
    grid.info.origin.orientation.w = 1.0
    grid.data = list(data)
    return grid


def map_transform(x=4.0, y=-2.0, yaw=0.5):
    return types.SimpleNamespace(
        transform=types.SimpleNamespace(
            translation=types.SimpleNamespace(x=x, y=y),
            rotation=types.SimpleNamespace(
                x=0.0, y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        )
    )


def navigation_bare_node(module):
    node = bare_node(module)
    node._map_error = None
    node._static_map = None
    node._global_costmap = None
    node._local_costmap = None
    node._path_snapshot = None
    node._localization_pose = None
    node._localization_error = None
    node._path_error = None
    node._local_transform_available = None
    node._local_transform_error = None
    node._local_map_from_source = None
    node._local_layer = (None, None, None, None)
    node._gate_mode = "automatic"
    node._odom_feedback = None
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(nanoseconds=0)
    )
    return node


def test_static_map_load_success_exposes_state_and_asset(node_module, monkeypatch):
    snapshot = _static_snapshot()
    monkeypatch.setattr(node_module, "load_nav2_pgm", lambda _path: snapshot)

    node = node_module.WebUiNode()

    assert node.navigation_state() == {
        "map_error": None,
        "localized": False,
        "localization": None,
        "localization_error": None,
        "path_error": None,
        "gate_mode": None,
        "motion": {
            "linear_x": None,
            "angular_z": None,
            "feedback_fresh": False,
        },
        "layers": {
            "static": {
                **snapshot.info.as_dict(),
                "revision": snapshot.binary.revision,
                "etag": snapshot.binary.etag,
            },
            "global_costmap": None,
            "local_costmap": None,
            "path": None,
        },
    }
    assert node.navigation_asset("static") is snapshot.binary
    node.destroy_node()


def test_localization_callback_exposes_map_pose_and_yaw(node_module):
    node = navigation_bare_node(node_module)
    rejected = node_module.Odometry()
    rejected.header.frame_id = "odom"

    node._localization_callback(rejected)

    assert node.navigation_state()["localization"] is None
    assert node.navigation_state()["localization_error"] == "expected map pose"
    message = node_module.Odometry()
    message.header.frame_id = "map"
    message.pose.pose.position.x = 1.25
    message.pose.pose.position.y = -2.5
    message.pose.pose.orientation.z = math.sin(0.7 / 2.0)
    message.pose.pose.orientation.w = math.cos(0.7 / 2.0)

    node._localization_callback(message)

    state = node.navigation_state()
    assert state["localized"] is True
    assert state["localization"] == {
        "frame_id": "map", "x": 1.25, "y": -2.5, "yaw": pytest.approx(0.7)
    }
    assert state["localization_error"] is None


def test_global_costmap_callback_revisions_only_on_content_or_metadata_change(
    node_module,
):
    node = navigation_bare_node(node_module)
    first = occupancy_grid(node_module, data=(-1, 37))

    node._global_costmap_callback(first)
    equal = node._global_costmap
    node._global_costmap_callback(occupancy_grid(node_module, data=(-1, 37)))
    assert node._global_costmap is equal
    node._global_costmap_callback(occupancy_grid(node_module, data=(-1, 38)))
    content_changed = node._global_costmap
    origin_changed = occupancy_grid(node_module, data=(-1, 38))
    origin_changed.info.origin.position.x += 0.1
    node._global_costmap_callback(origin_changed)

    assert equal.binary.data == bytes((255, 37))
    assert node._global_costmap is not equal
    assert content_changed.binary.revision == equal.binary.revision + 1
    assert node._global_costmap.binary.revision == content_changed.binary.revision + 1


def test_local_costmap_callback_stores_grid_and_separate_map_transform(
    node_module,
):
    node = navigation_bare_node(node_module)
    node._tf_buffer = node_module.Buffer()
    node._tf_buffer.transform = map_transform()
    grid = occupancy_grid(node_module, frame_id="odom", data=(12, 88))

    node._local_costmap_callback(grid)

    state = node.navigation_state()
    assert node._local_costmap.info.frame_id == "odom"
    assert node._local_costmap.binary.data == bytes((12, 88))
    assert state["layers"]["local_costmap"]["map_from_source"] == pytest.approx(
        [4.0, -2.0, 0.5]
    )
    assert state["layers"]["local_costmap"]["transform_available"] is True
    assert node._tf_buffer.lookups[0][:2] == ("map", "odom")


def test_local_costmap_tf_failure_preserves_other_layers_and_reports_unavailable(
    node_module,
):
    node = navigation_bare_node(node_module)
    node._global_costmap_callback(occupancy_grid(node_module))
    node._static_map = _static_snapshot()
    global_before = node._global_costmap
    static_before = node._static_map
    node._tf_buffer = node_module.Buffer()
    node._tf_buffer.transform = map_transform()
    node._local_costmap_callback(occupancy_grid(node_module, frame_id="odom"))
    local_before = node._local_costmap
    node._tf_buffer.error = node_module.TransformException("missing transform")

    node._local_costmap_callback(
        occupancy_grid(node_module, frame_id="odom", data=(1, 2))
    )

    assert node._global_costmap is global_before
    assert node._static_map is static_before
    assert node._local_costmap is local_before
    local = node.navigation_state()["layers"]["local_costmap"]
    assert local["map_from_source"] is None
    assert local["transform_available"] is False
    assert local["transform_error"] == "missing transform"


def test_plan_callback_encodes_only_xy_and_reuses_equal_revision(node_module):
    node = navigation_bare_node(node_module)
    plan = node_module.FakePath()
    plan.header.frame_id = "map"
    plan.poses = [
        types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=1.0, y=2.0, z=9.0),
                orientation=types.SimpleNamespace(x=1.0, y=2.0, z=3.0, w=4.0),
            )
        )
    ]

    node._plan_callback(plan)
    equal = node._path_snapshot
    plan.poses[0].pose.position.z = -9.0
    plan.poses[0].pose.orientation.w = -4.0
    node._plan_callback(plan)
    assert node._path_snapshot is equal
    plan.poses[0].pose.position.x = 1.5
    node._plan_callback(plan)

    assert node._path_snapshot.binary.revision == equal.binary.revision + 1
    assert equal.binary.data != node._path_snapshot.binary.data
    plan.header.frame_id = "odom"
    node._plan_callback(plan)
    assert node._path_snapshot.binary.revision == equal.binary.revision + 1
    assert node.navigation_state()["path_error"] == "expected map path"


def test_navigation_state_is_a_complete_immutable_projection(node_module):
    node = navigation_bare_node(node_module)
    node._static_map = _static_snapshot()
    node._global_costmap_callback(occupancy_grid(node_module, data=(7, 8)))
    node._local_costmap = node._global_costmap
    node._local_transform_available = True
    node._local_map_from_source = (1.0, 2.0, 0.3)
    node._local_layer = (
        node._local_costmap,
        node._local_map_from_source,
        node._local_transform_available,
        node._local_transform_error,
    )
    node._path_snapshot = update_path_snapshot(
        None, "map", [(3.0, 4.0)]
    )
    node._localization_pose = (5.0, 6.0, 0.7)

    state = node.navigation_state()
    state["layers"]["static"]["origin"][0] = 99.0
    state["layers"]["local_costmap"]["map_from_source"][0] = 99.0
    state["localization"]["x"] = 99.0
    fresh = node.navigation_state()

    assert set(fresh) == {
        "map_error", "localized", "localization", "localization_error",
        "path_error", "gate_mode", "motion", "layers",
    }
    assert fresh["motion"] == node.motion_status()
    assert fresh["layers"]["static"]["origin"][0] == -1.0
    assert fresh["layers"]["local_costmap"]["map_from_source"][0] == 1.0
    assert fresh["localization"]["x"] == 5.0


def test_local_costmap_state_reads_one_atomic_layer_projection(node_module):
    node = navigation_bare_node(node_module)
    node._tf_buffer = node_module.Buffer()
    node._tf_buffer.transform = map_transform()
    node._local_costmap_callback(occupancy_grid(node_module, data=(1, 2)))
    old = node._local_costmap
    node._local_costmap_callback(occupancy_grid(node_module, data=(3, 4)))
    new = node._local_costmap
    node._local_costmap = old
    node._local_map_from_source = (1.0, 2.0, 0.3)
    node._local_transform_available = True
    node._local_transform_error = None
    node._local_layer = (new, (7.0, 8.0, 0.9), True, None)

    local = node.navigation_state()["layers"]["local_costmap"]

    assert local["revision"] == new.binary.revision
    assert local["map_from_source"] == [7.0, 8.0, 0.9]


def test_node_declares_exact_visualization_subscriptions_and_qos(node_module):
    node = node_module.WebUiNode()
    visualization = node.subscriptions[2:]

    assert [(kind.__name__, topic, callback.__name__) for kind, topic, callback, _qos in visualization] == [
        ("FakeOdometry", "/localization", "_localization_callback"),
        ("FakePath", "/plan", "_plan_callback"),
        ("FakeOccupancyGrid", "/global_costmap/costmap", "_global_costmap_callback"),
        ("FakeOccupancyGrid", "/local_costmap/costmap", "_local_costmap_callback"),
    ]
    assert visualization[0][3].settings["reliability"] is node_module.ReliabilityPolicy.RELIABLE
    assert visualization[0][3].settings["depth"] == 1
    assert visualization[2][3].settings["durability"] is node_module.DurabilityPolicy.TRANSIENT_LOCAL
    assert len(node.subscriptions) == 6
    node.destroy_node()


@pytest.mark.parametrize(
    "loader_error",
    [
        OSError("missing"),
        TypeError("bad type"),
        ValueError("bad map"),
        yaml.YAMLError("bad yaml"),
    ],
)
def test_static_map_load_failure_is_reported_without_breaking_manual_controls(
    node_module, monkeypatch, loader_error
):
    def fail(_path):
        raise loader_error

    monkeypatch.setattr(node_module, "load_nav2_pgm", fail)
    node = node_module.WebUiNode()
    node._gate_mode = "automatic"

    result = node.manual_command("stop", 20)

    assert result == "automatic"
    message = node._manual_publisher.messages[-1]
    assert message.header.stamp is not None
    assert message.header.frame_id == "base_link"
    assert message.twist.linear.x == 0.0
    assert message.twist.angular.z == 0.0
    assert node._static_map is None
    assert loader_error.__class__.__name__ in node._map_error
    node.destroy_node()


def test_navigation_asset_rejects_unknown_names(node_module):
    node = bare_node(node_module)
    node._static_map = None

    with pytest.raises(KeyError):
        node.navigation_asset("unknown")


def set_clock(node, *seconds):
    readings = iter(seconds)
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(
            nanoseconds=int(next(readings) * 1_000_000_000)
        )
    )


def test_manual_command_publishes_one_stamped_base_link_message(node_module):
    node = bare_node(node_module)
    publisher = FakePublisher()
    stamp = object()
    node._max_linear = SYNTHETIC_MAX_LINEAR_SPEED
    node._max_angular = SYNTHETIC_MAX_ANGULAR_SPEED
    node._gate_mode = "manual"
    node._manual_publisher = publisher
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: stamp)
    )

    assert node.manual_command("forward", 20) == "manual"

    assert len(publisher.messages) == 1
    message = publisher.messages[0]
    assert message.header.stamp is stamp
    assert message.header.frame_id == "base_link"
    assert message.twist.linear.x == pytest.approx(
        SYNTHETIC_MAX_LINEAR_SPEED * 20 / 100.0
    )
    assert message.twist.angular.z == 0.0


def test_nonzero_manual_command_is_rejected_outside_manual_mode(node_module):
    node = bare_node(node_module)
    publisher = FakePublisher()
    node._max_linear = SYNTHETIC_MAX_LINEAR_SPEED
    node._max_angular = SYNTHETIC_MAX_ANGULAR_SPEED
    node._gate_mode = "automatic"
    node._manual_publisher = publisher

    with pytest.raises(
        node_module.ActionConflict,
        match="manual control is not active",
    ) as raised:
        node.manual_command("forward", 20)

    assert raised.value.mode == "automatic"
    assert publisher.messages == []


def test_zero_manual_command_remains_a_safe_noop_outside_manual_mode(
    node_module,
):
    node = bare_node(node_module)
    publisher = FakePublisher()
    node._max_linear = SYNTHETIC_MAX_LINEAR_SPEED
    node._max_angular = SYNTHETIC_MAX_ANGULAR_SPEED
    node._gate_mode = "automatic"
    node._manual_publisher = publisher
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: object())
    )

    assert node.manual_command("stop", 20) == "automatic"

    assert len(publisher.messages) == 1
    assert publisher.messages[0].twist.linear.x == 0.0
    assert publisher.messages[0].twist.angular.z == 0.0


def test_gate_mode_callback_tracks_authoritative_mode(node_module):
    node = bare_node(node_module)
    node._gate_mode = None

    node._gate_mode_callback(types.SimpleNamespace(data="manual"))

    assert node._gate_mode == "manual"


def test_motion_status_is_absent_before_odometry_feedback(node_module):
    node = bare_node(node_module)
    node._odom_feedback = None

    assert node.motion_status() == {
        "linear_x": None,
        "angular_z": None,
        "feedback_fresh": False,
    }


def test_motion_status_returns_fresh_odometry_snapshot(
    node_module,
):
    node = bare_node(node_module)
    set_clock(node, 10.0, 10.49)
    message = node_module.Odometry()
    message.twist.twist.linear.x = 0.3
    message.twist.twist.angular.z = -0.2

    node._odom_callback(message)

    assert node._odom_feedback == (0.3, -0.2, 10.0)
    assert node.motion_status() == {
        "linear_x": 0.3,
        "angular_z": -0.2,
        "feedback_fresh": True,
    }


def test_motion_status_expires_at_timeout_boundary(
    node_module,
):
    node = bare_node(node_module)
    set_clock(node, 10.0, 10.5)
    message = node_module.Odometry()
    message.twist.twist.linear.x = 0.3
    message.twist.twist.angular.z = -0.2

    node._odom_callback(message)

    assert node.motion_status() == {
        "linear_x": None,
        "angular_z": None,
        "feedback_fresh": False,
    }


@pytest.mark.parametrize("reported_mode", ["", "stopped", "MANUAL"])
def test_gate_mode_callback_ignores_non_stable_modes(
    node_module, reported_mode
):
    node = bare_node(node_module)
    node._gate_mode = "automatic"

    node._gate_mode_callback(types.SimpleNamespace(data=reported_mode))

    assert node._gate_mode == "automatic"


def test_mode_service_rejects_unavailable_client(node_module):
    node = bare_node(node_module)
    client = FakeClient(FakeFuture(), ready=False)
    node._takeover_client = client

    with pytest.raises(node_module.ActionUnavailable, match="unavailable"):
        node.takeover_manual()

    assert client.requests == []


def test_mode_service_propagates_success_false(node_module):
    node = bare_node(node_module)
    client = FakeClient(
        FakeFuture(response=response(False, "gate denied"))
    )
    node._takeover_client = client

    with pytest.raises(node_module.ActionUnavailable, match="gate denied"):
        node.takeover_manual()

    assert len(client.requests) == 1


def test_mode_service_rejects_none_response(node_module):
    node = bare_node(node_module)
    node._gate_mode = "automatic"
    node._takeover_client = FakeClient(FakeFuture(response=None))

    with pytest.raises(
        node_module.ActionUnavailable,
        match="manual takeover service returned no response",
    ):
        node.takeover_manual()


def test_future_exception_is_success_when_target_is_observed(node_module):
    node = bare_node(node_module)
    node._gate_mode = "manual"
    node._takeover_client = FakeClient(
        FakeFuture(error=RuntimeError("transport failed"))
    )

    assert node.takeover_manual() == "manual"
    assert len(node._takeover_client.requests) == 1


def test_future_exception_is_pending_when_target_is_not_observed(
    node_module,
):
    node = bare_node(node_module)
    node._gate_mode = "automatic"
    node._takeover_client = FakeClient(
        FakeFuture(error=RuntimeError("transport failed"))
    )

    with pytest.raises(
        node_module.ActionPending,
        match="transport failed",
    ) as raised:
        node.takeover_manual()

    assert raised.value.mode == "automatic"
    assert len(node._takeover_client.requests) == 1


def test_timeout_is_success_when_target_is_observed(
    node_module, monkeypatch
):
    waits = []

    class TimeoutEvent:
        def set(self):
            pass

        def wait(self, timeout):
            waits.append(timeout)
            return False

    monkeypatch.setattr(node_module.threading, "Event", TimeoutEvent)
    node = bare_node(node_module)
    node._gate_mode = "manual"
    node._takeover_client = FakeClient(FakeFuture(complete=False))

    assert node.takeover_manual() == "manual"
    assert waits == [1.0]
    assert len(node._takeover_client.requests) == 1


def test_timeout_is_pending_when_target_is_not_observed(
    node_module, monkeypatch
):
    waits = []

    class TimeoutEvent:
        def set(self):
            pass

        def wait(self, timeout):
            waits.append(timeout)
            return False

    monkeypatch.setattr(node_module.threading, "Event", TimeoutEvent)
    node = bare_node(node_module)
    node._gate_mode = None
    node._takeover_client = FakeClient(FakeFuture(complete=False))

    with pytest.raises(
        node_module.ActionPending,
        match="timed out",
    ) as raised:
        node.takeover_manual()

    assert waits == [1.0]
    assert raised.value.mode is None
    assert len(node._takeover_client.requests) == 1


def test_mode_service_accepts_successful_async_completion(node_module):
    node = bare_node(node_module)
    node._gate_mode = None
    client = FakeClient(
        FakeFuture(response=response(), delay=0.01)
    )
    node._resume_client = client

    assert node.resume_automatic() == "automatic"

    assert len(client.requests) == 1


def test_successful_mode_services_update_local_observation(node_module):
    node = bare_node(node_module)
    node._gate_mode = "automatic"
    node._takeover_client = FakeClient(FakeFuture(response=response()))
    node._resume_client = FakeClient(FakeFuture(response=response()))

    assert node.takeover_manual() == "manual"
    assert node._gate_mode == "manual"

    assert node.resume_automatic() == "automatic"
    assert node._gate_mode == "automatic"


def test_destroy_cleans_up_once_and_is_idempotent(node_module):
    class FakeServer:
        def __init__(self):
            self.shutdown_calls = 0
            self.close_calls = 0

        def shutdown(self):
            self.shutdown_calls += 1

        def server_close(self):
            self.close_calls += 1

    class FakeThread:
        def __init__(self):
            self.alive_checks = 0
            self.join_timeouts = []

        def is_alive(self):
            self.alive_checks += 1
            return True

        def join(self, timeout):
            self.join_timeouts.append(timeout)

    node = bare_node(node_module)
    server = FakeServer()
    thread = FakeThread()
    node._destroy_started = False
    node._http_server = server
    node._http_thread = thread

    node.destroy_node()
    node.destroy_node()

    assert server.shutdown_calls == 1
    assert server.close_calls == 1
    assert thread.alive_checks == 1
    assert thread.join_timeouts == [1.0]
    assert node.base_destroy_calls == 1
