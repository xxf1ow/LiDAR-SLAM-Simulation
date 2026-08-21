import importlib
import json
import math
import sys
import threading
import types
from pathlib import Path

import pytest
import yaml

from robot_web_ui.map_snapshot import (
    BinarySnapshot,
    GridInfo,
    GridSnapshot,
    update_grid_snapshot,
    update_path_snapshot,
)
from robot_web_ui.navigation_request import MapRevisionConflict


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

        def create_publisher(self, *args):
            publisher = FakePublisher(args)
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

    class FakePoseWithCovarianceStamped:
        def __init__(self):
            self.header = types.SimpleNamespace(stamp=None, frame_id="")
            self.pose = types.SimpleNamespace(
                pose=types.SimpleNamespace(
                    position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                    orientation=types.SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0
                    ),
                ),
                covariance=[0.0] * 36,
            )

    class FakeNavigateToPose:
        class Goal:
            def __init__(self):
                self.pose = types.SimpleNamespace(
                    header=types.SimpleNamespace(stamp=None, frame_id=""),
                    pose=types.SimpleNamespace(
                        position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                        orientation=types.SimpleNamespace(
                            x=0.0, y=0.0, z=0.0, w=1.0
                        ),
                    ),
                )
                self.behavior_tree = ""

    class FakeGoalStatus:
        STATUS_UNKNOWN = 0
        STATUS_ACCEPTED = 1
        STATUS_EXECUTING = 2
        STATUS_CANCELING = 3
        STATUS_SUCCEEDED = 4
        STATUS_CANCELED = 5
        STATUS_ABORTED = 6

    class FakeCancelGoal:
        class Response:
            def __init__(self, goals_canceling=()):
                self.return_code = 0
                self.goals_canceling = list(goals_canceling)

    class FakeActionClient:
        instances = []

        def __init__(self, node, action_type, action_name):
            self.args = (node, action_type, action_name)
            self.ready = True
            self.goals = []
            self.feedback_callbacks = []
            self.send_future = FakeFuture(complete=False)
            self.__class__.instances.append(self)

        def server_is_ready(self):
            return self.ready

        def send_goal_async(self, goal, feedback_callback=None):
            self.goals.append(goal)
            self.feedback_callbacks.append(feedback_callback)
            return self.send_future

        def emit_feedback(self, feedback_message):
            self.feedback_callbacks[-1](feedback_message)

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
    rclpy_action = types.ModuleType("rclpy.action")
    rclpy_action.ActionClient = FakeActionClient
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
    action_msgs = types.ModuleType("action_msgs")
    action_msgs_msg = types.ModuleType("action_msgs.msg")
    action_msgs_msg.GoalStatus = FakeGoalStatus
    action_msgs_srv = types.ModuleType("action_msgs.srv")
    action_msgs_srv.CancelGoal = FakeCancelGoal
    geometry_msgs = types.ModuleType("geometry_msgs")
    geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
    geometry_msgs_msg.TwistStamped = FakeTwistStamped
    geometry_msgs_msg.PoseWithCovarianceStamped = FakePoseWithCovarianceStamped
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
    nav2_msgs = types.ModuleType("nav2_msgs")
    nav2_msgs_action = types.ModuleType("nav2_msgs.action")
    nav2_msgs_action.NavigateToPose = FakeNavigateToPose
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
        "rclpy.action": rclpy_action,
        "rclpy.node": rclpy_node,
        "rclpy.parameter": rclpy_parameter,
        "rclpy.time": rclpy_time,
        "rclpy.qos": rclpy_qos,
        "ament_index_python": ament,
        "ament_index_python.packages": ament_packages,
        "action_msgs": action_msgs,
        "action_msgs.msg": action_msgs_msg,
        "action_msgs.srv": action_msgs_srv,
        "geometry_msgs": geometry_msgs,
        "geometry_msgs.msg": geometry_msgs_msg,
        "nav_msgs": nav_msgs,
        "nav_msgs.msg": nav_msgs_msg,
        "nav2_msgs": nav2_msgs,
        "nav2_msgs.action": nav2_msgs_action,
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
    module.FakeActionClient = FakeActionClient
    module.FakeCancelGoal = FakeCancelGoal
    module.FakeGoalStatus = FakeGoalStatus
    module.FakeNavigateToPose = FakeNavigateToPose
    yield module
    sys.modules.pop(module_name, None)


class FakePublisher:
    def __init__(self, args=()):
        self.args = args
        self.messages = []
        self.subscription_count = 0

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return self.subscription_count


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
        self.callbacks = []

    def add_done_callback(self, callback):
        if not self.complete:
            self.callbacks.append(callback)
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

    def set_result(self, response):
        self.response = response
        self.error = None
        self._finish()

    def set_exception(self, error):
        self.error = error
        self._finish()

    def _finish(self):
        self.complete = True
        callbacks, self.callbacks = self.callbacks, []
        for callback in callbacks:
            callback(self)


class FakeClientGoalHandle:
    def __init__(
        self,
        *,
        accepted=True,
        result_future=None,
        cancel_future=None,
        goal_id=None,
    ):
        self.accepted = accepted
        self.goal_id = goal_id or fake_goal_id(1)
        self.result_future = result_future or FakeFuture(complete=False)
        self.cancel_future = cancel_future or FakeFuture(complete=False)
        self.get_result_calls = 0
        self.cancel_goal_calls = 0

    def get_result_async(self):
        self.get_result_calls += 1
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_goal_calls += 1
        return self.cancel_future


def fake_goal_id(value):
    return types.SimpleNamespace(uuid=[value] * 16)


def navigation_feedback(distance_remaining, goal_id=None):
    return types.SimpleNamespace(
        goal_id=goal_id or fake_goal_id(1),
        feedback=types.SimpleNamespace(distance_remaining=distance_remaining)
    )


def navigation_result(status):
    return types.SimpleNamespace(status=status, result=types.SimpleNamespace())


def cancel_response(module, *, accepted):
    goals = [types.SimpleNamespace()] if accepted else []
    return module.FakeCancelGoal.Response(goals_canceling=goals)


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
    node._path_snapshot = None
    node._localization_pose = None
    node._localization_error = None
    node._path_error = None
    node._local_layer = (None, None, None, None)
    node._gate_mode = "automatic"
    node._odom_feedback = None
    node._initial_pose_publisher = FakePublisher()
    node._goal_lock = threading.Lock()
    node._goal_status = "idle"
    node._goal_handle = None
    node._goal_distance = None
    node._goal_message = None
    node._navigation_client = module.FakeActionClient(
        node,
        module.FakeNavigateToPose,
        "/navigate_to_pose",
    )
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
        "navigation": {
            "initial_pose_ready": False,
            "action_server_ready": True,
            "goal_status": "idle",
            "cancel_available": False,
            "distance_remaining": None,
            "message": None,
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


def test_global_costmap_wrong_frame_retains_last_valid_snapshot(node_module):
    node = navigation_bare_node(node_module)
    node._global_costmap_callback(
        occupancy_grid(node_module, frame_id="map", data=(12, 34))
    )
    valid = node._global_costmap

    node._global_costmap_callback(
        occupancy_grid(node_module, frame_id="odom", data=(56, 78))
    )

    assert node._global_costmap is valid
    assert node._global_costmap.binary.data == bytes((12, 34))


def test_local_costmap_callback_stores_grid_and_separate_map_transform(
    node_module,
):
    node = navigation_bare_node(node_module)
    node._tf_buffer = node_module.Buffer()
    node._tf_buffer.transform = map_transform()
    grid = occupancy_grid(node_module, frame_id="odom", data=(12, 88))

    node._local_costmap_callback(grid)

    state = node.navigation_state()
    assert node._local_layer[0].info.frame_id == "odom"
    assert node._local_layer[0].binary.data == bytes((12, 88))
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
    local_before = node._local_layer[0]
    node._tf_buffer.error = node_module.TransformException("missing transform")

    node._local_costmap_callback(
        occupancy_grid(node_module, frame_id="odom", data=(1, 2))
    )

    assert node._global_costmap is global_before
    assert node._static_map is static_before
    assert node._local_layer[0] is local_before
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
    node._local_layer = (
        node._global_costmap,
        (1.0, 2.0, 0.3),
        True,
        None,
    )
    node._path_snapshot = update_path_snapshot(
        None, "map", [(3.0, 4.0)]
    )
    node._localization_pose = (5.0, 6.0, 0.7)

    state = node.navigation_state()
    state["layers"]["static"]["origin"][0] = 99.0
    state["layers"]["local_costmap"]["map_from_source"][0] = 99.0
    state["localization"]["x"] = 99.0
    state["navigation"]["message"] = "modified"
    fresh = node.navigation_state()

    assert set(fresh) == {
        "map_error", "localized", "localization", "localization_error",
        "path_error", "gate_mode", "motion", "navigation", "layers",
    }
    assert fresh["motion"] == node.motion_status()
    assert fresh["layers"]["static"]["origin"][0] == -1.0
    assert fresh["layers"]["local_costmap"]["map_from_source"][0] == 1.0
    assert fresh["localization"]["x"] == 5.0
    assert fresh["navigation"] == {
        "initial_pose_ready": False,
        "action_server_ready": True,
        "goal_status": "idle",
        "cancel_available": False,
        "distance_remaining": None,
        "message": None,
    }
    assert fresh["navigation"] is not state["navigation"]


def test_local_costmap_state_reads_one_atomic_layer_projection(node_module):
    node = navigation_bare_node(node_module)
    node._tf_buffer = node_module.Buffer()
    node._tf_buffer.transform = map_transform()
    node._local_costmap_callback(occupancy_grid(node_module, data=(1, 2)))
    old = node._local_layer[0]
    node._local_costmap_callback(occupancy_grid(node_module, data=(3, 4)))
    new = node._local_layer[0]
    node._local_costmap = old
    node._local_map_from_source = (1.0, 2.0, 0.3)
    node._local_transform_available = True
    node._local_transform_error = None
    node._local_layer = (new, (7.0, 8.0, 0.9), True, None)

    local = node.navigation_state()["layers"]["local_costmap"]

    assert local["revision"] == new.binary.revision
    assert local["map_from_source"] == [7.0, 8.0, 0.9]


def test_local_costmap_asset_reads_the_same_atomic_layer_as_state(node_module):
    node = navigation_bare_node(node_module)
    info = GridInfo(2, 1, 0.4, -3.0, 1.5, 0.0, "odom")
    stale = update_grid_snapshot(None, info, b"\x01\x02")
    current = update_grid_snapshot(stale, info, b"\x03\x04")
    node._local_costmap = stale
    node._local_layer = (current, (7.0, 8.0, 0.9), True, None)

    local = node.navigation_state()["layers"]["local_costmap"]

    assert local["revision"] == current.binary.revision
    assert node.navigation_asset("local_costmap") is current.binary


def test_local_costmap_callback_builds_revisions_from_atomic_layer(node_module):
    node = navigation_bare_node(node_module)
    node._tf_buffer = node_module.Buffer()
    node._tf_buffer.transform = map_transform()
    info = GridInfo(2, 1, 0.4, -3.0, 1.5, 0.0, "odom")
    previous = update_grid_snapshot(None, info, b"\x01\x02")
    node._local_layer = (previous, (4.0, -2.0, 0.5), True, None)
    node._local_costmap_callback(
        occupancy_grid(node_module, frame_id="odom", data=(3, 4))
    )

    current = node._local_layer[0]
    assert current.binary.revision == previous.binary.revision + 1
    assert current.binary.data == b"\x03\x04"


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


def initial_pose_payload(**changes):
    payload = {
        "x": -0.95,
        "y": 2.05,
        "yaw": 0.4,
        "map_revision": 3,
    }
    payload.update(changes)
    return payload


def ready_initial_pose_node(module):
    node = navigation_bare_node(module)
    node._static_map = _static_snapshot()
    node._initial_pose_publisher.subscription_count = 1
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: "stamp")
    )
    return node


def test_node_declares_initial_pose_publisher_contract(node_module):
    node = node_module.WebUiNode()

    assert len(node.publishers) == 2
    message_type, topic, qos = node.publishers[1].args
    assert message_type is node_module.PoseWithCovarianceStamped
    assert topic == "/initialpose"
    assert qos.settings["history"] is node_module.HistoryPolicy.KEEP_LAST
    assert qos.settings["depth"] == 1
    assert qos.settings["reliability"] is node_module.ReliabilityPolicy.RELIABLE
    assert qos.settings["durability"] is node_module.DurabilityPolicy.VOLATILE
    node.destroy_node()


@pytest.mark.parametrize("static_map, subscribers", [(None, 1), (_static_snapshot(), 0)])
def test_initial_pose_rejects_unavailable_map_or_subscribers(
    node_module, static_map, subscribers
):
    node = navigation_bare_node(node_module)
    node._static_map = static_map
    node._initial_pose_publisher.subscription_count = subscribers

    with pytest.raises(node_module.ActionUnavailable):
        node.publish_initial_pose(initial_pose_payload())

    assert node._initial_pose_publisher.messages == []


@pytest.mark.parametrize(
    "gate_mode, localization",
    [
        ("manual", None),
        ("manual", (4.0, 5.0, 0.2)),
        ("automatic", None),
        ("automatic", (4.0, 5.0, 0.2)),
    ],
)
def test_initial_pose_publishes_without_mode_or_localization_requirement(
    node_module, gate_mode, localization
):
    node = ready_initial_pose_node(node_module)
    node._gate_mode = gate_mode
    node._localization_pose = localization

    node.publish_initial_pose(initial_pose_payload())

    assert len(node._initial_pose_publisher.messages) == 1
    assert node._localization_pose is localization


@pytest.mark.parametrize("status", ["sending", "navigating", "canceling"])
def test_initial_pose_rejects_active_navigation_without_publishing(
    node_module, status
):
    node = ready_initial_pose_node(node_module)
    node._goal_status = status

    with pytest.raises(node_module.ActionConflict):
        node.publish_initial_pose(initial_pose_payload())

    assert node._initial_pose_publisher.messages == []


def test_initial_pose_message_is_stamped_map_pose_with_zero_covariance(
    node_module,
):
    node = ready_initial_pose_node(node_module)

    node.publish_initial_pose(initial_pose_payload(yaw=math.pi / 2.0))

    message = node._initial_pose_publisher.messages[0]
    assert message.header.stamp == "stamp"
    assert message.header.frame_id == "map"
    assert message.pose.pose.position.x == -0.95
    assert message.pose.pose.position.y == 2.05
    assert message.pose.pose.position.z == 0.0
    assert message.pose.pose.orientation.x == 0.0
    assert message.pose.pose.orientation.y == 0.0
    assert message.pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert message.pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))
    assert message.pose.covariance == [0.0] * 36


@pytest.mark.parametrize(
    "payload, error",
    [
        (initial_pose_payload(map_revision=2), MapRevisionConflict),
        (initial_pose_payload(x=-1.1), ValueError),
        (initial_pose_payload(yaw=float("nan")), ValueError),
        ({"x": -0.95}, ValueError),
    ],
)
def test_invalid_initial_pose_payloads_do_not_publish(
    node_module, payload, error
):
    node = ready_initial_pose_node(node_module)

    with pytest.raises(error):
        node.publish_initial_pose(payload)

    assert node._initial_pose_publisher.messages == []


def test_repeated_initial_pose_requests_publish_once_per_call(node_module):
    node = ready_initial_pose_node(node_module)
    payload = initial_pose_payload()

    node.publish_initial_pose(payload)
    node.publish_initial_pose(payload)

    assert len(node._initial_pose_publisher.messages) == 2
    assert node._initial_pose_publisher.messages[0] is not node._initial_pose_publisher.messages[1]


def test_initial_pose_publish_is_atomic_with_navigation_goal_claim(node_module):
    node = ready_initial_pose_node(node_module)
    node._localization_pose = (0.0, 0.0, 0.0)
    publish_started = threading.Event()
    allow_publish = threading.Event()
    goal_scheduled = threading.Event()
    order = []
    outcomes = []
    real_publish = node._initial_pose_publisher.publish
    real_send_goal = node._navigation_client.send_goal_async

    def blocked_publish(message):
        order.append("publish-started")
        publish_started.set()
        assert allow_publish.wait(timeout=1.0)
        real_publish(message)
        order.append("published")

    def recorded_send_goal(goal, feedback_callback=None):
        order.append("goal-scheduled")
        goal_scheduled.set()
        return real_send_goal(goal, feedback_callback=feedback_callback)

    node._initial_pose_publisher.publish = blocked_publish
    node._navigation_client.send_goal_async = recorded_send_goal
    publish_thread = threading.Thread(
        target=lambda: node.publish_initial_pose(initial_pose_payload())
    )
    goal_thread = threading.Thread(
        target=lambda: outcomes.append(
            node.send_navigation_goal(initial_pose_payload())
        )
    )

    publish_thread.start()
    assert publish_started.wait(timeout=1.0)
    goal_thread.start()
    scheduled_while_publishing = goal_scheduled.wait(timeout=0.05)
    allow_publish.set()
    publish_thread.join(timeout=1.0)
    goal_thread.join(timeout=1.0)

    assert not publish_thread.is_alive()
    assert not goal_thread.is_alive()
    assert not scheduled_while_publishing
    assert outcomes == ["sending"]
    assert len(node._initial_pose_publisher.messages) == 1
    assert order == ["publish-started", "published", "goal-scheduled"]


def test_navigation_goal_claim_blocks_initial_pose_publish(node_module):
    node = ready_initial_pose_node(node_module)
    node._localization_pose = (0.0, 0.0, 0.0)
    goal_claimed = threading.Event()
    allow_send = threading.Event()
    real_send_goal = node._navigation_client.send_goal_async

    def blocked_send_goal(goal, feedback_callback=None):
        goal_claimed.set()
        assert allow_send.wait(timeout=1.0)
        return real_send_goal(goal, feedback_callback=feedback_callback)

    node._navigation_client.send_goal_async = blocked_send_goal
    goal_thread = threading.Thread(
        target=lambda: node.send_navigation_goal(initial_pose_payload())
    )

    goal_thread.start()
    assert goal_claimed.wait(timeout=1.0)
    with pytest.raises(node_module.ActionConflict, match="active"):
        node.publish_initial_pose(initial_pose_payload())
    assert node._initial_pose_publisher.messages == []
    allow_send.set()
    goal_thread.join(timeout=1.0)
    assert not goal_thread.is_alive()


def test_navigation_projection_is_json_native_and_fresh(node_module):
    node = ready_initial_pose_node(node_module)

    state = node.navigation_state()
    state["navigation"]["goal_status"] = "changed"
    fresh = node.navigation_state()

    assert fresh["navigation"] == {
        "initial_pose_ready": True,
        "action_server_ready": True,
        "goal_status": "idle",
        "cancel_available": False,
        "distance_remaining": None,
        "message": None,
    }
    assert fresh["navigation"] is not state["navigation"]


def ready_navigation_node(module):
    node = navigation_bare_node(module)
    node._static_map = _static_snapshot()
    node._initial_pose_publisher.subscription_count = 1
    node._localization_pose = (0.0, 0.0, 0.0)
    node.get_clock = lambda: types.SimpleNamespace(
        now=lambda: types.SimpleNamespace(to_msg=lambda: "goal-stamp")
    )
    return node


def start_navigation(module, node=None):
    node = ready_navigation_node(module) if node is None else node
    handle = FakeClientGoalHandle()
    assert node.send_navigation_goal(initial_pose_payload()) == "sending"
    node._navigation_client.send_future.set_result(handle)
    assert node._goal_status == "navigating"
    return node, handle


def test_node_creates_exact_navigate_to_pose_action_client(node_module):
    node = node_module.WebUiNode()

    assert len(node_module.FakeActionClient.instances) == 1
    client = node_module.FakeActionClient.instances[0]
    assert client.args == (
        node,
        node_module.FakeNavigateToPose,
        "/navigate_to_pose",
    )
    assert node._navigation_client is client
    node.destroy_node()


def test_navigation_goal_rejects_unavailable_server_before_send(node_module):
    node = ready_navigation_node(node_module)
    node._navigation_client.ready = False

    with pytest.raises(node_module.ActionUnavailable, match="action server"):
        node.send_navigation_goal(initial_pose_payload())

    assert node._navigation_client.goals == []
    assert node._goal_status == "idle"


@pytest.mark.parametrize(
    ("gate_mode", "localization", "message"),
    [
        ("manual", (0.0, 0.0, 0.0), "automatic control"),
        ("automatic", None, "localized"),
    ],
)
def test_navigation_goal_rejects_manual_or_unlocalized_before_send(
    node_module, gate_mode, localization, message
):
    node = ready_navigation_node(node_module)
    node._gate_mode = gate_mode
    node._localization_pose = localization

    with pytest.raises(node_module.ActionConflict, match=message):
        node.send_navigation_goal(initial_pose_payload())

    assert node._navigation_client.goals == []
    assert node._goal_status == "idle"


@pytest.mark.parametrize("status", ["sending", "navigating", "canceling"])
def test_navigation_goal_rejects_active_phase_before_send(node_module, status):
    node = ready_navigation_node(node_module)
    node._goal_status = status

    with pytest.raises(node_module.ActionConflict, match="active"):
        node.send_navigation_goal(initial_pose_payload())

    assert node._navigation_client.goals == []
    assert node._goal_status == status


@pytest.mark.parametrize(
    ("payload", "error"),
    [
        (initial_pose_payload(map_revision=2), MapRevisionConflict),
        (initial_pose_payload(x=-1.1), ValueError),
        (initial_pose_payload(yaw=float("inf")), ValueError),
        ({"x": -0.95}, ValueError),
    ],
)
def test_navigation_goal_reuses_pose_validation_before_send(
    node_module, payload, error
):
    node = ready_navigation_node(node_module)

    with pytest.raises(error):
        node.send_navigation_goal(payload)

    assert node._navigation_client.goals == []
    assert node._goal_status == "idle"


def test_navigation_goal_is_exact_stamped_map_pose(node_module):
    node = ready_navigation_node(node_module)

    assert node.send_navigation_goal(
        initial_pose_payload(yaw=math.pi / 2.0)
    ) == "sending"

    assert len(node._navigation_client.goals) == 1
    goal = node._navigation_client.goals[0]
    assert goal.pose.header.frame_id == "map"
    assert goal.pose.header.stamp == "goal-stamp"
    assert goal.pose.pose.position.x == -0.95
    assert goal.pose.pose.position.y == 2.05
    assert goal.pose.pose.position.z == 0.0
    assert goal.pose.pose.orientation.x == 0.0
    assert goal.pose.pose.orientation.y == 0.0
    assert goal.pose.pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert goal.pose.pose.orientation.w == pytest.approx(math.sqrt(0.5))
    assert goal.behavior_tree == ""
    assert node._navigation_client.feedback_callbacks == [
        node._navigation_feedback_callback
    ]


def test_navigation_goal_sets_sending_before_nonblocking_action_call(node_module):
    node = ready_navigation_node(node_module)
    observed = []
    pending = FakeFuture(complete=False)

    def send_goal(goal, feedback_callback=None):
        observed.append(node.navigation_state()["navigation"]["goal_status"])
        return pending

    node._navigation_client.send_goal_async = send_goal

    assert node.send_navigation_goal(initial_pose_payload()) == "sending"
    assert observed == ["sending"]
    assert pending.complete is False


def test_two_concurrent_navigation_goal_calls_schedule_once(node_module):
    node = ready_navigation_node(node_module)
    barrier = threading.Barrier(3)
    outcomes = []

    def send():
        barrier.wait()
        try:
            outcomes.append(node.send_navigation_goal(initial_pose_payload()))
        except BaseException as exc:
            outcomes.append(exc)

    threads = [threading.Thread(target=send) for _ in range(2)]
    for thread in threads:
        thread.start()
    barrier.wait()
    for thread in threads:
        thread.join(timeout=1.0)

    assert all(not thread.is_alive() for thread in threads)
    assert outcomes.count("sending") == 1
    conflicts = [
        outcome
        for outcome in outcomes
        if isinstance(outcome, node_module.ActionConflict)
    ]
    assert len(conflicts) == 1
    assert len(node._navigation_client.goals) == 1


def test_navigation_goal_response_accepts_and_tracks_result(node_module):
    node = ready_navigation_node(node_module)
    handle = FakeClientGoalHandle()

    node.send_navigation_goal(initial_pose_payload())
    node._navigation_client.send_future.set_result(handle)

    assert node._goal_status == "navigating"
    assert node._goal_handle is handle
    assert node._goal_distance is None
    assert node._goal_message is None
    assert handle.get_result_calls == 1
    assert len(handle.result_future.callbacks) == 1


@pytest.mark.parametrize("future_error", [False, True])
def test_navigation_goal_response_rejection_or_error_fails(
    node_module, future_error
):
    node = ready_navigation_node(node_module)
    node.send_navigation_goal(initial_pose_payload())

    if future_error:
        node._navigation_client.send_future.set_exception(
            RuntimeError("response failed")
        )
    else:
        node._navigation_client.send_future.set_result(
            FakeClientGoalHandle(accepted=False)
        )

    assert node._goal_status == "failed"
    assert node._goal_handle is None
    assert node._goal_distance is None
    assert isinstance(node._goal_message, str)


@pytest.mark.parametrize("error", [KeyboardInterrupt(), SystemExit()])
def test_navigation_goal_response_does_not_swallow_process_control_exceptions(
    node_module, error
):
    node = ready_navigation_node(node_module)
    node._goal_status = "sending"

    with pytest.raises(type(error)):
        node._navigation_goal_response_callback(FakeFuture(error=error))

    assert node._goal_status == "sending"


def test_navigation_goal_immediate_send_error_fails_without_scheduling(
    node_module,
):
    node = ready_navigation_node(node_module)

    def fail_send(*_args, **_kwargs):
        raise RuntimeError("send failed")

    node._navigation_client.send_goal_async = fail_send

    with pytest.raises(node_module.ActionUnavailable, match="send failed"):
        node.send_navigation_goal(initial_pose_payload())

    assert node._goal_status == "failed"
    assert node._goal_handle is None
    assert node._goal_distance is None
    assert "send failed" in node._goal_message


def test_navigation_feedback_accepts_only_finite_active_distance(node_module):
    node, handle = start_navigation(node_module)

    node._navigation_client.emit_feedback(
        navigation_feedback(7.25, handle.goal_id)
    )
    assert node._goal_distance == 7.25
    node._navigation_client.emit_feedback(
        navigation_feedback(float("nan"), handle.goal_id)
    )
    node._navigation_client.emit_feedback(
        navigation_feedback(float("inf"), handle.goal_id)
    )
    assert node._goal_distance == 7.25

    node._goal_status = "succeeded"
    node._navigation_client.emit_feedback(
        navigation_feedback(1.0, handle.goal_id)
    )
    assert node._goal_distance == 7.25
    assert node._goal_status == "succeeded"


def test_navigation_feedback_ignores_another_goal_id(node_module):
    node, handle = start_navigation(node_module)
    stale_goal_id = fake_goal_id(2)

    node._navigation_client.emit_feedback(
        navigation_feedback(1.0, stale_goal_id)
    )
    assert node._goal_distance is None

    node._navigation_client.emit_feedback(
        navigation_feedback(2.5, handle.goal_id)
    )
    assert node._goal_distance == 2.5


@pytest.mark.parametrize(
    ("result_status", "expected"),
    [
        ("STATUS_SUCCEEDED", "succeeded"),
        ("STATUS_CANCELED", "canceled"),
        ("STATUS_ABORTED", "failed"),
    ],
)
def test_navigation_result_maps_goal_status_and_clears_active_facts(
    node_module, result_status, expected
):
    node, handle = start_navigation(node_module)
    node._goal_distance = 2.5

    handle.result_future.set_result(
        navigation_result(getattr(node_module.FakeGoalStatus, result_status))
    )

    assert node._goal_status == expected
    assert node._goal_handle is None
    assert node._goal_distance is None
    if expected == "failed":
        assert isinstance(node._goal_message, str)
    else:
        assert node._goal_message is None


def test_navigation_result_future_error_fails_and_clears(node_module):
    node, handle = start_navigation(node_module)
    node._goal_distance = 2.5

    handle.result_future.set_exception(RuntimeError("result failed"))

    assert node._goal_status == "failed"
    assert node._goal_handle is None
    assert node._goal_distance is None
    assert "result failed" in node._goal_message


@pytest.mark.parametrize(
    "status",
    ["idle", "sending", "canceling", "succeeded", "canceled", "failed"],
)
def test_navigation_cancel_rejects_non_navigating_phase(node_module, status):
    node = ready_navigation_node(node_module)
    node._goal_status = status

    with pytest.raises(node_module.ActionConflict, match="not active"):
        node.cancel_navigation()

    assert node._goal_status == status


def test_navigation_cancel_requires_current_goal_handle(node_module):
    node = ready_navigation_node(node_module)
    node._goal_status = "navigating"

    with pytest.raises(node_module.ActionConflict, match="not active"):
        node.cancel_navigation()

    assert node._goal_status == "navigating"


def test_navigation_cancel_schedules_once_and_waits_for_result(node_module):
    node, handle = start_navigation(node_module)

    assert node.cancel_navigation() == "canceling"
    with pytest.raises(node_module.ActionConflict):
        node.cancel_navigation()

    assert handle.cancel_goal_calls == 1
    handle.cancel_future.set_result(cancel_response(node_module, accepted=True))
    assert node._goal_status == "canceling"
    assert node._goal_handle is handle

    handle.result_future.set_result(
        navigation_result(node_module.FakeGoalStatus.STATUS_CANCELED)
    )
    assert node._goal_status == "canceled"
    assert node._goal_handle is None


@pytest.mark.parametrize("future_error", [False, True])
def test_navigation_cancel_rejection_or_error_restores_navigating(
    node_module, future_error
):
    node, handle = start_navigation(node_module)
    node.cancel_navigation()

    if future_error:
        handle.cancel_future.set_exception(RuntimeError("cancel failed"))
    else:
        handle.cancel_future.set_result(
            cancel_response(node_module, accepted=False)
        )

    assert node._goal_status == "navigating"
    assert node._goal_handle is handle
    assert isinstance(node._goal_message, str)


def test_navigation_cancel_immediate_error_restores_navigating(node_module):
    node, handle = start_navigation(node_module)

    def fail_cancel():
        raise RuntimeError("cancel failed")

    handle.cancel_goal_async = fail_cancel

    with pytest.raises(node_module.ActionUnavailable, match="cancel failed"):
        node.cancel_navigation()

    assert node._goal_status == "navigating"
    assert node._goal_handle is handle
    assert "cancel failed" in node._goal_message


def test_late_navigation_callbacks_cannot_reopen_terminal_phase(node_module):
    node = ready_navigation_node(node_module)
    late_handle = FakeClientGoalHandle()
    node.send_navigation_goal(initial_pose_payload())
    node._goal_status = "succeeded"

    node._navigation_client.send_future.set_result(late_handle)
    node._navigation_feedback_callback(
        navigation_feedback(1.0, late_handle.goal_id)
    )
    node._navigation_cancel_response_callback(
        FakeFuture(response=cancel_response(node_module, accepted=False)),
        late_handle,
    )
    node._navigation_result_callback(
        FakeFuture(
            response=navigation_result(
                node_module.FakeGoalStatus.STATUS_ABORTED
            )
        )
    )

    assert node._goal_status == "succeeded"
    assert node._goal_handle is None
    assert node._goal_distance is None
    assert node._goal_message is None
    assert late_handle.get_result_calls == 0


def test_stale_cancel_response_cannot_modify_a_new_canceling_goal(node_module):
    node, handle_a = start_navigation(node_module)
    node.cancel_navigation()
    handle_a.result_future.set_result(
        navigation_result(node_module.FakeGoalStatus.STATUS_CANCELED)
    )

    node._navigation_client.send_future = FakeFuture(complete=False)
    handle_b = FakeClientGoalHandle()
    assert node.send_navigation_goal(initial_pose_payload()) == "sending"
    node._navigation_client.send_future.set_result(handle_b)
    assert node.cancel_navigation() == "canceling"
    message_b = node._goal_message

    handle_a.cancel_future.set_result(
        cancel_response(node_module, accepted=False)
    )

    assert node._goal_status == "canceling"
    assert node._goal_handle is handle_b
    assert node._goal_message is message_b


@pytest.mark.parametrize("terminal", ["succeeded", "canceled", "failed"])
def test_terminal_navigation_phase_permits_new_goal(node_module, terminal):
    node = ready_navigation_node(node_module)
    node._goal_status = terminal
    node._goal_message = "old message"

    assert node.send_navigation_goal(initial_pose_payload()) == "sending"

    assert len(node._navigation_client.goals) == 1
    assert node._goal_status == "sending"
    assert node._goal_handle is None
    assert node._goal_distance is None
    assert node._goal_message is None


@pytest.mark.parametrize(
    ("status", "has_handle", "cancel_available"),
    [
        ("idle", False, False),
        ("sending", False, False),
        ("navigating", False, False),
        ("navigating", True, True),
        ("canceling", True, False),
        ("succeeded", False, False),
        ("canceled", False, False),
        ("failed", False, False),
    ],
)
def test_navigation_state_copies_locked_goal_facts_and_exact_cancel_availability(
    node_module, status, has_handle, cancel_available
):
    node = ready_navigation_node(node_module)
    node._goal_status = status
    node._goal_handle = FakeClientGoalHandle() if has_handle else None
    node._goal_distance = 3.5
    node._goal_message = "working"

    state = node.navigation_state()
    json.dumps(state, allow_nan=False)
    state["navigation"]["goal_status"] = "changed"
    fresh = node.navigation_state()["navigation"]

    assert fresh == {
        "initial_pose_ready": True,
        "action_server_ready": True,
        "goal_status": status,
        "cancel_available": cancel_available,
        "distance_remaining": 3.5,
        "message": "working",
    }


def test_navigation_state_initial_pose_ready_is_subscriber_only(node_module):
    node = navigation_bare_node(node_module)
    node._goal_status = "navigating"
    node._goal_handle = FakeClientGoalHandle()
    node._initial_pose_publisher.subscription_count = 1

    state = node.navigation_state()

    assert state["layers"]["static"] is None
    assert state["navigation"]["goal_status"] == "navigating"
    assert state["navigation"]["initial_pose_ready"] is True


def test_navigation_state_reads_live_action_server_readiness(node_module):
    node = ready_navigation_node(node_module)

    node._navigation_client.ready = False
    assert node.navigation_state()["navigation"]["action_server_ready"] is False
    node._navigation_client.ready = True
    assert node.navigation_state()["navigation"]["action_server_ready"] is True


def test_navigation_source_has_no_copied_status_or_extra_goal_mechanisms():
    source = (
        Path(__file__).parents[1]
        / "robot_web_ui"
        / "web_ui_node.py"
    ).read_text(encoding="utf-8")

    assert "GoalStatus.STATUS_SUCCEEDED" in source
    assert "GoalStatus.STATUS_CANCELED" in source
    assert "GoalStatus.STATUS_ABORTED" in source
    assert "STATUS_SUCCEEDED =" not in source
    assert "STATUS_CANCELED =" not in source
    assert "STATUS_ABORTED =" not in source
    assert "goal_handles" not in source
    assert "goal_generation" not in source
    assert "cancel_all_goals" not in source
    assert "wait_for_server" not in source


def test_navigation_lifecycle_catches_only_ordinary_exceptions():
    source = (
        Path(__file__).parents[1]
        / "robot_web_ui"
        / "web_ui_node.py"
    ).read_text(encoding="utf-8")
    lifecycle = source[
        source.index("    def send_navigation_goal"):
        source.index("    def _now_seconds")
    ]

    assert "except BaseException" not in lifecycle
    assert lifecycle.count("except Exception") == 6
