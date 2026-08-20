import ast
from pathlib import Path
from xml.etree import ElementTree

import yaml


ROOT = Path(__file__).parents[1]
NODE_PATH = ROOT / "robot_web_ui" / "web_ui_node.py"
TEMPLATE_PATH = (
    ROOT.parent
    / "system_bringup"
    / "config"
    / "templates"
    / "robot_web_ui.yaml"
)


def _native_parameter_type(value):
    return {
        bool: "Parameter.Type.BOOL",
        int: "Parameter.Type.INTEGER",
        float: "Parameter.Type.DOUBLE",
        str: "Parameter.Type.STRING",
    }[type(value)]


def _template_parameter_types():
    parameters = yaml.safe_load(TEMPLATE_PATH.read_text(encoding="utf-8"))[
        "robot_web_ui"
    ]["ros__parameters"]
    return {
        name: _native_parameter_type(value)
        for name, value in parameters.items()
        if name != "use_sim_time"
    }


def test_package_declares_only_required_map_tf_dependencies():
    tree = ElementTree.parse(ROOT / "package.xml")
    dependencies = {node.text for node in tree.findall(".//exec_depend")}

    assert dependencies == {
        "ament_index_python",
        "geometry_msgs",
        "nav_msgs",
        "python3-yaml",
        "rclpy",
        "std_msgs",
        "std_srvs",
        "tf2_ros",
    }


def test_setup_installs_web_asset_and_entry_point():
    source = (ROOT / "setup.py").read_text(encoding="utf-8")

    assert '"robot_web_ui/web/index.html"' in source
    assert '"robot_web_ui/web/map_view.js"' in source
    assert (
        '"robot_web_ui = robot_web_ui.web_ui_node:main"'
        in source
    )


def test_map_yaml_path_is_the_only_required_runtime_only_parameter():
    source = NODE_PATH.read_text(encoding="utf-8")
    tree = ast.parse(source)
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]

    publishers = [
        call
        for call in calls
        if isinstance(call.func, ast.Attribute)
        and call.func.attr == "create_publisher"
    ]
    assert len(publishers) == 1
    assert ast.unparse(publishers[0].args[0]) == "TwistStamped"
    assert ast.literal_eval(publishers[0].args[1]) == "/cmd_vel_manual"

    subscriptions = [
        call
        for call in calls
        if isinstance(call.func, ast.Attribute)
        and call.func.attr == "create_subscription"
    ]
    assert [
        (
            ast.unparse(call.args[0]),
            ast.literal_eval(call.args[1]),
            ast.unparse(call.args[2]),
        )
        for call in subscriptions[:2]
    ] == [
        ("String", "/cmd_vel_gate/mode", "self._gate_mode_callback"),
        ("Odometry", "/base_controller/odom", "self._odom_callback"),
    ]
    assert ast.unparse(subscriptions[0].args[3]) == "mode_qos"
    assert ast.literal_eval(subscriptions[1].args[3]) == 10

    qos_profiles = [
        call
        for call in calls
        if isinstance(call.func, ast.Name)
        and call.func.id == "QoSProfile"
    ]
    assert len(qos_profiles) == 3
    assert {
        keyword.arg: ast.unparse(keyword.value)
        for keyword in qos_profiles[0].keywords
    } == {
        "history": "HistoryPolicy.KEEP_LAST",
        "depth": "1",
        "reliability": "ReliabilityPolicy.RELIABLE",
        "durability": "DurabilityPolicy.TRANSIENT_LOCAL",
    }

    clients = [
        call
        for call in calls
        if isinstance(call.func, ast.Attribute)
        and call.func.attr == "create_client"
    ]
    assert {
        (ast.unparse(call.args[0]), ast.literal_eval(call.args[1]))
        for call in clients
    } == {
        ("Trigger", "/cmd_vel_gate/takeover_manual"),
        ("Trigger", "/cmd_vel_gate/resume_automatic"),
    }

    parameters = [
        (
            ast.literal_eval(call.args[0]),
            ast.unparse(call.args[1]) if len(call.args) > 1 else None,
            len(call.args),
            len(call.keywords),
        )
        for call in calls
        if isinstance(call.func, ast.Attribute)
        and call.func.attr == "declare_parameter"
    ]
    expected_parameters = _template_parameter_types()
    template_parameters = [
        parameter for parameter in parameters if parameter[0] != "map_yaml_path"
    ]
    runtime_parameters = [
        parameter for parameter in parameters if parameter[0] == "map_yaml_path"
    ]
    parameter_names = [name for name, _type, _args, _keywords in template_parameters]
    assert len(template_parameters) == len(expected_parameters)
    assert len(parameter_names) == len(set(parameter_names))
    assert {
        name: parameter_type
        for name, parameter_type, _args, _keywords in template_parameters
    } == expected_parameters
    assert all(
        argument_count == 2 and keyword_count == 0
        for _name, _type, argument_count, keyword_count in template_parameters
    )
    assert runtime_parameters == [
        ("map_yaml_path", "Parameter.Type.STRING", 2, 0)
    ]
    assert any(
        isinstance(node, ast.ImportFrom)
        and node.module == "rclpy.parameter"
        and any(
            alias.name == "Parameter" and alias.asname is None
            for alias in node.names
        )
        for node in tree.body
    )


def test_node_stamps_manual_messages_and_bounds_service_wait():
    source = NODE_PATH.read_text(encoding="utf-8")

    assert 'message.header.frame_id = "base_link"' in source
    assert "self.get_clock().now().to_msg()" in source
    assert "threading.Event()" in source
    assert ".wait(timeout=1.0)" in source
    assert "call_async(Trigger.Request())" in source
    assert "serve_forever" in source
    assert "server_close()" in source
    assert "join(timeout=1.0)" in source
    assert "/driver" not in source
    assert "/motor_speed" not in source
