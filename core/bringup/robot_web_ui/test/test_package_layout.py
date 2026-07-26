import ast
from pathlib import Path
from xml.etree import ElementTree


ROOT = Path(__file__).parents[1]
NODE_PATH = ROOT / "robot_web_ui" / "web_ui_node.py"


def test_package_declares_runtime_dependencies():
    tree = ElementTree.parse(ROOT / "package.xml")
    dependencies = {node.text for node in tree.findall(".//exec_depend")}

    assert dependencies == {
        "ament_index_python",
        "geometry_msgs",
        "rclpy",
        "std_srvs",
    }


def test_setup_installs_web_asset_and_entry_point():
    source = (ROOT / "setup.py").read_text(encoding="utf-8")

    assert '"robot_web_ui/web/index.html"' in source
    assert (
        '"robot_web_ui = robot_web_ui.web_ui_node:main"'
        in source
    )


def test_node_has_exact_ros_contract_and_parameters():
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

    parameters = {
        (
            ast.literal_eval(call.args[0]),
            ast.literal_eval(call.args[1]),
        )
        for call in calls
        if isinstance(call.func, ast.Attribute)
        and call.func.attr == "declare_parameter"
    }
    assert parameters == {
        ("max_linear_speed", 1.5),
        ("max_angular_speed", 2.0),
        ("host", "0.0.0.0"),
        ("port", 8080),
    }


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
