import ast
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
MODULE_ROOT = PACKAGE_ROOT.parent
LAUNCH_FILE = PACKAGE_ROOT / "launch" / "can_driver_web_test.launch.py"
README_FILE = MODULE_ROOT / "README.md"


def _string_keyword(call, name):
    keyword = next(item for item in call.keywords if item.arg == name)
    return keyword.value.value


def test_launch_starts_both_nodes_and_shuts_down_with_web_node():
    assert LAUNCH_FILE.is_file(), f"missing launch file: {LAUNCH_FILE}"
    tree = ast.parse(LAUNCH_FILE.read_text(encoding="utf-8"))

    generate = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "generate_launch_description"
    )
    assignments = {
        statement.targets[0].id: statement.value
        for statement in generate.body
        if isinstance(statement, ast.Assign)
        and isinstance(statement.targets[0], ast.Name)
    }
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    node_calls = [
        call
        for call in calls
        if isinstance(call.func, ast.Name) and call.func.id == "Node"
    ]
    configured_nodes = {
        (_string_keyword(call, "package"), _string_keyword(call, "executable"))
        for call in node_calls
    }
    assert configured_nodes == {
        ("can_driver", "can_driver_8030"),
        ("can_driver_web_control", "can_driver_web_control"),
    }
    driver_call = next(
        call
        for call in node_calls
        if _string_keyword(call, "package") == "can_driver"
    )
    assert _string_keyword(driver_call, "output") == "log"
    parameters = next(
        item.value for item in driver_call.keywords if item.arg == "parameters"
    )
    assert ast.literal_eval(parameters) == [{"auto_enable_on_start": False}]

    handler = assignments["stop_all_if_web_exits"]
    assert isinstance(handler, ast.Call)
    assert isinstance(handler.func, ast.Name)
    assert handler.func.id == "RegisterEventHandler"
    on_exit = handler.args[0]
    assert isinstance(on_exit, ast.Call)
    assert isinstance(on_exit.func, ast.Name)
    assert on_exit.func.id == "OnProcessExit"
    target = next(item.value for item in on_exit.keywords if item.arg == "target_action")
    assert isinstance(target, ast.Name)
    assert target.id == "web_node"

    on_exit_actions = next(
        item.value for item in on_exit.keywords if item.arg == "on_exit"
    )
    assert isinstance(on_exit_actions, ast.List)
    emit_event = on_exit_actions.elts[0]
    assert isinstance(emit_event, ast.Call)
    assert isinstance(emit_event.func, ast.Name)
    assert emit_event.func.id == "EmitEvent"
    shutdown = next(
        item.value for item in emit_event.keywords if item.arg == "event"
    )
    assert isinstance(shutdown, ast.Call)
    assert isinstance(shutdown.func, ast.Name)
    assert shutdown.func.id == "Shutdown"

    returned = next(
        statement.value
        for statement in generate.body
        if isinstance(statement, ast.Return)
    )
    assert isinstance(returned, ast.Call)
    assert isinstance(returned.func, ast.Name)
    assert returned.func.id == "LaunchDescription"
    assert [item.id for item in returned.args[0].elts] == [
        "driver_node",
        "web_node",
        "stop_all_if_web_exits",
    ]


def test_readme_has_copy_build_run_and_hardware_checks():
    assert README_FILE.is_file(), f"missing module README: {README_FILE}"
    source = README_FILE.read_text(encoding="utf-8")

    required_commands = [
        "mkdir -p ~/can_test_ws/src",
        "cp -a /path/to/can_driver_8030D_sdk ~/can_test_ws/src/",
        "cp -a /path/to/can_driver_web_control ~/can_test_ws/src/",
        "sudo cp ~/can_test_ws/src/can_driver_8030D_sdk/lib/libcontrolcan.so /usr/local/lib/",
        "sudo ldconfig",
        "source /opt/ros/humble/setup.bash",
        "colcon build --packages-select can_driver can_driver_web_control",
        "source install/setup.bash",
        "colcon test --packages-select can_driver_web_control",
        "colcon test-result --verbose",
        "ros2 launch can_driver_web_control can_driver_web_test.launch.py",
        "hostname -I",
    ]
    for command in required_commands:
        assert command in source

    required_text = [
        "Ubuntu 22.04",
        "ROS 2 Humble",
        "ARM aarch64",
        "HARDWARE_SETUP.md",
        "udev",
        "http://<构建机IP>:8080",
        "架空",
        "物理",
        "没有使能成功/失败反馈",
        "第一路取反",
        "[右轮, 左轮]",
        "不能作为生产控制链",
    ]
    for text in required_text:
        assert text in source
