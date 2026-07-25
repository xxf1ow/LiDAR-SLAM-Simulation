import ast
from pathlib import Path
from xml.etree import ElementTree


ROOT = Path(__file__).parents[1]
NODE_PATH = ROOT / "can_driver_web_control" / "web_control_node.py"
RUNTIME_DEPENDENCIES = {
    "ament_index_python",
    "can_driver",
    "launch",
    "launch_ros",
    "rclpy",
    "std_msgs",
}


def test_package_declares_exact_runtime_dependencies():
    tree = ElementTree.parse(ROOT / "package.xml")
    dependencies = {node.text for node in tree.findall(".//exec_depend")}
    assert dependencies == RUNTIME_DEPENDENCIES


def _evaluate_setup_expression(node):
    if isinstance(node, ast.Constant):
        return node.value
    if isinstance(node, ast.Name) and node.id == "package_name":
        return "can_driver_web_control"
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return _evaluate_setup_expression(node.left) + _evaluate_setup_expression(
            node.right
        )
    if isinstance(node, ast.List):
        return [_evaluate_setup_expression(element) for element in node.elts]
    if isinstance(node, ast.Tuple):
        return tuple(_evaluate_setup_expression(element) for element in node.elts)
    if isinstance(node, ast.Dict):
        return {
            _evaluate_setup_expression(key): _evaluate_setup_expression(value)
            for key, value in zip(node.keys, node.values)
        }
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "glob"
    ):
        return ("glob", _evaluate_setup_expression(node.args[0]))
    if (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "join"
        and isinstance(node.func.value, ast.Attribute)
        and node.func.value.attr == "path"
        and isinstance(node.func.value.value, ast.Name)
        and node.func.value.value.id == "os"
    ):
        return "/".join(_evaluate_setup_expression(argument) for argument in node.args)
    raise AssertionError(f"unsupported setup expression: {ast.dump(node)}")


def _setup_keywords():
    tree = ast.parse((ROOT / "setup.py").read_text(encoding="utf-8"))
    package_names = [
        statement.value.value
        for statement in tree.body
        if isinstance(statement, ast.Assign)
        and len(statement.targets) == 1
        and isinstance(statement.targets[0], ast.Name)
        and statement.targets[0].id == "package_name"
        and isinstance(statement.value, ast.Constant)
    ]
    assert package_names == ["can_driver_web_control"]
    calls = [
        statement.value
        for statement in tree.body
        if isinstance(statement, ast.Expr)
        and isinstance(statement.value, ast.Call)
        and isinstance(statement.value.func, ast.Name)
        and statement.value.func.id == "setup"
    ]
    assert len(calls) == 1
    return {keyword.arg: keyword.value for keyword in calls[0].keywords}


def test_setup_structurally_installs_entry_point_launch_and_web_asset():
    keywords = _setup_keywords()
    assert _evaluate_setup_expression(keywords["data_files"]) == [
        (
            "share/ament_index/resource_index/packages",
            ["resource/can_driver_web_control"],
        ),
        ("share/can_driver_web_control", ["package.xml"]),
        (
            "share/can_driver_web_control/launch",
            ("glob", "launch/*.launch.py"),
        ),
        (
            "share/can_driver_web_control/web",
            ["can_driver_web_control/web/index.html"],
        ),
    ]
    assert _evaluate_setup_expression(keywords["entry_points"]) == {
        "console_scripts": [
            "can_driver_web_control = can_driver_web_control.web_control_node:main"
        ]
    }


def _node_tree():
    return ast.parse(NODE_PATH.read_text(encoding="utf-8"))


def _function(tree, name):
    functions = [
        node
        for node in ast.walk(tree)
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == name
    ]
    assert len(functions) == 1
    return functions[0]


def _attribute_calls(tree, name):
    return [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == name
    ]


def _self_method_call(tree, receiver, name):
    calls = [
        node
        for node in _attribute_calls(tree, name)
        if isinstance(node.func.value, ast.Attribute)
        and isinstance(node.func.value.value, ast.Name)
        and node.func.value.value.id == "self"
        and node.func.value.attr == receiver
    ]
    assert len(calls) == 1
    return calls[0]


def _name(node):
    assert isinstance(node, ast.Name)
    return node.id


def _constant(node):
    assert isinstance(node, ast.Constant)
    return node.value


def _self_attribute(node):
    assert isinstance(node, ast.Attribute)
    assert isinstance(node.value, ast.Name)
    assert node.value.id == "self"
    return node.attr


def _dump_expression(source):
    return ast.dump(ast.parse(source, mode="eval").body, include_attributes=False)


def test_ros_node_structurally_uses_required_topics_types_timers_and_connection():
    init = _function(_node_tree(), "__init__")
    publishers = {
        (_name(call.args[0]), _constant(call.args[1]))
        for call in _attribute_calls(init, "create_publisher")
    }
    assert publishers == {
        ("Int16MultiArray", "/motor_speed"),
        ("Int8", "/driver"),
    }

    subscriptions = {
        (
            _name(call.args[0]),
            _constant(call.args[1]),
            _self_attribute(call.args[2]),
        )
        for call in _attribute_calls(init, "create_subscription")
    }
    assert subscriptions == {
        ("Int16MultiArray", "/current_speed", "_feedback_callback")
    }

    timers = {
        (_constant(call.args[0]), _self_attribute(call.args[1]))
        for call in _attribute_calls(init, "create_timer")
    }
    assert timers == {
        (0.05, "_publish_motor"),
        (0.5, "_publish_driver"),
    }

    publish_motor = _function(_node_tree(), "_publish_motor")
    connection_call = _self_method_call(
        publish_motor, "_state", "set_driver_connected"
    )
    assert ast.dump(
        connection_call.args[0], include_attributes=False
    ) == _dump_expression(
        "self._motor_publisher.get_subscription_count() > 0"
    )


def _is_inside(node, statements):
    return any(
        candidate is node
        for statement in statements
        for candidate in ast.walk(statement)
    )


def _runs_in_finally_after(function, before, after):
    return any(
        isinstance(node, ast.Try)
        and _is_inside(before, node.body)
        and _is_inside(after, node.finalbody)
        for node in ast.walk(function)
    )


def _super_destroy_call(function):
    calls = [
        call
        for call in _attribute_calls(function, "destroy_node")
        if isinstance(call.func.value, ast.Call)
        and isinstance(call.func.value.func, ast.Name)
        and call.func.value.func.id == "super"
    ]
    assert len(calls) == 1
    return calls[0]


def test_destroy_is_idempotent_and_every_cleanup_stage_is_in_a_finally():
    destroy = _function(_node_tree(), "destroy_node")
    guard = destroy.body[0]
    assert isinstance(guard, ast.If)
    assert ast.dump(guard.test, include_attributes=False) == _dump_expression(
        "self._destroy_started"
    )
    assert len(guard.body) == 1 and isinstance(guard.body[0], ast.Return)

    mark_started = destroy.body[1]
    assert isinstance(mark_started, ast.Assign)
    assert len(mark_started.targets) == 1
    assert _self_attribute(mark_started.targets[0]) == "_destroy_started"
    assert _constant(mark_started.value) is True

    cleanup_calls = [
        _self_method_call(destroy, "_state", "set_enabled"),
        _self_method_call(destroy, "_motor_publisher", "publish"),
        _self_method_call(destroy, "_driver_publisher", "publish"),
        _self_method_call(destroy, "_http_server", "shutdown"),
        _self_method_call(destroy, "_http_server", "server_close"),
        _self_method_call(destroy, "_http_thread", "join"),
        _super_destroy_call(destroy),
    ]
    for before, after in zip(cleanup_calls, cleanup_calls[1:]):
        assert _runs_in_finally_after(destroy, before, after)

    assert len(_self_method_call(destroy, "_http_thread", "is_alive").args) == 0


def _none_assignment_line(function, attribute):
    lines = [
        node.lineno
        for node in ast.walk(function)
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Attribute)
        and _self_attribute(node.targets[0]) == attribute
        and isinstance(node.value, ast.Constant)
        and node.value.value is None
    ]
    assert len(lines) == 1
    return lines[0]


def test_constructor_binds_before_ros_entities_and_rolls_back_on_failure():
    init = _function(_node_tree(), "__init__")
    server_calls = [
        call
        for call in ast.walk(init)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "create_server"
    ]
    assert len(server_calls) == 1
    create_server_call = server_calls[0]
    entity_calls = [
        call
        for name in ("create_publisher", "create_subscription", "create_timer")
        for call in _attribute_calls(init, name)
    ]
    assert entity_calls
    assert all(create_server_call.lineno < call.lineno for call in entity_calls)

    for attribute in (
        "_http_server",
        "_http_thread",
        "_motor_publisher",
        "_driver_publisher",
    ):
        assert _none_assignment_line(init, attribute) < create_server_call.lineno

    setup_try = next(
        node
        for node in ast.walk(init)
        if isinstance(node, ast.Try)
        and _is_inside(create_server_call, node.body)
        and all(_is_inside(call, node.body) for call in entity_calls)
    )
    rollback_handlers = [
        handler
        for handler in setup_try.handlers
        if isinstance(handler.type, ast.Name) and handler.type.id == "BaseException"
    ]
    assert len(rollback_handlers) == 1
    rollback = rollback_handlers[0]
    rollback_calls = [
        call
        for call in _attribute_calls(rollback, "destroy_node")
        if isinstance(call.func.value, ast.Name) and call.func.value.id == "self"
    ]
    assert len(rollback_calls) == 1
    assert any(isinstance(node, ast.Raise) for node in rollback.body)


def test_main_shutdown_is_in_finally_after_node_cleanup():
    main = _function(_node_tree(), "main")
    spin_try = next(node for node in main.body if isinstance(node, ast.Try))
    assert len(spin_try.finalbody) == 1
    cleanup_try = spin_try.finalbody[0]
    assert isinstance(cleanup_try, ast.Try)
    node_cleanup = [
        call
        for call in _attribute_calls(cleanup_try, "destroy_node")
        if isinstance(call.func.value, ast.Name) and call.func.value.id == "node"
    ]
    shutdown = [
        call
        for call in _attribute_calls(cleanup_try, "shutdown")
        if isinstance(call.func.value, ast.Name) and call.func.value.id == "rclpy"
    ]
    assert len(node_cleanup) == 1
    assert len(shutdown) == 1
    assert _is_inside(node_cleanup[0], cleanup_try.body)
    assert _is_inside(shutdown[0], cleanup_try.finalbody)
