import ast
from pathlib import Path
from xml.etree import ElementTree


ROOT = Path(__file__).parents[1]
NODE_PATH = ROOT / "cmd_vel_gate" / "gate_node.py"
LAUNCH_TEST_PATH = ROOT / "test" / "test_gate_node_launch.py"


def _setup_call():
    tree = ast.parse((ROOT / "setup.py").read_text(encoding="utf-8"))
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "setup"
    ]
    assert len(calls) == 1
    return calls[0]


def _setup_keyword(name):
    keywords = {
        keyword.arg: keyword.value
        for keyword in _setup_call().keywords
    }
    return keywords[name]


def _node_tree():
    return ast.parse(NODE_PATH.read_text(encoding="utf-8"))


def _function(name):
    functions = [
        node
        for node in ast.walk(_node_tree())
        if isinstance(node, ast.FunctionDef) and node.name == name
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


def test_package_and_executable_names():
    package = ElementTree.parse(ROOT / "package.xml").getroot()
    assert package.findtext("name") == "cmd_vel_gate"

    name_keyword = _setup_keyword("name")
    assert isinstance(name_keyword, ast.Name)
    package_assignments = [
        node
        for node in ast.parse(
            (ROOT / "setup.py").read_text(encoding="utf-8")
        ).body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == name_keyword.id
            for target in node.targets
        )
    ]
    assert len(package_assignments) == 1
    assert _constant(package_assignments[0].value) == "cmd_vel_gate"

    entry_points = _setup_keyword("entry_points")
    assert isinstance(entry_points, ast.Dict)
    assert [_constant(key) for key in entry_points.keys] == ["console_scripts"]
    scripts = entry_points.values[0]
    assert isinstance(scripts, ast.List)
    assert [_constant(item) for item in scripts.elts] == [
        "cmd_vel_gate = cmd_vel_gate.gate_node:main"
    ]


def test_package_dependencies_are_minimal_and_complete():
    package = ElementTree.parse(ROOT / "package.xml").getroot()
    assert {node.text for node in package.findall("exec_depend")} == {
        "rclpy",
        "geometry_msgs",
        "std_msgs",
        "std_srvs",
    }
    assert {node.text for node in package.findall("test_depend")} == {
        "ament_pytest",
        "launch_testing",
        "launch_testing_ros",
        "python3-pytest",
    }


def test_node_uses_only_the_required_interfaces():
    init = _function("__init__")

    subscriptions = [
        (
            _name(call.args[0]),
            _constant(call.args[1]),
            _self_attribute(call.args[2]),
            _constant(call.args[3]),
        )
        for call in _attribute_calls(init, "create_subscription")
    ]
    assert len(subscriptions) == 2
    assert set(subscriptions) == {
        ("TwistStamped", "/cmd_vel_auto", "_automatic_callback", 1),
        ("TwistStamped", "/cmd_vel_manual", "_manual_callback", 1),
    }

    publishers = _attribute_calls(init, "create_publisher")
    assert len(publishers) == 2
    assert (
        _name(publishers[0].args[0]),
        _constant(publishers[0].args[1]),
        _constant(publishers[0].args[2]),
    ) == ("TwistStamped", "/cmd_vel", 1)
    assert (
        _name(publishers[1].args[0]),
        _constant(publishers[1].args[1]),
        _name(publishers[1].args[2]),
    ) == ("String", "/cmd_vel_gate/mode", "mode_qos")

    services = [
        (
            _name(call.args[0]),
            _constant(call.args[1]),
            _self_attribute(call.args[2]),
        )
        for call in _attribute_calls(init, "create_service")
    ]
    assert len(services) == 2
    assert set(services) == {
        ("Trigger", "/cmd_vel_gate/takeover_manual", "_takeover_manual"),
        ("Trigger", "/cmd_vel_gate/resume_automatic", "_resume_automatic"),
    }

    tree = _node_tree()
    created_interface_literals = {
        _constant(call.args[1])
        for method in (
            "create_subscription",
            "create_publisher",
            "create_service",
        )
        for call in _attribute_calls(tree, method)
    }
    assert "/driver" not in created_interface_literals
    assert "/motor_speed" not in created_interface_literals


def test_mode_publisher_uses_latched_reliable_depth_one_qos():
    init = _function("__init__")
    qos_calls = [
        call
        for call in ast.walk(init)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "QoSProfile"
    ]
    assert len(qos_calls) == 1
    keywords = {
        keyword.arg: ast.unparse(keyword.value)
        for keyword in qos_calls[0].keywords
    }
    assert keywords == {
        "history": "HistoryPolicy.KEEP_LAST",
        "depth": "1",
        "reliability": "ReliabilityPolicy.RELIABLE",
        "durability": "DurabilityPolicy.TRANSIENT_LOCAL",
    }

    source = NODE_PATH.read_text(encoding="utf-8")
    assert "from std_msgs.msg import String" in source
    assert "self._publish_mode(Mode.AUTOMATIC)" in source

    publish_mode = _function("_publish_mode")
    assert any(
        isinstance(call.func, ast.Attribute)
        and call.func.attr == "publish"
        and ast.unparse(call.args[0]) == "String(data=mode.value)"
        for call in ast.walk(publish_mode)
        if isinstance(call, ast.Call)
    )


def test_node_uses_exact_timeout_and_zero_period():
    constants = {
        target.id: node.value.value
        for node in _node_tree().body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and isinstance(node.value, ast.Constant)
        for target in node.targets
    }
    assert constants["SOURCE_TIMEOUT"] == 0.5
    assert constants["ZERO_PERIOD"] == 0.05


def test_switch_stops_before_zero_and_selects_after_zero():
    switch = _function("_switch")
    stop_calls = [
        call
        for call in _attribute_calls(switch, "stop")
        if isinstance(call.func.value, ast.Attribute)
        and _self_attribute(call.func.value) == "_state"
    ]
    zero_calls = [
        call
        for call in _attribute_calls(switch, "_publish_zero")
        if isinstance(call.func.value, ast.Name)
        and call.func.value.id == "self"
    ]
    select_calls = [
        call
        for call in _attribute_calls(switch, "select")
        if isinstance(call.func.value, ast.Attribute)
        and _self_attribute(call.func.value) == "_state"
    ]
    mode_calls = [
        call
        for call in _attribute_calls(switch, "_publish_mode")
        if isinstance(call.func.value, ast.Name)
        and call.func.value.id == "self"
    ]

    assert (
        len(stop_calls)
        == len(zero_calls)
        == len(select_calls)
        == len(mode_calls)
        == 1
    )
    assert (
        stop_calls[0].lineno
        < zero_calls[0].lineno
        < select_calls[0].lineno
        < mode_calls[0].lineno
    )


def test_launch_test_is_wired_for_pytest():
    tree = ast.parse(LAUNCH_TEST_PATH.read_text(encoding="utf-8"))
    entrypoints = [
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "generate_test_description"
    ]
    assert len(entrypoints) == 1
    assert [
        ast.unparse(decorator)
        for decorator in entrypoints[0].decorator_list
    ] == ["pytest.mark.launch_test"]


def test_launch_mode_subscription_is_created_after_gate_is_ready():
    tree = ast.parse(LAUNCH_TEST_PATH.read_text(encoding="utf-8"))
    setup = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == "setUp"
    ]
    assert len(setup) == 1

    service_waits = _attribute_calls(setup[0], "wait_for_service")
    mode_subscriptions = [
        call
        for call in _attribute_calls(setup[0], "create_subscription")
        if len(call.args) >= 2
        and isinstance(call.args[1], ast.Constant)
        and call.args[1].value == "/cmd_vel_gate/mode"
    ]

    assert len(service_waits) == 2
    assert len(mode_subscriptions) == 1
    assert (
        min(call.lineno for call in service_waits)
        < mode_subscriptions[0].lineno
    )
