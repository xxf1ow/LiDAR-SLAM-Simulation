import ast
from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "navigation.launch.py"
TEMPLATE_PACKAGES = {
    "nav2_map_server",
    "nav2_planner",
    "nav2_controller",
    "nav2_behaviors",
    "nav2_bt_navigator",
}


def _tree():
    return ast.parse(LAUNCH.read_text(encoding="utf-8"))


def _function(tree, name):
    return next(
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def _calls(node, name):
    return [
        item
        for item in ast.walk(node)
        if isinstance(item, ast.Call)
        and isinstance(item.func, ast.Name)
        and item.func.id == name
    ]


def _keyword(call, name):
    return next(keyword.value for keyword in call.keywords if keyword.arg == name)


def _string(node):
    return node.value if isinstance(node, ast.Constant) and isinstance(node.value, str) else None


def _dict_value(dictionary, key):
    for item_key, value in zip(dictionary.keys, dictionary.values):
        if _string(item_key) == key:
            return value
    raise AssertionError(f"dictionary key {key!r} not found")


def _node_calls(function):
    return _calls(function, "Node")


def _node_package(call):
    return _string(_keyword(call, "package"))


def _node(function, package):
    return next(call for call in _node_calls(function) if _node_package(call) == package)


def _declaration_default(tree, name):
    declaration = next(
        call
        for call in _calls(tree, "DeclareLaunchArgument")
        if call.args and _string(call.args[0]) == name
    )
    return _keyword(declaration, "default_value")


def _assert_clock_parameter(call):
    parameters = _keyword(call, "parameters")
    assert isinstance(parameters, ast.List)
    assert len(parameters.elts) == 1
    parameter_dict = parameters.elts[0]
    assert isinstance(parameter_dict, ast.Dict)
    clock = _dict_value(parameter_dict, "use_sim_time")
    assert isinstance(clock, ast.Name)
    assert clock.id == "use_sim_time"


def test_navigation_exposes_only_quaternion_weld_arguments():
    tree = _tree()
    declared = {
        _string(call.args[0])
        for call in _calls(tree, "DeclareLaunchArgument")
        if call.args and _string(call.args[0]).startswith("weld_")
    }
    assert declared == {
        "weld_x", "weld_y", "weld_z",
        "weld_qx", "weld_qy", "weld_qz", "weld_qw",
    }
    for name, expected in {
        "weld_x": "0.0", "weld_y": "0.0",
        "weld_qx": "0.0", "weld_qy": "0.0", "weld_qz": "0.0", "weld_qw": "1.0",
    }.items():
        assert _string(_declaration_default(tree, name)) == expected
    assert ast.unparse(_declaration_default(tree, "weld_z")) == "f'{_WELD_Z:.4f}'"

    function = _function(tree, "generate_launch_description")
    publisher = _node(function, "tf2_ros")
    arguments = _keyword(publisher, "arguments")
    assert isinstance(arguments, ast.List)
    transform = {
        _string(arguments.elts[index]): arguments.elts[index + 1]
        for index in range(0, 14, 2)
    }
    assert set(transform) == {"--x", "--y", "--z", "--qx", "--qy", "--qz", "--qw"}
    for option, variable in (
        ("--x", "weld_x"), ("--y", "weld_y"), ("--z", "weld_z"),
        ("--qx", "weld_qx"), ("--qy", "weld_qy"),
        ("--qz", "weld_qz"), ("--qw", "weld_qw"),
    ):
        assert isinstance(transform[option], ast.Name)
        assert transform[option].id == variable


def test_nav2_template_nodes_load_only_generated_params_file():
    function = _function(_tree(), "generate_launch_description")
    nodes = {
        _node_package(call): call
        for call in _node_calls(function)
        if _node_package(call) in TEMPLATE_PACKAGES
    }
    assert set(nodes) == TEMPLATE_PACKAGES

    for package in TEMPLATE_PACKAGES - {"nav2_map_server"}:
        parameters = _keyword(nodes[package], "parameters")
        assert isinstance(parameters, ast.List)
        assert len(parameters.elts) == 1
        assert isinstance(parameters.elts[0], ast.Name)
        assert parameters.elts[0].id == "params_file"

    map_parameters = _keyword(nodes["nav2_map_server"], "parameters")
    assert isinstance(map_parameters, ast.List)
    assert len(map_parameters.elts) == 2
    assert isinstance(map_parameters.elts[0], ast.Name)
    assert map_parameters.elts[0].id == "params_file"
    override = map_parameters.elts[1]
    assert isinstance(override, ast.Dict)
    assert {_string(key) for key in override.keys} == {"yaml_filename"}


def test_non_template_nodes_share_launch_clock_and_command_route():
    function = _function(_tree(), "generate_launch_description")
    for package in ("tf2_ros", "robot_navigation", "nav2_lifecycle_manager", "rviz2"):
        _assert_clock_parameter(_node(function, package))

    stamper_parameters = _keyword(_node(function, "robot_navigation"), "parameters").elts[0]
    assert _string(_dict_value(stamper_parameters, "input_topic")) == "/cmd_vel_nav"
    output = _dict_value(stamper_parameters, "output_topic")
    assert isinstance(output, ast.Name)
    assert output.id == "cmd_vel_output_topic"
