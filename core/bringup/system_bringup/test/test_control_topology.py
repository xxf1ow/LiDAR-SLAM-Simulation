import ast
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[4]
NAVIGATION = ROOT / "core/navigation/robot_navigation/launch/navigation.launch.py"
SLAM_STACK = ROOT / "core/bringup/system_bringup/launch/slam_stack.launch.py"
BRINGUP = ROOT / "core/bringup/system_bringup/launch/bringup.launch.py"
MANIFEST = ROOT / "core/bringup/system_bringup/package.xml"


def _tree(path):
    return ast.parse(path.read_text(encoding="utf-8"))


def _function(tree, name):
    return next(
        node
        for node in tree.body
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


def _declaration_default(tree, argument):
    for call in _calls(tree, "DeclareLaunchArgument"):
        if call.args and _string(call.args[0]) == argument:
            return _string(_keyword(call, "default_value"))
    return None


def _launch_configuration_assignment(function, variable, argument, performed=False):
    assignment = next(
        node
        for node in ast.walk(function)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == variable for target in node.targets)
    )
    value = assignment.value
    if performed:
        assert isinstance(value, ast.Call)
        assert isinstance(value.func, ast.Attribute)
        assert value.func.attr == "perform"
        value = value.func.value
    return (
        isinstance(value, ast.Call)
        and isinstance(value.func, ast.Name)
        and value.func.id == "LaunchConfiguration"
        and len(value.args) == 1
        and _string(value.args[0]) == argument
    )


def _node_call(function, package, executable):
    return next(
        call
        for call in _calls(function, "Node")
        if _string(_keyword(call, "package")) == package
        and _string(_keyword(call, "executable")) == executable
    )


def _include_arguments(function, package, launch_file):
    include = next(
        call
        for call in _calls(function, "_inc")
        if len(call.args) >= 3
        and _string(call.args[0]) == package
        and _string(call.args[1]) == launch_file
    )
    assert isinstance(include.args[2], ast.Dict)
    return include.args[2]


def _platform_branch(function, platform):
    return next(
        node
        for node in function.body
        if isinstance(node, ast.If)
        and isinstance(node.test, ast.Compare)
        and isinstance(node.test.left, ast.Name)
        and node.test.left.id == "platform"
        and any(_string(comparator) == platform for comparator in node.test.comparators)
    )


def _leftmost_add_name(node):
    while isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        node = node.left
    return node.id if isinstance(node, ast.Name) else None


def _subscript_path(node):
    path = []
    while isinstance(node, ast.Subscript):
        path.append(
            node.slice.id if isinstance(node.slice, ast.Name) else _string(node.slice)
        )
        node = node.value
    return node.id if isinstance(node, ast.Name) else None, tuple(reversed(path))


def test_navigation_declares_cmd_vel_output_topic_with_legacy_default():
    assert _declaration_default(_tree(NAVIGATION), "cmd_vel_output_topic") == "/cmd_vel"


def test_navigation_stamper_uses_configured_output_and_keeps_nav_input():
    function = _function(_tree(NAVIGATION), "generate_launch_description")
    assert _launch_configuration_assignment(
        function, "cmd_vel_output_topic", "cmd_vel_output_topic"
    )
    stamper = _node_call(function, "robot_navigation", "twist_stamper")
    parameters = _keyword(stamper, "parameters")
    assert isinstance(parameters, ast.List)
    topic_parameters = parameters.elts[0]
    assert isinstance(topic_parameters, ast.Dict)
    assert _string(_dict_value(topic_parameters, "input_topic")) == "/cmd_vel_nav"
    output_topic = _dict_value(topic_parameters, "output_topic")
    assert isinstance(output_topic, ast.Name)
    assert output_topic.id == "cmd_vel_output_topic"


def test_slam_stack_declares_cmd_vel_output_topic_with_legacy_default():
    assert _declaration_default(_tree(SLAM_STACK), "cmd_vel_output_topic") == "/cmd_vel"


def test_slam_stack_passes_cmd_vel_output_topic_to_navigation():
    function = _function(_tree(SLAM_STACK), "_stack")
    assert _launch_configuration_assignment(
        function, "cmd_vel_output_topic", "cmd_vel_output_topic", performed=True
    )
    arguments = _include_arguments(
        function, "robot_navigation", "launch/navigation.launch.py"
    )
    output_topic = _dict_value(arguments, "cmd_vel_output_topic")
    assert isinstance(output_topic, ast.Name)
    assert output_topic.id == "cmd_vel_output_topic"


def test_slam_stack_declares_profile_parameter_files():
    tree = _tree(SLAM_STACK)
    declared = {
        _string(call.args[0])
        for call in _calls(tree, "DeclareLaunchArgument")
        if call.args
    }
    assert {
        "lio_sam_params_file",
        "gicp_config_file",
        "nav2_params_file",
    } <= declared


def test_slam_stack_passes_profile_parameter_files_to_includes():
    function = _function(_tree(SLAM_STACK), "_stack")
    for variable, argument in (
        ("lio_sam_params", "lio_sam_params_file"),
        ("gicp_config", "gicp_config_file"),
        ("nav2_params", "nav2_params_file"),
    ):
        assert _launch_configuration_assignment(
            function, variable, argument, performed=True
        )

    mapping = _include_arguments(function, "lio_sam", "launch/run.launch.py")
    mapping_params = _dict_value(mapping, "params_file")
    assert isinstance(mapping_params, ast.Name)
    assert mapping_params.id == "lio_sam_params"

    gicp = _include_arguments(
        function, "gicp_localization", "launch/localization.launch.py"
    )
    gicp_params = _dict_value(gicp, "config_file")
    assert isinstance(gicp_params, ast.Name)
    assert gicp_params.id == "gicp_config"

    nav2 = _include_arguments(
        function, "robot_navigation", "launch/navigation.launch.py"
    )
    nav2_params = _dict_value(nav2, "params_file")
    assert isinstance(nav2_params, ast.Name)
    assert nav2_params.id == "nav2_params"


def test_slam_stack_waits_for_localization_and_base_controller_odom_before_nav2():
    function = _function(_tree(SLAM_STACK), "_stack")
    assert any(
        isinstance(call.args[0], ast.List)
        and {_string(item) for item in call.args[0].elts}
        == {"/localization", "/base_controller/odom"}
        for call in _calls(function, "ready_gate")
    )


def test_bringup_routes_navigation_output_to_cmd_vel_auto():
    function = _function(_tree(BRINGUP), "_bringup")
    arguments = _include_arguments(
        function, "system_bringup", "launch/slam_stack.launch.py"
    )
    assert _string(_dict_value(arguments, "cmd_vel_output_topic")) == "/cmd_vel_auto"


def test_bringup_selects_the_slam_profile_from_platform():
    function = _function(_tree(BRINGUP), "_bringup")
    profile_assignment = next(
        node
        for node in ast.walk(function)
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "profile"
            for target in node.targets
        )
    )
    assert _subscript_path(profile_assignment.value) == ("stack_cfg", ("platform",))


def test_bringup_passes_selected_profile_configs_to_shared_slam_stack():
    function = _function(_tree(BRINGUP), "_bringup")
    arguments = _include_arguments(
        function, "system_bringup", "launch/slam_stack.launch.py"
    )
    for argument, package, component in (
        ("lio_sam_params_file", "lio_sam", "lio_sam"),
        ("gicp_config_file", "gicp_localization", "gicp_localization"),
        ("nav2_params_file", "robot_navigation", "robot_navigation"),
    ):
        value = _dict_value(arguments, argument)
        assert isinstance(value, ast.Call)
        assert isinstance(value.func, ast.Name)
        assert value.func.id == "_pkg_config"
        assert _string(value.args[0]) == package
        assert _subscript_path(value.args[1]) == (
            "profile", (component, "config")
        )

    assert _subscript_path(_dict_value(arguments, "fast_lio_config")) == (
        "profile", ("fast_lio", "config")
    )


def test_bringup_creates_gate_and_web_nodes_before_platform_branching():
    function = _function(_tree(BRINGUP), "_bringup")
    first_platform_branch = min(
        _platform_branch(function, "sim").lineno,
        _platform_branch(function, "real").lineno,
    )
    for package in ("cmd_vel_gate", "robot_web_ui"):
        node = _node_call(function, package, package)
        assert node.lineno < first_platform_branch
        assert _string(_keyword(node, "output")) == "screen"


def test_bringup_prepends_control_layer_to_both_platform_returns():
    function = _function(_tree(BRINGUP), "_bringup")
    for platform in ("sim", "real"):
        branch = _platform_branch(function, platform)
        result = next(node.value for node in branch.body if isinstance(node, ast.Return))
        assert _leftmost_add_name(result) == "control_layer"


def test_manifest_exec_depends_on_control_packages():
    manifest = ET.parse(MANIFEST)
    dependencies = {
        element.text for element in manifest.getroot().findall("exec_depend")
    }
    assert {"cmd_vel_gate", "robot_web_ui"} <= dependencies
