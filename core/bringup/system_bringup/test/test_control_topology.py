import ast
import importlib.util
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[4]
NAVIGATION = ROOT / "core/navigation/robot_navigation/launch/navigation.launch.py"
SLAM_STACK = ROOT / "core/bringup/system_bringup/launch/slam_stack.launch.py"
BRINGUP = ROOT / "core/bringup/system_bringup/launch/bringup.launch.py"
ROBOT = ROOT / "core/robot/robot_bringup/launch/robot.launch.py"
REAL_CHASSIS = ROOT / "core/robot/robot_bringup/launch/real_chassis.launch.py"
SENSOR_GATE = ROOT / "core/bringup/system_bringup/system_bringup/sensor_gate_node.py"
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


def _declaration(tree, argument):
    return next(
        call
        for call in _calls(tree, "DeclareLaunchArgument")
        if call.args and _string(call.args[0]) == argument
    )


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


def _add_terms(node):
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        return _add_terms(node.left) + _add_terms(node.right)
    return [node]


def _subscript_path(node):
    path = []
    while isinstance(node, ast.Subscript):
        path.append(
            node.slice.id if isinstance(node.slice, ast.Name) else _string(node.slice)
        )
        node = node.value
    return node.id if isinstance(node, ast.Name) else None, tuple(reversed(path))


def _is_vanjee_config_call(node):
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "_pkg_config"
        and _string(node.args[0]) == "vanjee_lidar_ros"
    )


def _assigned_value(function, name):
    return next(
        node.value
        for node in ast.walk(function)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name for target in node.targets)
    )


def test_navigation_declares_cmd_vel_output_topic_with_legacy_default():
    assert _declaration_default(_tree(NAVIGATION), "cmd_vel_output_topic") == "/cmd_vel"


def test_navigation_map_server_uses_generated_params_with_only_map_override():
    function = _function(_tree(NAVIGATION), "generate_launch_description")
    assert _launch_configuration_assignment(function, "params_file", "params_file")
    map_function = next(
        node
        for node in function.body
        if isinstance(node, ast.FunctionDef) and node.name == "_map_server"
    )
    assert not any(
        isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "params_file"
            for target in node.targets
        )
        for node in ast.walk(map_function)
    )

    map_server = _node_call(map_function, "nav2_map_server", "map_server")
    parameters = _keyword(map_server, "parameters")
    assert isinstance(parameters, ast.List) and len(parameters.elts) == 2
    assert isinstance(parameters.elts[0], ast.Name)
    assert parameters.elts[0].id == "params_file"
    override = parameters.elts[1]
    assert isinstance(override, ast.Dict) and len(override.keys) == 1
    map_yaml = _dict_value(override, "yaml_filename")
    assert isinstance(map_yaml, ast.Name) and map_yaml.id == "map_yaml"


def test_navigation_accepts_complete_body_weld_transform():
    tree = _tree(NAVIGATION)
    assert {
        _string(call.args[0])
        for call in _calls(tree, "DeclareLaunchArgument")
        if call.args
        and _string(call.args[0]).startswith("weld_")
    } == {
        "weld_x", "weld_y", "weld_z",
        "weld_qx", "weld_qy", "weld_qz", "weld_qw",
    }

    function = _function(tree, "generate_launch_description")
    publisher = _node_call(function, "tf2_ros", "static_transform_publisher")
    arguments = _keyword(publisher, "arguments")
    assert isinstance(arguments, ast.List)
    values = {
        _string(arguments.elts[index]): arguments.elts[index + 1]
        for index in range(0, 14, 2)
    }
    for option, variable in (
        ("--x", "weld_x"), ("--y", "weld_y"), ("--z", "weld_z"),
        ("--qx", "weld_qx"), ("--qy", "weld_qy"),
        ("--qz", "weld_qz"), ("--qw", "weld_qw"),
    ):
        assert isinstance(values[option], ast.Name)
        assert values[option].id == variable


def test_slam_stack_forwards_only_quaternion_body_weld_arguments():
    tree = _tree(SLAM_STACK)
    declared_weld = {
        _string(call.args[0])
        for call in _calls(tree, "DeclareLaunchArgument")
        if call.args
        and _string(call.args[0]).startswith("weld_")
    }
    assert declared_weld == {
        "weld_x", "weld_y", "weld_z",
        "weld_qx", "weld_qy", "weld_qz", "weld_qw",
    }
    assert {
        name: _declaration_default(tree, name)
        for name in declared_weld
    } == {
        "weld_x": "0.0", "weld_y": "0.0", "weld_z": "-0.5560",
        "weld_qx": "0.0", "weld_qy": "0.0", "weld_qz": "0.0", "weld_qw": "1.0",
    }

    function = _function(tree, "_stack")
    weld = _assigned_value(function, "weld")
    assert isinstance(weld, ast.DictComp)
    assert {
        _string(item)
        for item in weld.generators[0].iter.elts
    } == declared_weld

    navigation = _include_arguments(
        function, "robot_navigation", "launch/navigation.launch.py"
    )
    assert any(
        key is None and isinstance(value, ast.Name) and value.id == "weld"
        for key, value in zip(navigation.keys, navigation.values)
    )


def test_robot_launch_accepts_runtime_geometry_and_controller_file():
    tree = _tree(ROBOT)
    declared = {
        _string(call.args[0])
        for call in _calls(tree, "DeclareLaunchArgument")
        if call.args
    }
    assert {
        "controllers_file",
        "base_length", "base_width", "base_height", "base_link_height",
        "wheel_radius", "wheel_width", "wheel_separation",
        "sensor_x", "sensor_y", "sensor_z",
        "sensor_roll", "sensor_pitch", "sensor_yaw",
        "use_sim_time",
    } <= declared

    function = _function(tree, "generate_launch_description")
    control_node = _node_call(function, "controller_manager", "ros2_control_node")
    parameters = _keyword(control_node, "parameters")
    assert isinstance(parameters, ast.List)
    assert isinstance(parameters.elts[0], ast.Name)
    assert parameters.elts[0].id == "controllers_file"
    control_clock = _dict_value(parameters.elts[1], "use_sim_time")
    assert isinstance(control_clock, ast.Name)
    assert control_clock.id == "use_sim_time"

    robot_description = _assigned_value(function, "robot_description")
    description_clock = _dict_value(robot_description, "use_sim_time")
    assert isinstance(description_clock, ast.Name)
    assert description_clock.id == "use_sim_time"

    rviz = _node_call(function, "rviz2", "rviz2")
    rviz_parameters = _keyword(rviz, "parameters")
    rviz_clock = _dict_value(rviz_parameters.elts[0], "use_sim_time")
    assert isinstance(rviz_clock, ast.Name)
    assert rviz_clock.id == "use_sim_time"

    command = _assigned_value(function, "robot_description_content")
    assert isinstance(command, ast.Call)
    command_parts = command.args[0]
    assert isinstance(command_parts, ast.List)
    configured = {
        _string(command_parts.elts[index]).removesuffix(":="):
        command_parts.elts[index + 1]
        for index in range(len(command_parts.elts) - 1)
        if _string(command_parts.elts[index])
        and _string(command_parts.elts[index]).endswith(":=")
    }
    for name in declared - {"gui", "controllers_file", "use_sim_time"}:
        value = configured[name]
        if name in {"use_mock_hardware", "prefix"}:
            assert isinstance(value, ast.Name)
        else:
            assert isinstance(value, ast.Call)
            assert isinstance(value.func, ast.Name)
            assert value.func.id == "LaunchConfiguration"
            assert _string(value.args[0]) == name


def test_real_chassis_passes_runtime_geometry_to_robot_launch():
    tree = _tree(REAL_CHASSIS)
    declared = {
        _string(call.args[0])
        for call in _calls(tree, "DeclareLaunchArgument")
        if call.args
    }
    assert {
        "controllers_file",
        "base_length", "base_width", "base_height", "base_link_height",
        "wheel_radius", "wheel_width", "wheel_separation",
        "sensor_x", "sensor_y", "sensor_z",
        "sensor_roll", "sensor_pitch", "sensor_yaw",
        "use_sim_time",
    } <= declared

    function = _function(tree, "generate_launch_description")
    geometry = _assigned_value(function, "geometry_arguments")
    assert isinstance(geometry, ast.DictComp)
    names = {
        _string(item)
        for item in geometry.generators[0].iter.elts
    }
    assert declared - {"gui"} == names
    assert "use_mock_hardware" not in declared

    robot = _assigned_value(function, "robot")
    launch_arguments = _keyword(robot, "launch_arguments")
    assert isinstance(launch_arguments, ast.Call)
    passed = launch_arguments.func.value
    assert isinstance(passed, ast.Dict)
    assert any(
        key is None and isinstance(value, ast.Name)
        and value.id == "geometry_arguments"
        for key, value in zip(passed.keys, passed.values)
    )
    assert _string(_dict_value(passed, "use_mock_hardware")) == "false"


def test_real_chassis_requires_authoritative_geometry_instead_of_sim_defaults():
    tree = _tree(REAL_CHASSIS)
    for name in (
        "controllers_file",
        "base_length", "base_width", "base_height", "base_link_height",
        "wheel_radius", "wheel_width", "wheel_separation",
        "sensor_x", "sensor_y", "sensor_z",
        "sensor_roll", "sensor_pitch", "sensor_yaw",
        "use_sim_time",
    ):
        declaration = _declaration(tree, name)
        assert not any(keyword.arg == "default_value" for keyword in declaration.keywords)


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


def test_slam_ready_gates_receive_the_existing_single_clock_value():
    stack = _function(_tree(SLAM_STACK), "_stack")
    gates = _calls(stack, "ready_gate")
    assert len(gates) == 2
    for gate in gates:
        value = _keyword(gate, "use_sim_time")
        assert isinstance(value, ast.Name)
        assert value.id == "use_sim"


def _fake_launch_module(monkeypatch, package_share):
    class StubAction:
        def __init__(self, *args, **kwargs):
            self.args = args
            self.kwargs = kwargs

    def module(name, **attributes):
        value = types.ModuleType(name)
        for key, item in attributes.items():
            setattr(value, key, item)
        monkeypatch.setitem(sys.modules, name, value)
        return value

    packages = module(
        "ament_index_python.packages",
        get_package_share_directory=lambda package: str(package_share),
    )
    module("ament_index_python", packages=packages)
    launch = module("launch", LaunchDescription=StubAction)
    actions = module(
        "launch.actions",
        IncludeLaunchDescription=StubAction,
        LogInfo=StubAction,
        OpaqueFunction=StubAction,
        RegisterEventHandler=StubAction,
    )
    event_handlers = module("launch.event_handlers", OnProcessExit=StubAction)
    sources = module(
        "launch.launch_description_sources",
        PythonLaunchDescriptionSource=StubAction,
    )
    launch.actions = actions
    launch.event_handlers = event_handlers
    launch.launch_description_sources = sources
    launch_ros = module("launch_ros")
    launch_ros.actions = module("launch_ros.actions", Node=StubAction)
    module("yaml", YAMLError=ValueError)

    import system_bringup

    consistency = module(
        "system_bringup.consistency_check",
        run_runtime_consistency=lambda repo_root, manifest: [],
        find_repo_root=lambda: (_ for _ in ()).throw(RuntimeError("legacy discovery")),
        load_bringup_config=lambda repo_root: {},
        run=lambda repo_root: [],
        derive_real_geometry=lambda config: {},
        real_geometry_launch_arguments=lambda geometry: {},
        require_runtime_config_file=lambda path, component: path,
        write_real_runtime_configs=lambda repo_root, config: {},
    )
    runtime_compiler = module(
        "system_bringup.runtime_config_compiler",
        compile_runtime_configs=lambda source: {},
    )
    ready_gate_module = module(
        "system_bringup.ready_gate", ready_gate=lambda *args, **kwargs: []
    )
    monkeypatch.setattr(system_bringup, "consistency_check", consistency, raising=False)
    monkeypatch.setattr(
        system_bringup, "runtime_config_compiler", runtime_compiler, raising=False
    )
    monkeypatch.setattr(system_bringup, "ready_gate", ready_gate_module, raising=False)

    spec = importlib.util.spec_from_file_location("formal_bringup_under_test", BRINGUP)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


@pytest.mark.parametrize(
    "share_relative",
    (
        Path("install/system_bringup/share/system_bringup"),
        Path("install/share/system_bringup"),
    ),
    ids=("isolated", "merged"),
)
def test_formal_bringup_resolves_source_config_without_repo_search(
    monkeypatch, tmp_path, share_relative
):
    workspace = tmp_path / "core"
    launch_module = _fake_launch_module(
        monkeypatch, workspace / share_relative
    )

    assert launch_module._source_bringup_config_path() == (
        workspace / "bringup/system_bringup/config/bringup.yaml"
    ).resolve()


def test_formal_bringup_compiles_and_validates_once_before_action_construction():
    tree = _tree(BRINGUP)
    function = _function(tree, "_bringup")
    compile_calls = _calls(function, "compile_runtime_configs")
    consistency_calls = _calls(function, "run_runtime_consistency")

    assert len(compile_calls) == 1
    assert len(consistency_calls) == 1
    assert isinstance(compile_calls[0].args[0], ast.Name)
    assert compile_calls[0].args[0].id == "source_config"
    assert [argument.id for argument in consistency_calls[0].args] == [
        "repo_root", "manifest"
    ]

    gate_index = consistency_calls[0].lineno
    action_calls = [
        call for name in ("Node", "_inc", "ready_gate", "LogInfo")
        for call in _calls(function, name)
    ]
    assert action_calls
    assert all(call.lineno > gate_index for call in action_calls)


def test_formal_bringup_has_no_active_legacy_runtime_path():
    source = BRINGUP.read_text(encoding="utf-8")
    for name in (
        "find_repo_root",
        "load_bringup_config",
        "derive_real_geometry",
        "build_real_runtime_configs",
        "write_real_runtime_configs",
        "real_geometry_launch_arguments",
        "consistency_check.run(",
    ):
        assert name not in source


@pytest.mark.parametrize("failure_stage", ("compile", "consistency"))
def test_formal_bringup_failures_construct_no_actions(
    monkeypatch, tmp_path, failure_stage
):
    launch_module = _fake_launch_module(monkeypatch, tmp_path / "share")
    source = (tmp_path / "core/bringup/system_bringup/config/bringup.yaml").resolve()
    manifest = {"platform": "sim"}
    calls = []
    actions = []
    monkeypatch.setattr(launch_module, "_source_bringup_config_path", lambda: source)

    def compile_once(path):
        calls.append(("compile", path))
        if failure_stage == "compile":
            raise ValueError("invalid profile")
        return manifest

    def validate_once(repo_root, received):
        calls.append(("consistency", repo_root, received))
        return ["generated nav2 drift"] if failure_stage == "consistency" else []

    monkeypatch.setattr(
        launch_module, "compile_runtime_configs", compile_once, raising=False
    )
    monkeypatch.setattr(
        launch_module, "run_runtime_consistency", validate_once, raising=False
    )
    for name in ("Node", "_inc", "ready_gate", "LogInfo"):
        monkeypatch.setattr(
            launch_module,
            name,
            lambda *args, _name=name, **kwargs: actions.append(_name),
        )

    expected = "invalid profile" if failure_stage == "compile" else "generated nav2 drift"
    with pytest.raises(RuntimeError, match=expected):
        launch_module._bringup(None)

    assert calls[0] == ("compile", source)
    if failure_stage == "compile":
        assert len(calls) == 1
    else:
        assert calls == [("compile", source), ("consistency", source.parents[4], manifest)]
    assert actions == []


@pytest.mark.parametrize(
    ("mode", "package", "filename", "component"),
    (
        ("mapping", "lio_sam", "params.yaml", "LIO-SAM"),
        ("navigation", "fast_lio", "gazebo_velodyne.yaml", "FAST-LIO"),
    ),
)
def test_missing_selected_slam_config_fails_before_action_construction(
    monkeypatch, tmp_path, mode, package, filename, component
):
    launch_module = _fake_launch_module(monkeypatch, tmp_path / "share")
    source = (tmp_path / "core/bringup/system_bringup/config/bringup.yaml").resolve()
    profile = {
        "lio_sam": {"config": "params.yaml"},
        "fast_lio": {"config": "gazebo_velodyne.yaml"},
    }
    manifest = {
        "platform": "sim",
        "mode": mode,
        "use_sim_time": True,
        "bringup_config": {
            "slam_stack": {"settling": 20.0, "sim": profile},
        },
    }
    required = []
    actions = []
    monkeypatch.setattr(launch_module, "_source_bringup_config_path", lambda: source)
    monkeypatch.setattr(
        launch_module, "compile_runtime_configs", lambda path: manifest
    )
    monkeypatch.setattr(
        launch_module, "run_runtime_consistency", lambda repo_root, value: []
    )
    monkeypatch.setattr(
        launch_module,
        "_pkg_config",
        lambda pkg, selected: f"/installed/{pkg}/config/{selected}",
    )

    def reject_missing(path, label):
        required.append((path, label))
        raise RuntimeError(f"{label} 运行时配置不存在: {path}")

    monkeypatch.setattr(
        launch_module, "require_runtime_config_file", reject_missing, raising=False
    )
    for name in ("Node", "_inc", "ready_gate", "LogInfo"):
        monkeypatch.setattr(
            launch_module,
            name,
            lambda *args, _name=name, **kwargs: actions.append(_name),
        )

    selected_path = f"/installed/{package}/config/{filename}"
    with pytest.raises(
        RuntimeError,
        match=rf"{component} 运行时配置不存在: {selected_path}",
    ):
        launch_module._bringup(None)

    assert required == [(selected_path, component)]
    assert actions == []


def test_bringup_constructs_one_shared_control_and_slam_layer_before_branching():
    function = _function(_tree(BRINGUP), "_bringup")
    sim = _platform_branch(function, "sim")
    real = _platform_branch(function, "real")
    first_branch = min(sim.lineno, real.lineno)

    shared_stack = [
        call for call in _calls(function, "_inc")
        if _string(call.args[0]) == "system_bringup"
        and _string(call.args[1]) == "launch/slam_stack.launch.py"
    ]
    assert len(shared_stack) == 1
    assert shared_stack[0].lineno < first_branch
    for package in ("cmd_vel_gate", "robot_web_ui"):
        nodes = [
            call for call in _calls(function, "Node")
            if _string(_keyword(call, "package")) == package
        ]
        assert len(nodes) == 1
        assert nodes[0].lineno < first_branch
    for branch in (sim, real):
        result = next(
            node.value for node in branch.body if isinstance(node, ast.Return)
        )
        assert _leftmost_add_name(result) == "control_layer"


def test_shared_control_consumes_only_manifest_clock_and_web_ui_file():
    function = _function(_tree(BRINGUP), "_bringup")
    gate = _node_call(function, "cmd_vel_gate", "cmd_vel_gate")
    gate_parameters = _keyword(gate, "parameters")
    assert isinstance(gate_parameters, ast.List) and len(gate_parameters.elts) == 1
    assert {
        _string(key) for key in gate_parameters.elts[0].keys
    } == {"use_sim_time"}
    assert _subscript_path(
        _dict_value(gate_parameters.elts[0], "use_sim_time")
    ) == ("manifest", ("use_sim_time",))

    web = _node_call(function, "robot_web_ui", "robot_web_ui")
    web_parameters = _keyword(web, "parameters")
    assert isinstance(web_parameters, ast.List) and len(web_parameters.elts) == 1
    config_path = web_parameters.elts[0]
    assert isinstance(config_path, ast.Call)
    assert isinstance(config_path.func, ast.Name) and config_path.func.id == "str"
    assert _subscript_path(config_path.args[0]) == ("manifest", ("web_ui_path",))


def test_shared_slam_consumes_manifest_paths_clock_profile_and_quaternion_weld():
    function = _function(_tree(BRINGUP), "_bringup")
    arguments = _include_arguments(
        function, "system_bringup", "launch/slam_stack.launch.py"
    )
    assert _string(_dict_value(arguments, "cmd_vel_output_topic")) == "/cmd_vel_auto"
    assert _subscript_path(_dict_value(arguments, "mode")) == ("manifest", ("mode",))
    nav2 = _dict_value(arguments, "nav2_params_file")
    assert isinstance(nav2, ast.Call) and nav2.func.id == "str"
    assert _subscript_path(nav2.args[0]) == ("manifest", ("nav2_path",))
    for argument, package, component in (
        ("lio_sam_params_file", "lio_sam", "lio_sam"),
        ("gicp_config_file", "gicp_localization", "gicp_localization"),
    ):
        value = _dict_value(arguments, argument)
        assert isinstance(value, ast.Call) and value.func.id == "_pkg_config"
        assert _string(value.args[0]) == package
        assert _subscript_path(value.args[1]) == ("profile", (component, "config"))
    assert _subscript_path(_dict_value(arguments, "fast_lio_config")) == (
        "profile", ("fast_lio", "config")
    )
    for name in ("x", "y", "z", "qx", "qy", "qz", "qw"):
        assert _subscript_path(_dict_value(arguments, f"weld_{name}")) == (
            "weld", (name,)
        )


def _assert_manifest_robot_interface(arguments):
    controller = _dict_value(arguments, "controllers_file")
    assert isinstance(controller, ast.Call) and controller.func.id == "str"
    assert _subscript_path(controller.args[0]) == ("manifest", ("controllers_path",))
    clock = _dict_value(arguments, "use_sim_time")
    assert isinstance(clock, ast.Name) and clock.id == "use_sim"
    assert any(
        key is None and isinstance(value, ast.Name) and value.id == "geometry"
        for key, value in zip(arguments.keys, arguments.values)
    )


def test_sim_backend_uses_manifest_controller_geometry_clock_and_existing_gate():
    function = _function(_tree(BRINGUP), "_bringup")
    sim = _platform_branch(function, "sim")
    arguments = _include_arguments(
        sim, "robot_gz_bringup", "launch/robot_gz.launch.py"
    )
    _assert_manifest_robot_interface(arguments)
    for name in ("gui", "rviz", "world", "spawn_x", "spawn_y", "spawn_z"):
        value = _dict_value(arguments, name)
        assert _subscript_path(value) == ("gz", (name,))
    gate = _calls(sim, "ready_gate")
    assert len(gate) == 1
    assert {_string(item) for item in gate[0].args[0].elts} == {
        "/points_raw", "/joint_states"
    }
    assert gate[0].args[1].value == 300.0
    assert _keyword(gate[0], "settling").id == "settling"
    assert _keyword(gate[0], "use_sim_time").id == "use_sim"


def test_real_backend_uses_same_interface_and_gates_shared_stack_after_drivers():
    function = _function(_tree(BRINGUP), "_bringup")
    real = _platform_branch(function, "real")
    chassis = _include_arguments(
        real, "robot_bringup", "launch/real_chassis.launch.py"
    )
    _assert_manifest_robot_interface(chassis)
    assert _string(_dict_value(chassis, "gui")) == "false"
    assert all(_string(key) != "use_mock_hardware" for key in chassis.keys)
    lidar = _include_arguments(
        real, "vanjee_lidar_ros", "launch/vanjee_lidar.launch.py"
    )
    lidar_config = _dict_value(lidar, "config_file")
    assert isinstance(lidar_config, ast.Name)
    assert lidar_config.id == "vanjee_config"

    result = next(node.value for node in real.body if isinstance(node, ast.Return))
    terms = _add_terms(result)
    gate_index = next(
        index for index, term in enumerate(terms)
        if isinstance(term, ast.Call)
        and isinstance(term.func, ast.Name)
        and term.func.id == "_real_sensor_gate"
    )
    preceding_names = {
        name.id for term in terms[:gate_index] for name in ast.walk(term)
        if isinstance(name, ast.Name)
    }
    assert {"chassis", "lidar"} <= preceding_names
    gate = terms[gate_index]
    assert [item.id for item in gate.args[0].elts] == ["slam_stack"]
    assert isinstance(gate.args[1], ast.Name) and gate.args[1].id == "use_sim_time"

    gate_function = _function(_tree(BRINGUP), "_real_sensor_gate")
    waiter = _node_call(gate_function, "system_bringup", "real_sensor_ready_gate")
    parameters = _keyword(waiter, "parameters").elts[0]
    assert _dict_value(parameters, "timeout").value == 300.0
    assert isinstance(_dict_value(parameters, "use_sim_time"), ast.Name)
    assert _dict_value(parameters, "use_sim_time").id == "use_sim_time"
    handler = next(call for call in _calls(gate_function, "OnProcessExit"))
    on_exit = _keyword(handler, "on_exit")
    assert isinstance(on_exit.body, ast.IfExp)
    assert isinstance(on_exit.body.test, ast.Compare)
    assert isinstance(on_exit.body.test.left, ast.Attribute)
    assert isinstance(on_exit.body.test.left.value, ast.Name)
    assert on_exit.body.test.left.value.id == "event"
    assert on_exit.body.test.left.attr == "returncode"
    assert isinstance(on_exit.body.test.ops[0], ast.Eq)
    assert on_exit.body.test.comparators[0].value == 0
    assert on_exit.body.body.id == "then_actions"
    abort = on_exit.body.orelse.elts[0]
    assert abort.func.id == "OpaqueFunction"
    assert _keyword(abort, "function").id == "_abort_real_sensor_gate"


def test_real_sensor_gate_finishes_callback_without_shutting_down_executor():
    finish = next(
        node for node in ast.walk(_tree(SENSOR_GATE))
        if isinstance(node, ast.FunctionDef) and node.name == "_finish"
    )
    assert not any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "shutdown"
        for node in ast.walk(finish)
    )


def test_real_sensor_gate_main_spins_only_until_result_is_finished():
    main = _function(_tree(SENSOR_GATE), "main")
    loops = [node for node in ast.walk(main) if isinstance(node, ast.While)]
    assert len(loops) == 1
    loop = loops[0]
    expected_test = ast.parse(
        "while rclpy.ok() and not node.finished:\n    pass"
    ).body[0].test
    assert ast.dump(loop.test) == ast.dump(expected_test)

    spins = [
        node for node in ast.walk(loop)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "spin_once"
    ]
    expected_spin = ast.parse(
        "rclpy.spin_once(node, timeout_sec=0.1)"
    ).body[0].value
    assert len(spins) == 1
    assert ast.dump(spins[0]) == ast.dump(expected_spin)


def test_real_branch_resolves_vanjee_config_only_after_platform_selection():
    function = _function(_tree(BRINGUP), "_bringup")
    real = _platform_branch(function, "real")
    assignment = next(
        node
        for node in real.body
        if isinstance(node, ast.Assign)
        and any(
            isinstance(target, ast.Name) and target.id == "vanjee_config"
            for target in node.targets
        )
    )

    assert _is_vanjee_config_call(assignment.value)
    assert _subscript_path(assignment.value.args[1]) == (
        "cfg", ("vanjee_lidar", "config")
    )
    vanjee_config_calls = [
        call for call in _calls(function, "_pkg_config") if _is_vanjee_config_call(call)
    ]
    assert vanjee_config_calls == [assignment.value]


def test_real_branch_does_not_publish_duplicate_lidar_static_tf():
    function = _function(_tree(BRINGUP), "_bringup")
    real = _platform_branch(function, "real")
    assert not any(
        _string(_keyword(call, "package")) == "tf2_ros"
        and _string(_keyword(call, "executable")) == "static_transform_publisher"
        for call in _calls(real, "Node")
    )


def test_manifest_exec_depends_on_control_packages():
    manifest = ET.parse(MANIFEST)
    dependencies = {
        element.text for element in manifest.getroot().findall("exec_depend")
    }
    assert {"cmd_vel_gate", "robot_web_ui"} <= dependencies
    assert {"robot_bringup", "vanjee_lidar_ros", "rclpy", "sensor_msgs"} <= dependencies
