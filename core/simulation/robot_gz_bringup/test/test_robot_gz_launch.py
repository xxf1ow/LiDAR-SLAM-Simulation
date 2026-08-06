import ast
from pathlib import Path
import re
from xml.etree import ElementTree


LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "robot_gz.launch.py"
PACKAGE_ROOT = LAUNCH_PATH.parents[1]
CMAKE_PATH = PACKAGE_ROOT / "CMakeLists.txt"
PACKAGE_XML_PATH = PACKAGE_ROOT / "package.xml"
RUNTIME_ARGUMENTS = {
    "controllers_file",
    "base_length", "base_width", "base_height", "base_link_height",
    "wheel_radius", "wheel_width", "wheel_separation",
    "sensor_x", "sensor_y", "sensor_z",
    "sensor_roll", "sensor_pitch", "sensor_yaw",
    "use_sim_time",
}


def _tree():
    return ast.parse(LAUNCH_PATH.read_text(encoding="utf-8"))


def _calls(node, name):
    return [
        item
        for item in ast.walk(node)
        if isinstance(item, ast.Call)
        and isinstance(item.func, ast.Name)
        and item.func.id == name
    ]


def _string(node):
    return node.value if isinstance(node, ast.Constant) and isinstance(node.value, str) else None


def _keyword(call, name):
    return next(keyword.value for keyword in call.keywords if keyword.arg == name)


def _assigned_value(tree, name):
    return next(
        node.value
        for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == name for target in node.targets)
    )


def _dict_value(dictionary, key):
    for item_key, value in zip(dictionary.keys, dictionary.values):
        if _string(item_key) == key:
            return value
    raise AssertionError(f"dictionary key {key!r} not found")


def _is_launch_configuration(node, argument):
    return (
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "LaunchConfiguration"
        and len(node.args) == 1
        and _string(node.args[0]) == argument
    )


def test_manifest_owned_inputs_are_required_launch_arguments():
    declarations = {
        _string(call.args[0]): call
        for call in _calls(_tree(), "DeclareLaunchArgument")
        if call.args
    }

    assert RUNTIME_ARGUMENTS <= declarations.keys()
    for name in RUNTIME_ARGUMENTS:
        assert not any(
            keyword.arg == "default_value" for keyword in declarations[name].keywords
        )


def test_launch_contract_test_is_registered_with_ament():
    cmake = CMAKE_PATH.read_text(encoding="utf-8")
    assert "if(BUILD_TESTING)" in cmake
    assert "find_package(ament_cmake_pytest REQUIRED)" in cmake
    assert re.search(
        r"ament_add_pytest_test\(\s*robot_gz_launch\s+"
        r"test/test_robot_gz_launch\.py\s*\)",
        cmake,
    )

    root = ElementTree.parse(PACKAGE_XML_PATH).getroot()
    test_dependencies = {
        element.text for element in root.findall("test_depend")
    }
    assert "ament_cmake_pytest" in test_dependencies


def test_xacro_receives_generated_controller_and_all_runtime_geometry():
    tree = _tree()
    command = _assigned_value(tree, "robot_description_content")
    command_parts = command.args[0]
    configured = {
        _string(command_parts.elts[index]).strip().removesuffix(":="):
        command_parts.elts[index + 1]
        for index in range(len(command_parts.elts) - 1)
        if _string(command_parts.elts[index])
        and _string(command_parts.elts[index]).strip().endswith(":=")
    }

    controller_value = configured["gz_controllers_file"]
    assert isinstance(controller_value, ast.Name)
    assert controller_value.id == "controllers_file"
    assert _is_launch_configuration(
        _assigned_value(tree, "controllers_file"), "controllers_file"
    )
    for name in RUNTIME_ARGUMENTS - {"controllers_file", "use_sim_time"}:
        assert _is_launch_configuration(configured[name], name)


def test_non_template_nodes_share_the_launch_clock_without_literal_true():
    tree = _tree()
    assert _is_launch_configuration(
        _assigned_value(tree, "use_sim_time"), "use_sim_time"
    )

    robot_description = _assigned_value(tree, "robot_description")
    clock = _dict_value(robot_description, "use_sim_time")
    assert isinstance(clock, ast.Name) and clock.id == "use_sim_time"

    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "generate_launch_description"
    )
    expected_nodes = {
        ("ros_gz_bridge", "parameter_bridge"),
        ("lidar_pointcloud_adapter", "adapter_node"),
        ("rviz2", "rviz2"),
    }
    checked = set()
    for call in _calls(function, "Node"):
        package = _string(_keyword(call, "package"))
        executable = _string(_keyword(call, "executable"))
        if (package, executable) not in expected_nodes:
            continue
        parameters = _keyword(call, "parameters")
        parameter_dict = parameters.elts[0]
        clock = _dict_value(parameter_dict, "use_sim_time")
        assert isinstance(clock, ast.Name) and clock.id == "use_sim_time"
        checked.add((package, executable))

    assert checked == expected_nodes
    assert not any(
        isinstance(node, ast.Constant) and node.value is True
        for node in ast.walk(function)
    )
