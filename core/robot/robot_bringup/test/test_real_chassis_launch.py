import ast
import importlib.util
from pathlib import Path
import re
from xml.etree import ElementTree

import pytest
from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.utilities import perform_substitutions
from launch_ros.substitutions import FindPackageShare


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "real_chassis.launch.py"
)
ROBOT_LAUNCH_PATH = LAUNCH_PATH.parent / "robot.launch.py"
PACKAGE_ROOT = LAUNCH_PATH.parents[1]
PACKAGE_CMAKE_PATH = PACKAGE_ROOT / "CMakeLists.txt"
PACKAGE_XML_PATH = Path(__file__).resolve().parents[1] / "package.xml"
HARDWARE_CHAIN_PATH = PACKAGE_ROOT / "test" / "test_real_hardware_chain.launch.py"
VENDOR_LAUNCH_PATH = (
    Path(__file__).resolve().parents[2]
    / "drivers"
    / "chassis_8030d"
    / "can_driver_8030D_sdk"
    / "launch"
    / "can_driver_8030.launch.py"
)
RUNTIME_ARGUMENTS = {
    "controllers_file",
    "base_length", "base_width", "base_height", "base_link_height",
    "wheel_radius", "wheel_width", "wheel_separation",
    "lidar_x", "lidar_y", "lidar_z",
    "lidar_roll", "lidar_pitch", "lidar_yaw",
    "imu_x", "imu_y", "imu_z",
    "imu_roll", "imu_pitch", "imu_yaw",
    "lidar_scan_lines", "lidar_columns_per_scan", "lidar_scan_rate_hz",
    "lidar_min_range", "lidar_max_range",
    "lidar_horizontal_start_angle", "lidar_horizontal_end_angle",
    "imu_rate_hz",
    "use_sim_time",
}
OPERATOR_ARGUMENTS = {"gui", "use_mock_hardware", "prefix"}


def load_launch_module():
    spec = importlib.util.spec_from_file_location("real_chassis_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def load_hardware_chain_module():
    spec = importlib.util.spec_from_file_location(
        "real_hardware_chain_launch", HARDWARE_CHAIN_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def launch_arguments(action):
    return {name: str(value) for name, value in action.launch_arguments}


def _declarations(path):
    tree = ast.parse(path.read_text(encoding="utf-8"))
    return {
        call.args[0].value: call
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and isinstance(call.args[0], ast.Constant)
        and isinstance(call.args[0].value, str)
    }


def _install_directory_sources(source):
    uncommented = re.sub(r"#.*", "", source)
    blocks = re.finditer(
        r"\binstall\s*\(\s*DIRECTORY\b(?P<directories>.*?)\bDESTINATION\b",
        uncommented,
        flags=re.IGNORECASE | re.DOTALL,
    )
    return [
        token.strip("\"'")
        for block in blocks
        for token in re.findall(
            r'"[^"]*"|[^\s()]+', block.group("directories")
        )
    ]


def _assert_no_config_directory_install(source):
    directories = _install_directory_sources(source)
    assert directories, "missing install(DIRECTORY ... DESTINATION ...) block"
    for directory in directories:
        path_parts = directory.replace("\\", "/").rstrip("/").split("/")
        assert "config" not in {part.lower() for part in path_parts}, (
            f"retired config directory is installed: {directory}"
        )


def include_for_filename(includes, filename):
    matches = [
        action for action in includes
        if filename in action.launch_description_source.location
    ]
    assert len(matches) == 1
    return matches[0]


def test_real_chassis_includes_vendor_and_real_robot_with_single_owner_settings():
    context = LaunchContext()
    description = load_launch_module().generate_launch_description()
    includes = [
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert len(includes) == 2

    vendor = include_for_filename(includes, "can_driver_8030.launch.py")
    robot = include_for_filename(includes, "robot.launch.py")
    vendor_arguments = launch_arguments(vendor)
    robot_arguments = launch_arguments(robot)
    vendor_arguments_raw = dict(vendor.launch_arguments)
    robot_arguments_raw = dict(robot.launch_arguments)
    vendor_share = Path(FindPackageShare("can_driver").perform(context))
    vendor.launch_description_source.get_launch_description(context)
    vendor_location = Path(vendor.launch_description_source.location)
    assert vendor_location == vendor_share / "can_driver_8030.launch.py"
    assert vendor_arguments["auto_enable_on_start"] == "false"
    assert vendor_arguments["log_level"] == "warn"
    assert (
        Path(vendor_arguments_raw["config_file"].perform(context))
        == vendor_share / "can_driver_params.yaml"
    )
    assert "use_mock_hardware" not in vendor_arguments
    assert robot_arguments["use_mock_hardware"] == "false"
    context.launch_configurations["use_sim_time"] = "clock-value"
    assert robot_arguments_raw["use_sim_time"].perform(context) == "clock-value"
    assert "prefix" not in robot_arguments
    assert "auto_enable_on_start" not in robot_arguments

    declarations = [
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    assert "prefix" not in {argument.name for argument in declarations}
    runtime_declarations = {
        argument.name: argument for argument in declarations
        if argument.name in RUNTIME_ARGUMENTS
    }
    assert set(runtime_declarations) == RUNTIME_ARGUMENTS
    assert all(not argument.default_value for argument in runtime_declarations.values())
    gui_arguments = [argument for argument in declarations if argument.name == "gui"]
    assert len(gui_arguments) == 1
    assert (
        perform_substitutions(LaunchContext(), gui_arguments[0].default_value)
        == "false"
    )


def test_robot_launch_requires_generated_inputs_and_keeps_operator_defaults():
    declarations = _declarations(ROBOT_LAUNCH_PATH)

    assert RUNTIME_ARGUMENTS <= set(declarations)
    for name in RUNTIME_ARGUMENTS:
        assert not any(
            keyword.arg == "default_value" for keyword in declarations[name].keywords
        )
    for name in OPERATOR_ARGUMENTS:
        assert any(
            keyword.arg == "default_value" for keyword in declarations[name].keywords
        )


def test_package_retires_controller_yaml_and_source_fixture():
    assert not (PACKAGE_ROOT / "config" / "robot_controllers.yaml").exists()
    assert not (PACKAGE_ROOT / "test" / "test_robot_controllers.py").exists()

    cmake = PACKAGE_CMAKE_PATH.read_text(encoding="utf-8")
    assert "DIRECTORY launch" in cmake
    _assert_no_config_directory_install(cmake)
    assert "robot_controllers_config" not in cmake
    assert "test_robot_controllers.py" not in cmake


def test_robot_retirement_guard_rejects_multiline_multi_directory_config():
    source = """\
install(
  DIRECTORY launch
            test_assets
            config
  DESTINATION share/${PROJECT_NAME}
)
"""

    with pytest.raises(AssertionError, match="retired config directory"):
        _assert_no_config_directory_install(source)


def test_real_hardware_chain_cleans_temporary_controller_file_after_shutdown():
    tree = ast.parse(HARDWARE_CHAIN_PATH.read_text(encoding="utf-8"))
    shutdown = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "TestShutdown"
    )
    assert any(
        isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "unlink"
        for node in ast.walk(shutdown)
    )


def test_real_hardware_chain_uses_one_complete_synthetic_fixture():
    module = load_hardware_chain_module()
    fixture = module.SYNTHETIC_HARDWARE_CHAIN_FIXTURE
    launch_arguments = module._synthetic_robot_launch_arguments()
    fixture_arguments = fixture["robot_launch_arguments"]
    expected_fixture_arguments = RUNTIME_ARGUMENTS - {
        "controllers_file",
        "use_sim_time",
    }

    assert set(fixture_arguments) == expected_fixture_arguments
    assert {
        name: launch_arguments[name] for name in fixture_arguments
    } == {
        name: str(value) for name, value in fixture_arguments.items()
    }
    assert module._expected_motor_command() == [
        -fixture["motor_rpm"],
        -fixture["motor_rpm"],
    ]
    expected_wheel_velocity = fixture["motor_rpm"] * 2.0 * module.math.pi / 60.0
    assert module._expected_wheel_velocity() == expected_wheel_velocity
    assert module._expected_odom_linear_x() == (
        fixture_arguments["wheel_radius"] * expected_wheel_velocity
    )


def test_package_declares_direct_launch_and_test_dependencies():
    root = ElementTree.parse(PACKAGE_XML_PATH).getroot()
    exec_dependencies = [element.text for element in root.findall("exec_depend")]
    test_dependencies = [element.text for element in root.findall("test_depend")]

    assert {"launch", "launch_ros"} <= set(exec_dependencies)
    assert {"ament_index_python", "launch", "launch_testing"} <= set(
        test_dependencies
    )
    assert len(exec_dependencies) == len(set(exec_dependencies))
    assert len(test_dependencies) == len(set(test_dependencies))


def test_vendor_launch_keeps_stdout_in_log_and_applies_process_log_level():
    tree = ast.parse(VENDOR_LAUNCH_PATH.read_text(encoding="utf-8"))
    declarations = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "log_level"
    ]
    assert len(declarations) == 1
    defaults = {
        keyword.arg: keyword.value for keyword in declarations[0].keywords
    }
    assert isinstance(defaults["default_value"], ast.Constant)
    assert defaults["default_value"].value == "info"

    nodes = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ]
    assert len(nodes) == 1
    keywords = {keyword.arg: keyword.value for keyword in nodes[0].keywords}
    output = keywords["output"]
    assert isinstance(output, ast.Constant)
    assert output.value == "log"
    ros_arguments = keywords["ros_arguments"]
    assert isinstance(ros_arguments, ast.List)
    assert isinstance(ros_arguments.elts[0], ast.Constant)
    assert ros_arguments.elts[0].value == "--log-level"
    level = ros_arguments.elts[1]
    assert isinstance(level, ast.Call)
    assert isinstance(level.func, ast.Name)
    assert level.func.id == "LaunchConfiguration"
    assert isinstance(level.args[0], ast.Constant)
    assert level.args[0].value == "log_level"
