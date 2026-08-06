import ast
import importlib.util
from pathlib import Path
from xml.etree import ElementTree

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
PACKAGE_XML_PATH = Path(__file__).resolve().parents[1] / "package.xml"
VENDOR_LAUNCH_PATH = (
    Path(__file__).resolve().parents[2]
    / "drivers"
    / "chassis_8030d"
    / "can_driver_8030D_sdk"
    / "launch"
    / "can_driver_8030.launch.py"
)


def load_launch_module():
    spec = importlib.util.spec_from_file_location("real_chassis_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def launch_arguments(action):
    return {name: str(value) for name, value in action.launch_arguments}


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
        if argument.name in {
            "controllers_file",
            "base_length", "base_width", "base_height", "base_link_height",
            "wheel_radius", "wheel_width", "wheel_separation",
            "sensor_x", "sensor_y", "sensor_z",
            "sensor_roll", "sensor_pitch", "sensor_yaw",
            "use_sim_time",
        }
    }
    assert len(runtime_declarations) == 15
    assert all(not argument.default_value for argument in runtime_declarations.values())
    gui_arguments = [argument for argument in declarations if argument.name == "gui"]
    assert len(gui_arguments) == 1
    assert (
        perform_substitutions(LaunchContext(), gui_arguments[0].default_value)
        == "false"
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
