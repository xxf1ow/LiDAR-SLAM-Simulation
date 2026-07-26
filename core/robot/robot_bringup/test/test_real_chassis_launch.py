import importlib.util
from pathlib import Path
from xml.etree import ElementTree

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.utilities import perform_substitutions


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "real_chassis.launch.py"
)
PACKAGE_XML_PATH = Path(__file__).resolve().parents[1] / "package.xml"


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
    assert vendor_arguments["auto_enable_on_start"] == "false"
    assert "use_mock_hardware" not in vendor_arguments
    assert robot_arguments["use_mock_hardware"] == "false"
    assert "prefix" not in robot_arguments
    assert "auto_enable_on_start" not in robot_arguments

    declarations = [
        entity for entity in description.entities
        if isinstance(entity, DeclareLaunchArgument)
    ]
    assert "prefix" not in {argument.name for argument in declarations}
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
