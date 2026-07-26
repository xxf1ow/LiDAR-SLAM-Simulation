import importlib.util
from pathlib import Path

from launch.actions import IncludeLaunchDescription


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "real_chassis.launch.py"
)


def load_launch_module():
    spec = importlib.util.spec_from_file_location("real_chassis_launch", LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def launch_arguments(action):
    return {name: str(value) for name, value in action.launch_arguments}


def test_real_chassis_includes_vendor_and_real_robot_with_single_owner_settings():
    description = load_launch_module().generate_launch_description()
    includes = [
        entity for entity in description.entities
        if isinstance(entity, IncludeLaunchDescription)
    ]
    assert len(includes) == 2

    argument_sets = [launch_arguments(action) for action in includes]
    assert any(
        args.get("auto_enable_on_start") == "false"
        for args in argument_sets
    )
    assert any(
        args.get("use_mock_hardware") == "false"
        for args in argument_sets
    )
