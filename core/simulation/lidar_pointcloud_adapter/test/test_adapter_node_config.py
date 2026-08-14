import ast
from pathlib import Path

import yaml


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "lidar_pointcloud_adapter"
    / "adapter_node.py"
)
TEMPLATE = (
    Path(__file__).resolve().parents[3]
    / "bringup"
    / "system_bringup"
    / "config"
    / "templates"
    / "lidar_adapter.yaml"
)


def _native_parameter_type(value):
    return {
        bool: "Parameter.Type.BOOL",
        int: "Parameter.Type.INTEGER",
        float: "Parameter.Type.DOUBLE",
        str: "Parameter.Type.STRING",
    }[type(value)]


def _template_parameter_types():
    parameters = yaml.safe_load(TEMPLATE.read_text(encoding="utf-8"))[
        "lidar_pointcloud_adapter"
    ]["ros__parameters"]
    return {
        name: _native_parameter_type(value)
        for name, value in parameters.items()
        if name != "use_sim_time"
    }


def test_adapter_configuration_parameters_are_typed_and_required():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "declare_parameter"
    ]
    actual = [
        (
            ast.literal_eval(call.args[0]),
            ast.unparse(call.args[1]) if len(call.args) > 1 else None,
            len(call.args),
            len(call.keywords),
        )
        for call in calls
    ]
    expected = _template_parameter_types()
    names = [name for name, _type, _args, _keywords in actual]
    assert len(actual) == len(expected)
    assert len(names) == len(set(names))
    assert {
        name: parameter_type
        for name, parameter_type, _args, _keywords in actual
    } == expected
    assert all(
        argument_count == 2 and keyword_count == 0
        for _name, _type, argument_count, keyword_count in actual
    )
