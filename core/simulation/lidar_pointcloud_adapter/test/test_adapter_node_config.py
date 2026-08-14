import ast
from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "lidar_pointcloud_adapter"
    / "adapter_node.py"
)


def test_adapter_configuration_parameters_are_typed_and_required():
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    calls = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "declare_parameter"
    ]
    actual = {
        (
            ast.literal_eval(call.args[0]),
            ast.unparse(call.args[1]),
            len(call.args),
            len(call.keywords),
        )
        for call in calls
    }
    assert actual == {
        ("input_topic", "Parameter.Type.STRING", 2, 0),
        ("output_topic", "Parameter.Type.STRING", 2, 0),
        ("output_frame", "Parameter.Type.STRING", 2, 0),
        ("scan_period", "Parameter.Type.DOUBLE", 2, 0),
    }
