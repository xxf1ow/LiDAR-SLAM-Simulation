import ast
import os
import re
import subprocess
from pathlib import Path

import yaml


PATCH = Path(__file__).parents[1] / "lio-sam.patch"
README = Path(__file__).parents[1] / "README.md"
UPSTREAM = Path(__file__).parents[1] / "LIO-SAM"


def _patch_targets(patch_text):
    pairs = re.findall(
        r"^diff --git a/(.+) b/(.+)$", patch_text, flags=re.MULTILINE
    )
    assert pairs and all(left == right for left, right in pairs)
    return [left for left, _ in pairs]


def _patch_section(patch_text, relative):
    marker = f"diff --git a/{relative} b/{relative}"
    return patch_text.split(marker, 1)[1].split("\ndiff --git ", 1)[0]


def _clean_patch_source(tmp_path):
    source = tmp_path / "lio-sam-clean"
    source.mkdir()
    patch_text = PATCH.read_text(encoding="utf-8")
    for relative in _patch_targets(patch_text):
        if "--- /dev/null" in _patch_section(patch_text, relative):
            continue
        result = subprocess.run(
            [
                "git",
                f"--git-dir={UPSTREAM / '.git'}",
                f"--work-tree={UPSTREAM}",
                "show",
                f"HEAD:{relative}",
            ],
            capture_output=True,
            check=False,
        )
        assert result.returncode == 0, result.stderr.decode(errors="replace")
        target = source / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(result.stdout)
    return source


def _apply_patch(source, *, check):
    args = ["git", "apply", "--no-index"]
    if check:
        args.append("--check")
    args.append(str(PATCH.resolve()))
    return subprocess.run(
        args,
        cwd=source,
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ, "GIT_CEILING_DIRECTORIES": str(source.parent)},
    )


def _added_file(patch_text: str, relative_path: str) -> str:
    marker = f"diff --git a/{relative_path} b/{relative_path}"
    try:
        section = patch_text.split(marker, 1)[1].split("diff --git ", 1)[0]
    except IndexError:
        return ""

    return "\n".join(
        line[1:]
        for line in section.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    )


def test_real_config_matches_accepted_vanjee_722_interface():
    patch_text = PATCH.read_text(encoding="utf-8")
    real_config = _added_file(patch_text, "config/params_real.yaml")

    assert real_config, "lio-sam.patch must add config/params_real.yaml"
    params = yaml.safe_load(real_config)["/**"]["ros__parameters"]

    assert params["use_sim_time"] is False
    assert params["pointCloudTopic"] == "/points_raw"
    assert params["imuTopic"] == "/imu/data"
    assert params["lidarFrame"] == "velodyne"
    assert params["baselinkFrame"] == "base_footprint"
    assert params["sensor"] == "velodyne"
    assert params["N_SCAN"] == 32
    assert params["Horizon_SCAN"] == 1200
    assert params["lidarMinRange"] == 0.3
    assert params["lidarMaxRange"] == 40.0
    assert params["savePCD"] is True
    assert params["savePCDDirectory"] == "/result/loam/"
    assert params["extrinsicTrans"] == [0.0, 0.0, 0.0]
    assert params["extrinsicRot"] == [1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0]
    assert params["extrinsicRPY"] == [1.0, 0.0, 0.0,
                                      0.0, 1.0, 0.0,
                                      0.0, 0.0, 1.0]

    unsupported_vendor_keys = {
        "maxFeatureNum", "maxVel", "maxIter",
        "maxNoise", "accGating", "gyrGating",
    }
    assert unsupported_vendor_keys.isdisjoint(params)


def test_rviz_inherits_use_sim_time_from_selected_params_file():
    patch_text = PATCH.read_text(encoding="utf-8")

    assert "+            parameters=[parameter_file]," in patch_text
    assert "+            parameters=[{'use_sim_time': True}]" not in patch_text


def test_lio_sam_patch_applies_to_clean_pinned_git_objects(tmp_path):
    source = _clean_patch_source(tmp_path)
    result = _apply_patch(source, check=True)
    assert result.returncode == 0, result.stderr


def test_lio_sam_launch_requires_one_shared_parameter_file(tmp_path):
    source = _clean_patch_source(tmp_path)
    result = _apply_patch(source, check=False)
    assert result.returncode == 0, result.stderr
    tree = ast.parse((source / "launch/run.launch.py").read_text(encoding="utf-8"))

    declarations = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and node.args[0].value == "params_file"
    ]
    assert len(declarations) == 1
    assert not any(item.arg == "default_value" for item in declarations[0].keywords)

    parameter_value_imports = [
        node for node in tree.body
        if isinstance(node, ast.ImportFrom)
        and node.module == "launch_ros.parameter_descriptions"
        and any(item.name == "ParameterValue" for item in node.names)
    ]
    assert len(parameter_value_imports) == 1
    parameter_value_alias = next(
        item for item in parameter_value_imports[0].names
        if item.name == "ParameterValue"
    )
    assert parameter_value_alias.asname is None

    parameter_file_assignments = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and node.targets[0].id == "parameter_file"
    ]
    assert len(parameter_file_assignments) == 1
    parameter_file_value = parameter_file_assignments[0].value
    assert isinstance(parameter_file_value, ast.Call)
    assert isinstance(parameter_file_value.func, ast.Name)
    assert parameter_file_value.func.id == "ParameterValue"
    assert len(parameter_file_value.args) == 1
    launch_configuration = parameter_file_value.args[0]
    assert isinstance(launch_configuration, ast.Call)
    assert isinstance(launch_configuration.func, ast.Name)
    assert launch_configuration.func.id == "LaunchConfiguration"
    assert len(launch_configuration.args) == 1
    assert not launch_configuration.keywords
    assert isinstance(launch_configuration.args[0], ast.Constant)
    assert launch_configuration.args[0].value == "params_file"
    assert len(parameter_file_value.keywords) == 1
    assert parameter_file_value.keywords[0].arg == "value_type"
    assert isinstance(parameter_file_value.keywords[0].value, ast.Name)
    assert parameter_file_value.keywords[0].value.id == "str"

    nodes = [
        node for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ]
    assert len(nodes) == 6
    for node in nodes:
        parameters = next(item.value for item in node.keywords if item.arg == "parameters")
        assert ast.unparse(parameters) == "[parameter_file]"


def test_readme_documents_the_real_lio_sam_entrypoint():
    readme = README.read_text(encoding="utf-8")

    assert "params_real.yaml" in readme
    assert "params_file:=" in readme
    assert "32×1200" in readme
    assert "外参" in readme and "待实测" in readme
