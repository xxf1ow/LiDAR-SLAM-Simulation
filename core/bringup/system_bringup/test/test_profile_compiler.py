import copy
import math
import os
from pathlib import Path

import pytest
import yaml

from system_bringup import profile_compiler as pc


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
BRINGUP = PACKAGE_ROOT / "config" / "bringup.yaml"


def _selection(tmp_path, platform="real"):
    profiles_dir = tmp_path / "profiles"
    profiles_dir.mkdir()
    source_dir = PACKAGE_ROOT / "config" / "profiles"
    for name in ("sim.yaml", "real.yaml"):
        (profiles_dir / name).write_text(
            (source_dir / name).read_text(encoding="utf-8"),
            encoding="utf-8",
        )
    config = {
        "platform": platform,
        "profiles": {
            "sim": "profiles/sim.yaml",
            "real": "profiles/real.yaml",
        },
    }
    path = tmp_path / "bringup.yaml"
    path.write_text(
        yaml.safe_dump(config, sort_keys=False),
        encoding="utf-8",
    )
    return path


def _pair(config_path):
    _, paths = pc.load_bringup_selection(config_path)
    return {
        name: (path, pc.load_profile(path))
        for name, path in paths.items()
    }


@pytest.mark.parametrize("platform", ["sim", "real"])
def test_load_selection_resolves_profiles_from_config_directory(
    tmp_path, monkeypatch, platform
):
    config = _selection(tmp_path, platform)
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(elsewhere)

    selected, paths = pc.load_bringup_selection(config)

    assert selected == platform
    assert paths == {
        "sim": (tmp_path / "profiles" / "sim.yaml").resolve(),
        "real": (tmp_path / "profiles" / "real.yaml").resolve(),
    }


@pytest.mark.parametrize(
    "change,expected",
    [
        (lambda cfg: cfg.update(platform="invalid"), "platform"),
        (lambda cfg: cfg.update(profiles={"sim": "profiles/sim.yaml"}), "profiles"),
        (
            lambda cfg: cfg["profiles"].update(real=""),
            "profiles.real",
        ),
    ],
)
def test_invalid_bringup_selection_fails(tmp_path, change, expected):
    config_path = _selection(tmp_path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    change(config)
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError, match=expected):
        pc.load_bringup_selection(config_path)


def test_absolute_profile_path_fails(tmp_path):
    config_path = _selection(tmp_path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    config["profiles"]["real"] = str(
        (tmp_path / "absolute-real.yaml").resolve()
    )
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError, match="relative"):
        pc.load_bringup_selection(config_path)


def test_both_profile_files_are_required(tmp_path):
    config = _selection(tmp_path)
    (tmp_path / "profiles" / "sim.yaml").unlink()

    with pytest.raises(ValueError, match="sim.yaml"):
        pc.load_bringup_selection(config)


@pytest.mark.parametrize("text,expected", [
    ("- not\n- a\n- mapping\n", "root must be a mapping"),
    ("robot: [\n", "invalid YAML"),
])
def test_profile_file_must_be_valid_yaml_mapping(tmp_path, text, expected):
    path = tmp_path / "profile.yaml"
    path.write_text(text, encoding="utf-8")

    with pytest.raises(ValueError, match=expected):
        pc.load_profile(path)


@pytest.mark.parametrize(
    "path,value,expected",
    [
        (("robot", "body", "front_extent"), True, "front_extent"),
        (("robot", "body", "front_extent"), float("nan"), "finite"),
        (("robot", "body", "front_extent"), float("inf"), "finite"),
        (("sensors", "lidar", "scan_lines"), 32.0, "scan_lines"),
        (("hardware", "lidar", "backend"), "", "backend"),
        (("motion", "max_linear_velocity"), "unset", "max_linear_velocity"),
    ],
)
def test_invalid_leaf_type_or_value_fails(tmp_path, path, value, expected):
    pair = _pair(_selection(tmp_path))
    target = pair["real"][1]
    node = target
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value

    with pytest.raises(ValueError, match=expected):
        pc.validate_profile_pair(pair)


@pytest.mark.parametrize("mutation,expected", [
    (
        lambda profile: profile["robot"]["body"].pop("height"),
        "missing.*height",
    ),
    (
        lambda profile: profile["robot"]["body"].update(extra=1.0),
        "unexpected.*extra",
    ),
])
def test_missing_or_extra_schema_key_fails(tmp_path, mutation, expected):
    pair = _pair(_selection(tmp_path))
    mutation(pair["sim"][1])

    with pytest.raises(ValueError, match=expected):
        pc.validate_profile_pair(pair)


def test_allowed_null_fields_pass_schema_validation(tmp_path):
    pair = _pair(_selection(tmp_path))
    pc.validate_profile_pair(pair)
