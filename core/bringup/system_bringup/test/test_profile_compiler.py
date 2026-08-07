from pathlib import Path, PurePosixPath

import pytest
import yaml

from system_bringup import profile_compiler as pc


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


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


def _set_pair_value(pair, platform, path, value):
    target = pair[platform][1]
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value


def test_load_bringup_context_returns_source_config_and_selection(tmp_path):
    config = _selection(tmp_path, "sim")

    source, parsed, platform, paths = pc.load_bringup_context(config)

    assert source == config.resolve()
    assert parsed["platform"] == "sim"
    assert platform == "sim"
    assert paths == {
        "sim": (tmp_path / "profiles" / "sim.yaml").resolve(),
        "real": (tmp_path / "profiles" / "real.yaml").resolve(),
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

    with pytest.raises(ValueError, match=r"profiles\.real must be relative$"):
        pc.load_bringup_selection(config_path)


@pytest.mark.parametrize("profile_path", ["/tmp/profile.yaml", "C:profile.yaml"])
def test_rooted_or_drive_relative_profile_path_fails(tmp_path, profile_path):
    config_path = _selection(tmp_path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    config["profiles"]["real"] = profile_path
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")

    with pytest.raises(ValueError, match=r"profiles\.real must be relative$"):
        pc.load_bringup_selection(config_path)


def test_windows_drive_relative_path_fails_under_posix_path_semantics(
    tmp_path, monkeypatch
):
    config_path = _selection(tmp_path)
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    config["profiles"]["real"] = "C:profile.yaml"
    config_path.write_text(yaml.safe_dump(config), encoding="utf-8")
    native_path = pc.Path

    def posix_path_for_windows_drive(value):
        if value == "C:profile.yaml":
            return PurePosixPath(value)
        return native_path(value)

    monkeypatch.setattr(pc, "Path", posix_path_for_windows_drive)

    with pytest.raises(ValueError, match=r"profiles\.real must be relative$"):
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


@pytest.mark.parametrize(
    "platform,path,value,expected",
    [
        ("sim", ("hardware", "lidar", "backend"), "vanjee", "gazebo/gpu_lidar"),
        ("sim", ("hardware", "lidar", "model"), "vanjee_722", "gazebo/gpu_lidar"),
        ("real", ("hardware", "lidar", "backend"), "gazebo", "vanjee/vanjee_722"),
        ("real", ("hardware", "lidar", "model"), "gpu_lidar", "vanjee/vanjee_722"),
        ("sim", ("hardware", "lidar", "host_address"), "192.168.2.88", "must be null"),
        ("sim", ("hardware", "lidar", "device_address"), "192.168.2.86", "must be null"),
        ("sim", ("hardware", "lidar", "host_msop_port"), 3001, "must be null"),
        ("sim", ("hardware", "lidar", "device_msop_port"), 3333, "must be null"),
        ("real", ("hardware", "lidar", "host_address"), "not-an-ip", "IPv4"),
        ("real", ("hardware", "lidar", "device_address"), "::1", "IPv4"),
        ("real", ("hardware", "lidar", "host_address"), None, "IPv4"),
        ("real", ("hardware", "lidar", "host_msop_port"), 0, "1..65535"),
        ("real", ("hardware", "lidar", "device_msop_port"), 65536, "1..65535"),
        ("real", ("hardware", "lidar", "device_msop_port"), None, "1..65535"),
        ("sim", ("sensors", "lidar", "point_time_field"), "t", "time/seconds/scan_start"),
        ("real", ("sensors", "lidar", "point_time_unit"), "milliseconds", "time/seconds/scan_start"),
        ("real", ("sensors", "lidar", "point_time_reference"), "scan_end", "time/seconds/scan_start"),
    ],
)
def test_sensor_platform_contract_rejects_unsupported_values(
    tmp_path, platform, path, value, expected
):
    pair = _pair(_selection(tmp_path))
    _set_pair_value(pair, platform, path, value)

    with pytest.raises(ValueError, match=expected):
        pc.validate_profile_pair(pair)


@pytest.mark.parametrize(
    "path,value,expected",
    [
        (("sensors", "imu", "rate_hz"), 0.0, "rate_hz"),
        (("sensors", "lidar", "min_range"), -0.1, "min_range"),
        (("sensors", "lidar", "max_range"), 0.05, "max_range"),
        (("sensors", "lidar", "horizontal_end_angle"), 0.0, "horizontal"),
        (("sensors", "lidar", "horizontal_end_angle"), 7.0, r"2\*pi"),
    ],
)
@pytest.mark.parametrize("platform", ["sim", "real"])
def test_sensor_numeric_semantics_are_strict(
    tmp_path, platform, path, value, expected
):
    pair = _pair(_selection(tmp_path))
    if path[-1] == "max_range":
        _set_pair_value(
            pair,
            platform,
            ("sensors", "lidar", "min_range"),
            0.05,
        )
    if path[-1] == "horizontal_end_angle" and value == 0.0:
        _set_pair_value(
            pair,
            platform,
            ("sensors", "lidar", "horizontal_start_angle"),
            0.0,
        )
    _set_pair_value(pair, platform, path, value)

    with pytest.raises(ValueError, match=expected):
        pc.validate_profile_pair(pair)


@pytest.mark.parametrize(
    "path,value",
    [
        (("robot", "body", "front_extent"), 0.0),
        (("robot", "body", "rear_extent"), -0.1),
        (("robot", "body", "left_extent"), 0.0),
        (("robot", "body", "right_extent"), -0.1),
        (("robot", "body", "height"), 0.0),
        (("robot", "body", "ground_clearance"), -0.001),
        (("robot", "drive", "wheel_radius"), 0.0),
        (("robot", "drive", "wheel_width"), -0.1),
        (("robot", "drive", "wheel_separation"), 0.0),
        (("sensors", "lidar", "scan_lines"), 0),
        (("sensors", "lidar", "columns_per_scan"), -1),
        (("sensors", "lidar", "scan_rate_hz"), 0.0),
    ],
)
def test_nonphysical_derived_input_fails(tmp_path, path, value):
    pair = _pair(_selection(tmp_path))
    node = pair["real"][1]
    for key in path[:-1]:
        node = node[key]
    node[path[-1]] = value

    with pytest.raises(ValueError, match=path[-1]):
        pc.validate_profile_pair(pair)


def test_nonzero_mount_rotation_is_valid(tmp_path):
    pair = _pair(_selection(tmp_path))
    pair["real"][1]["robot"]["mounts"]["lidar"]["yaw"] = 1.5707963267948966
    pc.validate_profile_pair(pair)


def test_compile_profile_preserves_all_mount_rotations(tmp_path):
    config = _selection(tmp_path, "real")
    real_path = tmp_path / "profiles" / "real.yaml"
    real = yaml.safe_load(real_path.read_text(encoding="utf-8"))
    expected = {
        "lidar": {"roll": 0.1, "pitch": -0.2, "yaw": 0.3},
        "imu": {"roll": -0.4, "pitch": 0.5, "yaw": -0.6},
    }
    for mount_name, rotations in expected.items():
        real["robot"]["mounts"][mount_name].update(rotations)
    real_path.write_text(yaml.safe_dump(real, sort_keys=False), encoding="utf-8")

    output = pc.compile_profile(config, tmp_path / "output")
    mounts = yaml.safe_load(output.read_text(encoding="utf-8"))["derived"][
        "geometry"
    ]["mounts_relative_to_base_link"]

    assert {
        mount_name: {
            axis: mounts[mount_name][axis]
            for axis in ("roll", "pitch", "yaw")
        }
        for mount_name in expected
    } == expected


@pytest.mark.parametrize(
    "platform,expected_points,expected_period",
    [("sim", 28800, 0.1), ("real", 38400, 0.1)],
)
def test_compile_profile_derives_sensor_contract(
    tmp_path, platform, expected_points, expected_period
):
    config = _selection(tmp_path, platform)
    output = tmp_path / "output"
    path = pc.compile_profile(config, output)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))

    assert path == output / "effective_profile.generated.yaml"
    assert data["platform"] == platform
    assert data["source_profile"] == str(
        (tmp_path / "profiles" / f"{platform}.yaml").resolve()
    )
    assert data["derived"]["use_sim_time"] is (platform == "sim")
    assert data["derived"]["sensor_contract"] == {
        "points_per_scan": expected_points,
        "scan_period": expected_period,
    }


def test_real_derived_geometry_matches_confirmed_legacy_baseline(tmp_path):
    path = pc.compile_profile(_selection(tmp_path, "real"), tmp_path / "out")
    geometry = yaml.safe_load(path.read_text(encoding="utf-8"))["derived"]["geometry"]

    assert geometry["body"] == {
        "length": 0.96,
        "width": 0.61,
        "height": 0.377,
        "center_x": 0.0,
        "center_y": 0.0,
        "base_link_height": 0.3315,
    }
    assert geometry["drive"] == {
        "wheel_radius": 0.1025,
        "wheel_width": 0.101,
        "wheel_separation": 0.463,
    }
    assert geometry["footprint"] == [
        [0.480, 0.305],
        [0.480, -0.305],
        [-0.480, -0.305],
        [-0.480, 0.305],
    ]
    assert geometry["mounts_relative_to_base_link"]["lidar"] == {
        "x": 0.443,
        "y": 0.0,
        "z": 0.5735,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }


def test_sim_derived_geometry_matches_current_xacro(tmp_path):
    path = pc.compile_profile(_selection(tmp_path, "sim"), tmp_path / "out")
    geometry = yaml.safe_load(path.read_text(encoding="utf-8"))["derived"]["geometry"]
    assert geometry["body"]["length"] == 0.75
    assert geometry["body"]["width"] == 0.55
    assert geometry["body"]["base_link_height"] == 0.32
    assert geometry["drive"]["wheel_radius"] == 0.12
    assert geometry["drive"]["wheel_separation"] == 0.55
    assert geometry["mounts_relative_to_base_link"]["lidar"]["z"] == pytest.approx(0.236)


def test_automatic_output_uses_private_temp_directory(tmp_path, monkeypatch):
    private = tmp_path / "private"

    def fake_mkdtemp(prefix):
        assert prefix == "system_bringup-profile-"
        private.mkdir()
        return str(private)

    monkeypatch.setattr(pc.tempfile, "mkdtemp", fake_mkdtemp)
    path = pc.compile_profile(_selection(tmp_path))
    assert path == private / "effective_profile.generated.yaml"


def test_explicit_output_overwrites_only_generated_file(tmp_path):
    output = tmp_path / "output"
    output.mkdir()
    keep = output / "keep.txt"
    keep.write_text("keep", encoding="utf-8")
    generated = output / "effective_profile.generated.yaml"
    generated.write_text("old", encoding="utf-8")

    path = pc.compile_profile(_selection(tmp_path), output)

    assert path == generated
    assert keep.read_text(encoding="utf-8") == "keep"
    assert yaml.safe_load(generated.read_text(encoding="utf-8"))["platform"] == "real"


def test_failed_compilation_does_not_create_final_file(tmp_path):
    config = _selection(tmp_path)
    real_path = tmp_path / "profiles" / "real.yaml"
    real = yaml.safe_load(real_path.read_text(encoding="utf-8"))
    real["robot"]["body"]["front_extent"] = 0.0
    real_path.write_text(yaml.safe_dump(real), encoding="utf-8")
    output = tmp_path / "output"

    with pytest.raises(ValueError, match="front_extent"):
        pc.compile_profile(config, output)

    assert not (output / "effective_profile.generated.yaml").exists()


def test_cli_prints_absolute_generated_path(tmp_path, capsys):
    output = tmp_path / "output"
    result = pc.main([
        "--bringup-config", str(_selection(tmp_path)),
        "--output-dir", str(output),
    ])
    captured = capsys.readouterr()
    assert result == 0
    assert captured.err == ""
    assert captured.out.strip() == str(
        (output / "effective_profile.generated.yaml").resolve()
    )


def test_cli_reports_validation_error_without_traceback(tmp_path, capsys):
    config = _selection(tmp_path)
    value = yaml.safe_load(config.read_text(encoding="utf-8"))
    value["platform"] = "bad"
    config.write_text(yaml.safe_dump(value), encoding="utf-8")

    result = pc.main(["--bringup-config", str(config)])
    captured = capsys.readouterr()
    assert result == 1
    assert "platform" in captured.err
    assert "Traceback" not in captured.err


def test_cli_reports_mixed_type_bringup_profile_keys_without_traceback(
    tmp_path, capsys
):
    config = _selection(tmp_path)
    value = yaml.safe_load(config.read_text(encoding="utf-8"))
    value["profiles"].update(
        {1: "profiles/sim.yaml", "extra": "profiles/real.yaml"}
    )
    config.write_text(
        yaml.safe_dump(value, sort_keys=False),
        encoding="utf-8",
    )

    result = pc.main(["--bringup-config", str(config)])
    captured = capsys.readouterr()

    assert result == 1
    assert captured.out == ""
    assert captured.err == (
        f"profile compilation failed: {config.resolve()}: profiles keys invalid; "
        "missing=[], unexpected=[1, 'extra']\n"
    )
    assert "Traceback" not in captured.err


@pytest.mark.parametrize(
    "mutation,expected",
    [
        (
            lambda profile: profile["robot"]["body"].update(
                {1: 0.0, "extra": 0.0}
            ),
            "unexpected keys",
        ),
        (
            lambda profile: profile["robot"]["body"].update(
                front_extent=-(10**1000)
            ),
            "front_extent",
        ),
    ],
    ids=["mixed-type-key", "extremely-large-integer"],
)
def test_cli_reports_legal_yaml_validation_errors_without_traceback(
    tmp_path, capsys, mutation, expected
):
    config = _selection(tmp_path)
    real_path = tmp_path / "profiles" / "real.yaml"
    real = yaml.safe_load(real_path.read_text(encoding="utf-8"))
    mutation(real)
    real_path.write_text(yaml.safe_dump(real, sort_keys=False), encoding="utf-8")

    result = pc.main(["--bringup-config", str(config)])
    captured = capsys.readouterr()

    assert result == 1
    assert captured.out == ""
    assert captured.err.startswith("profile compilation failed: ")
    assert expected in captured.err
    assert captured.err.count("\n") == 1
    assert "Traceback" not in captured.err


def test_cli_rejects_extremely_large_positive_integer_without_traceback(
    tmp_path, capsys
):
    config = _selection(tmp_path)
    real_path = tmp_path / "profiles" / "real.yaml"
    real = yaml.safe_load(real_path.read_text(encoding="utf-8"))
    real["robot"]["body"]["front_extent"] = 10**1000
    real_path.write_text(yaml.safe_dump(real, sort_keys=False), encoding="utf-8")

    try:
        result = pc.main(["--bringup-config", str(config)])
    except Exception as exc:
        pytest.fail(f"CLI leaked {type(exc).__name__}: {exc}")
    captured = capsys.readouterr()

    assert result == 1
    assert captured.out == ""
    assert "front_extent" in captured.err
    assert "finite" in captured.err
    assert "Traceback" not in captured.err


def test_cli_missing_required_argument_returns_one_without_traceback(capsys):
    result = pc.main([])
    captured = capsys.readouterr()
    assert result == 1
    assert captured.out == ""
    assert captured.err == (
        "profile compilation failed: the following arguments are required: "
        "--bringup-config\n"
    )


def test_formal_bringup_does_not_import_profile_compiler():
    launch = (
        PACKAGE_ROOT / "launch" / "bringup.launch.py"
    ).read_text(encoding="utf-8")
    assert "profile_compiler" not in launch
    assert "compile_profile" not in launch


def test_setup_registers_compiler_and_installs_profiles():
    setup_source = (PACKAGE_ROOT / "setup.py").read_text(encoding="utf-8")
    assert (
        "compile_profile = system_bringup.profile_compiler:main"
        in setup_source
    )
    assert "config/profiles/*.yaml" in setup_source
    assert "config/templates/*.yaml" in setup_source
    assert (
        "compile_runtime_configs = "
        "system_bringup.runtime_config_compiler:main"
    ) in setup_source
    assert "bringup.yaml" not in setup_source
    assert "glob('config/*.yaml')" not in setup_source
