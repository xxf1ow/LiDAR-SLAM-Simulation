"""Cross-module source contracts that do not belong to runtime validation."""
import os
from pathlib import Path
import re
import subprocess

import pytest
import yaml

from system_bringup import consistency_check as cc
from system_bringup import runtime_config_compiler as rcc


ROOT = Path(__file__).resolve().parents[4]
FAST_LIO_PATCH = ROOT / "core/localization/fast-lio2.patch"
GICP_TEMPLATE = ROOT / "core/bringup/system_bringup/config/templates/gicp.yaml"
GICP_NODE_SOURCE = ROOT / "core/localization/gicp_localization/src/gicp_localization_node.cpp"
GICP_ALIGNER_HEADER = ROOT / "core/localization/gicp_localization/include/gicp_localization/gicp_aligner.hpp"
GICP_NODE_HEADER = ROOT / "core/localization/gicp_localization/include/gicp_localization/gicp_localization_node.hpp"
VANJEE_TEMPLATE = ROOT / "core/bringup/system_bringup/config/templates/vanjee_lidar.yaml"
VANJEE_NODE_SOURCE = ROOT / "core/robot/drivers/lidar_vanjee_722/vanjee_lidar_ros/src/vanjee_lidar_node.cpp"
VANJEE_DRIVER_CONFIG = ROOT / "core/robot/drivers/lidar_vanjee_722/vanjee_lidar_ros/include/vanjee_lidar_ros/driver_config.hpp"
VANJEE_LAUNCH = ROOT / "core/robot/drivers/lidar_vanjee_722/vanjee_lidar_ros/launch/vanjee_lidar.launch.py"
VANJEE_CMAKE = ROOT / "core/robot/drivers/lidar_vanjee_722/vanjee_lidar_ros/CMakeLists.txt"
RETIRED_VANJEE_CONFIG = ROOT / "core/robot/drivers/lidar_vanjee_722/vanjee_lidar_ros/config/vanjee_722.yaml"


def _load_yaml(path):
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def _patch_file_section(text, relative_path):
    marker = f"diff --git a/{relative_path} b/{relative_path}"
    start = text.find(marker)
    if start < 0:
        raise ValueError(f"patch missing file: {relative_path}")
    return text[start:].split("\ndiff --git ", 1)[0]


def _calls_named(source, marker):
    calls = []
    start = 0
    while True:
        index = source.find(marker, start)
        if index < 0:
            return calls
        open_index = source.find("(", index + len(marker))
        depth = 0
        for end in range(open_index, len(source)):
            if source[end] == "(":
                depth += 1
            elif source[end] == ")":
                depth -= 1
                if depth == 0:
                    calls.append(source[index:end + 1])
                    start = end + 1
                    break
        else:
            raise AssertionError(f"unterminated call: {marker}")


def _top_level_argument_count(call):
    inner = call[call.find("(") + 1:-1]
    depth = 0
    count = 1
    for char in inner:
        if char in "(<[{":
            depth += 1
        elif char in ")>]}":
            depth -= 1
        elif char == "," and depth == 0:
            count += 1
    return count


def _native_cpp_parameter_type(value):
    if type(value) is list:
        assert value and all(type(item) is float for item in value)
        return "std::vector<double>"
    return {
        bool: "bool",
        int: "int",
        float: "double",
        str: "std::string",
    }[type(value)]


def _template_cpp_parameter_types(path, node_name):
    parameters = _load_yaml(path)[node_name]["ros__parameters"]
    return {
        name: _native_cpp_parameter_type(value)
        for name, value in parameters.items()
        if name != "use_sim_time"
    }


def _cpp_parameter_declarations(source):
    declarations = []
    for call in _calls_named(source, "declare_parameter"):
        match = re.match(
            r'^declare_parameter<(?P<type>.+)>\s*\(\s*"(?P<name>[^"]+)"',
            call,
            flags=re.DOTALL,
        )
        assert match is not None, f"untyped parameter declaration: {call}"
        declarations.append(
            (
                match.group("name"),
                match.group("type"),
                _top_level_argument_count(call),
            )
        )
    return declarations


def _assert_cpp_parameter_contract(source, expected_types):
    declarations = _cpp_parameter_declarations(source)
    names = [name for name, _type, _arguments in declarations]
    assert len(declarations) == len(expected_types), (
        "declaration count differs"
    )
    assert len(names) == len(set(names)), "duplicate parameter declaration"
    assert set(names) == set(expected_types), "parameter keys differ"
    actual_types = {
        name: parameter_type
        for name, parameter_type, _arguments in declarations
    }
    assert actual_types == expected_types, "native declaration types differ"
    assert all(
        argument_count == 1
        for _name, _type, argument_count in declarations
    ), "parameter declaration has a usable default"


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


def test_repository_fixture_resolves_from_test_location():
    assert (ROOT / "core/bringup/system_bringup").is_dir()


def test_fast_lio_patch_contains_only_the_imu_qos_source_change():
    patch = FAST_LIO_PATCH.read_text(encoding="utf-8")
    headers = [
        line for line in patch.splitlines() if line.startswith("diff --git ")
    ]
    assert headers == [
        "diff --git a/src/laserMapping.cpp b/src/laserMapping.cpp"
    ]
    assert "config/gazebo_velodyne.yaml" not in patch
    assert "config/vanjee_722.yaml" not in patch
    section = _patch_file_section(patch, "src/laserMapping.cpp")
    hunks = [line for line in section.splitlines() if line.startswith("@@ ")]
    assert hunks == ["@@ -926,7 +926,7 @@ public:"]
    removed = [
        line
        for line in section.splitlines()
        if line.startswith("-") and not line.startswith("---")
    ]
    added = [
        line
        for line in section.splitlines()
        if line.startswith("+") and not line.startswith("+++")
    ]
    assert removed == [
        "-        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>"
        "(imu_topic, 10, imu_cbk);"
    ]
    assert added == [
        "+        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>"
        "(imu_topic, rclcpp::SensorDataQoS(), imu_cbk);"
    ]


def test_fast_lio_patch_passes_git_apply_check_against_pinned_context(tmp_path):
    source = tmp_path / "src/laserMapping.cpp"
    source.parent.mkdir(parents=True)
    source.write_text(
        "// pinned-equivalent filler\n" * 925
        + "        {\n"
        + "            sub_pcl_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(lid_topic, rclcpp::SensorDataQoS(), standard_pcl_cbk);\n"
        + "        }\n"
        + "        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, 10, imu_cbk);\n"
        + "        pubLaserCloudFull_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(\"/cloud_registered\", 20);\n"
        + "        pubLaserCloudFull_body_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(\"/cloud_registered_body\", 20);\n"
        + "        pubLaserCloudEffect_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(\"/cloud_effected\", 20);\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [
            "git",
            "apply",
            "--check",
            "--no-index",
            "--verbose",
            str(FAST_LIO_PATCH),
        ],
        cwd=tmp_path,
        capture_output=True,
        text=True,
        check=False,
        env={**os.environ, "GIT_CEILING_DIRECTORIES": str(tmp_path.parent)},
    )

    assert result.returncode == 0, result.stderr


def test_legacy_checker_does_not_parse_generated_adapter_config_from_launch():
    assert not hasattr(cc, "_adapter_scan_period")


def test_retired_legacy_source_interfaces_are_absent():
    assert not hasattr(cc, "FASTLIO_CONFIG")
    assert not hasattr(cc, "F_NAV_LAUNCH")
    assert not hasattr(cc, "_launch_floats")


def test_gicp_template_parameters_are_declared_once_without_defaults():
    expected = _template_cpp_parameter_types(
        GICP_TEMPLATE, "gicp_localization"
    )
    _assert_cpp_parameter_contract(
        GICP_NODE_SOURCE.read_text(encoding="utf-8"), expected
    )


def test_cpp_parameter_contract_oracle_rejects_wrong_native_type():
    source = GICP_NODE_SOURCE.read_text(encoding="utf-8")
    wrong_type = source.replace(
        'declare_parameter<std::string>("map_frame")',
        'declare_parameter<bool>("map_frame")',
        1,
    )
    assert wrong_type != source
    expected = _template_cpp_parameter_types(
        GICP_TEMPLATE, "gicp_localization"
    )

    with pytest.raises(AssertionError, match="native declaration types differ"):
        _assert_cpp_parameter_contract(wrong_type, expected)


def test_gicp_source_structs_and_members_have_no_tuning_fallbacks():
    aligner_header = GICP_ALIGNER_HEADER.read_text(encoding="utf-8")
    node_header = GICP_NODE_HEADER.read_text(encoding="utf-8")

    for field in (
        "map_voxel_size",
        "scan_voxel_size",
        "max_corr_dist",
        "num_neighbors",
        "num_threads",
        "max_iterations",
    ):
        assert f"{field} =" not in aligner_header
    assert "fitness_threshold_{" not in node_header
    assert "min_scan_points_{" not in node_header


def test_vanjee_template_parameters_are_declared_once_without_defaults():
    expected = _template_cpp_parameter_types(VANJEE_TEMPLATE, "vanjee_lidar")
    _assert_cpp_parameter_contract(
        VANJEE_NODE_SOURCE.read_text(encoding="utf-8"), expected
    )


def test_vanjee_launch_requires_an_explicit_generated_config_file():
    source = VANJEE_LAUNCH.read_text(encoding="utf-8")
    calls = _calls_named(source, "DeclareLaunchArgument")
    config_file_call = next(call for call in calls if '"config_file"' in call)

    assert "default_value" not in config_file_call
    assert "FindPackageShare" not in source
    assert "PathJoinSubstitution" not in source


def test_vanjee_retired_package_config_is_not_protected_or_installed():
    assert not RETIRED_VANJEE_CONFIG.exists()
    assert "vanjee_722.yaml" not in Path(rcc.__file__).read_text(encoding="utf-8")
    _assert_no_config_directory_install(
        VANJEE_CMAKE.read_text(encoding="utf-8")
    )


def test_vanjee_retirement_guard_rejects_multiline_multi_directory_config():
    source = """\
install(
  DIRECTORY launch
            doc
            config/
  DESTINATION share/${PROJECT_NAME}
)
"""

    with pytest.raises(AssertionError, match="retired config directory"):
        _assert_no_config_directory_install(source)


def test_vanjee_driver_config_has_only_empty_or_zero_unconfigured_members():
    source = VANJEE_DRIVER_CONFIG.read_text(encoding="utf-8")

    assert not re.search(
        r"\b(?:std::string|uint16_t|float|bool)\s+\w+\s*\{[^}]+\};",
        source,
    )
