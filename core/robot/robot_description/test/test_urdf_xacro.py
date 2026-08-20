# 各 arg 组合下 robot.urdf.xacro 能展开成合法 URDF。
# 在构建机执行(需 xacro + liburdfdom-tools)：colcon test --packages-select robot_description
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

XACRO = os.path.join(
    get_package_share_directory("robot_description"), "urdf", "robot.urdf.xacro"
)

# (use_gazebo, use_mock_hardware) —— 三态：真机 / mock / gz
COMBOS = [
    ("false", "false"),  # 真机硬件分支
    ("false", "true"),   # mock 硬件分支
    ("true", "false"),   # Gz 仿真分支
]
GENERATED_CONTROLLERS_PATH = (
    "/tmp/system_bringup-runtime-test/generated_robot_controllers.yaml"
)


@pytest.mark.parametrize("use_gazebo,use_mock_hardware", COMBOS)
def test_xacro_expands_to_valid_urdf(use_gazebo, use_mock_hardware):
    xacro_bin = shutil.which("xacro")
    check_urdf = shutil.which("check_urdf")
    assert xacro_bin, "xacro 不在 PATH"
    assert check_urdf, "check_urdf 不在 PATH(需 liburdfdom-tools)"

    urdf = subprocess.check_output(
        [
            xacro_bin, XACRO,
            f"use_gazebo:={use_gazebo}",
            f"use_mock_hardware:={use_mock_hardware}",
            f"gz_controllers_file:={GENERATED_CONTROLLERS_PATH}",
        ],
        text=True,
    )
    assert "<robot" in urdf
    assert "left_wheel_joint" in urdf
    assert "right_wheel_joint" in urdf
    root = ET.fromstring(urdf)
    ros2_control = root.find("ros2_control")
    assert ros2_control is not None
    joints = [joint.attrib["name"] for joint in ros2_control.findall("joint")]
    assert joints == ["left_wheel_joint", "right_wheel_joint"]

    if use_gazebo == "false" and use_mock_hardware == "false":
        hardware = ros2_control.find("hardware")
        assert hardware is not None
        params = {
            param.attrib["name"]: (param.text or "").strip()
            for param in hardware.findall("param")
        }
        assert params == {
            "activation_wait_sec": "5.0",
            "feedback_timeout_sec": "0.5",
            "max_motor_rpm": "256",
        }

    # Phase 2 新增顶置传感器挂载帧
    assert 'link name="velodyne"' in urdf
    assert 'link name="imu_link"' in urdf
    # Phase 5a：base_footprint 根帧存在,且有 base_footprint_joint
    assert 'link name="base_footprint"' in urdf
    assert 'joint name="base_footprint_joint"' in urdf

    # gz 分支必含 gz 插件且不含 mock/真机插件；非 gz 分支反之
    if use_gazebo == "true":
        assert "gz_ros2_control/GazeboSimSystem" in urdf
        assert "gz_ros2_control-system" in urdf  # gazebo 插件标签也注入了
        plugin = next(
            item
            for item in root.findall("./gazebo/plugin")
            if item.attrib.get("filename") == "gz_ros2_control-system"
        )
        parameters = plugin.find("parameters")
        assert parameters is not None
        assert (parameters.text or "").strip() == GENERATED_CONTROLLERS_PATH
    elif use_mock_hardware == "true":
        assert "mock_components/GenericSystem" in urdf
    else:
        assert "robot_hardware/DiffDriveSystem" in urdf

    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(urdf)
        path = f.name
    try:
        subprocess.check_call([check_urdf, path])
    finally:
        os.remove(path)


def test_real_geometry_arguments_place_body_wheels_and_independent_sensor_mounts():
    xacro_bin = shutil.which("xacro")
    assert xacro_bin, "xacro 不在 PATH"
    urdf = subprocess.check_output(
        [
            xacro_bin, XACRO,
            "use_gazebo:=false",
            "use_mock_hardware:=false",
            "base_length:=0.960",
            "base_width:=0.610",
            "base_height:=0.377",
            "base_link_height:=0.3315",
            "wheel_radius:=0.1025",
            "wheel_width:=0.101",
            "wheel_separation:=0.463",
            "lidar_x:=0.443",
            "lidar_y:=0.071",
            "lidar_z:=0.5735",
            "lidar_roll:=0.13",
            "lidar_pitch:=-0.21",
            "lidar_yaw:=0.34",
            "imu_x:=0.317",
            "imu_y:=-0.052",
            "imu_z:=0.491",
            "imu_roll:=-0.19",
            "imu_pitch:=0.27",
            "imu_yaw:=-0.38",
        ],
        text=True,
    )
    root = ET.fromstring(urdf)

    def origin(joint_name):
        return root.find(f"./joint[@name='{joint_name}']/origin").attrib

    def numbers(value):
        return pytest.approx([float(item) for item in value.split()])

    assert numbers(root.find(
        "./link[@name='base_link']/collision/geometry/box"
    ).attrib["size"]) == [0.960, 0.610, 0.377]
    assert numbers(origin("base_footprint_joint")["xyz"]) == [0.0, 0.0, 0.3315]
    assert numbers(origin("left_wheel_joint")["xyz"]) == [0.0, 0.2315, -0.229]
    assert numbers(origin("right_wheel_joint")["xyz"]) == [0.0, -0.2315, -0.229]
    assert numbers(origin("velodyne_joint")["xyz"]) == [0.443, 0.071, 0.5735]
    assert numbers(origin("velodyne_joint")["rpy"]) == [0.13, -0.21, 0.34]
    assert numbers(origin("imu_joint")["xyz"]) == [0.317, -0.052, 0.491]
    assert numbers(origin("imu_joint")["rpy"]) == [-0.19, 0.27, -0.38]
    assert origin("imu_joint") != origin("velodyne_joint")


def test_gazebo_sensor_arguments_set_profile_owned_values():
    xacro_bin = shutil.which("xacro")
    assert xacro_bin, "xacro 不在 PATH"
    urdf = subprocess.check_output(
        [
            xacro_bin, XACRO,
            "use_gazebo:=true",
            f"gz_controllers_file:={GENERATED_CONTROLLERS_PATH}",
            "lidar_scan_lines:=32",
            "lidar_columns_per_scan:=1024",
            "lidar_scan_rate_hz:=15",
            "lidar_min_range:=1.2",
            "lidar_max_range:=72.5",
            "lidar_horizontal_start_angle:=-2.4",
            "lidar_horizontal_end_angle:=2.7",
            "imu_rate_hz:=250",
        ],
        text=True,
    )
    root = ET.fromstring(urdf)

    def text(path):
        return root.find(path).text.strip()

    lidar = "./gazebo[@reference='velodyne']/sensor[@name='gpu_lidar']"
    imu = "./gazebo[@reference='imu_link']/sensor[@name='imu']"
    assert text(f"{lidar}/lidar/scan/vertical/samples") == "32"
    assert text(f"{lidar}/lidar/scan/horizontal/samples") == "1024"
    assert text(f"{lidar}/update_rate") == "15"
    assert text(f"{lidar}/lidar/range/min") == "1.2"
    assert text(f"{lidar}/lidar/range/max") == "72.5"
    assert text(f"{lidar}/lidar/scan/horizontal/min_angle") == "-2.4"
    assert text(f"{lidar}/lidar/scan/horizontal/max_angle") == "2.7"
    assert text(f"{imu}/update_rate") == "250"
