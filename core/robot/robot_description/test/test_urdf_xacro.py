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
            "gz_controllers_file:=/tmp/robot_controllers.yaml",
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


def test_real_geometry_arguments_place_body_wheels_and_sensor():
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
            "sensor_x:=0.443",
            "sensor_y:=0.0",
            "sensor_z:=0.5735",
            "sensor_roll:=0.0",
            "sensor_pitch:=0.0",
            "sensor_yaw:=0.0",
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
    assert numbers(origin("velodyne_joint")["xyz"]) == [0.443, 0.0, 0.5735]
    assert numbers(origin("velodyne_joint")["rpy"]) == [0.0, 0.0, 0.0]
    assert origin("imu_joint") == origin("velodyne_joint")
