# 各 arg 组合下 robot.urdf.xacro 能展开成合法 URDF。
# 在构建机执行(需 xacro + liburdfdom-tools)：colcon test --packages-select robot_description
import os
import shutil
import subprocess
import tempfile

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
    # Phase 2 新增顶置传感器挂载帧(根仍 base_link，未加 base_footprint)
    assert 'link name="velodyne"' in urdf
    assert 'link name="imu_link"' in urdf

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
