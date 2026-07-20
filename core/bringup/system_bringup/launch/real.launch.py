"""真机全栈启动(本期仅骨架,未实现底层驱动)。可变参数在 config/bringup.yaml(改它不用 rebuild)。

与 sim.launch.py 同构:① 一致性闸门 → ② 真机底层(TODO) → ③ slam_stack(mode)。
本期真机驱动包尚不存在,② 留 TODO;上真机时填底层 include 即可。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource

from system_bringup import consistency_check
from system_bringup.ready_gate import ready_gate


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _bringup(context, *args, **kwargs):
    repo_root = consistency_check.find_repo_root()
    cfg = consistency_check.load_bringup_config(repo_root)
    flow = lambda m: LogInfo(msg="======== [system_bringup] %s" % m)
    failures = consistency_check.run(repo_root)
    if failures:
        raise RuntimeError("一致性闸门未通过(已中止):\n" + "\n".join(failures))

    actions = [
        flow("真机骨架 | MODE=%s | config=源码 bringup.yaml(改它不 rebuild)" % cfg["mode"]),
        LogInfo(msg="[real.launch] 真机底层未实现(骨架);上真机时在此 include robot_bringup(真实硬件)+ velodyne/imu 驱动。"),
    ]
    # TODO(真机): 取消下行注释并补真机底层 ——
    #   robot_bringup robot.launch.py use_mock_hardware:=false + 真实 velodyne/imu 驱动节点。
    # actions.append(_inc("robot_bringup", "launch/robot.launch.py", {"use_mock_hardware": "false"}))

    slam_stack = _inc(
        "system_bringup", "launch/slam_stack.launch.py",
        {"mode": cfg["mode"], "use_sim_time": "false",
         "fast_lio_config": cfg["fast_lio_config"], "prior_map_path": cfg["prior_map_path"],
         "nav_map": cfg["nav_map"], "settling": str(cfg["settling"])})
    # 真机底层起好后,等 velodyne 发 /points_raw + settling 后再起上层栈(非阻塞);真机驱动 TODO
    actions += ready_gate("/points_raw", 300.0, "真机 velodyne→/points_raw", [slam_stack], settling=cfg["settling"])
    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_bringup)])
