"""全栈启动。config/bringup.yaml 选 platform(sim/real) + mode(nav/map),改 config 不 rebuild。

唯一入口:ros2 launch system_bringup bringup.launch.py
流程:① 一致性闸门(失败即中止)② 底层(platform=sim: robot_gz; platform=real: chassis+Vanjee)
③ ready_gate 等 lidar(+ sim controller)④ slam_stack(mode=navigation: fast_lio→gicp→nav2;
mode=mapping: lio_sam)。
切 platform/mode 改 config/bringup.yaml 顶层两行即可,不用 rebuild。
use_sim_time 从 platform 推断(sim=true, real=false),不在 config。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from system_bringup import consistency_check
from system_bringup.consistency_check import (
    derive_real_geometry,
    real_geometry_launch_arguments,
    write_real_runtime_configs,
)
from system_bringup.ready_gate import ready_gate


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _pkg_config(package, filename):
    return os.path.join(get_package_share_directory(package), "config", filename)


def _abort_real_sensor_gate(context, *args, **kwargs):
    raise RuntimeError(
        "真实传感器 gate 未通过；详细原因见 real_sensor_ready_gate 日志，"
        "已中止整个 launch。"
    )


def _real_sensor_gate(then_actions):
    waiter = Node(
        package="system_bringup",
        executable="real_sensor_ready_gate",
        name="real_sensor_ready_gate",
        output="screen",
        parameters=[{"timeout": 300.0}],
    )
    handler = RegisterEventHandler(
        OnProcessExit(
            target_action=waiter,
            on_exit=lambda event, context: (
                then_actions
                if event.returncode == 0
                else [OpaqueFunction(function=_abort_real_sensor_gate)]
            ),
        )
    )
    return [waiter, handler]


def _bringup(context, *args, **kwargs):
    repo_root = consistency_check.find_repo_root()
    cfg = consistency_check.load_bringup_config(repo_root)
    platform = cfg["platform"]          # sim | real
    mode = cfg["mode"]                  # navigation | mapping
    use_sim = "true" if platform == "sim" else "false"
    stack_cfg = cfg["slam_stack"]
    flow = lambda m: LogInfo(msg="======== [system_bringup] %s" % m)

    failures = consistency_check.run(repo_root)
    if failures:
        raise RuntimeError("一致性闸门未通过(已中止,未启动任何节点):\n" + "\n".join(failures))

    profile = stack_cfg[platform]
    slam_stack = _inc(
        "system_bringup", "launch/slam_stack.launch.py",
        {"mode": mode, "use_sim_time": use_sim,
         "lio_sam_params_file": _pkg_config("lio_sam", profile["lio_sam"]["config"]),
         "fast_lio_config": profile["fast_lio"]["config"],
         "gicp_config_file": _pkg_config(
             "gicp_localization", profile["gicp_localization"]["config"]),
         "prior_map_path": profile["gicp_localization"]["prior_map_path"],
         "nav2_params_file": _pkg_config(
             "robot_navigation", profile["robot_navigation"]["config"]),
         "nav_map": profile["robot_navigation"]["map"],
         "cmd_vel_output_topic": "/cmd_vel_auto",
         "settling": str(stack_cfg["settling"])})

    control_layer = [
        Node(
            package="cmd_vel_gate",
            executable="cmd_vel_gate",
            output="screen",
            parameters=[{"use_sim_time": use_sim == "true"}],
        ),
        Node(
            package="robot_web_ui",
            executable="robot_web_ui",
            output="screen",
            parameters=[{"use_sim_time": use_sim == "true"}],
        ),
    ]

    if platform == "sim":
        gz = cfg["robot_gz"]
        base = _inc("robot_gz_bringup", "launch/robot_gz.launch.py",
                    {"gui": gz.get("gui", "true"), "rviz": gz.get("rviz", "false"),
                     "world": gz.get("world", "factory.sdf"),
                     "spawn_x": gz.get("spawn_x", "4.0"),
                     "spawn_y": gz.get("spawn_y", "0.0"),
                     "spawn_z": gz.get("spawn_z", "0.05")})
        flow_log = [
            flow("一致性闸门通过 | platform=sim | mode=%s | config=源码 bringup.yaml(改它不 rebuild)" % mode),
            flow("① 起 robot_gz 仿真底层 → ready_gate 等 /points_raw+/joint_states + settling %ss 后起 slam_stack" % stack_cfg["settling"]),
        ]
        return control_layer + flow_log + [base] + ready_gate(
            ["/points_raw", "/joint_states"], 300.0, "robot_gz(lidar+controller)",
            [flow("手机访问 http://<机器人或仿真主机IP>:8080\n"
                  "mapping：点击“人工接管”后按住方向按钮驾驶\n"
                  "navigation：默认自动；点击“人工接管”屏蔽 Nav2，"
                  "点击“恢复自动导航”恢复"),
             slam_stack],
            settling=stack_cfg["settling"])

    if platform == "real":
        runtime_paths = write_real_runtime_configs(repo_root, cfg)
        geometry_arguments = real_geometry_launch_arguments(
            derive_real_geometry(cfg)
        )
        # 覆盖上面的共享默认：真机 Nav2 参数和 body 焊接均由 real_geometry 派生。
        slam_stack = _inc(
            "system_bringup", "launch/slam_stack.launch.py",
            {"mode": mode, "use_sim_time": use_sim,
             "lio_sam_params_file": _pkg_config("lio_sam", profile["lio_sam"]["config"]),
             "fast_lio_config": profile["fast_lio"]["config"],
             "gicp_config_file": _pkg_config(
                 "gicp_localization", profile["gicp_localization"]["config"]),
             "prior_map_path": profile["gicp_localization"]["prior_map_path"],
             "nav2_params_file": str(runtime_paths["nav2"]),
             "nav_map": profile["robot_navigation"]["map"],
             "cmd_vel_output_topic": "/cmd_vel_auto",
             "settling": str(stack_cfg["settling"]),
             "weld_x": geometry_arguments["navigation"]["weld_x"],
             "weld_y": geometry_arguments["navigation"]["weld_y"],
             "weld_z": geometry_arguments["navigation"]["weld_z"],
             "weld_roll": geometry_arguments["navigation"]["weld_roll"],
             "weld_pitch": geometry_arguments["navigation"]["weld_pitch"],
             "weld_yaw": geometry_arguments["navigation"]["weld_yaw"]})
        vanjee_config = _pkg_config(
            "vanjee_lidar_ros", cfg["vanjee_lidar"]["config"]
        )
        chassis = _inc(
            "robot_bringup",
            "launch/real_chassis.launch.py",
            {"gui": "false",
             "controllers_file": str(runtime_paths["controllers"]),
             "base_length": geometry_arguments["robot"]["base_length"],
             "base_width": geometry_arguments["robot"]["base_width"],
             "base_height": geometry_arguments["robot"]["base_height"],
             "base_link_height": geometry_arguments["robot"]["base_link_height"],
             "wheel_radius": geometry_arguments["robot"]["wheel_radius"],
             "wheel_width": geometry_arguments["robot"]["wheel_width"],
             "wheel_separation": geometry_arguments["robot"]["wheel_separation"],
             "sensor_x": geometry_arguments["robot"]["sensor_x"],
             "sensor_y": geometry_arguments["robot"]["sensor_y"],
             "sensor_z": geometry_arguments["robot"]["sensor_z"],
             "sensor_roll": geometry_arguments["robot"]["sensor_roll"],
             "sensor_pitch": geometry_arguments["robot"]["sensor_pitch"],
             "sensor_yaw": geometry_arguments["robot"]["sensor_yaw"]},
        )
        lidar = _inc(
            "vanjee_lidar_ros",
            "launch/vanjee_lidar.launch.py",
            {"config_file": vanjee_config},
        )
        flow_log = [
            flow("一致性闸门通过 | platform=real | mode=%s | config=源码 bringup.yaml(改它不 rebuild)" % mode),
            flow("① 起真实底盘+Vanjee 722 → 真实传感器 gate 连续验收 2s → 起共享 slam_stack"),
        ]
        return (
            control_layer
            + flow_log
            + [chassis, lidar]
            + _real_sensor_gate([slam_stack])
        )

    raise RuntimeError("未知 platform='%s'(应为 sim|real)" % platform)


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_bringup)])
