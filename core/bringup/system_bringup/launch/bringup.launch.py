"""Compile one source profile and launch the selected full-stack topology."""
import os
from pathlib import Path

import yaml

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

from system_bringup.consistency_check import (
    run_runtime_consistency,
)
from system_bringup.ready_gate import ready_gate
from system_bringup.runtime_config_compiler import compile_runtime_configs


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _source_bringup_config_path():
    """Map the installed package share to the colcon workspace source tree."""
    share = Path(get_package_share_directory("system_bringup")).resolve()
    prefix = share.parents[1]
    if prefix.name == "install":
        workspace = prefix.parent
    elif prefix.parent.name == "install":
        workspace = prefix.parent.parent
    else:
        raise RuntimeError(
            "无法从 system_bringup package share 定位 colcon 工作区: "
            f"{share} (期望 merged 或 isolated install 布局)"
        )
    return (workspace / "bringup/system_bringup/config/bringup.yaml").resolve()


def _abort_sensor_gate(context, *args, **kwargs):
    raise RuntimeError(
        "传感器契约 gate 未通过；详细原因见 sensor_contract_gate 日志，"
        "已中止整个 launch。"
    )


def _sensor_gate(then_actions, manifest):
    waiter = Node(
        package="system_bringup",
        executable="sensor_contract_gate",
        name="sensor_contract_gate",
        output="screen",
        parameters=[str(manifest["sensor_gate_path"])],
    )
    handler = RegisterEventHandler(
        OnProcessExit(
            target_action=waiter,
            on_exit=lambda event, context: (
                then_actions
                if event.returncode == 0
                else [OpaqueFunction(function=_abort_sensor_gate)]
            ),
        )
    )
    return [waiter, handler]


def _bringup(context, *args, **kwargs):
    source_config = _source_bringup_config_path()
    repo_root = source_config.parents[4]
    try:
        manifest = compile_runtime_configs(source_config)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise RuntimeError(
            f"运行时配置编译失败(已中止,未启动任何节点): {source_config}: {exc}"
        ) from exc

    try:
        failures = run_runtime_consistency(repo_root, manifest)
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        raise RuntimeError(
            "运行时一致性检查失败(已中止,未启动任何节点): "
            f"{source_config}: {exc}"
        ) from exc
    if failures:
        raise RuntimeError(
            f"运行时一致性闸门未通过(已中止,未启动任何节点): {source_config}\n"
            + "\n".join(failures)
        )

    platform = manifest["platform"]
    mode = manifest["mode"]
    use_sim_time = manifest["use_sim_time"]
    if platform not in ("sim", "real"):
        raise RuntimeError(f"manifest platform={platform!r}(应为 sim|real)")
    if mode not in ("navigation", "mapping"):
        raise RuntimeError(f"manifest mode={mode!r}(应为 navigation|mapping)")
    if not isinstance(use_sim_time, bool):
        raise RuntimeError("manifest use_sim_time 必须是 bool")

    cfg = manifest["bringup_config"]
    stack_cfg = cfg["slam_stack"]
    map_artifacts = cfg["map_artifacts"]
    geometry = manifest["robot_launch_arguments"]
    bridge = manifest["fast_lio_body_bridge_arguments"]
    use_sim = "true" if use_sim_time else "false"
    settling = stack_cfg["settling"]
    flow = lambda message: LogInfo(
        msg="======== [system_bringup] %s" % message
    )

    control_layer = [
        Node(
            package="cmd_vel_gate",
            executable="cmd_vel_gate",
            output="screen",
            parameters=[{"use_sim_time": manifest["use_sim_time"]}],
        ),
        Node(
            package="robot_web_ui",
            executable="robot_web_ui",
            output="screen",
            parameters=[
                str(manifest["web_ui_path"]),
                {"map_yaml_path": map_artifacts["nav2_map"]},
            ],
        ),
    ]
    slam_stack = _inc(
        "system_bringup",
        "launch/slam_stack.launch.py",
        {
            "mode": manifest["mode"],
            "use_sim_time": use_sim,
            "lio_sam_params_file": str(manifest["lio_sam_path"]),
            "fast_lio_params_file": str(manifest["fast_lio_path"]),
            "gicp_config_file": str(manifest["gicp_path"]),
            "prior_map_path": map_artifacts["prior_pcd"],
            "nav2_params_file": str(manifest["nav2_path"]),
            "nav_map": map_artifacts["nav2_map"],
            "cmd_vel_output_topic": "/cmd_vel_auto",
            "settling": str(settling),
            **{
                f"fast_lio_body_bridge_{name}": bridge[name]
                for name in ("x", "y", "z", "qx", "qy", "qz", "qw")
            },
        },
    )

    if platform == "sim":
        gz = cfg["robot_gz"]
        base = _inc(
            "robot_gz_bringup",
            "launch/robot_gz.launch.py",
            {
                "gui": gz["gui"],
                "rviz": gz["rviz"],
                "world": gz["world"],
                "spawn_x": gz["spawn_x"],
                "spawn_y": gz["spawn_y"],
                "spawn_z": gz["spawn_z"],
                "controllers_file": str(manifest["controllers_path"]),
                "lidar_adapter_config": str(manifest["lidar_adapter_path"]),
                "use_sim_time": use_sim,
                **geometry,
            },
        )
        flow_log = [
            flow(
                "运行时一致性闸门通过 | platform=sim | mode=%s | source=%s"
                % (mode, source_config)
            ),
            flow(
                "① 起 robot_gz 仿真底层 → ready_gate 等 "
                "/joint_states + settling %ss → 传感器契约 gate → 起 slam_stack"
                % settling
            ),
        ]
        sensor_gate_actions = _sensor_gate(
            [*flow_log,
             flow("手机访问 http://<机器人或仿真主机IP>:8080\n"
                  "mapping：点击“人工接管”后按住方向按钮驾驶\n"
                  "navigation：默认自动；点击“人工接管”屏蔽 Nav2，"
                  "点击“恢复自动导航”恢复"),
             slam_stack],
            manifest,
        )
        return control_layer + [base] + ready_gate(
            ["/joint_states"], 300.0, "robot_gz(controller)",
            sensor_gate_actions,
            use_sim_time=use_sim, settling=settling)

    if platform == "real":
        chassis = _inc(
            "robot_bringup",
            "launch/real_chassis.launch.py",
            {
                "gui": "false",
                "controllers_file": str(manifest["controllers_path"]),
                "use_sim_time": use_sim,
                **geometry,
            },
        )
        lidar = _inc(
            "vanjee_lidar_ros",
            "launch/vanjee_lidar.launch.py",
            {"config_file": str(manifest["vanjee_lidar_path"])},
        )
        flow_log = [
            flow(
                "运行时一致性闸门通过 | platform=real | mode=%s | source=%s"
                % (mode, source_config)
            ),
            flow("① 起真实底盘+Vanjee 722 → 真实传感器 gate 连续验收 2s → 起共享 slam_stack"),
        ]
        sensor_gate_actions = _sensor_gate([slam_stack], manifest)
        return (
            control_layer
            + flow_log
            + [chassis, lidar]
            + sensor_gate_actions
        )


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_bringup)])
