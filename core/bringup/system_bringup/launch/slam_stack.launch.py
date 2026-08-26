"""共享上层 SLAM/定位/导航栈(被 sim/real 内部 include,不直接用命令行调)。

mode=navigation:fast_lio 里程计 → (错峰) gicp 定位 → (错峰) nav2。
mode=mapping   :lio_sam 建图(与导航互斥,产先验图)。
参数由父级 launch 透传(launch_arguments),本文件用 DeclareLaunchArgument 接收。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from system_bringup.ready_gate import ready_gate


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _stack(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    use_sim = LaunchConfiguration("use_sim_time").perform(context)
    lio_sam_params = LaunchConfiguration("lio_sam_params_file").perform(context)
    gicp_config = LaunchConfiguration("gicp_config_file").perform(context)
    prior = LaunchConfiguration("prior_map_path").perform(context)
    nav2_params = LaunchConfiguration("nav2_params_file").perform(context)
    nav_map = LaunchConfiguration("nav_map").perform(context)
    cmd_vel_output_topic = LaunchConfiguration("cmd_vel_output_topic").perform(context)
    settling = float(LaunchConfiguration("settling").perform(context))
    flow = lambda m: LogInfo(msg="======== [slam_stack] %s" % m)

    if mode == "mapping":
        # lio_sam 的 use_sim_time 来自所选参数文件,不接收 launch arg。
        return [
            flow("MODE=mapping → ② 起 lio_sam 建图(4 节点 + 自带 RViz);建完用 save_map.sh 存盘"),
            _inc("lio_sam", "launch/run.launch.py", {"params_file": lio_sam_params}),
        ]
    if mode == "navigation":
        fast_lio_params_file = LaunchConfiguration(
            "fast_lio_params_file"
        ).perform(context)
        fast_lio_config_path = os.path.dirname(fast_lio_params_file)
        fast_lio_config_file = os.path.basename(fast_lio_params_file)
        if not os.path.isabs(fast_lio_params_file) or not fast_lio_config_file:
            raise RuntimeError(
                "fast_lio_params_file must be an absolute generated YAML path"
            )

        bridge = {
            name: LaunchConfiguration(
                f"fast_lio_body_bridge_{name}"
            ).perform(context)
            for name in ("x", "y", "z", "qx", "qy", "qz", "qw")
        }

        fast_lio = _inc(
            "fast_lio",
            "launch/mapping.launch.py",
            {
                "config_path": fast_lio_config_path,
                "config_file": fast_lio_config_file,
                "use_sim_time": use_sim,
                "rviz": "false",
            },
        )
        body_bridge = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="fast_lio_body_to_base_footprint",
            output="screen",
            arguments=[
                "--x", bridge["x"], "--y", bridge["y"], "--z", bridge["z"],
                "--qx", bridge["qx"], "--qy", bridge["qy"],
                "--qz", bridge["qz"], "--qw", bridge["qw"],
                "--frame-id", "body", "--child-frame-id", "base_footprint",
            ],
            parameters=[
                {
                    "use_sim_time": ParameterValue(
                        LaunchConfiguration("use_sim_time"), value_type=bool
                    )
                }
            ],
        )
        gicp = _inc("gicp_localization", "launch/localization.launch.py",
                    {"config_file": gicp_config, "prior_map_path": prior})
        rviz = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            condition=IfCondition(LaunchConfiguration("stack_rviz")),
            arguments=[
                "-d",
                os.path.join(
                    get_package_share_directory("robot_navigation"),
                    "config/nav2.rviz",
                ),
            ],
            parameters=[{"use_sim_time": use_sim == "true"}],
        )
        nav2 = _inc("robot_navigation", "launch/navigation.launch.py",
                    {"params_file": nav2_params, "map": nav_map, "use_rviz": "false",
                     "use_sim_time": use_sim, "cmd_vel_output_topic": cmd_vel_output_topic})
        # 链式就绪闸门(非阻塞):等上游真发出关键话题 + settling 后才起下个,超时中止
        return [
            flow("MODE=navigation → ② fast_lio → gate(/Odometry+/cloud_registered_body) → gicp → gate(/localization+/base_controller/odom) → nav2"),
            body_bridge,
            fast_lio,
            rviz,
        ] + ready_gate(["/Odometry", "/cloud_registered_body"], 60.0,
                       "fast_lio→/Odometry+/cloud_registered_body",
                       [gicp] + ready_gate(
                           ["/localization", "/base_controller/odom"], 60.0,
                           "gicp+base_controller→/localization+/base_controller/odom",
                           [nav2], use_sim_time=use_sim, settling=settling),
                       use_sim_time=use_sim, settling=settling)
    raise RuntimeError("未知 mode='%s'(应为 navigation|mapping)" % mode)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode"),
        DeclareLaunchArgument("use_sim_time"),
        DeclareLaunchArgument("lio_sam_params_file"),
        DeclareLaunchArgument("fast_lio_params_file"),
        DeclareLaunchArgument("gicp_config_file"),
        DeclareLaunchArgument("prior_map_path"),
        DeclareLaunchArgument("nav2_params_file"),
        DeclareLaunchArgument("nav_map"),
        DeclareLaunchArgument("cmd_vel_output_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("stack_rviz"),
        DeclareLaunchArgument("fast_lio_body_bridge_x"),
        DeclareLaunchArgument("fast_lio_body_bridge_y"),
        DeclareLaunchArgument("fast_lio_body_bridge_z"),
        DeclareLaunchArgument("fast_lio_body_bridge_qx"),
        DeclareLaunchArgument("fast_lio_body_bridge_qy"),
        DeclareLaunchArgument("fast_lio_body_bridge_qz"),
        DeclareLaunchArgument("fast_lio_body_bridge_qw"),
        DeclareLaunchArgument("settling"),
        OpaqueFunction(function=_stack),
    ])
