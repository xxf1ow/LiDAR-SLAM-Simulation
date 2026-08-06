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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from system_bringup.ready_gate import ready_gate


def _inc(pkg, rel, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(pkg), rel)),
        launch_arguments=(args or {}).items())


def _stack(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    use_sim = LaunchConfiguration("use_sim_time").perform(context)
    lio_sam_params = LaunchConfiguration("lio_sam_params_file").perform(context)
    fast_cfg = LaunchConfiguration("fast_lio_config").perform(context)
    gicp_config = LaunchConfiguration("gicp_config_file").perform(context)
    prior = LaunchConfiguration("prior_map_path").perform(context)
    nav2_params = LaunchConfiguration("nav2_params_file").perform(context)
    nav_map = LaunchConfiguration("nav_map").perform(context)
    cmd_vel_output_topic = LaunchConfiguration("cmd_vel_output_topic").perform(context)
    weld = {
        name: LaunchConfiguration(name).perform(context)
        for name in (
            "weld_x", "weld_y", "weld_z",
            "weld_qx", "weld_qy", "weld_qz", "weld_qw",
        )
    }
    settling = float(LaunchConfiguration("settling").perform(context))
    flow = lambda m: LogInfo(msg="======== [slam_stack] %s" % m)

    if mode == "mapping":
        # lio_sam 的 use_sim_time 来自所选参数文件,不接收 launch arg。
        return [
            flow("MODE=mapping → ② 起 lio_sam 建图(4 节点 + 自带 RViz);建完用 save_map.sh 存盘"),
            _inc("lio_sam", "launch/run.launch.py", {"params_file": lio_sam_params}),
        ]
    if mode == "navigation":
        fast_lio = _inc("fast_lio", "launch/mapping.launch.py",
                        {"config_file": fast_cfg, "use_sim_time": use_sim, "rviz": "false"})
        gicp = _inc("gicp_localization", "launch/localization.launch.py",
                    {"config_file": gicp_config, "prior_map_path": prior,
                     "use_sim_time": use_sim})
        nav2 = _inc("robot_navigation", "launch/navigation.launch.py",
                    {"params_file": nav2_params, "map": nav_map, "use_rviz": "true",
                     "use_sim_time": use_sim, "cmd_vel_output_topic": cmd_vel_output_topic,
                     **weld})
        # 链式就绪闸门(非阻塞):等上游真发出关键话题 + settling 后才起下个,超时中止
        return [
            flow("MODE=navigation → ② fast_lio → gate(/Odometry+/cloud_registered) → gicp → gate(/localization+/base_controller/odom) → nav2"),
            fast_lio,
        ] + ready_gate(["/Odometry", "/cloud_registered"], 60.0,
                       "fast_lio→/Odometry+/cloud_registered",
                       [gicp] + ready_gate(
                           ["/localization", "/base_controller/odom"], 60.0,
                           "gicp+base_controller→/localization+/base_controller/odom",
                           [nav2], use_sim_time=use_sim, settling=settling),
                       use_sim_time=use_sim, settling=settling)
    raise RuntimeError("未知 mode='%s'(应为 navigation|mapping)" % mode)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="navigation"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "lio_sam_params_file",
            default_value=os.path.join(
                get_package_share_directory("lio_sam"), "config", "params.yaml")),
        DeclareLaunchArgument("fast_lio_config", default_value="gazebo_velodyne.yaml"),
        DeclareLaunchArgument(
            "gicp_config_file",
            default_value=os.path.join(
                get_package_share_directory("gicp_localization"), "config",
                "gicp_localization.yaml")),
        DeclareLaunchArgument("prior_map_path", default_value="~/result/GlobalMap.pcd"),
        DeclareLaunchArgument(
            "nav2_params_file",
            default_value=os.path.join(
                get_package_share_directory("robot_navigation"), "config", "nav2_params.yaml")),
        DeclareLaunchArgument("nav_map", default_value="~/result/factory_map.yaml"),
        DeclareLaunchArgument("cmd_vel_output_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("weld_x", default_value="0.0"),
        DeclareLaunchArgument("weld_y", default_value="0.0"),
        DeclareLaunchArgument("weld_z", default_value="-0.5560"),
        DeclareLaunchArgument("weld_qx", default_value="0.0"),
        DeclareLaunchArgument("weld_qy", default_value="0.0"),
        DeclareLaunchArgument("weld_qz", default_value="0.0"),
        DeclareLaunchArgument("weld_qw", default_value="1.0"),
        DeclareLaunchArgument("settling", default_value="20.0"),
        OpaqueFunction(function=_stack),
    ])
