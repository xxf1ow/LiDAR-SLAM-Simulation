"""Gazebo Harmonic 仿真侧 bringup：起带传感器系统插件的测试世界 + spawn 机器人(自带 gpu_lidar/imu)
+ robot_state_publisher + 控制器 spawner + ros_gz_bridge(clock/lidar/imu) + ring/time 适配节点。
controller_manager 由 URDF 里的 gz_ros2_control 插件提供(无独立 ros2_control_node)。"""
import glob
import os

from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("gui", default_value="true",
                              description="true=带 GUI 起 Gz 与 RViz；false=headless。"),
        DeclareLaunchArgument("prefix", default_value="",
                              description="link/joint 名前缀。"),
        DeclareLaunchArgument("world", default_value="factory.sdf",
                              description="worlds/ 下的世界文件名(factory.sdf=工厂; test_world.sdf=冒烟回退)。"),
        DeclareLaunchArgument(
            "factory_models_path",
            # 默认指向构建机仓库布局 ~/LiDAR-SLAM-Simulation/models/factory_model(用 $HOME,
            # 不写死用户名),免得每次手填;仓库不在此处就传 factory_models_path:= 覆盖,
            # 或置空依赖已 export 的 GZ_SIM_RESOURCE_PATH。
            default_value=os.path.expanduser("~/LiDAR-SLAM-Simulation/models/factory_model"),
            description="model:// 资产父目录(.../models/factory_model)的绝对路径;"
                        "默认 ~/LiDAR-SLAM-Simulation/models/factory_model,可覆盖或置空。"),
        DeclareLaunchArgument("spawn_x", default_value="4.0", description="机器人 spawn X(避开原点 workcell)。"),
        DeclareLaunchArgument("spawn_y", default_value="0.0", description="机器人 spawn Y。"),
        DeclareLaunchArgument("spawn_z", default_value="0.05",
                              description="机器人 spawn Z(根=base_footprint 在地面，留 5cm 落地余量)。"),
    ]
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")

    pkg_gz = FindPackageShare("robot_gz_bringup")
    world = PathJoinSubstitution([pkg_gz, "worlds", LaunchConfiguration("world")])
    bridge_cfg = PathJoinSubstitution([pkg_gz, "config", "bridge.yaml"])
    # GZ_SIM_RESOURCE_PATH 需含:① model:// 资产父目录(解析 model://workcell 等),
    # ② 各模型 materials/textures 目录(mesh 的 mtl/dae 用裸文件名引贴图 Floor.png 等,
    #    否则 Gz 报 "Could not resolve file [...]"、模型无纹理)。后者在 launch 时按实际目录展开。
    def _set_resource_paths(context, *args, **kwargs):
        models = LaunchConfiguration("factory_models_path").perform(context)
        if not models:
            return []
        paths = [models]
        paths += sorted(glob.glob(os.path.join(models, "*", "materials", "textures")))
        return [AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.pathsep.join(paths))]

    set_resource_path = OpaqueFunction(function=_set_resource_paths)
    gz_controllers_file = PathJoinSubstitution(
        [FindPackageShare("robot_bringup"), "config", "robot_controllers.yaml"])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution([FindPackageShare("robot_description"), "urdf", "robot.urdf.xacro"]),
        " ", "use_gazebo:=true",
        " ", "prefix:=", prefix,
        " ", "gz_controllers_file:=", gz_controllers_file,
    ])
    # robot_description 必须显式声明为 str：含 <gazebo> 传感器块的 URDF 不是合法 YAML，
    # launch_ros 默认会 yaml.safe_load 推断类型而报错(见 launch 报错提示)。
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str),
        "use_sim_time": True,
    }
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robot_description"), "rviz", "robot.rviz"])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", [" -r -v 3 ", world])],
        condition=IfCondition(gui),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", [" --headless-rendering -s -r -v 3 ", world])],
        condition=UnlessCondition(gui),
    )

    # 桥接：clock + /lidar/points + /imu→/imu_plugin/out
    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="ros_gz_bridge", output="screen",
        parameters=[{"config_file": bridge_cfg, "use_sim_time": True}],
    )

    # ring/time 适配：Gz 组织化点云 → Velodyne 风格 /points_raw(补 time、透传 native ring)
    lidar_adapter = Node(
        package="lidar_pointcloud_adapter", executable="adapter_node",
        name="lidar_pointcloud_adapter", output="screen",
        parameters=[{
            "input_topic": "/lidar/points",
            "output_topic": "/points_raw",
            "output_frame": "velodyne",
            "scan_period": 0.1,
            "use_sim_time": True,
        }],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "/robot_description", "-name", "robot", "-allow_renaming", "true",
                   "-x", LaunchConfiguration("spawn_x"),
                   "-y", LaunchConfiguration("spawn_y"),
                   "-z", LaunchConfiguration("spawn_z")],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher", executable="robot_state_publisher",
        output="both", parameters=[robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    robot_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["base_controller", "--controller-manager", "/controller_manager"],
    )
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2", output="log",
        arguments=["-d", rviz_config_file], parameters=[{"use_sim_time": True}],
        condition=IfCondition(gui),
    )

    nodes = [
        set_resource_path,
        gz,
        gz_headless,
        bridge,
        lidar_adapter,
        robot_state_pub_node,
        gz_spawn_entity,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        rviz_node,
    ]
    return LaunchDescription(declared_arguments + nodes)
