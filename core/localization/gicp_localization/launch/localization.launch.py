import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('gicp_localization')
    default_cfg = os.path.join(pkg, 'config', 'gicp_localization.yaml')

    config_file = LaunchConfiguration('config_file')
    prior_map_path = LaunchConfiguration('prior_map_path')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_cfg),
        # 先验图默认路径用 expanduser 展开 ~(LIO-SAM 5b save_map 落 ~/result/GlobalMap.pcd);
        # 否则字面 "~" 传给 PCL loadPCDFile 不会展开、加载失败。路径不同才传该 arg 覆盖。
        DeclareLaunchArgument('prior_map_path',
                              default_value=os.path.expanduser('~/result/GlobalMap.pcd')),
        Node(
            package='gicp_localization',
            executable='gicp_localization_node',
            name='gicp_localization',
            output='screen',
            parameters=[config_file, {'prior_map_path': prior_map_path}],
        ),
    ])
