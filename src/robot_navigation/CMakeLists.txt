cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

# 纯配置/启动/工具包：仅安装资源，无编译目标。
install(DIRECTORY launch config tools
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # 离线 PCD→2D 工具的纯逻辑单测（build 机 colcon test 也能 gate）
  ament_add_pytest_test(test_pcd_to_occupancy test/test_pcd_to_occupancy.py)
  ament_add_pytest_test(test_nav2_navigation_params test/test_nav2_navigation_params.py)
endif()

ament_package()
