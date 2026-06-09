cmake_minimum_required(VERSION 3.8)
project(robot_navigation)

find_package(ament_cmake REQUIRED)

# 纯配置/启动/工具包：仅安装资源，无编译目标。
install(DIRECTORY launch config tools
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
