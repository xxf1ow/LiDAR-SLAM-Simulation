cmake_minimum_required(VERSION 3.16)
project(robot_gz_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

# 仿真期手动开车用的 sticky 键盘 teleop(ros2 run robot_gz_bringup sticky_teleop.py)
install(
  PROGRAMS scripts/sticky_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
