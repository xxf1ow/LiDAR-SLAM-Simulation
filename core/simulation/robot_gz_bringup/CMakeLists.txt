cmake_minimum_required(VERSION 3.16)
project(robot_gz_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    robot_gz_launch
    test/test_robot_gz_launch.py
  )
endif()

ament_package()
