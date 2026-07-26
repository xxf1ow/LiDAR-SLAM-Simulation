cmake_minimum_required(VERSION 3.5)
project(can_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Pre-built executable
install(PROGRAMS
  bin/can_driver_8030
  DESTINATION lib/${PROJECT_NAME}
)

# Shared library
install(FILES lib/libcontrolcan.so
  DESTINATION lib/${PROJECT_NAME}
)

# Public headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Launch & config
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
