cmake_minimum_required(VERSION 3.16)
project(vanjee_lidar_ros LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Threads REQUIRED)

set(VANJEE_DRIVER_INCLUDE
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/vanjee_driver/src"
)

add_library(vanjee_lidar_support STATIC
  src/driver_config.cpp
)
target_compile_features(vanjee_lidar_support PUBLIC cxx_std_17)
target_compile_definitions(vanjee_lidar_support PUBLIC DISABLE_PCAP_PARSE)
target_include_directories(vanjee_lidar_support PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${VANJEE_DRIVER_INCLUDE}"
)
target_link_libraries(vanjee_lidar_support PUBLIC Threads::Threads)
ament_target_dependencies(vanjee_lidar_support PUBLIC
  builtin_interfaces
  sensor_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_driver_config test/test_driver_config.cpp)
  target_link_libraries(test_driver_config vanjee_lidar_support)
endif()

ament_package()
