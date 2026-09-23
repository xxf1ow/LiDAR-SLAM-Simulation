cmake_minimum_required(VERSION 3.16)
project(lidar_target_tracking)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

add_library(reference_tracking INTERFACE)
target_compile_features(reference_tracking INTERFACE cxx_std_17)
target_include_directories(reference_tracking INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/reference>
  $<INSTALL_INTERFACE:include/reference>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(reference_tracking INTERFACE ${OpenCV_LIBS})

install(DIRECTORY include/reference/ DESTINATION include/reference)
install(TARGETS reference_tracking EXPORT export_reference_tracking)
ament_export_targets(export_reference_tracking HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_reference_tracking test/test_reference_tracking.cpp)
  target_link_libraries(test_reference_tracking reference_tracking)
  ament_target_dependencies(test_reference_tracking rclcpp sensor_msgs geometry_msgs)
endif()

ament_package()
