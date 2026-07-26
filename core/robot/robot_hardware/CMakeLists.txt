cmake_minimum_required(VERSION 3.16)
project(robot_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(robot_hardware SHARED
  src/diff_drive_system.cpp
  src/drive_conversion.cpp
)
target_compile_features(robot_hardware PUBLIC cxx_std_17)
target_include_directories(robot_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/robot_hardware>
)
ament_target_dependencies(robot_hardware PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)

install(DIRECTORY include/ DESTINATION include/robot_hardware)
install(TARGETS robot_hardware
  EXPORT export_robot_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_robot_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_drive_conversion test/test_drive_conversion.cpp)
  target_link_libraries(test_drive_conversion robot_hardware)

  ament_add_gtest(test_diff_drive_system test/test_diff_drive_system.cpp)
  target_link_libraries(test_diff_drive_system robot_hardware)
  ament_target_dependencies(test_diff_drive_system
    hardware_interface
    rclcpp
    rclcpp_lifecycle
    std_msgs
  )

  ament_add_gtest(test_plugin_load test/test_plugin_load.cpp)
  ament_target_dependencies(test_plugin_load hardware_interface pluginlib)
endif()

ament_package()
