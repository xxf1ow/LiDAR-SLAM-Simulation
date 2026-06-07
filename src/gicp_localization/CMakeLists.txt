cmake_minimum_required(VERSION 3.8)
project(gicp_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

option(GICP_DIAGNOSTICS "Build GICP diagnostics msg + publisher" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)
find_package(small_gicp REQUIRED)
find_package(OpenMP)

add_library(gicp_localization_core
  src/pose_math.cpp
  src/prior_map.cpp
  src/gicp_aligner.cpp
)
target_include_directories(gicp_localization_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(gicp_localization_core
  Eigen3::Eigen ${PCL_LIBRARIES} small_gicp::small_gicp)
ament_target_dependencies(gicp_localization_core pcl_conversions)
if(OpenMP_CXX_FOUND)
  target_link_libraries(gicp_localization_core OpenMP::OpenMP_CXX)
endif()

# 诊断实现：构建时二选一（主逻辑零开关）
if(GICP_DIAGNOSTICS)
  find_package(rosidl_default_generators REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/GicpDiagnostics.msg"
    DEPENDENCIES builtin_interfaces
  )
  add_library(gicp_localization_diag src/diagnostics_real.cpp)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(gicp_localization_diag ${cpp_typesupport_target})
  target_compile_definitions(gicp_localization_diag PUBLIC GICP_DIAGNOSTICS)
else()
  add_library(gicp_localization_diag src/diagnostics_null.cpp)
endif()
target_include_directories(gicp_localization_diag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(gicp_localization_diag rclcpp)

add_executable(gicp_localization_node
  src/gicp_localization_node.cpp
  src/main.cpp
)
target_include_directories(gicp_localization_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(gicp_localization_node
  gicp_localization_core
  gicp_localization_diag
)
ament_target_dependencies(gicp_localization_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros tf2_eigen pcl_conversions)

install(TARGETS gicp_localization_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pose_math test/test_pose_math.cpp)
  target_link_libraries(test_pose_math gicp_localization_core)
  ament_add_gtest(test_prior_map test/test_prior_map.cpp)
  target_link_libraries(test_prior_map gicp_localization_core)
  ament_add_gtest(test_gicp_aligner test/test_gicp_aligner.cpp)
  target_link_libraries(test_gicp_aligner gicp_localization_core)
endif()

ament_package()
