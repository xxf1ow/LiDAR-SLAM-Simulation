cmake_minimum_required(VERSION 3.16)
project(robot_web_ui)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(tl-expected REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(CPPHTTPLIB REQUIRED IMPORTED_TARGET cpp-httplib)

add_library(robot_web_ui_core
  src/manual_command.cpp
  src/map_snapshot.cpp
  src/navigation_request.cpp
  src/navigation_tracker.cpp
  src/parking_point_store.cpp
)
target_compile_features(robot_web_ui_core PUBLIC cxx_std_17)
target_include_directories(robot_web_ui_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(robot_web_ui_core PUBLIC
  nlohmann_json::nlohmann_json
  tl::expected
)
target_link_libraries(robot_web_ui_core PRIVATE
  yaml-cpp
  OpenSSL::Crypto
  ZLIB::ZLIB
)

add_library(robot_web_ui_ros src/web_ui_node.cpp)
target_link_libraries(robot_web_ui_ros robot_web_ui_core)
ament_target_dependencies(robot_web_ui_ros rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs std_msgs std_srvs tf2_msgs)

add_library(robot_web_ui_http src/http_server.cpp)
target_link_libraries(robot_web_ui_http PUBLIC robot_web_ui_core)
target_link_libraries(robot_web_ui_http PRIVATE PkgConfig::CPPHTTPLIB OpenSSL::Crypto)

add_executable(robot_web_ui src/main.cpp)
target_link_libraries(robot_web_ui robot_web_ui_ros robot_web_ui_http)
ament_target_dependencies(robot_web_ui rclcpp ament_index_cpp)
install(TARGETS robot_web_ui DESTINATION lib/${PROJECT_NAME})
install(FILES robot_web_ui/web/index.html robot_web_ui/web/map_view.js DESTINATION share/${PROJECT_NAME}/web)

if(BUILD_TESTING)
  ament_add_gtest(test_http_server test/test_http_server.cpp)
  target_link_libraries(test_http_server robot_web_ui_http PkgConfig::CPPHTTPLIB)

  ament_add_gtest(test_manual_command test/test_manual_command.cpp)
  target_link_libraries(test_manual_command robot_web_ui_core)

  ament_add_gtest(test_navigation_request test/test_navigation_request.cpp)
  target_link_libraries(test_navigation_request robot_web_ui_core)

  ament_add_gtest(test_map_snapshot test/test_map_snapshot.cpp)
  target_link_libraries(test_map_snapshot robot_web_ui_core ZLIB::ZLIB)

  ament_add_gtest(test_parking_point_store test/test_parking_point_store.cpp)
  target_link_libraries(test_parking_point_store robot_web_ui_core)

  ament_add_gtest(test_navigation_tracker test/test_navigation_tracker.cpp)
  target_link_libraries(test_navigation_tracker robot_web_ui_core)

  ament_add_gtest(test_web_ui_node test/test_web_ui_node.cpp)
  target_link_libraries(test_web_ui_node robot_web_ui_ros)
  target_link_options(test_web_ui_node PRIVATE "LINKER:--wrap=rcl_publish")
  ament_target_dependencies(test_web_ui_node rclcpp rclcpp_action geometry_msgs nav_msgs nav2_msgs std_msgs std_srvs tf2_msgs)
endif()

ament_package()
