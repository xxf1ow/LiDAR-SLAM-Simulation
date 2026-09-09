cmake_minimum_required(VERSION 3.16)
project(robot_web_ui)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(tl-expected REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ZLIB REQUIRED)

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

if(BUILD_TESTING)
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
endif()

ament_package()
