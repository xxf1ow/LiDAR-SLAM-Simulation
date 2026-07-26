cmake_minimum_required(VERSION 3.16)
project(robot_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    robot_controllers_config
    test/test_robot_controllers.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(
    real_chassis_launch
    test/test_real_chassis_launch.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
  ament_add_pytest_test(
    fake_8030d_node
    test/test_fake_8030d_node.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )

  find_package(ament_cmake_ros REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  set(RUNNER "${ament_cmake_ros_DIR}/run_test_isolated.py")
  add_launch_test(
    test/test_real_hardware_chain.launch.py
    RUNNER "${RUNNER}"
    TIMEOUT 90
  )
endif()

ament_package()
