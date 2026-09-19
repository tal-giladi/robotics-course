cmake_minimum_required(VERSION 3.8)
project(karmel_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Renders the xacro for sim and real and checks the copy of the robot config is in sync.
  ament_add_pytest_test(test_description test/test_description.py
    TIMEOUT 120)
endif()

ament_package()
