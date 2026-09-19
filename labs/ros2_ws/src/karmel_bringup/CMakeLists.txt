cmake_minimum_required(VERSION 3.8)
project(karmel_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # controllers.yaml and nav2_params.yaml agree with karmel.yaml; launch files import.
  ament_add_pytest_test(test_config test/test_config.py TIMEOUT 60)
endif()

ament_package()
