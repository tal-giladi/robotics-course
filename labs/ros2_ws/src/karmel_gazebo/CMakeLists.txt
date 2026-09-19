cmake_minimum_required(VERSION 3.8)
project(karmel_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY worlds DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.sdf")

ament_package()
