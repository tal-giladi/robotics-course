cmake_minimum_required(VERSION 3.8)
project(karmel_tutorial_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generates C, C++ and Python code (plus type support for the RMW) for every interface.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BatteryHealth.msg"
  "srv/SetLowThreshold.srv"
  "action/DriveDistance.action"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
