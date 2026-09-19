# Lesson 03.09 — the smallest useful CMakeLists, so you can read the ones in labs/ros2_ws.
#
#   cmake -S . -B build && cmake --build build && ./build/karmel_telemetry
#
# What each line means (FC.04 goes deeper):
#   cmake_minimum_required  the policy set CMake should behave like — not "the version you have"
#   project()               names the build and enables the languages, which finds the compilers
#   add_executable()        one target = one thing to build; sources listed explicitly, never globbed
#   target_compile_*        properties attached to the TARGET, not to the whole directory. This is
#                           "modern CMake": everything is a property of a target, and targets
#                           propagate their PUBLIC properties to whatever links them.
#
# A ROS 2 package's CMakeLists (see labs/ros2_ws/src/karmel_hardware/CMakeLists.txt) is this plus
# find_package() for its dependencies, ament_target_dependencies()/target_link_libraries() to wire
# them in, pluginlib_export_plugin_description_file() to register the plugin, and install() rules.

cmake_minimum_required(VERSION 3.22)  # the version Ubuntu 24.04 ships
project(karmel_telemetry_demo CXX)

add_executable(karmel_telemetry telemetry.cpp)

target_compile_features(karmel_telemetry PRIVATE cxx_std_20)
target_compile_options(karmel_telemetry PRIVATE -Wall -Wextra -Wpedantic)

# Robotics builds are almost always -O2 with debug info: you need the speed AND a usable stack
# trace when a node segfaults on the robot at 2 a.m.
set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "build type")
