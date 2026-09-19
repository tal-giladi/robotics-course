cmake_minimum_required(VERSION 3.16)
project(karmel_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# The plugin library: KarmelSystem + the protocol and serial helpers it uses.
add_library(${PROJECT_NAME} SHARED
  src/karmel_system.cpp
  src/protocol.cpp
  src/serial_port.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Tell pluginlib that this library provides a hardware_interface plugin (see karmel_hardware.xml).
pluginlib_export_plugin_description_file(hardware_interface karmel_hardware.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Protocol encode/decode, checked against lines produced by the Python implementation.
  ament_add_gtest(test_protocol test/test_protocol.cpp src/protocol.cpp)
  target_include_directories(test_protocol PRIVATE include)

  # The plugin can be found and instantiated through pluginlib (what the controller_manager does).
  ament_add_gtest(test_load_plugin test/test_load_plugin.cpp)
  target_link_libraries(test_load_plugin ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
