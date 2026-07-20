cmake_minimum_required(VERSION 3.16)
project(robot_board_hardware LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

generate_parameter_library(robot_board_hardware_parameters
  src/robot_board_hardware_parameters.yaml
)

add_library(${PROJECT_NAME} SHARED
  src/packet_protocol.cpp
  src/serial_port.cpp
  src/robot_board_system_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
)

target_link_libraries(${PROJECT_NAME} PUBLIC robot_board_hardware_parameters)

pluginlib_export_plugin_description_file(hardware_interface robot_board_hardware_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} robot_board_hardware_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_srvs
)

ament_package()
