cmake_minimum_required(VERSION 3.16)
project(robot_arm_controller LANGUAGES C CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Generate ROS 2 message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "interface/msg/JointCommand.msg"
  "interface/msg/JointGroupCommand.msg"
  "interface/srv/SetJointAngles.srv"
  "interface/srv/GetJointAngles.srv"
)

generate_parameter_library(
  robot_arm_controller_parameters
  src/robot_arm_controller_parameters.yaml
)

add_library(robot_arm_controller SHARED
  src/robot_arm_controller.cpp
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_msgs "rosidl_typesupport_cpp")
target_link_libraries(robot_arm_controller PUBLIC ${cpp_typesupport_target})
add_dependencies(robot_arm_controller ${PROJECT_NAME}_msgs)
target_compile_features(robot_arm_controller PUBLIC cxx_std_17)
target_include_directories(robot_arm_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include/robot_arm_controller>
)
target_link_libraries(robot_arm_controller PUBLIC
  robot_arm_controller_parameters
)
ament_target_dependencies(robot_arm_controller PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(robot_arm_controller PRIVATE "ROBOT_ARM_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface forward_command_plugin.xml)


if(BUILD_TESTING)
  message(STATUS "Tests disabled: test source files not available")
  # Tests require implementation of test files in test/ directory
  # find_package(ament_cmake_gmock REQUIRED)
  # find_package(controller_manager REQUIRED)
  # find_package(ros2_control_test_assets REQUIRED)
  # ... test targets commented out until test sources are created
endif()

install(
  DIRECTORY include/
  DESTINATION include/robot_arm_controller
)
install(
  TARGETS
    robot_arm_controller
    robot_arm_controller_parameters
  EXPORT export_robot_arm_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)
install(
  DIRECTORY interface/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_robot_arm_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
