#ifndef ROBOT_ARM_CONTROLLER__ROBOT_ARM_CONTROLLER_HPP_
#define ROBOT_ARM_CONTROLLER__ROBOT_ARM_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "robot_arm_controller/visibility_control.h"
#include "robot_arm_controller/msg/joint_command.hpp"
#include "robot_arm_controller/msg/joint_group_command.hpp"
#include "robot_arm_controller/srv/set_joint_angles.hpp"
#include "robot_arm_controller/srv/get_joint_angles.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "robot_arm_controller/robot_arm_controller_parameters.hpp"

namespace robot_arm_controller
{
using JointGroupCmdType = robot_arm_controller::msg::JointGroupCommand;

/**
 * \brief Robot arm controller with per-joint position and duration control.
 *
 * This controller subscribes to JointGroupCommand messages on /joints_command
 * and forwards both position and duration commands to the hardware interfaces
 * for each joint. Each joint receives its own target position (radians) and
 * execution duration (0-2000ms).
 *
 * \param joints Names of the joints to control.
 * \param interface_names Names of the interfaces to command (default: ["position", "duration"]).
 *
 * Subscribes to:
 * - \b /joints_command (robot_arm_controller::msg::JointGroupCommand) :
 *   Array of joint commands, each with joint_id, position_rad, and duration_ms.
 */
class RobotArmController : public controller_interface::ControllerInterface
{
public:
  ROBOT_ARM_CONTROLLER_PUBLIC
  RobotArmController();

  ROBOT_ARM_CONTROLLER_PUBLIC
  ~RobotArmController() = default;

  ROBOT_ARM_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ROBOT_ARM_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ROBOT_ARM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ROBOT_ARM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_ARM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_ARM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_ARM_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  void declare_parameters();
  controller_interface::CallbackReturn read_parameters();

  /// Write a joint pose array to the position+duration command interfaces
  void write_pose_to_hardware(const std::vector<double> & pose, const char * pose_name);

  /// Service callback: set target joint angles
  void set_joint_angles_callback(
    const std::shared_ptr<robot_arm_controller::srv::SetJointAngles::Request> request,
    std::shared_ptr<robot_arm_controller::srv::SetJointAngles::Response> response);

  /// Service callback: get current joint angles
  void get_joint_angles_callback(
    const std::shared_ptr<robot_arm_controller::srv::GetJointAngles::Request> request,
    std::shared_ptr<robot_arm_controller::srv::GetJointAngles::Response> response);

  std::vector<std::string> joint_names_;
  std::vector<std::string> command_interface_types_;
  std::vector<std::string> state_interface_types_;

  std::shared_ptr<robot_arm_controller::ParamListener> param_listener_;
  robot_arm_controller::Params params_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<JointGroupCmdType>> rt_group_command_ptr_;
  rclcpp::Subscription<JointGroupCmdType>::SharedPtr group_command_subscriber_;

  rclcpp::Service<robot_arm_controller::srv::SetJointAngles>::SharedPtr set_joint_angles_service_;
  rclcpp::Service<robot_arm_controller::srv::GetJointAngles>::SharedPtr get_joint_angles_service_;

  std::vector<double> current_positions_;
};

} 

#endif 