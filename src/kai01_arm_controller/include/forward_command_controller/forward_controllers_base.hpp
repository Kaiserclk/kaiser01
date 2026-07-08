
#ifndef FORWARD_COMMAND_CONTROLLER__FORWARD_CONTROLLERS_BASE_HPP_
#define FORWARD_COMMAND_CONTROLLER__FORWARD_CONTROLLERS_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "forward_command_controller/visibility_control.h"
#include "forward_command_controller/msg/joint_command.hpp"
#include "forward_command_controller/msg/joint_group_command.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace forward_command_controller
{
using CmdType = std_msgs::msg::Float64MultiArray;
using SingleJointCmdType = forward_command_controller::msg::JointCommand;
using JointGroupCmdType = forward_command_controller::msg::JointGroupCommand;

/**
 * \brief Forward command controller for a set of joints and interfaces.
 *
 * This class forwards the command signal down to a set of joints or interfaces.
 *
 * Subscribes to:
 * - \b commands (std_msgs::msg::Float64MultiArray) : The commands to apply.
 */
class ForwardControllersBase : public controller_interface::ControllerInterface
{
public:
  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ForwardControllersBase();

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  ~ForwardControllersBase() = default;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  FORWARD_COMMAND_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /**
   * Process multi-joint position command (Float64MultiArray).
   * Sets position values on command interfaces. Duration interfaces are left unchanged.
   */
  controller_interface::return_type process_multi_joint_command();

  /**
   * Process single-joint command with duration (JointCommand).
   * Sets both position and duration on the specified joint's command interfaces.
   */
  controller_interface::return_type process_single_joint_command();

  /**
   * Derived controllers have to declare parameters in this method.
   * Error handling does not have to be done. It is done in `on_init`-method of this class.
   */
  virtual void declare_parameters() = 0;

  /**
   * Derived controllers have to read parameters in this method and set `command_interface_types_`
   * variable. The variable is then used to propagate the command interface configuration to
   * controller manager. The method is called from `on_configure`-method of this class.
   *
   * It is expected that error handling of exceptions is done.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are successfully read and
   * their values are allowed, controller_interface::CallbackReturn::ERROR otherwise.
   */
  virtual controller_interface::CallbackReturn read_parameters() = 0;

  std::vector<std::string> joint_names_;
  std::string interface_name_;

  std::vector<std::string> command_interface_types_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

  realtime_tools::RealtimeBuffer<std::shared_ptr<SingleJointCmdType>> single_joint_rt_command_ptr_;
  rclcpp::Subscription<SingleJointCmdType>::SharedPtr single_joint_command_subscriber_;
};

}  // namespace forward_command_controller

#endif  // FORWARD_COMMAND_CONTROLLER__FORWARD_CONTROLLERS_BASE_HPP_
