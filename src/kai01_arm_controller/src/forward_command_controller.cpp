
#include "forward_command_controller/forward_command_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace forward_command_controller
{
ForwardCommandController::ForwardCommandController() : ForwardControllersBase() {}

void ForwardCommandController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn ForwardCommandController::read_parameters()
{
  if (!param_listener_)
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }
  params_ = param_listener_->get_params();

  if (params_.joints.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Store joint names for index lookups
  joint_names_ = params_.joints;

  command_interface_types_.clear();
  // Register position command interfaces
  for (const auto & joint : params_.joints)
  {
    command_interface_types_.push_back(joint + "/" + params_.interface_name);
  }
  // Register duration command interfaces (for per-servo execution time control)
  for (const auto & joint : params_.joints)
  {
    command_interface_types_.push_back(joint + "/duration");
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ForwardCommandController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Call base class on_configure first (creates Float64MultiArray subscriber on ~/commands)
  auto ret = ForwardControllersBase::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Create single-joint command subscriber on ~/single_joint_command
  single_joint_command_subscriber_ = get_node()->create_subscription<SingleJointCmdType>(
    "~/single_joint_command", rclcpp::SystemDefaultsQoS(),
    [this](const SingleJointCmdType::SharedPtr msg) {
      single_joint_rt_command_ptr_.writeFromNonRT(msg);
    });

  RCLCPP_INFO(get_node()->get_logger(),
    "ForwardCommandController: subscribed to ~/commands (Float64MultiArray) "
    "and ~/single_joint_command (JointCommand)");

  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::ForwardCommandController, controller_interface::ControllerInterface)
