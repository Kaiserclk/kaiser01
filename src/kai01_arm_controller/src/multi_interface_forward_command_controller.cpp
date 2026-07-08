#include "forward_command_controller/multi_interface_forward_command_controller.hpp"

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
MultiInterfaceForwardCommandController::MultiInterfaceForwardCommandController()
: ForwardControllersBase()
{
}

void MultiInterfaceForwardCommandController::declare_parameters()
{
  param_listener_ =
    std::make_shared<multi_interface_forward_command_controller::ParamListener>(get_node());
}

controller_interface::CallbackReturn MultiInterfaceForwardCommandController::read_parameters()
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

  if (params_.interface_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_names' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Store joint names for index lookups
  joint_names_ = params_.joints;

  command_interface_types_.clear();
  // Register command interfaces: for each joint, register each interface name
  for (const auto & joint : params_.joints)
  {
    for (const auto & iface : params_.interface_names)
    {
      command_interface_types_.push_back(joint + "/" + iface);
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MultiInterfaceForwardCommandController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Call base class on_init-style setup first (read_parameters already called by base on_configure)
  auto ret = ForwardControllersBase::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // Subscribe to multi-joint command topic
  group_command_subscriber_ = get_node()->create_subscription<JointGroupCmdType>(
    "~/multi_joint_command", rclcpp::SystemDefaultsQoS(),
    [this](const JointGroupCmdType::SharedPtr msg) {
      rt_group_command_ptr_.writeFromNonRT(msg);
    });

  RCLCPP_INFO(get_node()->get_logger(),
    "MultiInterfaceForwardCommandController: subscribed to ~/multi_joint_command (JointGroupCommand) "
    "for %zu joints with %zu interfaces each",
    params_.joints.size(), params_.interface_names.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type MultiInterfaceForwardCommandController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  auto group_cmd = rt_group_command_ptr_.readFromRT();

  // no command received yet
  if (!group_cmd || !(*group_cmd))
  {
    return controller_interface::return_type::OK;
  }

  const auto & commands = (*group_cmd)->commands;
  size_t num_joints = joint_names_.size();
  size_t num_interfaces = params_.interface_names.size();

  for (const auto & cmd : commands)
  {
    uint8_t joint_id = cmd.joint_id;

    // joint_id is 1-based; validate range
    if (joint_id < 1 || joint_id > static_cast<uint8_t>(num_joints))
    {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *(get_node()->get_clock()), 1000,
        "Multi-joint command: invalid joint_id %d (valid: 1-%zu), skipping",
        joint_id, num_joints);
      continue;
    }

    // Clamp duration to valid range [0, 2000] ms
    uint16_t duration_ms = cmd.duration_ms;
    if (duration_ms > 2000)
    {
      duration_ms = 2000;
    }

    // Calculate base index for this joint in the command_interfaces_ vector
    // Layout: [joint0/iface0, joint0/iface1, ..., joint1/iface0, joint1/iface1, ...]
    size_t base_index = static_cast<size_t>(joint_id - 1) * num_interfaces;

    // Find position and duration interface indices
    for (size_t i = 0; i < num_interfaces; ++i)
    {
      size_t idx = base_index + i;
      if (idx >= command_interfaces_.size())
      {
        break;
      }

      const auto & iface_name = params_.interface_names[i];
      if (iface_name == "position")
      {
        command_interfaces_[idx].set_value(cmd.position_rad);
      }
      else if (iface_name == "duration")
      {
        command_interfaces_[idx].set_value(static_cast<double>(duration_ms));
      }
      // Other interface types (if any) are left unchanged
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace forward_command_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  forward_command_controller::MultiInterfaceForwardCommandController,
  controller_interface::ControllerInterface)
