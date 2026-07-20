#include "robot_arm_controller/robot_arm_controller.hpp"
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
namespace robot_arm_controller
{
  RobotArmController::RobotArmController() : controller_interface::ControllerInterface(),
                                             rt_group_command_ptr_(nullptr), group_command_subscriber_(nullptr)
  {
  }

  void RobotArmController::declare_parameters()
  {
    param_listener_ = std::make_shared<robot_arm_controller::ParamListener>(get_node());
  }

  controller_interface::CallbackReturn RobotArmController::read_parameters()
  {
    if (!param_listener_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during read_parameters init");
      return controller_interface::CallbackReturn::ERROR;
    }
    params_ = param_listener_->get_params();

    if (params_.joints.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty during read_parameters");
      return controller_interface::CallbackReturn::ERROR;
    }

    if (params_.interface_names.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'interface_names' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    joint_names_ = params_.joints;

    command_interface_types_.clear();
    // Register command interfaces: for each joint, register each interface name
    for (const auto &joint : params_.joints)
    {
      for (const auto &iface : params_.interface_names)
      {
        command_interface_types_.push_back(joint + "/" + iface);
      }
    }

    // Register state interfaces: position only, one per joint
    state_interface_types_.clear();
    for (const auto &joint : params_.joints)
    {
      state_interface_types_.push_back(joint + "/position");
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn RobotArmController::on_init()
  {
    try
    {
      declare_parameters();
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn RobotArmController::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto ret = this->read_parameters();
    if (ret != controller_interface::CallbackReturn::SUCCESS)
    {
      return ret;
    }

    // Subscribe to multi-joint command topic (absolute name)
    group_command_subscriber_ = get_node()->create_subscription<JointGroupCmdType>(
        "/joints_command", rclcpp::SystemDefaultsQoS(),
        [this](const JointGroupCmdType::SharedPtr msg)
        {
          rt_group_command_ptr_.writeFromNonRT(msg);
        });

    RCLCPP_INFO(get_node()->get_logger(),
                "RobotArmController: subscribed to /joints_command (JointGroupCommand) "
                "for %zu joints with %zu interfaces each",
                params_.joints.size(), params_.interface_names.size());

    // Initialize current positions from state (will be updated in update())
    current_positions_.resize(joint_names_.size(), 0.0);

    // Create service: set joint angles
    set_joint_angles_service_ =
        get_node()->create_service<robot_arm_controller::srv::SetJointAngles>(
            "~/set_joint_angles",
            [this](
                const std::shared_ptr<robot_arm_controller::srv::SetJointAngles::Request> request,
                std::shared_ptr<robot_arm_controller::srv::SetJointAngles::Response> response)
            { set_joint_angles_callback(request, response); });

    // Create service: get joint angles
    get_joint_angles_service_ =
        get_node()->create_service<robot_arm_controller::srv::GetJointAngles>(
            "~/get_joint_angles",
            [this](
                const std::shared_ptr<robot_arm_controller::srv::GetJointAngles::Request> request,
                std::shared_ptr<robot_arm_controller::srv::GetJointAngles::Response> response)
            { get_joint_angles_callback(request, response); });

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  RobotArmController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names = command_interface_types_;
    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration RobotArmController::state_interface_configuration()
      const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = state_interface_types_;
    return state_interfaces_config;
  }

  controller_interface::CallbackReturn RobotArmController::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> ordered_interfaces;
    if (
        !controller_interface::get_ordered_interfaces(
            command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
        command_interface_types_.size() != ordered_interfaces.size())
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
          command_interface_types_.size(), ordered_interfaces.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    // reset command buffer if a command came through callback when controller was inactive
    rt_group_command_ptr_ =
        realtime_tools::RealtimeBuffer<std::shared_ptr<JointGroupCmdType>>(nullptr);

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn RobotArmController::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // Arm home/rest pose management now lives in robot_board_hardware.
    // reset command buffer
    rt_group_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<JointGroupCmdType>>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type RobotArmController::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // Read current joint positions from hardware state interfaces
    if (!state_interfaces_.empty())
    {
      for (size_t j = 0; j < joint_names_.size() && j < state_interfaces_.size(); ++j)
      {
        current_positions_[j] = state_interfaces_[j].get_value();
      }
    }

    auto group_cmd = rt_group_command_ptr_.readFromRT();

    // no command received yet
    if (!group_cmd || !(*group_cmd))
    {
      return controller_interface::return_type::OK;
    }

    const auto &commands = (*group_cmd)->commands;
    size_t num_joints = joint_names_.size();
    size_t num_interfaces = params_.interface_names.size();

    for (const auto &cmd : commands)
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
      duration_ms = std::clamp(duration_ms, static_cast<uint16_t>(0), static_cast<uint16_t>(2000));

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

        const auto &iface_name = params_.interface_names[i];
        if (iface_name == "position")
        {
          command_interfaces_[idx].set_value(cmd.position_rad);
        }
        else if (iface_name == "duration")
        {
          command_interfaces_[idx].set_value(static_cast<double>(duration_ms));
        }
      }
    }

    return controller_interface::return_type::OK;
  }

  void RobotArmController::set_joint_angles_callback(
      const std::shared_ptr<robot_arm_controller::srv::SetJointAngles::Request> request,
      std::shared_ptr<robot_arm_controller::srv::SetJointAngles::Response> response)
  {
    size_t num_joints = joint_names_.size();

    if (request->angles.size() != num_joints)
    {
      response->success = false;
      response->message =
          "Expected " + std::to_string(num_joints) + " angles, got " +
          std::to_string(request->angles.size());
      RCLCPP_WARN(get_node()->get_logger(), "%s", response->message.c_str());
      return;
    }

    size_t num_interfaces = params_.interface_names.size();
    for (size_t j = 0; j < num_joints; ++j)
    {
      size_t base_index = j * num_interfaces;
      for (size_t i = 0; i < num_interfaces; ++i)
      {
        size_t idx = base_index + i;
        if (idx >= command_interfaces_.size())
        {
          break;
        }
        const auto &iface_name = params_.interface_names[i];
        if (iface_name == "position")
        {
          command_interfaces_[idx].set_value(request->angles[j]);
        }
        else if (iface_name == "duration")
        {
          command_interfaces_[idx].set_value(1500); // default 1000ms
        }
      }
    }
    // Track positions
    response->success = true;
    response->message = "Joint angles set successfully";
    RCLCPP_INFO(get_node()->get_logger(), "Service set_joint_angles: %zu joints written", num_joints);
  }

  void RobotArmController::get_joint_angles_callback(
      const std::shared_ptr<robot_arm_controller::srv::GetJointAngles::Request> /*request*/,
      std::shared_ptr<robot_arm_controller::srv::GetJointAngles::Response> response)
  {
    response->angles = current_positions_;
  }

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_arm_controller::RobotArmController, controller_interface::ControllerInterface)
