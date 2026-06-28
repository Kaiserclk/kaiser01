#ifndef ROBOT_BOARD_HARDWARE__ROBOT_BOARD_SYSTEM_INTERFACE_HPP_
#define ROBOT_BOARD_HARDWARE__ROBOT_BOARD_SYSTEM_INTERFACE_HPP_

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "robot_board_hardware/serial_port.hpp"
#include "robot_board_hardware/visibility_control.h"

namespace robot_board_hardware
{

// Unit conversion constants
constexpr double SERVO_RAW_MAX = 1000.0;
constexpr double SERVO_ANGLE_RANGE_DEG = 240.0;
constexpr double SERVO_ANGLE_RANGE_RAD = SERVO_ANGLE_RANGE_DEG * M_PI / 180.0;
constexpr double GRAVITY = 9.80665;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr double TWO_PI = 2.0 * M_PI;

// Radians per encoder tick: 240 degrees / 1000 ticks
constexpr double RADIANS_PER_TICK = SERVO_ANGLE_RANGE_RAD / SERVO_RAW_MAX;
constexpr double TICKS_PER_RADIAN = SERVO_RAW_MAX / SERVO_ANGLE_RANGE_RAD;

class RobotBoardSystemInterface : public hardware_interface::SystemInterface
{
public:
  ROBOT_BOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;

  ROBOT_BOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_BOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_BOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_BOARD_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  ROBOT_BOARD_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROBOT_BOARD_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROBOT_BOARD_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ROBOT_BOARD_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Get the logger of the SystemInterface
  rclcpp::Logger get_logger() const { return *logger_; }
  // Get the clock of the SystemInterface
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  // Serial communication
  std::unique_ptr<SerialPort> serial_port_;
  std::string serial_device_;
  int baud_rate_ = 1000000;

  // Joint configuration: wheel motor names and IDs
  struct WheelConfig
  {
    std::string name;       // 轮子名称
    uint8_t motor_id = 0;  // 0-based ID for protocol
    bool negate = false;    // true for rear wheels (mounted reversed)
  };
  std::vector<WheelConfig> wheels_;

  // 机械臂关节舵机
  struct ServoConfig
  {
    std::string name;       // 关节
    uint8_t servo_id = 0;  // 舵机ID
    
    // Calibration parameters (from URDF)
    int zero_pulse = 500;   // Pulse width at 0 radians (zero position)
    int min_pulse = 0;      // Minimum pulse width
    int max_pulse = 1000;   // Maximum pulse width
    bool flipped = false;   // True if min_pulse > max_pulse (reversed direction)
    uint16_t servo_duration = 500; // Movement duration in ms (0=fastest)

    // Initial joint angle at startup (radians)
    double init_rad = 0.0;

    // Computed joint limits in radians
    double min_rad = 0.0;
    double max_rad = 0.0;

    // Convert radians to pulse width
    uint16_t rad_to_pulse(double rad) const
    {
      double raw = rad * TICKS_PER_RADIAN;
      double pulse = flipped ? (zero_pulse - raw) : (zero_pulse + raw);
      pulse = std::clamp(pulse, static_cast<double>(min_pulse), static_cast<double>(max_pulse));
      return static_cast<uint16_t>(std::round(pulse));
    }

    // Convert pulse width to radians
    double pulse_to_rad(uint16_t pulse) const
    {
      double rad = flipped
        ? (zero_pulse - static_cast<double>(pulse)) * RADIANS_PER_TICK
        : (static_cast<double>(pulse) - zero_pulse) * RADIANS_PER_TICK;
      return rad;
    }
  };
  std::vector<ServoConfig> arm_servos_;

  // Cached previous arm commands for change detection
  std::vector<double> prev_arm_cmd_raw_;

  // Cached previous wheel motor commands for change detection (in RPS)
  std::vector<float> prev_motor_speeds_;
  bool prev_motor_speeds_initialized_ = false;

  // GPIO state tracking for change detection
  double prev_led_on_time_ = 0.0;
  double prev_buzzer_freq_ = 0.0;

  // State and command storage
  std::unordered_map<std::string, double> hw_states_;
  std::unordered_map<std::string, double> hw_commands_;

  // ROS service for servo torque control
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr servo_torque_service_;
  
  // Custom node for ROS communication
  rclcpp::Node::SharedPtr custom_node_;
  std::thread node_spin_thread_;  // Thread to spin the node
  
  // Torque enabled state tracking
  std::atomic<bool> servo_torque_enabled_{true};
  
  // Service callback
  void servo_torque_callback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;


};


}  // namespace robot_board_hardware

#endif  // ROBOT_BOARD_HARDWARE__ROBOT_BOARD_SYSTEM_INTERFACE_HPP_
