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

inline double raw_to_rad(double raw) { return raw * SERVO_ANGLE_RANGE_RAD / SERVO_RAW_MAX; }
inline double rad_to_raw(double rad) { return rad * SERVO_RAW_MAX / SERVO_ANGLE_RANGE_RAD; }

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
  };
  std::vector<ServoConfig> arm_servos_;

  // Cached previous arm commands for change detection
  std::vector<double> prev_arm_cmd_raw_;

  // GPIO state tracking for change detection
  double prev_led_on_time_ = 0.0;
  double prev_buzzer_freq_ = 0.0;

  // State and command storage
  std::unordered_map<std::string, double> hw_states_;
  std::unordered_map<std::string, double> hw_commands_;


  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;


};


}  // namespace robot_board_hardware

#endif  // ROBOT_BOARD_HARDWARE__ROBOT_BOARD_SYSTEM_INTERFACE_HPP_
