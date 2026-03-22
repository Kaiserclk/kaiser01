#include "robot_board_hardware/robot_board_system_interface.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_board_hardware
{

hardware_interface::CallbackReturn RobotBoardSystemInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hw_info = info_;

  // Read hardware parameters
  if (hw_info.hardware_parameters.count("serial_port")) {
    serial_device_ = hw_info.hardware_parameters.at("serial_port");
  } else {
    serial_device_ = "/dev/ttyACM0";
  }

  if (hw_info.hardware_parameters.count("baud_rate")) {
    baud_rate_ = std::stoi(hw_info.hardware_parameters.at("baud_rate"));
  } else {
    baud_rate_ = 1000000;
  }

  // Parse joints: identify wheels vs arm servos by their parameters
  wheels_.clear();
  arm_servos_.clear();

  for (const auto & joint : hw_info.joints) {
    if (joint.parameters.count("motor_id")) {
      // This is a wheel motor joint
      WheelConfig wc;
      wc.name = joint.name;
      uint8_t motor_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("motor_id")));
      wc.motor_id = motor_id - 1;  // convert to 0-based for protocol
      // Rear wheels (motor 3, 4) are mounted in reverse
      wc.negate = (motor_id == 3 || motor_id == 4);
      wheels_.push_back(wc);
    } else if (joint.parameters.count("servo_id")) {
      // This is an arm servo joint
      ServoConfig sc;
      sc.name = joint.name;
      sc.servo_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("servo_id")));
      arm_servos_.push_back(sc);
    }
  }

  prev_arm_cmd_raw_.resize(arm_servos_.size(), 500.0);  // default mid-position

  RCLCPP_INFO(
    rclcpp::get_logger("RobotBoardSystemInterface"),
    "RobotBoardSystemInterface initialized: %zu wheels, %zu arm servos, "
    "serial=%s @ %d baud",
    wheels_.size(), arm_servos_.size(), serial_device_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotBoardSystemInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Wheel velocity states
  for (const auto & wc : wheels_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      wc.name, hardware_interface::HW_IF_VELOCITY, &hw_states_[wc.name + "/" + hardware_interface::HW_IF_VELOCITY]));
  }

  // Arm servo position states
  for (const auto & sc : arm_servos_) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      sc.name, hardware_interface::HW_IF_POSITION, &hw_states_[sc.name + "/" + hardware_interface::HW_IF_POSITION]));
  }

  // IMU states
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.x", &hw_states_["imu_sensor/orientation.x"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.y", &hw_states_["imu_sensor/orientation.y"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.z", &hw_states_["imu_sensor/orientation.z"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "orientation.w", &hw_states_["imu_sensor/orientation.w"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.x", &hw_states_["imu_sensor/angular_velocity.x"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.y", &hw_states_["imu_sensor/angular_velocity.y"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "angular_velocity.z", &hw_states_["imu_sensor/angular_velocity.z"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.x", &hw_states_["imu_sensor/linear_acceleration.x"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.y", &hw_states_["imu_sensor/linear_acceleration.y"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "imu_sensor", "linear_acceleration.z", &hw_states_["imu_sensor/linear_acceleration.z"]));

  // Battery state
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "battery_sensor", "voltage", &hw_states_["battery_sensor/voltage"]));

  // Button states
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "button_sensor", "button1_state", &hw_states_["button_sensor/button1_state"]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "button_sensor", "button2_state", &hw_states_["button_sensor/button2_state"]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotBoardSystemInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Wheel velocity commands
  for (const auto & wc : wheels_) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      wc.name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[wc.name + "/" + hardware_interface::HW_IF_VELOCITY]));
  }

  // Arm servo position commands
  for (const auto & sc : arm_servos_) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      sc.name, hardware_interface::HW_IF_POSITION, &hw_commands_[sc.name + "/" + hardware_interface::HW_IF_POSITION]));
  }

  // LED commands
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "led", "on_time", &hw_commands_["led/on_time"]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "led", "off_time", &hw_commands_["led/off_time"]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "led", "repeat", &hw_commands_["led/repeat"]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "led", "led_id", &hw_commands_["led/led_id"]));

  // Buzzer commands
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "buzzer", "frequency", &hw_commands_["buzzer/frequency"]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "buzzer", "on_time", &hw_commands_["buzzer/on_time"]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "buzzer", "off_time", &hw_commands_["buzzer/off_time"]));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    "buzzer", "repeat", &hw_commands_["buzzer/repeat"]));

  return command_interfaces;
}

hardware_interface::CallbackReturn RobotBoardSystemInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  serial_port_ = std::make_unique<SerialPort>();

  if (!serial_port_->open(serial_device_, baud_rate_)) {
    RCLCPP_ERROR(rclcpp::get_logger("RobotBoardSystemInterface"), "Failed to open serial port: %s", serial_device_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_->start_recv_thread();

  // Stop all motors
  std::vector<std::pair<uint8_t, float>> zero_speeds;
  for (const auto & wc : wheels_) {
    zero_speeds.emplace_back(wc.motor_id, 0.0f);
  }
  auto motor_data = PacketProtocol::build_motor_speed_cmd(zero_speeds);
  serial_port_->send_packet(
    static_cast<uint8_t>(PacketFunction::MOTOR), motor_data);

  RCLCPP_INFO(rclcpp::get_logger("RobotBoardSystemInterface"), "Serial port configured and recv thread started");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotBoardSystemInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Enable torque on all arm servos
  for (const auto & sc : arm_servos_) {
    auto data = PacketProtocol::build_bus_servo_enable_torque_cmd(sc.servo_id, true);
    serial_port_->send_packet(
      static_cast<uint8_t>(PacketFunction::BUS_SERVO), data);
    // Small delay between torque enable commands (matching Python SDK's 20ms sleep)
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Initialize state interfaces with default values
  for (const auto & wc : wheels_) {
    hw_states_[wc.name + "/" + hardware_interface::HW_IF_VELOCITY] = 0.0;
  }

  for (size_t i = 0; i < arm_servos_.size(); ++i) {
    double default_rad = raw_to_rad(500.0);  // mid-position
    hw_states_[arm_servos_[i].name + "/" + hardware_interface::HW_IF_POSITION] = default_rad;
  }

  // Initialize IMU with identity orientation and zero readings
  hw_states_["imu_sensor/orientation.x"] = 0.0;
  hw_states_["imu_sensor/orientation.y"] = 0.0;
  hw_states_["imu_sensor/orientation.z"] = 0.0;
  hw_states_["imu_sensor/orientation.w"] = 1.0;
  hw_states_["imu_sensor/angular_velocity.x"] = 0.0;
  hw_states_["imu_sensor/angular_velocity.y"] = 0.0;
  hw_states_["imu_sensor/angular_velocity.z"] = 0.0;
  hw_states_["imu_sensor/linear_acceleration.x"] = 0.0;
  hw_states_["imu_sensor/linear_acceleration.y"] = 0.0;
  hw_states_["imu_sensor/linear_acceleration.z"] = 0.0;

  hw_states_["battery_sensor/voltage"] = 0.0;
  hw_states_["button_sensor/button1_state"] = 0.0;
  hw_states_["button_sensor/button2_state"] = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("RobotBoardSystemInterface"), "Hardware activated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotBoardSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Stop all motors
  std::vector<std::pair<uint8_t, float>> zero_speeds;
  for (const auto & wc : wheels_) {
    zero_speeds.emplace_back(wc.motor_id, 0.0f);
  }
  auto motor_data = PacketProtocol::build_motor_speed_cmd(zero_speeds);
  serial_port_->send_packet(
    static_cast<uint8_t>(PacketFunction::MOTOR), motor_data);

  RCLCPP_INFO(rclcpp::get_logger("RobotBoardSystemInterface"), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotBoardSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_port_) {
    serial_port_->stop_recv_thread();
    serial_port_->close();
    serial_port_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("RobotBoardSystemInterface"), "Hardware cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotBoardSystemInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_) {
    return hardware_interface::return_type::ERROR;
  }

  // Read IMU data
  auto imu_data = serial_port_->get_latest_imu();
  if (imu_data.has_value()) {
    const auto & imu = imu_data.value();
    // Convert: raw acceleration (g units) -> m/s^2, raw gyro (deg/s) -> rad/s
    hw_states_["imu_sensor/linear_acceleration.x"] = static_cast<double>(imu.ax) * GRAVITY;
    hw_states_["imu_sensor/linear_acceleration.y"] = static_cast<double>(imu.ay) * GRAVITY;
    hw_states_["imu_sensor/linear_acceleration.z"] = static_cast<double>(imu.az) * GRAVITY;
    hw_states_["imu_sensor/angular_velocity.x"] = static_cast<double>(imu.gx) * DEG_TO_RAD;
    hw_states_["imu_sensor/angular_velocity.y"] = static_cast<double>(imu.gy) * DEG_TO_RAD;
    hw_states_["imu_sensor/angular_velocity.z"] = static_cast<double>(imu.gz) * DEG_TO_RAD;
    // No orientation estimation from MCU
    hw_states_["imu_sensor/orientation.x"] = 0.0;
    hw_states_["imu_sensor/orientation.y"] = 0.0;
    hw_states_["imu_sensor/orientation.z"] = 0.0;
    hw_states_["imu_sensor/orientation.w"] = 1.0;
  }

  // Read battery data
  auto battery_data = serial_port_->get_latest_battery();
  if (battery_data.has_value()) {
    hw_states_["battery_sensor/voltage"] = static_cast<double>(battery_data.value().voltage_mv);
  }

  // Read button data
  auto button_data = serial_port_->get_latest_button();
  if (button_data.has_value()) {
    const auto & btn = button_data.value();
    if (btn.id == 1) {
      hw_states_["button_sensor/button1_state"] = static_cast<double>(btn.event);
    } else if (btn.id == 2) {
      hw_states_["button_sensor/button2_state"] = static_cast<double>(btn.event);
    }
  }

  // Wheel velocity states: echo back command values (open-loop, no encoders)
  for (const auto & wc : wheels_) {
    std::string cmd_name = wc.name + "/" + hardware_interface::HW_IF_VELOCITY;
    hw_states_[cmd_name] = hw_commands_[cmd_name];
  }

  // Arm servo position states: echo back command values
  for (const auto & sc : arm_servos_) {
    std::string cmd_name = sc.name + "/" + hardware_interface::HW_IF_POSITION;
    hw_states_[cmd_name] = hw_commands_[cmd_name];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotBoardSystemInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_) {
    return hardware_interface::return_type::ERROR;
  }

  // Write wheel motor speeds
  {
    std::vector<std::pair<uint8_t, float>> motor_speeds;
    for (const auto & wc : wheels_) {
      double cmd_vel = hw_commands_[wc.name + "/" + hardware_interface::HW_IF_VELOCITY];

      // Convert from rad/s to RPS
      double rps = cmd_vel / TWO_PI;

      // Negate for rear wheels (mounted reversed)
      if (wc.negate) {
        rps = -rps;
      }

      motor_speeds.emplace_back(wc.motor_id, static_cast<float>(rps));
    }

    auto data = PacketProtocol::build_motor_speed_cmd(motor_speeds);
    serial_port_->send_packet(
      static_cast<uint8_t>(PacketFunction::MOTOR), data);
  }

  // Write arm servo positions
  {
    std::vector<std::pair<uint8_t, uint16_t>> positions;
    bool has_changes = false;

    for (size_t i = 0; i < arm_servos_.size(); ++i) {
      double cmd_rad = hw_commands_[arm_servos_[i].name + "/" + hardware_interface::HW_IF_POSITION];

      // Convert radians to raw 0-1000
      double raw = rad_to_raw(cmd_rad);
      raw = std::clamp(raw, 0.0, SERVO_RAW_MAX);

      // Only send if changed (threshold 0.5 raw units)
      if (std::abs(raw - prev_arm_cmd_raw_[i]) > 0.5) {
        has_changes = true;
        prev_arm_cmd_raw_[i] = raw;
      }

      positions.emplace_back(arm_servos_[i].servo_id, static_cast<uint16_t>(std::round(raw)));
    }

    if (has_changes) {
      auto data = PacketProtocol::build_bus_servo_position_cmd(0, positions);  // duration=0 (immediate)
      serial_port_->send_packet(
        static_cast<uint8_t>(PacketFunction::BUS_SERVO), data);
    }
  }

  // Write GPIO: LED
  {
    double led_on_time = hw_commands_["led/on_time"];
    if (std::abs(led_on_time - prev_led_on_time_) > 0.001 && led_on_time > 0.001) {
      double led_off_time = hw_commands_["led/off_time"];
      double led_repeat = hw_commands_["led/repeat"];
      double led_id_val = hw_commands_["led/led_id"];

      auto data = PacketProtocol::build_led_cmd(
        static_cast<uint8_t>(std::max(1.0, led_id_val)),
        static_cast<uint16_t>(led_on_time * 1000),
        static_cast<uint16_t>(led_off_time * 1000),
        static_cast<uint16_t>(led_repeat));
      serial_port_->send_packet(
        static_cast<uint8_t>(PacketFunction::LED), data);
      prev_led_on_time_ = led_on_time;
    }
  }

  // Write GPIO: Buzzer
  {
    double buzzer_freq = hw_commands_["buzzer/frequency"];
    if (std::abs(buzzer_freq - prev_buzzer_freq_) > 0.001 && buzzer_freq > 0.001) {
      double buzzer_on_time = hw_commands_["buzzer/on_time"];
      double buzzer_off_time = hw_commands_["buzzer/off_time"];
      double buzzer_repeat = hw_commands_["buzzer/repeat"];

      auto data = PacketProtocol::build_buzzer_cmd(
        static_cast<uint16_t>(buzzer_freq),
        static_cast<uint16_t>(buzzer_on_time * 1000),
        static_cast<uint16_t>(buzzer_off_time * 1000),
        static_cast<uint16_t>(buzzer_repeat));
      serial_port_->send_packet(
        static_cast<uint8_t>(PacketFunction::BUZZER), data);
      prev_buzzer_freq_ = buzzer_freq;
    }
  }

  return hardware_interface::return_type::OK;
}

}  // namespace robot_board_hardware

PLUGINLIB_EXPORT_CLASS(
  robot_board_hardware::RobotBoardSystemInterface,
  hardware_interface::SystemInterface)
