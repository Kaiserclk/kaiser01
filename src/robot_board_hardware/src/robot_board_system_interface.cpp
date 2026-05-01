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

  /**
   * @brief 初始化硬件接口信息和通信参数
   * @param hardware_info
   * @return hardware_interface::CallbackReturn::SUCCESS or ERROR
   */
  hardware_interface::CallbackReturn RobotBoardSystemInterface::on_init(
      const hardware_interface::HardwareInfo &hardware_info)
  {
    // Initialize base class
    if (hardware_interface::SystemInterface::on_init(hardware_info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get logger and clock
    logger_ = std::make_shared<rclcpp::Logger>(
        rclcpp::get_logger("RobotBoardSystemInterface"));
    clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());
    const auto &hw_info = info_;

    // Read hardware parameters
    if (!hw_info.hardware_parameters.count("serial_port"))
    {
      RCLCPP_ERROR(get_logger(), "Missing serial port parameter");
      return hardware_interface::CallbackReturn::ERROR;
    }
    serial_device_ = hw_info.hardware_parameters.at("serial_port");

    if (!hw_info.hardware_parameters.count("baud_rate"))
    {
      RCLCPP_ERROR(get_logger(), "Missing baud rate parameter");
      return hardware_interface::CallbackReturn::ERROR;
    }
    baud_rate_ = std::stoi(hw_info.hardware_parameters.at("baud_rate"));

    // Parse joints: identify wheels vs arm servos by their parameters
    wheels_.clear();
    arm_servos_.clear();

    for (const auto &joint : hw_info.joints)
    {
      if (joint.parameters.count("motor_id"))
      {
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
          RCLCPP_ERROR(get_logger(), "Joint %s must use velocity interface", joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }

        // Init wheel config
        WheelConfig wc;
        wc.name = joint.name;
        uint8_t motor_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("motor_id")));
        wc.motor_id = motor_id - 1;                   // convert to 0-based for protocol
        wc.negate = (motor_id == 3 || motor_id == 4); // Rear wheels (motor 3, 4) are mounted in reverse
        wheels_.push_back(wc);
      }
      else if (joint.parameters.count("servo_id"))
      {
        if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
          RCLCPP_ERROR(get_logger(), "Joint %s must use position interface", joint.name.c_str());
          return hardware_interface::CallbackReturn::ERROR;
        }

        // Init arm servo config
        ServoConfig sc;
        sc.name = joint.name;
        sc.servo_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("servo_id")));
        
        // Read calibration parameters (optional, with defaults)
        if (joint.parameters.count("zero")) {
          sc.zero_pulse = std::stoi(joint.parameters.at("zero"));
        }
        if (joint.parameters.count("init")) {
          sc.init_rad = std::stod(joint.parameters.at("init"));
        }
        if (joint.parameters.count("min")) {
          sc.min_pulse = std::stoi(joint.parameters.at("min"));
        }
        if (joint.parameters.count("max")) {
          sc.max_pulse = std::stoi(joint.parameters.at("max"));
        }
        
        // Determine if joint is flipped (min > max)
        sc.flipped = sc.min_pulse > sc.max_pulse;
        if (sc.flipped) {
          // Swap min/max for proper clamping
          std::swap(sc.min_pulse, sc.max_pulse);
        }
        
        // Read servo movement duration (ms), default 500ms
        if (joint.parameters.count("servo_duration")) {
          sc.servo_duration = static_cast<uint16_t>(std::stoi(joint.parameters.at("servo_duration")));
        }
        
        // Compute joint limits in radians
        sc.min_rad = sc.pulse_to_rad(sc.min_pulse);
        sc.max_rad = sc.pulse_to_rad(sc.max_pulse);
        
        // For flipped joints, min_rad > max_rad; swap for correct clamp behavior
        if (sc.min_rad > sc.max_rad) {
          std::swap(sc.min_rad, sc.max_rad);
        }
        
        RCLCPP_INFO(get_logger(), "Servo %s: id=%d, zero=%d, init=%.3f rad, range=[%d,%d], flipped=%s, duration=%dms, rad=[%.2f,%.2f]",
            sc.name.c_str(), sc.servo_id, sc.zero_pulse, sc.init_rad,
            sc.min_pulse, sc.max_pulse, sc.flipped ? "true" : "false",
            sc.servo_duration, sc.min_rad, sc.max_rad);
        
        arm_servos_.push_back(sc);
      }
    }

    prev_arm_cmd_raw_.resize(arm_servos_.size());
    for (size_t i = 0; i < arm_servos_.size(); ++i)
    {
      // Initialize prev to init_rad's pulse so that the first write at init_rad is NOT sent
      prev_arm_cmd_raw_[i] = static_cast<double>(arm_servos_[i].rad_to_pulse(arm_servos_[i].init_rad));
    }

    RCLCPP_INFO(
        get_logger(), "RobotBoardSystemInterface initialized: %zu wheels, %zu arm servos, "
                      "serial=%s @ %d baud",
        wheels_.size(), arm_servos_.size(), serial_device_.c_str(), baud_rate_);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> RobotBoardSystemInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    // Pre-reserve to avoid rehash during insertion.
    // std::unordered_map rehash invalidates ALL pointers/references stored in StateInterface.
    // Count: wheels + arm_servos + 10(IMU) + 1(battery) + 2(buttons) = N+13
    hw_states_.reserve((wheels_.size() + arm_servos_.size() + 13) * 2);

    // Wheel position and velocity states
    for (const auto &wc : wheels_)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          wc.name, hardware_interface::HW_IF_POSITION, &hw_states_[wc.name + "/" + hardware_interface::HW_IF_POSITION]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          wc.name, hardware_interface::HW_IF_VELOCITY, &hw_states_[wc.name + "/" + hardware_interface::HW_IF_VELOCITY]));
    }

    // Arm servo position states
    for (const auto &sc : arm_servos_)
    {
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

    // Pre-reserve to avoid rehash during insertion.
    // std::unordered_map rehash invalidates ALL pointers/references stored in CommandInterface.
    // Count: wheels + arm_servos + 4(LED) + 4(buzzer) = N+8
    hw_commands_.reserve((wheels_.size() + arm_servos_.size() + 8) * 2);

    // Wheel velocity commands
    for (const auto &wc : wheels_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          wc.name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[wc.name + "/" + hardware_interface::HW_IF_VELOCITY]));
    }

    // Arm servo position commands
    for (const auto &sc : arm_servos_)
    {
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

    if (!serial_port_->open(serial_device_, baud_rate_))
    {
      RCLCPP_FATAL(get_logger(), "Failed to open serial port: %s", serial_device_.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    serial_port_->start_recv_thread();

    // Stop all motors
    std::vector<std::pair<uint8_t, float>> zero_speeds;
    for (const auto &wc : wheels_)
    {
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
    for (const auto &sc : arm_servos_)
    {
      auto data = PacketProtocol::build_bus_servo_enable_torque_cmd(sc.servo_id, true);
      serial_port_->send_packet(
          static_cast<uint8_t>(PacketFunction::BUS_SERVO), data);
      // Small delay between torque enable commands (matching Python SDK's 20ms sleep)
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Initialize state interfaces with default values
    for (const auto &wc : wheels_)
    {
      hw_states_[wc.name + "/" + hardware_interface::HW_IF_POSITION] = 0.0;
      hw_states_[wc.name + "/" + hardware_interface::HW_IF_VELOCITY] = 0.0;
    }

    // Send init position command and sync state/command interfaces to init_rad
    {
      std::vector<std::pair<uint8_t, uint16_t>> init_positions;
      for (size_t i = 0; i < arm_servos_.size(); ++i)
      {
        uint16_t pulse = arm_servos_[i].rad_to_pulse(arm_servos_[i].init_rad);
        init_positions.emplace_back(arm_servos_[i].servo_id, pulse);

        // Sync state and command interfaces to init_rad
        hw_states_[arm_servos_[i].name + "/" + hardware_interface::HW_IF_POSITION] = arm_servos_[i].init_rad;
        hw_commands_[arm_servos_[i].name + "/" + hardware_interface::HW_IF_POSITION] = arm_servos_[i].init_rad;

        // Sync prev_arm_cmd_raw_ so first write() won't resend immediately
        prev_arm_cmd_raw_[i] = static_cast<double>(pulse);
      }
      uint16_t duration = arm_servos_.empty() ? 500 : arm_servos_[0].servo_duration;
      auto data = PacketProtocol::build_bus_servo_position_cmd(duration, init_positions);
      serial_port_->send_packet(static_cast<uint8_t>(PacketFunction::BUS_SERVO), data);
      RCLCPP_INFO(rclcpp::get_logger("RobotBoardSystemInterface"),
          "Sent init position command (duration=%dms)", duration);
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
    for (const auto &wc : wheels_)
    {
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
    if (serial_port_)
    {
      serial_port_->stop_recv_thread();
      serial_port_->close();
      serial_port_.reset();
    }

    RCLCPP_INFO(rclcpp::get_logger("RobotBoardSystemInterface"), "Hardware cleaned up");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type RobotBoardSystemInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
  {
    if (!serial_port_)
    {
      return hardware_interface::return_type::ERROR;
    }

    // Read IMU data
    auto imu_data = serial_port_->get_latest_imu();
    if (imu_data.has_value())
    {
      const auto &imu = imu_data.value();
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
    if (battery_data.has_value())
    {
      hw_states_["battery_sensor/voltage"] = static_cast<double>(battery_data.value().voltage_mv);
    }

    // Read button data
    auto button_data = serial_port_->get_latest_button();
    if (button_data.has_value())
    {
      const auto &btn = button_data.value();
      if (btn.id == 1)
      {
        hw_states_["button_sensor/button1_state"] = static_cast<double>(btn.event);
      }
      else if (btn.id == 2)
      {
        hw_states_["button_sensor/button2_state"] = static_cast<double>(btn.event);
      }
    }

    // Wheel velocity states: echo back command values (open-loop, no encoders)
    // Wheel position states: integrate velocity over period
    double dt = period.seconds();
    for (const auto &wc : wheels_)
    {
      std::string vel_key = wc.name + "/" + hardware_interface::HW_IF_VELOCITY;
      hw_states_[vel_key] = hw_commands_[vel_key];

      std::string pos_key = wc.name + "/" + hardware_interface::HW_IF_POSITION;
      hw_states_[pos_key] += hw_states_[vel_key] * dt;
    }

    // Arm servo position states: echo back command values
    for (const auto &sc : arm_servos_)
    {
      std::string cmd_name = sc.name + "/" + hardware_interface::HW_IF_POSITION;
      hw_states_[cmd_name] = hw_commands_[cmd_name];
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type RobotBoardSystemInterface::write(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    if (!serial_port_)
    {
      return hardware_interface::return_type::ERROR;
    }

    // Write wheel motor speeds
    {
      std::vector<std::pair<uint8_t, float>> motor_speeds;
      for (const auto &wc : wheels_)
      {
        double cmd_vel = hw_commands_[wc.name + "/" + hardware_interface::HW_IF_VELOCITY];

        // Convert from rad/s to RPS
        double rps = cmd_vel / TWO_PI;

        // Negate for rear wheels (mounted reversed)
        if (wc.negate)
        {
          rps = -rps;
        }

        // if (std::abs(cmd_vel) > 1e-6)
        // {
        //   has_nonzero = true;
        // }

        motor_speeds.emplace_back(wc.motor_id, static_cast<float>(rps));
      }

      // if (has_nonzero)
      // {
      //   RCLCPP_INFO(get_logger(),
      //       "Motor cmd: [id%d=%.4f, id%d=%.4f, id%d=%.4f, id%d=%.4f] (rps)",
      //       motor_speeds[0].first, motor_speeds[0].second,
      //       motor_speeds[1].first, motor_speeds[1].second,
      //       motor_speeds[2].first, motor_speeds[2].second,
      //       motor_speeds[3].first, motor_speeds[3].second);
      // }

      auto data = PacketProtocol::build_motor_speed_cmd(motor_speeds);
      serial_port_->send_packet(
          static_cast<uint8_t>(PacketFunction::MOTOR), data);
    }

    // Write arm servo positions
    {
      std::vector<std::pair<uint8_t, uint16_t>> positions;
      bool has_changes = false;

      for (size_t i = 0; i < arm_servos_.size(); ++i)
      {
        double cmd_rad = hw_commands_[arm_servos_[i].name + "/" + hardware_interface::HW_IF_POSITION];

        // Clamp to joint limits
        cmd_rad = std::clamp(cmd_rad, arm_servos_[i].min_rad, arm_servos_[i].max_rad);

        // Convert radians to pulse using calibrated parameters
        uint16_t pulse = arm_servos_[i].rad_to_pulse(cmd_rad);

        // Only send if changed (threshold 1 pulse)
        if (std::abs(static_cast<double>(pulse) - prev_arm_cmd_raw_[i]) > 1.0)
        {
          has_changes = true;
          prev_arm_cmd_raw_[i] = static_cast<double>(pulse);
        }

        positions.emplace_back(arm_servos_[i].servo_id, pulse);
      }

      if (has_changes)
      {
        // Use first servo's duration as the movement time for all servos in this batch
        uint16_t duration = arm_servos_.empty() ? 500 : arm_servos_[0].servo_duration;
        auto data = PacketProtocol::build_bus_servo_position_cmd(duration, positions);
        serial_port_->send_packet(
            static_cast<uint8_t>(PacketFunction::BUS_SERVO), data);
      }
    }

    // Write GPIO: LED
    {
      double led_on_time = hw_commands_["led/on_time"];
      if (std::abs(led_on_time - prev_led_on_time_) > 0.001 && led_on_time > 0.001)
      {
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
      if (std::abs(buzzer_freq - prev_buzzer_freq_) > 0.001 && buzzer_freq > 0.001)
      {
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

} // namespace robot_board_hardware

PLUGINLIB_EXPORT_CLASS(
    robot_board_hardware::RobotBoardSystemInterface,
    hardware_interface::SystemInterface)
