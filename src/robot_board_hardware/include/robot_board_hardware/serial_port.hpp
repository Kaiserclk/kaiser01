#ifndef ROBOT_BOARD_HARDWARE__SERIAL_PORT_HPP_
#define ROBOT_BOARD_HARDWARE__SERIAL_PORT_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "robot_board_hardware/packet_protocol.hpp"

namespace robot_board_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();

  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  bool open(const std::string & device, int baudrate);
  void close();

  void start_recv_thread();
  void stop_recv_thread();

  // Thread-safe send: builds and sends a complete packet
  bool send_packet(uint8_t function, const std::vector<uint8_t> & data);

  // Thread-safe getters for latest received data (non-blocking)
  std::optional<ImuData> get_latest_imu();
  std::optional<BatteryData> get_latest_battery();
  std::optional<ButtonData> get_latest_button();

  // Synchronous request-response for bus servo position (blocking, timeout 1s)
  std::optional<int16_t> read_bus_servo_position(uint8_t servo_id);

private:
  int fd_ = -1;
  std::atomic<bool> recv_running_{false};
  std::thread recv_thread_;

  // Write protection
  std::mutex write_mutex_;

  // Latest sensor data storage
  std::mutex imu_mutex_;
  std::optional<ImuData> latest_imu_;

  std::mutex battery_mutex_;
  std::optional<BatteryData> latest_battery_;

  std::mutex button_mutex_;
  std::optional<ButtonData> latest_button_;

  // Synchronous servo read support
  std::mutex servo_read_lock_;  // only one servo read at a time
  std::mutex servo_resp_mutex_;
  std::condition_variable servo_resp_cv_;
  std::optional<std::vector<uint8_t>> servo_response_;

  // Receive state machine
  enum class RecvState
  {
    STARTBYTE1,
    STARTBYTE2,
    FUNCTION,
    LENGTH,
    DATA,
    CHECKSUM
  };

  void recv_loop();
  void dispatch_packet(uint8_t function, const std::vector<uint8_t> & data);

  // Raw serial write (no locking - caller must hold write_mutex_)
  bool raw_write(const std::vector<uint8_t> & data);
};

}  // namespace robot_board_hardware

#endif  // ROBOT_BOARD_HARDWARE__SERIAL_PORT_HPP_
