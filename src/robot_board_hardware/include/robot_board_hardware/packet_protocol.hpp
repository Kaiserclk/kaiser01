#ifndef ROBOT_BOARD_HARDWARE__PACKET_PROTOCOL_HPP_
#define ROBOT_BOARD_HARDWARE__PACKET_PROTOCOL_HPP_

#include <cstdint>
#include <cstring>
#include <vector>
#include <utility>

namespace robot_board_hardware
{

enum class PacketFunction : uint8_t
{
  SYS = 0,
  LED = 1,
  BUZZER = 2,
  MOTOR = 3,
  PWM_SERVO = 4,
  BUS_SERVO = 5,
  KEY = 6,
  IMU = 7,
  GAMEPAD = 8,
  SBUS = 9,
  NONE = 10
};

struct ImuData
{
  float ax = 0.0f;
  float ay = 0.0f;
  float az = 0.0f;
  float gx = 0.0f;
  float gy = 0.0f;
  float gz = 0.0f;
};

struct BatteryData//电压
{
  uint16_t voltage_mv = 0;
};

struct ButtonData//按键
{
  uint8_t id = 0;
  uint8_t event = 0;
};

class PacketProtocol
{
public:
  static constexpr uint8_t HEADER_BYTE1 = 0xAA;
  static constexpr uint8_t HEADER_BYTE2 = 0x55;

  // CRC-8 Dallas checksum
  static uint8_t checksum_crc8(const uint8_t * data, size_t len);

  // Build a complete packet: [0xAA, 0x55, function, length, data..., crc8]
  static std::vector<uint8_t> build_packet(uint8_t function, const std::vector<uint8_t> & data);

  // Motor speed command: sub-cmd 0x01, each motor as (id_0based, speed_float)
  static std::vector<uint8_t> build_motor_speed_cmd(
    const std::vector<std::pair<uint8_t, float>> & speeds);

  // Bus servo position command: sub-cmd 0x01
  static std::vector<uint8_t> build_bus_servo_position_cmd(
    uint16_t duration_ms,
    const std::vector<std::pair<uint8_t, uint16_t>> & positions);

  // Bus servo read position request: sub-cmd 0x05
  static std::vector<uint8_t> build_bus_servo_read_position_cmd(uint8_t servo_id);

  // Bus servo enable/disable torque: sub-cmd 0x0B (enable) / 0x0C (disable)
  static std::vector<uint8_t> build_bus_servo_enable_torque_cmd(uint8_t servo_id, bool enable);

  // LED command
  static std::vector<uint8_t> build_led_cmd(
    uint8_t led_id, uint16_t on_time_ms, uint16_t off_time_ms, uint16_t repeat);

  // Buzzer command
  static std::vector<uint8_t> build_buzzer_cmd(
    uint16_t freq, uint16_t on_time_ms, uint16_t off_time_ms, uint16_t repeat);

  // Parse received data
  static bool parse_imu(const std::vector<uint8_t> & data, ImuData & out);
  static bool parse_battery(const std::vector<uint8_t> & data, BatteryData & out);
  static bool parse_button(const std::vector<uint8_t> & data, ButtonData & out);

  // Parse bus servo read position response: returns (success, position)
  // Response format: servo_id(u8), cmd(u8), success(i8), position(i16 LE)
  static bool parse_bus_servo_position(const std::vector<uint8_t> & data, int16_t & position);

private:
  static constexpr uint8_t crc8_table_[256] = {
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
  };

  // Helper: append a little-endian value to a vector
  template <typename T>
  static void append_le(std::vector<uint8_t> & buf, T value)
  {
    uint8_t bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    buf.insert(buf.end(), bytes, bytes + sizeof(T));
  }

  // Helper: read a little-endian value from a buffer
  template <typename T>
  static T read_le(const uint8_t * data)
  {
    T value;
    std::memcpy(&value, data, sizeof(T));
    return value;
  }
};

}  // namespace robot_board_hardware

#endif  // ROBOT_BOARD_HARDWARE__PACKET_PROTOCOL_HPP_
