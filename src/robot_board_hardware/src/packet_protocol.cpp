#include "robot_board_hardware/packet_protocol.hpp"

namespace robot_board_hardware
{

uint8_t PacketProtocol::checksum_crc8(const uint8_t * data, size_t len)
{
  uint8_t check = 0;
  for (size_t i = 0; i < len; ++i) {
    check = crc8_table_[check ^ data[i]];
  }
  return check;
}

/**
 * @brief Build a packet with the given function code and data.
 *
 * @param function The function code.
 * @param data The data to be sent.
 * @return std::vector<uint8_t> The built packet.
 */
std::vector<uint8_t> PacketProtocol ::build_packet(
  uint8_t function, const std::vector<uint8_t> & data)
{
  // Format: [0xAA, 0x55, function, length, data..., crc8]
  // CRC8 covers [function, length, data...]
  std::vector<uint8_t> packet;
  packet.reserve(4 + data.size() + 1);// 预分配容量，避免多次扩容
  packet.push_back(HEADER_BYTE1);// 第1个字节：帧头1
  packet.push_back(HEADER_BYTE2);// 第2个字节：帧头2
  packet.push_back(function);// 第3个字节：功能码
  packet.push_back(static_cast<uint8_t>(data.size()));// 第4个字节：数据长度
  packet.insert(packet.end(), data.begin(), data.end());//  插入数据
  // CRC8 over bytes starting from function (index 2) to end of data
  uint8_t crc = checksum_crc8(packet.data() + 2, packet.size() - 2);
  packet.push_back(crc);

  return packet;
}

/**
 * @brief Build a motor speed command packet.
 *
 * @param speeds A vector of pairs containing motor ID and speed.
 * @return std::vector<uint8_t> The built packet.
 */
std::vector<uint8_t> PacketProtocol::build_motor_speed_cmd(
  const std::vector<std::pair<uint8_t, float>> & speeds)
{
  // Data format: [0x01, count, id0(u8) speed0(f32 LE), id1(u8) speed1(f32 LE), ...]
  // Note: motor ID in packet is 0-based (Python SDK does i[0] - 1)
  std::vector<uint8_t> data;
  data.push_back(0x01);//命令字（设置速度）
  data.push_back(static_cast<uint8_t>(speeds.size()));//电机数量
  for (const auto & [id, speed] : speeds) { //结构化绑定，解包 pair
    data.push_back(id);  // 放入电机 ID（1 字节）
    append_le<float>(data, speed);// 放入速度（4 字节）
  }
  return data;
}

/**
 * @brief Build a bus servo position command packet.
 *
 * @param duration_ms The duration for the position change.
 * @param positions A vector of pairs containing servo ID and position.
 * @return std::vector<uint8_t> The built packet.
 */
std::vector<uint8_t> PacketProtocol::build_bus_servo_position_cmd(
  uint16_t duration_ms,
  const std::vector<std::pair<uint8_t, uint16_t>> & positions)
{
  // Data format: [0x01, dur_lo, dur_hi, count, id(u8) pos(u16 LE), ...]
  std::vector<uint8_t> data;
  data.push_back(0x01);//命令字（设置速度）
  data.push_back(static_cast<uint8_t>(duration_ms & 0xFF));//低字节
  data.push_back(static_cast<uint8_t>((duration_ms >> 8) & 0xFF));//高字节
  data.push_back(static_cast<uint8_t>(positions.size()));//控制舵机个数
  for (const auto & [id, pos] : positions) {
    data.push_back(id);
    append_le<uint16_t>(data, pos);
  }
  return data;
}


/**
 * @brief 构建读总线舵机位置命令包
 *
 * @param servo_id The ID of the servo to read.
 * @return std::vector<uint8_t> The built packet.
 */
std::vector<uint8_t> PacketProtocol::build_bus_servo_read_position_cmd(uint8_t servo_id)
{
  // Data format: [0x05, servo_id]
  return {0x05, servo_id};//命令字（读取位置），舵机ID
}

/**
 * @brief 构建总线舵机使能扭矩命令包
 *
 * @param servo_id The ID of the servo.
 * @param enable True to enable torque, false to disable.
 * @return std::vector<uint8_t> The built packet.
 * 
 * @note 重要：该舵机固件的力矩命令语义与常规相反！
 *       - 0x0B 实际效果是 DISABLE torque
 *       - 0x0C 实际效果是 ENABLE torque
 *       因此这里需要反转命令码
 */
std::vector<uint8_t> PacketProtocol::build_bus_servo_enable_torque_cmd(
  uint8_t servo_id, bool enable)
{
  // Due to firmware bug, command codes are inverted:
  // - 0x0B actually DISABLES torque
  // - 0x0C actually ENABLES torque
  // So we need to invert the command based on desired state
  return {static_cast<uint8_t>(enable ? 0x0C : 0x0B), servo_id};
}

/**
 * @brief 构建LED控制命令包
 *
 * @param led_id The ID of the LED.
 * @param on_time_ms The on time in milliseconds.
 * @param off_time_ms The off time in milliseconds.
 * @param repeat The repeat count.
 * @return std::vector<uint8_t> The built packet.
 */
std::vector<uint8_t> PacketProtocol::build_led_cmd(
  uint8_t led_id, uint16_t on_time_ms, uint16_t off_time_ms, uint16_t repeat)
{
  // Data format: led_id(B), on_time_ms(H LE), off_time_ms(H LE), repeat(H LE)
  std::vector<uint8_t> data;
  data.push_back(led_id);
  append_le<uint16_t>(data, on_time_ms);
  append_le<uint16_t>(data, off_time_ms);
  append_le<uint16_t>(data, repeat);
  return data;
}

/**
 * @brief 构建蜂鸣器控制命令包
 *
 * @param freq The frequency in Hz.
 * @param on_time_ms The on time in milliseconds.
 * @param off_time_ms The off time in milliseconds.
 * @param repeat The repeat count.
 * @return std::vector<uint8_t> The built packet.
 */
std::vector<uint8_t> PacketProtocol::build_buzzer_cmd(
  uint16_t freq, uint16_t on_time_ms, uint16_t off_time_ms, uint16_t repeat)
{
  // Data format: freq(H LE), on_time_ms(H LE), off_time_ms(H LE), repeat(H LE)
  std::vector<uint8_t> data;
  append_le<uint16_t>(data, freq);
  append_le<uint16_t>(data, on_time_ms);
  append_le<uint16_t>(data, off_time_ms);
  append_le<uint16_t>(data, repeat);
  return data;
}

/**
 * @brief 解析IMU数据
 *
 * @param data The data to be parsed.
 * @param out The parsed IMU data.
 * @return true If parsing is successful.
 * @return false If parsing fails.
 */
bool PacketProtocol::parse_imu(const std::vector<uint8_t> & data, ImuData & out)
{
  // IMU data: 24 bytes = 6 x float32 LE (ax, ay, az, gx, gy, gz)
  if (data.size() < 24) {
    return false;
  }
  out.ax = read_le<float>(data.data() + 0);
  out.ay = read_le<float>(data.data() + 4);
  out.az = read_le<float>(data.data() + 8);
  out.gx = read_le<float>(data.data() + 12);
  out.gy = read_le<float>(data.data() + 16);
  out.gz = read_le<float>(data.data() + 20);
  return true;
}

/**
 * @brief 解析电池数据
 *
 * @param data The data to be parsed.
 * @param out The parsed battery data.
 * @return true If parsing is successful.
 * @return false If parsing fails.
 */
bool PacketProtocol::parse_battery(const std::vector<uint8_t> & data, BatteryData & out)
{
  // Battery data: [0x04, voltage(u16 LE)]
  if (data.size() < 3 || data[0] != 0x04) {
    return false;
  }
  out.voltage_mv = read_le<uint16_t>(data.data() + 1);
  out.voltage=out.voltage_mv/1000;
  return true;
}

/**
 * @brief 解析按键数据
 *
 * @param data The data to be parsed.
 * @param out The parsed button data.
 * @return true If parsing is successful.
 * @return false If parsing fails.
 */
bool PacketProtocol::parse_button(const std::vector<uint8_t> & data, ButtonData & out)
{
  // Button data: [key_id(u8), event(u8)]
  if (data.size() < 2) {
    return false;
  }
  out.id = data[0];
  out.event = data[1];
  return true;
}

/**
 * @brief 解析总线舵机位置
 *
 * @param data The data to be parsed.
 * @param position The parsed position.
 * @return true If parsing is successful.
 * @return false If parsing fails.
 */
bool PacketProtocol::parse_bus_servo_position(
  const std::vector<uint8_t> & data, int16_t & position)
{
  // Response format: servo_id(u8), cmd(u8), success(i8), position(i16 LE)
  if (data.size() < 5) {
    return false;
  }
  int8_t success = static_cast<int8_t>(data[2]);
  if (success != 0) {
    return false;
  }
  position = read_le<int16_t>(data.data() + 3);
  return true;
}

}  // namespace robot_board_hardware
