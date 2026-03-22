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

std::vector<uint8_t> PacketProtocol::build_packet(
  uint8_t function, const std::vector<uint8_t> & data)
{
  // Format: [0xAA, 0x55, function, length, data..., crc8]
  // CRC8 covers [function, length, data...]
  std::vector<uint8_t> packet;
  packet.reserve(4 + data.size() + 1);
  packet.push_back(HEADER_BYTE1);
  packet.push_back(HEADER_BYTE2);
  packet.push_back(function);
  packet.push_back(static_cast<uint8_t>(data.size()));
  packet.insert(packet.end(), data.begin(), data.end());

  // CRC8 over bytes starting from function (index 2) to end of data
  uint8_t crc = checksum_crc8(packet.data() + 2, packet.size() - 2);
  packet.push_back(crc);

  return packet;
}

std::vector<uint8_t> PacketProtocol::build_motor_speed_cmd(
  const std::vector<std::pair<uint8_t, float>> & speeds)
{
  // Data format: [0x01, count, id0(u8) speed0(f32 LE), id1(u8) speed1(f32 LE), ...]
  // Note: motor ID in packet is 0-based (Python SDK does i[0] - 1)
  std::vector<uint8_t> data;
  data.push_back(0x01);
  data.push_back(static_cast<uint8_t>(speeds.size()));
  for (const auto & [id, speed] : speeds) {
    data.push_back(id);  // already 0-based
    append_le<float>(data, speed);
  }
  return data;
}

std::vector<uint8_t> PacketProtocol::build_bus_servo_position_cmd(
  uint16_t duration_ms,
  const std::vector<std::pair<uint8_t, uint16_t>> & positions)
{
  // Data format: [0x01, dur_lo, dur_hi, count, id(u8) pos(u16 LE), ...]
  std::vector<uint8_t> data;
  data.push_back(0x01);
  data.push_back(static_cast<uint8_t>(duration_ms & 0xFF));
  data.push_back(static_cast<uint8_t>((duration_ms >> 8) & 0xFF));
  data.push_back(static_cast<uint8_t>(positions.size()));
  for (const auto & [id, pos] : positions) {
    data.push_back(id);
    append_le<uint16_t>(data, pos);
  }
  return data;
}

std::vector<uint8_t> PacketProtocol::build_bus_servo_read_position_cmd(uint8_t servo_id)
{
  // Data format: [0x05, servo_id]
  return {0x05, servo_id};
}

std::vector<uint8_t> PacketProtocol::build_bus_servo_enable_torque_cmd(
  uint8_t servo_id, bool enable)
{
  // Enable: [0x0B, servo_id], Disable: [0x0C, servo_id]
  return {static_cast<uint8_t>(enable ? 0x0B : 0x0C), servo_id};
}

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

bool PacketProtocol::parse_battery(const std::vector<uint8_t> & data, BatteryData & out)
{
  // Battery data: [0x04, voltage(u16 LE)]
  if (data.size() < 3 || data[0] != 0x04) {
    return false;
  }
  out.voltage_mv = read_le<uint16_t>(data.data() + 1);
  return true;
}

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
