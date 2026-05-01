#include "robot_board_hardware/serial_port.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>


namespace robot_board_hardware
{

SerialPort::~SerialPort()
{
  stop_recv_thread();
  close();
}

bool SerialPort::open(const std::string & device, int baudrate)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    return false;
  }

  // Clear O_NONBLOCK after open (we want blocking reads with timeout in recv thread)
  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));

  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Map baudrate to termios constant
  speed_t baud;
  switch (baudrate) {
    case 9600: baud = B9600; break;
    case 19200: baud = B19200; break;
    case 38400: baud = B38400; break;
    case 57600: baud = B57600; break;
    case 115200: baud = B115200; break;
    case 230400: baud = B230400; break;
    case 460800: baud = B460800; break;
    case 500000: baud = B500000; break;
    case 576000: baud = B576000; break;
    case 921600: baud = B921600; break;
    case 1000000: baud = B1000000; break;
    case 1152000: baud = B1152000; break;
    case 1500000: baud = B1500000; break;
    case 2000000: baud = B2000000; break;
    default:
      ::close(fd_);
      fd_ = -1;
      return false;
  }

  cfsetispeed(&tty, baud);
  cfsetospeed(&tty, baud);

  // 8N1, no parity, no hardware flow control
  tty.c_cflag &= ~PARENB; //禁用奇偶校验
  tty.c_cflag &= ~CSTOPB; //禁用停止位
  tty.c_cflag &= ~CSIZE;  //清除数据位掩码
  tty.c_cflag |= CS8;     //设置数据位为8
  tty.c_cflag &= ~CRTSCTS; //禁用硬件流控制
  tty.c_cflag |= CREAD | CLOCAL; //启用接收者，忽略 modem 控制

  // Raw input mode
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 禁用规范模式、回显、信号
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); //禁用软件流控制
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // 禁用特殊字符处理

  // Raw output
  tty.c_oflag &= ~OPOST; // 禁用输出处理

  // Read with timeout: VMIN=0, VTIME=1 (100ms timeout)
  tty.c_cc[VMIN] = 0;   // 最少字符数：0（非阻塞）
  tty.c_cc[VTIME] = 1;  // 100ms

  tcflush(fd_, TCIFLUSH);   // 清除输入队列中的旧数据

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // Disable RTS and DTR (matching Python SDK: rts=False, dtr=False)
  int modem_bits = 0;
  ioctl(fd_, TIOCMGET, &modem_bits); // 获取当前状态
  modem_bits &= ~TIOCM_RTS; // 清除 RTS
  modem_bits &= ~TIOCM_DTR;// 清除 DTR
  ioctl(fd_, TIOCMSET, &modem_bits); // 应用设置

  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

/*
@brief Start the receive thread
*/
void SerialPort::start_recv_thread()
{
  if (recv_running_) {
    return;
  }
  recv_running_ = true;
  recv_thread_ = std::thread(&SerialPort::recv_loop, this);
}

/*
@brief Stop the receive thread
*/
void SerialPort::stop_recv_thread()
{
  recv_running_ = false;
  if (recv_thread_.joinable()) {
    recv_thread_.join();
  }
}

/*
@brief Write raw data to the serial port
@param data The data to write
@return True if the data was written successfully, false otherwise
*/
bool SerialPort::raw_write(const std::vector<uint8_t> & data)
{
  if (fd_ < 0) {
    return false;
  }
  ssize_t written = ::write(fd_, data.data(), data.size());
  if (written != static_cast<ssize_t>(data.size())) {
    return false;
  }
  // Ensure data is physically transmitted (not just buffered in OS)
  tcdrain(fd_);
  return true;
}

/*
@brief Send a packet with the given function and data
@param function The function code
@param data The data to send
@return True if the packet was sent successfully, false otherwise
*/
bool SerialPort::send_packet(uint8_t function, const std::vector<uint8_t> & data)
{
  auto packet = PacketProtocol::build_packet(function, data);

  // Debug: hex dump for motor packets (function==3), throttled
  // if (function == 3) {
  //   static int motor_dump_count = 0;
  //   if (motor_dump_count < 3) {
  //     motor_dump_count++;
  //     std::string hex;
  //     hex.reserve(packet.size() * 3);
  //     for (uint8_t b : packet) {
  //       char buf[4];
  //       snprintf(buf, sizeof(buf), "%02X ", b);
  //       hex += buf;
  //     }
  //     RCLCPP_INFO(rclcpp::get_logger("SerialPort"),
  //         "TX motor packet (%zu bytes): %s", packet.size(), hex.c_str());
  //   }
  // }

  std::lock_guard<std::mutex> lock(write_mutex_);
  return raw_write(packet);
}

/*
@brief Get the latest IMU data
@return The latest IMU data, or std::nullopt if no data is available
*/
std::optional<ImuData> SerialPort::get_latest_imu()
{
  std::lock_guard<std::mutex> lock(imu_mutex_);
  auto val = latest_imu_;
  latest_imu_.reset();
  return val;
}

/*
@brief Get the latest battery data
@return The latest battery data, or std::nullopt if no data is available
*/
std::optional<BatteryData> SerialPort::get_latest_battery()
{
  std::lock_guard<std::mutex> lock(battery_mutex_);
  auto val = latest_battery_;
  latest_battery_.reset();
  return val;
}

/*
@brief Get the latest button data
@return The latest button data, or std::nullopt if no data is available
*/
std::optional<ButtonData> SerialPort::get_latest_button()
{
  std::lock_guard<std::mutex> lock(button_mutex_);
  auto val = latest_button_;
  latest_button_.reset();
  return val;
}

/*
@brief 读取指定编号的舵机位置
@param servo_id The ID of the servo to read
@return The position of the servo, or std::nullopt if the read failed
*/
std::optional<int16_t> SerialPort::read_bus_servo_position(uint8_t servo_id)
{
  // Only one servo read at a time
  std::lock_guard<std::mutex> read_lock(servo_read_lock_);

  // Clear any previous response
  {
    std::lock_guard<std::mutex> resp_lock(servo_resp_mutex_);
    servo_response_.reset();
  }

  // Send read position command
  auto cmd_data = PacketProtocol::build_bus_servo_read_position_cmd(servo_id);
  {
    std::lock_guard<std::mutex> wlock(write_mutex_);
    if (!raw_write(PacketProtocol::build_packet(
        static_cast<uint8_t>(PacketFunction::BUS_SERVO), cmd_data)))
    {
      return std::nullopt;
    }
  }

  // Wait for response with timeout
  std::unique_lock<std::mutex> resp_lock(servo_resp_mutex_);
  if (!servo_resp_cv_.wait_for(resp_lock, std::chrono::seconds(1),
    [this]() { return servo_response_.has_value(); }))
  {
    return std::nullopt;
  }

  // Parse response
  int16_t position = 0;
  if (PacketProtocol::parse_bus_servo_position(servo_response_.value(), position)) {
    servo_response_.reset();
    return position;
  }

  servo_response_.reset();
  return std::nullopt;
}

/*
@brief Receive loop to process incoming data
*/
void SerialPort::recv_loop()
{
  RecvState state = RecvState::STARTBYTE1;
  std::vector<uint8_t> frame;
  uint8_t frame_function = 0;
  uint8_t frame_length = 0;
  uint8_t recv_count = 0;

  uint8_t buf[256];

  while (recv_running_) {
    if (fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) {
      continue;
    }

    for (ssize_t i = 0; i < n; ++i) {
      uint8_t dat = buf[i];

      switch (state) {
        case RecvState::STARTBYTE1:
          if (dat == PacketProtocol::HEADER_BYTE1) {
            state = RecvState::STARTBYTE2;
          }
          break;

        case RecvState::STARTBYTE2:
          if (dat == PacketProtocol::HEADER_BYTE2) {
            state = RecvState::FUNCTION;
          } else {
            state = RecvState::STARTBYTE1;
          }
          break;

        case RecvState::FUNCTION:
          if (dat < static_cast<uint8_t>(PacketFunction::NONE)) {
            frame_function = dat;
            frame_length = 0;
            frame.clear();
            state = RecvState::LENGTH;
          } else {
            state = RecvState::STARTBYTE1;
          }
          break;

        case RecvState::LENGTH:
          frame_length = dat;
          recv_count = 0;
          if (dat == 0) {
            state = RecvState::CHECKSUM;
          } else {
            frame.reserve(dat);
            state = RecvState::DATA;
          }
          break;

        case RecvState::DATA:
          frame.push_back(dat);
          recv_count++;
          if (recv_count >= frame_length) {
            state = RecvState::CHECKSUM;
          }
          break;

        case RecvState::CHECKSUM: {
          // Build the CRC input: [function, length, data...]
          std::vector<uint8_t> crc_input;
          crc_input.push_back(frame_function);
          crc_input.push_back(frame_length);
          crc_input.insert(crc_input.end(), frame.begin(), frame.end());

          uint8_t expected_crc = PacketProtocol::checksum_crc8(
            crc_input.data(), crc_input.size());

          if (expected_crc == dat) {
            dispatch_packet(frame_function, frame);
          }

          state = RecvState::STARTBYTE1;
          break;
        }
      }
    }
  }
}

/*
@brief Dispatch a received packet to the appropriate handler
@param function The function code of the packet
@param data The data payload of the packet
*/
void SerialPort::dispatch_packet(uint8_t function, const std::vector<uint8_t> & data)
{
  auto func = static_cast<PacketFunction>(function);

  switch (func) {
    case PacketFunction::IMU: {
      ImuData imu;
      if (PacketProtocol::parse_imu(data, imu)) {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_ = imu;
      }
      break;
    }

    case PacketFunction::SYS: {
      BatteryData battery;
      if (PacketProtocol::parse_battery(data, battery)) {
        std::lock_guard<std::mutex> lock(battery_mutex_);
        latest_battery_ = battery;
      }
      break;
    }

    case PacketFunction::KEY: {
      ButtonData button;
      if (PacketProtocol::parse_button(data, button)) {
        std::lock_guard<std::mutex> lock(button_mutex_);
        latest_button_ = button;
      }
      break;
    }

    case PacketFunction::BUS_SERVO: {
      // Route to servo response for synchronous reads
      std::lock_guard<std::mutex> lock(servo_resp_mutex_);
      servo_response_ = data;
      servo_resp_cv_.notify_one();
      break;
    }

    default:
      // Ignore other packet types
      break;
  }
}

}  // namespace robot_board_hardware
