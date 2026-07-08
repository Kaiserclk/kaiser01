#!/usr/bin/env python3
"""
底盘串口数据更新频率测试工具

测试底盘通过串口主动推送的各类型数据包的实际频率:
  - IMU  (func=7): 加速度+角速度, 24字节
  - SYS  (func=0): 电池电压, 3字节
  - KEY  (func=6): 按键状态, 2字节

协议格式 (与 C++ robot_board_hardware 一致):
  [0xAA, 0x55, function(u8), length(u8), data..., crc8]
  CRC8 覆盖 [function, length, data...]

用法:
  python3 test_serial_frequency.py [-d /dev/robot_board] [-b 1000000] [-t 30]
"""

import sys
import time
import struct
import threading
import argparse
from collections import defaultdict

try:
    import serial
except ImportError:
    print("需要 pyserial: pip3 install pyserial")
    sys.exit(1)


# ======================== CRC8 表 (与 C++ 一致) ========================
CRC8_TABLE = [
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
]


def checksum_crc8(data):
    check = 0
    for b in data:
        check = CRC8_TABLE[check ^ b]
    return check & 0xFF


# ======================== 协议常量 ========================
HEADER1 = 0xAA
HEADER2 = 0x55

FUNC_NAMES = {
    0:  "SYS(电池)",
    1:  "LED",
    2:  "BUZZER",
    3:  "MOTOR",
    4:  "PWM_SERVO",
    5:  "BUS_SERVO",
    6:  "KEY(按键)",
    7:  "IMU",
    8:  "GAMEPAD",
    9:  "SBUS",
}


def safe_div(a, b):
    return a / b if b > 0 else 0.0


class PacketReceiver:
    """串口接收器：解析协议帧并分类计数"""

    def __init__(self, port):
        self.port = port
        self.running = False

        # 接收状态机
        self.state = 0  # 0=START1, 1=START2, 2=FUNC, 3=LEN, 4=DATA, 5=CRC
        self.frame_func = 0
        self.frame_len = 0
        self.frame_data = bytearray()
        self.recv_count = 0

        # 统计
        self.lock = threading.Lock()
        self.total_packets = 0
        self.crc_errors = 0
        self.per_func = defaultdict(int)  # {func: count}
        self.per_func_bytes = defaultdict(int)  # {func: total_bytes}

        # 最新数据（用于显示采样值）
        self.latest = {}

    def _dispatch(self, func, data):
        with self.lock:
            self.total_packets += 1
            self.per_func[func] += 1
            self.per_func_bytes[func] += len(data)

        # 解析并缓存最新值
        try:
            if func == 7 and len(data) >= 24:  # IMU
                ax, ay, az, gx, gy, gz = struct.unpack('<6f', data[:24])
                self.latest['imu'] = (ax, ay, az, gx, gy, gz)
            elif func == 0 and len(data) >= 3 and data[0] == 0x04:  # SYS/battery
                voltage_mv = struct.unpack('<H', data[1:3])[0]
                self.latest['battery'] = voltage_mv
            elif func == 6 and len(data) >= 2:  # KEY
                self.latest['key'] = (data[0], data[1])
        except Exception:
            pass

    def recv_loop(self):
        """主接收循环，使用状态机解析协议帧"""
        while self.running:
            try:
                raw = self.port.read(256)
            except serial.SerialException:
                break
            if not raw:
                continue

            for byte in raw:
                if self.state == 0:  # START1
                    if byte == HEADER1:
                        self.state = 1

                elif self.state == 1:  # START2
                    self.state = 2 if byte == HEADER2 else 0

                elif self.state == 2:  # FUNCTION
                    if byte < 10:
                        self.frame_func = byte
                        self.frame_len = 0
                        self.frame_data = bytearray()
                        self.state = 3
                    else:
                        self.state = 0

                elif self.state == 3:  # LENGTH
                    self.frame_len = byte
                    self.recv_count = 0
                    if byte == 0:
                        self.state = 5
                    else:
                        self.frame_data = bytearray(byte)
                        self.state = 4

                elif self.state == 4:  # DATA
                    self.frame_data[self.recv_count] = byte
                    self.recv_count += 1
                    if self.recv_count >= self.frame_len:
                        self.state = 5

                elif self.state == 5:  # CHECKSUM
                    crc_input = bytes([self.frame_func, self.frame_len]) + bytes(self.frame_data)
                    if checksum_crc8(crc_input) == byte:
                        self._dispatch(self.frame_func, bytes(self.frame_data))
                    else:
                        with self.lock:
                            self.crc_errors += 1
                    self.state = 0

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self.recv_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def get_stats(self):
        """获取快照统计"""
        with self.lock:
            return {
                'total': self.total_packets,
                'crc_errors': self.crc_errors,
                'per_func': dict(self.per_func),
                'per_func_bytes': dict(self.per_func_bytes),
            }


def format_freq(count, seconds):
    """格式化频率显示"""
    hz = safe_div(count, seconds)
    period_ms = safe_div(seconds * 1000.0, count) if count > 0 else float('inf')
    return f"{hz:8.1f} Hz  ({period_ms:6.1f} ms)"

def main():
    parser = argparse.ArgumentParser(
        description="底盘串口数据更新频率测试工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 test_serial_frequency.py                    # 默认 /dev/robot_board, 测试30秒
  python3 test_serial_frequency.py -t 10              # 测试10秒
  python3 test_serial_frequency.py -d /dev/ttyUSB0    # 指定串口
        """)
    parser.add_argument('-d', '--device', default='/dev/robot_board',
                        help='串口设备路径 (默认: /dev/robot_board)')
    parser.add_argument('-b', '--baudrate', type=int, default=1000000,
                        help='波特率 (默认: 1000000)')
    parser.add_argument('-t', '--time', type=int, default=30,
                        help='测试时长(秒) (默认: 30)')
    args = parser.parse_args()

    # 打开串口
    print(f"打开串口 {args.device} @ {args.baudrate} bps ...")
    try:
        port = serial.Serial(args.device, args.baudrate, timeout=0.5)
        port.rts = False
        port.dtr = False
    except serial.SerialException as e:
        print(f"错误: 无法打开串口 - {e}")
        print("提示: 请确认设备连接、权限及 ros2 程序未占用串口")
        sys.exit(1)

    # 清空缓冲区
    port.reset_input_buffer()

    # 启动接收
    receiver = PacketReceiver(port)
    receiver.start()
    print(f"接收线程已启动，测试 {args.time} 秒...\n")

    # 主循环：每秒打印统计
    start_time = time.time()
    last_stats = {'total': 0, 'per_func': {}}
    last_time = start_time

    header_shown = False

    try:
        while time.time() - start_time < args.time:
            time.sleep(1.0)
            now = time.time()
            interval = now - last_time
            stats = receiver.get_stats()

            # 增量统计（本次间隔内的新增包数）
            incr_total = stats['total'] - last_stats['total']
            incr_per_func = {}
            for func, cnt in stats['per_func'].items():
                incr_per_func[func] = cnt - last_stats['per_func'].get(func, 0)

            if not header_shown and incr_total > 0:
                print(f"{'功能':<18} {'频率':<24} {'包数':>6}  {'字节数':>8}  {'占比':>6}")
                print("-" * 68)
                header_shown = True

            if incr_total > 0:
                for func in sorted(incr_per_func.keys()):
                    cnt = incr_per_func.get(func, 0)
                    name = FUNC_NAMES.get(func, f"UNKNOWN({func})")
                    freq_str = format_freq(cnt, interval)
                    pct = safe_div(cnt, incr_total) * 100 if incr_total > 0 else 0
                    print(f"{name:<18} {freq_str}  {cnt:>6}  {stats['per_func_bytes'].get(func, 0):>8}  {pct:>5.1f}%")

            # 显示总计（非第一次）
            if header_shown:
                total_freq = format_freq(incr_total, interval)
                crc_info = f" CRC错误:{stats['crc_errors'] - last_stats.get('crc_errors', 0)}" if stats['crc_errors'] > last_stats.get('crc_errors', 0) else ""
                print(f"{'[本间隔总计]':<18} {total_freq}  {incr_total:>6}{crc_info}")
                print()

            last_stats = {
                'total': stats['total'],
                'per_func': dict(stats['per_func']),
                'crc_errors': stats['crc_errors'],
            }
            last_time = now

    except KeyboardInterrupt:
        print("\n用户中断")

    # 最终汇总
    elapsed = time.time() - start_time
    stats = receiver.get_stats()
    receiver.stop()
    port.close()

    print("=" * 68)
    print(f"{'总测试时长:':<12} {elapsed:.1f} 秒")
    print(f"{'总包数:':<12} {stats['total']}")
    print(f"{'CRC 错误:':<12} {stats['crc_errors']}")
    print()
    print(f"{'功能':<18} {'平均频率':<24} {'总包数':>6}  {'总字节':>8}")
    print("-" * 68)
    for func in sorted(stats['per_func'].keys()):
        cnt = stats['per_func'][func]
        name = FUNC_NAMES.get(func, f"UNKNOWN({func})")
        freq_str = format_freq(cnt, elapsed)
        print(f"{name:<18} {freq_str}  {cnt:>6}  {stats['per_func_bytes'].get(func, 0):>8}")

    print("-" * 68)
    total_freq = format_freq(stats['total'], elapsed)
    print(f"{'[总计]':<18} {total_freq}  {stats['total']:>6}")

    # 显示最新采样值
    print()
    print("最新采样值:")
    if 'imu' in receiver.latest:
        ax, ay, az, gx, gy, gz = receiver.latest['imu']
        print(f"  IMU:    ax={ax:+.4f}g  ay={ay:+.4f}g  az={az:+.4f}g  "
              f"gx={gx:+.2f}°/s  gy={gy:+.2f}°/s  gz={gz:+.2f}°/s")
    if 'battery' in receiver.latest:
        v = receiver.latest['battery']
        print(f"  电池:   {v} mV ({v/1000:.2f} V)")
    if 'key' in receiver.latest:
        kid, kev = receiver.latest['key']
        print(f"  按键:   id={kid}, event=0x{kev:02X}")


if __name__ == "__main__":
    main()
