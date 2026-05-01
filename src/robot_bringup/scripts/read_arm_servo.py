#!/usr/bin/env python3
# encoding: utf-8
"""
直接通过串口读取机械臂所有关节舵机的当前位置（脉宽值）。
运行方式：
  python3 src/robot_bringup/scripts/read_arm_servo.py           # 读取配置的6个关节
  python3 src/robot_bringup/scripts/read_arm_servo.py --scan   # 扫描所有在线舵机(ID 1-15)
  python3 src/robot_bringup/scripts/read_arm_servo.py /dev/ttyACM0 --scan
"""

import sys
import time
import math

# 将参考 SDK 加入路径
SDK_PATH = "/home/sunrise/kaiser01/参考文件/driver/ros_robot_controller/ros_robot_controller"
sys.path.insert(0, SDK_PATH)

from ros_robot_controller_sdk import Board  # type: ignore

# ========== 配置（与 robot.urdf.xacro 保持一致） ==========
SERVO_IDS = [1, 2, 3, 4, 5, 10]  # 机械臂舵机 ID 列表（ID 10 为夹爪）

SERVO_CONFIG = {
    1:  {"joint": "arm_joint_1", "zero": 500, "min": 1000, "max": 0},
    2:  {"joint": "arm_joint_2", "zero": 500, "min": 1000, "max": 0},
    3:  {"joint": "arm_joint_3", "zero": 500, "min": 1000, "max": 0},
    4:  {"joint": "arm_joint_4", "zero": 500, "min": 1000, "max": 0},
    5:  {"joint": "arm_joint_5", "zero": 500, "min": 1000, "max": 0},
    10: {"joint": "arm_joint_6", "zero": 700, "min": 1000, "max": 0},
}

SERVO_ANGLE_RANGE_DEG = 240.0
SERVO_RAW_MAX = 1000.0
RADIANS_PER_TICK = (SERVO_ANGLE_RANGE_DEG * math.pi / 180.0) / SERVO_RAW_MAX


def pulse_to_rad(pulse: int, zero: int, min_pulse: int, max_pulse: int) -> float:
    """脉宽 → 弧度（与 C++ ServoConfig::pulse_to_rad 一致）"""
    flipped = min_pulse > max_pulse
    if flipped:
        return (zero - pulse) * RADIANS_PER_TICK
    else:
        return (pulse - zero) * RADIANS_PER_TICK


def scan_all_servos(board):
    """扫描 ID 1-15，打印所有响应的舵机及其位置"""
    print("\n[扫描模式] 正在逐一查询舵机 ID 1-15 ...\n")
    print("=" * 52)
    print(f"{'舵机ID':>6} {'状态':>8} {'脉宽':>6} {'弧度 (rad)':>12} {'角度 (°)':>10}")
    print("-" * 52)
    online = []
    for sid in range(1, 16):
        result = board.bus_servo_read_position(sid)
        if result is not None:
            pulse = result[0]
            zero, mn, mx = 500, 1000, 0
            rad = pulse_to_rad(pulse, zero, mn, mx)
            deg = math.degrees(rad)
            print(f"{sid:>6}  {'在线':>8} {pulse:>6}    {rad:>10.4f}    {deg:>8.2f}°")
            online.append(sid)
        else:
            print(f"{sid:>6}  {'无响应':>8}")
    print("=" * 52)
    print(f"\n在线舵机 ID 列表: {online}")
    return online


def read_arm_servos(board):
    """读取配置的6个机械臂关节舵机位置"""
    print("\n" + "=" * 66)
    print(f"{'关节':<14} {'舵机ID':>6} {'脉宽':>6} {'弧度 (rad)':>12} {'角度 (°)':>10}")
    print("-" * 66)
    for sid in SERVO_IDS:
        result = board.bus_servo_read_position(sid)
        cfg = SERVO_CONFIG[sid]
        if result is not None:
            pulse = result[0]
            rad = pulse_to_rad(pulse, cfg["zero"], cfg["min"], cfg["max"])
            deg = math.degrees(rad)
            print(f"{cfg['joint']:<14} {sid:>6} {pulse:>6}    {rad:>10.4f}    {deg:>8.2f}°")
        else:
            print(f"{cfg['joint']:<14} {sid:>6}   读取失败（舵机未响应）")
    print("=" * 66)


def main():
    args = sys.argv[1:]
    scan_mode = "--scan" in args
    device_args = [a for a in args if not a.startswith("-")]
    device = device_args[0] if device_args else "/dev/ttyACM0"

    print(f"连接串口: {device} @ 1000000 baud ...")
    board = Board(device=device, baudrate=1000000)
    board.enable_reception()
    time.sleep(0.1)  # 等待接收线程就绪

    if scan_mode:
        scan_all_servos(board)
    else:
        read_arm_servos(board)

    board.enable_recv = False


if __name__ == "__main__":
    main()
