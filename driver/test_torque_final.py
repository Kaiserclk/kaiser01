#!/usr/bin/env python3
# encoding: utf-8
"""
最终验证：修复后的舵机力矩控制测试
"""

import sys
import time
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'driver', 'ros_robot_controller'))

from ros_robot_controller.ros_robot_controller_sdk import Board

def final_verification():
    print("=" * 70)
    print("最终验证：修复后的舵机力矩控制")
    print("=" * 70)
    print("\n修复内容：")
    print("  - 反转了力矩控制命令码（0x0B ↔ 0x0C）")
    print("  - enable=true  → 发送 0x0C（实际效果：ENABLE）")
    print("  - enable=false → 发送 0x0B（实际效果：DISABLE）")
    print()
    
    try:
        board = Board(device="/dev/robot_board", baudrate=1000000)
        print("✓ 控制板初始化成功\n")
    except Exception as e:
        print(f"✗ 控制板初始化失败: {e}")
        return False
    
    board.enable_reception()
    time.sleep(0.5)
    
    servo_ids = [1, 2, 3, 4, 5, 10]
    print(f"测试舵机ID: {servo_ids}\n")
    
    # 测试1：开启力矩
    print("=" * 70)
    print("测试1: 开启舵机力矩 (enable=True → 发送 0x0C)")
    print("=" * 70)
    for servo_id in servo_ids:
        print(f"  开启舵机 {servo_id:2d}...", end=" ")
        board.bus_servo_enable_torque(servo_id, True)  # 现在应该真正开启力矩
        print("✓")
        time.sleep(0.02)
    
    time.sleep(1)
    print("\n请用手尝试移动机械臂：")
    print("  预期：应该感到有阻力，机械臂锁死（力矩已开启）")
    input("确认有力矩后按回车继续...\n")
    
    # 测试2：关闭力矩
    print("=" * 70)
    print("测试2: 关闭舵机力矩 (enable=False → 发送 0x0B)")
    print("=" * 70)
    for servo_id in servo_ids:
        print(f"  关闭舵机 {servo_id:2d}...", end=" ")
        board.bus_servo_enable_torque(servo_id, False)  # 现在应该真正关闭力矩
        print("✓")
        time.sleep(0.02)
    
    time.sleep(1)
    print("\n请用手尝试移动机械臂：")
    print("  预期：应该可以轻松移动，没有阻力（力矩已关闭）")
    print("  等待5秒后会自动重新开启力矩...")
    
    for i in range(5, 0, -1):
        time.sleep(1)
        print(f"  {i}...")
    
    # 测试3：重新开启力矩
    print("\n" + "=" * 70)
    print("测试3: 重新开启舵机力矩")
    print("=" * 70)
    for servo_id in servo_ids:
        print(f"  开启舵机 {servo_id:2d}...", end=" ")
        board.bus_servo_enable_torque(servo_id, True)
        print("✓")
        time.sleep(0.02)
    
    time.sleep(1)
    print("\n请确认机械臂又锁死了（力矩重新开启）")
    
    print("\n" + "=" * 70)
    print("验证结果")
    print("=" * 70)
    print("如果以上测试都符合预期，说明修复成功！")
    print("\n修复内容总结：")
    print("  ✓ C++ 代码：packet_protocol.cpp 中的 build_bus_servo_enable_torque_cmd")
    print("  ✓ Python SDK：ros_robot_controller_sdk.py 中的 bus_servo_enable_torque")
    print("  ✓ 命令码反转：0x0B → DISABLE, 0x0C → ENABLE")
    
    return True

if __name__ == "__main__":
    try:
        final_verification()
    except KeyboardInterrupt:
        print("\n\n测试被用户中断")
    except Exception as e:
        print(f"\n✗ 测试过程中出现异常: {e}")
        import traceback
        traceback.print_exc()
