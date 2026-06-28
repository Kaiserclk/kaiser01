#!/usr/bin/env python3
# encoding: utf-8
"""
简化版舵机力矩测试 - 通过实际感受验证力矩是否关闭
"""

import sys
import time
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'driver', 'ros_robot_controller'))

from ros_robot_controller.ros_robot_controller_sdk import Board

def simple_torque_test():
    print("=" * 60)
    print("简化版舵机力矩测试")
    print("=" * 60)
    
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
    
    # 测试1：先发送位置命令，确保舵机有力矩
    print("步骤1: 发送位置命令，确保舵机有力矩")
    print("-" * 60)
    board.bus_servo_set_position(0.5, [[sid, 500] for sid in servo_ids])
    time.sleep(1)
    print("✓ 已发送位置命令到中间点（pulse=500）")
    print("  请用手尝试移动机械臂，应该感到有阻力（力矩开启）")
    input("确认有力矩后按回车继续...\n")
    
    # 测试2：关闭力矩
    print("步骤2: 关闭舵机力矩")
    print("-" * 60)
    for servo_id in servo_ids:
        print(f"  关闭舵机 {servo_id:2d}...", end=" ")
        board.bus_servo_enable_torque(servo_id, False)
        print("✓")
        time.sleep(0.02)
    
    print("\n⚠  力矩关闭命令已发送！")
    print("  请用手尝试移动机械臂关节：")
    print("  - 如果可以轻松移动 → 力矩已成功关闭 ✓")
    print("  - 如果仍然锁死 → 力矩未关闭 ✗")
    print("\n等待5秒后会自动重新开启力矩...")
    
    # 等待用户测试
    for i in range(5, 0, -1):
        time.sleep(1)
        print(f"  {i}...")
    
    # 测试3：重新开启力矩
    print("\n步骤3: 重新开启舵机力矩")
    print("-" * 60)
    for servo_id in servo_ids:
        print(f"  开启舵机 {servo_id:2d}...", end=" ")
        board.bus_servo_enable_torque(servo_id, True)
        print("✓")
        time.sleep(0.02)
    
    time.sleep(1)
    print("\n✓ 力矩已重新开启，请确认机械臂又锁死了")
    
    print("\n" + "=" * 60)
    print("测试问题：")
    print("=" * 60)
    print("请回答：")
    print("1. 步骤2中，关闭力矩后，机械臂是否可以轻松移动？(是/否)")
    print("2. 步骤3中，开启力矩后，机械臂是否又锁死了？(是/否)")
    print("\n如果问题1回答'是'，问题2回答'否'，")
    print("说明力矩控制命令是有效的，只是状态读取有问题！")
    
    return True

if __name__ == "__main__":
    try:
        simple_torque_test()
    except KeyboardInterrupt:
        print("\n\n测试被用户中断")
    except Exception as e:
        print(f"\n✗ 测试过程中出现异常: {e}")
        import traceback
        traceback.print_exc()
