#!/usr/bin/env python3
# encoding: utf-8
"""
测试总线舵机的力矩控制功能
"""

import sys
import time
import os

# 添加 driver 路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'driver', 'ros_robot_controller'))

from ros_robot_controller.ros_robot_controller_sdk import Board

def test_servo_torque_control():
    print("=" * 60)
    print("总线舵机力矩控制测试")
    print("=" * 60)
    
    # 初始化控制板
    try:
        board = Board(device="/dev/robot_board", baudrate=1000000)
        print("✓ 控制板初始化成功")
    except Exception as e:
        print(f"✗ 控制板初始化失败: {e}")
        return False
    
    # 使能接收
    board.enable_reception()
    time.sleep(0.5)
    
    # 要测试的舵机ID列表（根据你的实际情况修改）
    servo_ids = [1, 2, 3, 4, 5, 10]
    
    print(f"\n准备测试舵机ID: {servo_ids}")
    
    # 测试1：先读取当前的力矩状态
    print("\n" + "-" * 60)
    print("测试1: 读取当前力矩状态")
    print("-" * 60)
    for servo_id in servo_ids:
        try:
            torque_state = board.bus_servo_read_torque_state(servo_id)
            if torque_state is not None:
                state = "ENABLED" if torque_state[0] == 1 else "DISABLED"
                print(f"  舵机 {servo_id:2d}: 力矩状态 = {state} (raw={torque_state})")
            else:
                print(f"  舵机 {servo_id:2d}: 读取失败")
        except Exception as e:
            print(f"  舵机 {servo_id:2d}: 异常 - {e}")
    
    # 测试2：关闭力矩
    print("\n" + "-" * 60)
    print("测试2: 关闭所有舵机力矩")
    print("-" * 60)
    input("请用手握住机械臂，然后按回车继续...")
    
    for servo_id in servo_ids:
        try:
            print(f"  关闭舵机 {servo_id:2d} 的力矩...", end=" ")
            board.bus_servo_enable_torque(servo_id, False)
            print("✓")
            time.sleep(0.02)
        except Exception as e:
            print(f"✗ 失败: {e}")
    
    print("\n等待2秒后检查力矩状态...")
    time.sleep(2)
    
    # 检查力矩状态
    print("当前力矩状态:")
    for servo_id in servo_ids:
        try:
            torque_state = board.bus_servo_read_torque_state(servo_id)
            if torque_state is not None:
                state = "ENABLED" if torque_state[0] == 1 else "DISABLED"
                print(f"  舵机 {servo_id:2d}: 力矩状态 = {state} (raw={torque_state})")
                
                # 测试是否可以手动移动
                if torque_state[0] == 0:
                    print(f"           → 力矩已关闭，请尝试手动移动该关节")
            else:
                print(f"  舵机 {servo_id:2d}: 读取失败")
        except Exception as e:
            print(f"  舵机 {servo_id:2d}: 异常 - {e}")
    
    # 测试3：重新开启力矩
    print("\n" + "-" * 60)
    print("测试3: 重新开启所有舵机力矩")
    print("-" * 60)
    input("按回车键重新开启力矩...")
    
    for servo_id in servo_ids:
        try:
            print(f"  开启舵机 {servo_id:2d} 的力矩...", end=" ")
            board.bus_servo_enable_torque(servo_id, True)
            print("✓")
            time.sleep(0.02)
        except Exception as e:
            print(f"✗ 失败: {e}")
    
    time.sleep(1)
    
    # 最终检查
    print("\n最终力矩状态:")
    for servo_id in servo_ids:
        try:
            torque_state = board.bus_servo_read_torque_state(servo_id)
            if torque_state is not None:
                state = "ENABLED" if torque_state[0] == 1 else "DISABLED"
                print(f"  舵机 {servo_id:2d}: 力矩状态 = {state} (raw={torque_state})")
            else:
                print(f"  舵机 {servo_id:2d}: 读取失败")
        except Exception as e:
            print(f"  舵机 {servo_id:2d}: 异常 - {e}")
    
    print("\n" + "=" * 60)
    print("测试完成！")
    print("=" * 60)
    
    return True

if __name__ == "__main__":
    try:
        test_servo_torque_control()
    except KeyboardInterrupt:
        print("\n\n测试被用户中断")
    except Exception as e:
        print(f"\n✗ 测试过程中出现异常: {e}")
        import traceback
        traceback.print_exc()
