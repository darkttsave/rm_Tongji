#!/usr/bin/env python3
"""
监控系统测试数据发送器
用于测试监控程序是否正常工作，无需运行实际的C++程序
"""

import socket
import json
import time
import math
import argparse


def generate_test_data(t):
    """生成模拟的测试数据"""
    # 模拟小陀螺运动
    radius = 0.3  # 0.3米半径
    angular_velocity = 10.0  # 10 rad/s
    distance = 2.5  # 2.5米距离

    # 目标位置（圆周运动）
    angle = angular_velocity * t
    target_x = distance + radius * math.cos(angle)
    target_y = radius * math.sin(angle)
    target_z = 0.4 + 0.05 * math.sin(2 * t)  # 轻微上下波动

    # 目标速度
    target_vx = -radius * angular_velocity * math.sin(angle)
    target_vy = radius * angular_velocity * math.cos(angle)
    target_vz = 0.1 * math.cos(2 * t)

    # 云台跟踪（有延迟和抖动）
    yaw_target = math.atan2(target_y, target_x)
    yaw = yaw_target + 0.02 * math.sin(5 * t)  # 模拟跟踪误差
    pitch = math.atan2(target_z, math.sqrt(target_x**2 + target_y**2))
    pitch += 0.01 * math.cos(7 * t)  # 模拟抖动

    # 射击判断（模拟）
    tracking_error = abs(yaw - yaw_target)
    shoot = tracking_error < 0.05  # 误差小于0.05rad时开火

    data = {
        # 机器人状态
        "timestamp": time.time(),
        "yaw": yaw,
        "pitch": pitch,
        "shoot": shoot,
        "control": True,
        "bullet_speed": 28.0,

        # 目标状态
        "target_x": target_x,
        "target_y": target_y,
        "target_z": target_z,
        "target_vx": target_vx,
        "target_vy": target_vy,
        "target_vz": target_vz,
        "target_yaw": angle,
        "target_yaw_velocity": angular_velocity,

        # 装甲板信息
        "armor_type": "small",
        "armor_name": "three",
        "distance": math.sqrt(target_x**2 + target_y**2 + target_z**2),

        # 额外信息
        "frame_count": int(t * 100),
        "tracking_error": tracking_error
    }

    return data


def send_test_data(host='127.0.0.1', port=9870, frequency=100, duration=None, scenario='normal'):
    """发送测试数据"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("=" * 80)
    print("监控系统测试数据发送器")
    print("=" * 80)
    print(f"目标地址: {host}:{port}")
    print(f"发送频率: {frequency} Hz")
    print(f"测试场景: {scenario}")
    if duration:
        print(f"持续时间: {duration} 秒")
    else:
        print(f"持续时间: 无限 (Ctrl+C退出)")
    print("=" * 80)
    print()

    start_time = time.time()
    packet_count = 0
    interval = 1.0 / frequency

    try:
        while True:
            current_time = time.time()
            elapsed = current_time - start_time

            # 检查是否达到持续时间
            if duration and elapsed >= duration:
                break

            # 根据场景生成数据
            if scenario == 'normal':
                data = generate_test_data(elapsed)
            elif scenario == 'static':
                data = generate_static_target(elapsed)
            elif scenario == 'fast_spin':
                data = generate_fast_spin_target(elapsed)
            elif scenario == 'switch':
                data = generate_armor_switch(elapsed)
            else:
                data = generate_test_data(elapsed)

            # 发送数据
            json_data = json.dumps(data).encode('utf-8')
            sock.sendto(json_data, (host, port))

            packet_count += 1

            # 每秒显示一次统计
            if packet_count % frequency == 0:
                print(f"[{elapsed:.1f}s] 已发送 {packet_count} 个数据包 "
                      f"| Yaw: {data['yaw']:.3f} | Distance: {data['distance']:.2f}m "
                      f"| Shoot: {'■' if data['shoot'] else '□'}")

            # 等待下一个周期
            next_time = start_time + (packet_count + 1) * interval
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n正在停止...")
    finally:
        elapsed = time.time() - start_time
        print(f"\n发送完成！")
        print(f"总计发送: {packet_count} 个数据包")
        print(f"运行时间: {elapsed:.2f} 秒")
        print(f"平均频率: {packet_count/elapsed:.1f} Hz")
        sock.close()


def generate_static_target(t):
    """生成静态目标数据"""
    data = {
        "timestamp": time.time(),
        "yaw": 0.0 + 0.01 * math.sin(t),
        "pitch": 0.0 + 0.005 * math.cos(t),
        "shoot": True,
        "control": True,
        "bullet_speed": 28.0,

        "target_x": 3.0,
        "target_y": 0.0,
        "target_z": 0.5,
        "target_vx": 0.0,
        "target_vy": 0.0,
        "target_vz": 0.0,
        "target_yaw": 0.0,
        "target_yaw_velocity": 0.0,

        "armor_type": "small",
        "armor_name": "one",
        "distance": 3.0,
    }
    return data


def generate_fast_spin_target(t):
    """生成高速小陀螺数据"""
    radius = 0.4
    angular_velocity = 18.0  # 18 rad/s
    distance = 2.0

    angle = angular_velocity * t
    target_x = distance + radius * math.cos(angle)
    target_y = radius * math.sin(angle)
    target_z = 0.4

    data = {
        "timestamp": time.time(),
        "yaw": math.atan2(target_y, target_x),
        "pitch": math.atan2(target_z, distance),
        "shoot": False,  # 高速时较少开火
        "control": True,
        "bullet_speed": 28.0,

        "target_x": target_x,
        "target_y": target_y,
        "target_z": target_z,
        "target_vx": -radius * angular_velocity * math.sin(angle),
        "target_vy": radius * angular_velocity * math.cos(angle),
        "target_vz": 0.0,
        "target_yaw": angle,
        "target_yaw_velocity": angular_velocity,

        "armor_type": "small",
        "armor_name": "four",
        "distance": math.sqrt(target_x**2 + target_y**2 + target_z**2),
    }
    return data


def generate_armor_switch(t):
    """生成装甲板切换数据"""
    # 模拟装甲板在不同位置切换
    switch_period = 0.5  # 0.5秒切换一次
    current_armor = int(t / switch_period) % 4

    armor_names = ["one", "two", "three", "four"]
    angles = [0, math.pi/2, math.pi, 3*math.pi/2]

    base_angle = angles[current_armor]
    distance = 2.5
    radius = 0.15

    target_x = distance * math.cos(base_angle) + radius * math.cos(t * 2)
    target_y = distance * math.sin(base_angle) + radius * math.sin(t * 2)
    target_z = 0.4

    data = {
        "timestamp": time.time(),
        "yaw": math.atan2(target_y, target_x),
        "pitch": math.atan2(target_z, distance),
        "shoot": (t % switch_period) > 0.3,  # 切换后稍等再开火
        "control": True,
        "bullet_speed": 28.0,

        "target_x": target_x,
        "target_y": target_y,
        "target_z": target_z,
        "target_vx": -radius * 2 * math.sin(t * 2),
        "target_vy": radius * 2 * math.cos(t * 2),
        "target_vz": 0.0,
        "target_yaw": base_angle,
        "target_yaw_velocity": 5.0,

        "armor_type": "small",
        "armor_name": armor_names[current_armor],
        "distance": math.sqrt(target_x**2 + target_y**2 + target_z**2),
    }
    return data


def main():
    parser = argparse.ArgumentParser(description='监控系统测试数据发送器')
    parser.add_argument('--host', default='127.0.0.1', help='目标IP地址 (默认: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=9870, help='目标端口 (默认: 9870)')
    parser.add_argument('--frequency', type=int, default=100, help='发送频率Hz (默认: 100)')
    parser.add_argument('--duration', type=float, help='持续时间（秒），不指定则无限运行')
    parser.add_argument('--scenario', choices=['normal', 'static', 'fast_spin', 'switch'],
                        default='normal', help='测试场景 (默认: normal)')

    args = parser.parse_args()

    send_test_data(
        host=args.host,
        port=args.port,
        frequency=args.frequency,
        duration=args.duration,
        scenario=args.scenario
    )


if __name__ == '__main__':
    main()
