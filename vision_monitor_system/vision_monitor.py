#!/usr/bin/env python3
"""
RoboMaster视觉系统实时监控工具
接收来自C++程序的UDP数据包，实时显示目标状态和机器人状态
"""

import socket
import json
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import numpy as np

# GUI库导入（尝试多个选项）
try:
    import PyQt5
    USE_QT = True
except ImportError:
    USE_QT = False
    print("PyQt5未安装，将使用matplotlib作为备选方案")

import matplotlib
if not USE_QT:
    matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.gridspec import GridSpec


@dataclass
class RobotState:
    """机器人状态数据"""
    timestamp: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    shoot_command: bool = False
    control_active: bool = False
    bullet_speed: float = 0.0

    # 历史数据队列
    yaw_history: deque = field(default_factory=lambda: deque(maxlen=500))
    pitch_history: deque = field(default_factory=lambda: deque(maxlen=500))
    time_history: deque = field(default_factory=lambda: deque(maxlen=500))

    def update(self, data: dict):
        """更新状态数据"""
        self.timestamp = time.time()

        if 'yaw' in data:
            self.yaw = data['yaw']
        if 'pitch' in data:
            self.pitch = data['pitch']
        if 'shoot' in data:
            self.shoot_command = data['shoot']
        if 'control' in data:
            self.control_active = data['control']
        if 'bullet_speed' in data:
            self.bullet_speed = data['bullet_speed']

        # 记录历史
        self.time_history.append(self.timestamp)
        self.yaw_history.append(self.yaw)
        self.pitch_history.append(self.pitch)


@dataclass
class TargetState:
    """目标状态数据"""
    timestamp: float = 0.0
    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    velocity_z: float = 0.0
    yaw_angle: float = 0.0
    yaw_velocity: float = 0.0
    armor_type: str = "unknown"
    armor_name: str = "unknown"
    distance: float = 0.0

    # 历史数据队列
    pos_x_history: deque = field(default_factory=lambda: deque(maxlen=500))
    pos_y_history: deque = field(default_factory=lambda: deque(maxlen=500))
    pos_z_history: deque = field(default_factory=lambda: deque(maxlen=500))
    yaw_history: deque = field(default_factory=lambda: deque(maxlen=500))
    yaw_vel_history: deque = field(default_factory=lambda: deque(maxlen=500))
    distance_history: deque = field(default_factory=lambda: deque(maxlen=500))
    time_history: deque = field(default_factory=lambda: deque(maxlen=500))

    def update(self, data: dict):
        """更新目标数据"""
        self.timestamp = time.time()

        # 提取位置信息
        if 'target_x' in data:
            self.position_x = data['target_x']
        if 'target_y' in data:
            self.position_y = data['target_y']
        if 'target_z' in data:
            self.position_z = data['target_z']

        # 提取速度信息
        if 'target_vx' in data:
            self.velocity_x = data['target_vx']
        if 'target_vy' in data:
            self.velocity_y = data['target_vy']
        if 'target_vz' in data:
            self.velocity_z = data['target_vz']

        # 提取角度信息
        if 'target_yaw' in data:
            self.yaw_angle = data['target_yaw']
        if 'target_yaw_velocity' in data:
            self.yaw_velocity = data['target_yaw_velocity']

        # 提取装甲板信息
        if 'armor_type' in data:
            self.armor_type = data['armor_type']
        if 'armor_name' in data:
            self.armor_name = data['armor_name']

        # 计算距离
        if 'distance' in data:
            self.distance = data['distance']
        else:
            self.distance = np.sqrt(self.position_x**2 + self.position_y**2 + self.position_z**2)

        # 记录历史
        self.time_history.append(self.timestamp)
        self.pos_x_history.append(self.position_x)
        self.pos_y_history.append(self.position_y)
        self.pos_z_history.append(self.position_z)
        self.yaw_history.append(self.yaw_angle)
        self.yaw_vel_history.append(self.yaw_velocity)
        self.distance_history.append(self.distance)


class VisionMonitor:
    """视觉系统监控器"""

    def __init__(self, host='127.0.0.1', port=9870):
        self.host = host
        self.port = port
        self.socket = None
        self.running = False

        # 状态数据
        self.robot_state = RobotState()
        self.target_state = TargetState()

        # 统计信息
        self.packet_count = 0
        self.last_packet_time = 0
        self.fps = 0.0

    def start_receiver(self):
        """启动UDP接收线程"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind((self.host, self.port))
        self.socket.settimeout(0.1)  # 100ms超时

        self.running = True
        self.receiver_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.receiver_thread.start()
        print(f"开始监听 {self.host}:{self.port}")

    def _receive_loop(self):
        """接收数据循环"""
        while self.running:
            try:
                data, addr = self.socket.recvfrom(4096)
                self._process_packet(data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"接收数据错误: {e}")

    def _process_packet(self, data: bytes):
        """处理接收到的数据包"""
        try:
            json_data = json.loads(data.decode('utf-8'))

            # 更新统计信息
            self.packet_count += 1
            current_time = time.time()
            if self.last_packet_time > 0:
                dt = current_time - self.last_packet_time
                if dt > 0:
                    self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)  # 平滑FPS
            self.last_packet_time = current_time

            # 根据数据类型更新对应的状态
            # 这里需要根据你的实际JSON格式调整
            if 'yaw' in json_data or 'pitch' in json_data:
                self.robot_state.update(json_data)
            if any(key.startswith('target_') for key in json_data.keys()):
                self.target_state.update(json_data)

        except json.JSONDecodeError as e:
            print(f"JSON解析错误: {e}")
        except Exception as e:
            print(f"数据处理错误: {e}")

    def stop(self):
        """停止监控"""
        self.running = False
        if self.socket:
            self.socket.close()


class RealtimePlotter:
    """实时曲线绘制器"""

    def __init__(self, monitor: VisionMonitor):
        self.monitor = monitor

        # 创建图形窗口
        self.fig = plt.figure(figsize=(16, 10))
        self.fig.canvas.manager.set_window_title('RoboMaster 视觉系统监控')

        # 创建网格布局
        gs = GridSpec(3, 3, figure=self.fig, hspace=0.3, wspace=0.3)

        # 创建子图
        self.ax_gimbal_yaw = self.fig.add_subplot(gs[0, 0])
        self.ax_gimbal_pitch = self.fig.add_subplot(gs[0, 1])
        self.ax_target_pos = self.fig.add_subplot(gs[0, 2], projection='3d')

        self.ax_target_yaw = self.fig.add_subplot(gs[1, 0])
        self.ax_target_velocity = self.fig.add_subplot(gs[1, 1])
        self.ax_distance = self.fig.add_subplot(gs[1, 2])

        self.ax_status = self.fig.add_subplot(gs[2, :])
        self.ax_status.axis('off')

        # 初始化曲线
        self._init_plots()

    def _init_plots(self):
        """初始化所有子图"""
        # 云台Yaw角
        self.ax_gimbal_yaw.set_title('云台Yaw角度')
        self.ax_gimbal_yaw.set_xlabel('时间 (s)')
        self.ax_gimbal_yaw.set_ylabel('Yaw (rad)')
        self.ax_gimbal_yaw.grid(True, alpha=0.3)
        self.line_gimbal_yaw, = self.ax_gimbal_yaw.plot([], [], 'b-', linewidth=1.5, label='Yaw')
        self.ax_gimbal_yaw.legend()

        # 云台Pitch角
        self.ax_gimbal_pitch.set_title('云台Pitch角度')
        self.ax_gimbal_pitch.set_xlabel('时间 (s)')
        self.ax_gimbal_pitch.set_ylabel('Pitch (rad)')
        self.ax_gimbal_pitch.grid(True, alpha=0.3)
        self.line_gimbal_pitch, = self.ax_gimbal_pitch.plot([], [], 'g-', linewidth=1.5, label='Pitch')
        self.ax_gimbal_pitch.legend()

        # 目标3D位置
        self.ax_target_pos.set_title('目标3D位置轨迹')
        self.ax_target_pos.set_xlabel('X (m)')
        self.ax_target_pos.set_ylabel('Y (m)')
        self.ax_target_pos.set_zlabel('Z (m)')
        self.line_target_pos, = self.ax_target_pos.plot([], [], [], 'r-', linewidth=1.5, label='轨迹')
        self.ax_target_pos.legend()

        # 目标Yaw角
        self.ax_target_yaw.set_title('目标Yaw角度与角速度')
        self.ax_target_yaw.set_xlabel('时间 (s)')
        self.ax_target_yaw.set_ylabel('Yaw (rad)', color='tab:red')
        self.ax_target_yaw.tick_params(axis='y', labelcolor='tab:red')
        self.ax_target_yaw.grid(True, alpha=0.3)
        self.line_target_yaw, = self.ax_target_yaw.plot([], [], 'r-', linewidth=1.5, label='Yaw')

        self.ax_target_yaw_vel = self.ax_target_yaw.twinx()
        self.ax_target_yaw_vel.set_ylabel('Yaw速度 (rad/s)', color='tab:blue')
        self.ax_target_yaw_vel.tick_params(axis='y', labelcolor='tab:blue')
        self.line_target_yaw_vel, = self.ax_target_yaw_vel.plot([], [], 'b--', linewidth=1.5, label='Yaw速度')

        # 目标速度
        self.ax_target_velocity.set_title('目标三维速度')
        self.ax_target_velocity.set_xlabel('时间 (s)')
        self.ax_target_velocity.set_ylabel('速度 (m/s)')
        self.ax_target_velocity.grid(True, alpha=0.3)
        self.line_vel_x, = self.ax_target_velocity.plot([], [], 'r-', linewidth=1, label='Vx', alpha=0.7)
        self.line_vel_y, = self.ax_target_velocity.plot([], [], 'g-', linewidth=1, label='Vy', alpha=0.7)
        self.line_vel_z, = self.ax_target_velocity.plot([], [], 'b-', linewidth=1, label='Vz', alpha=0.7)
        self.ax_target_velocity.legend()

        # 距离
        self.ax_distance.set_title('目标距离')
        self.ax_distance.set_xlabel('时间 (s)')
        self.ax_distance.set_ylabel('距离 (m)')
        self.ax_distance.grid(True, alpha=0.3)
        self.line_distance, = self.ax_distance.plot([], [], 'purple', linewidth=1.5, label='距离')
        self.ax_distance.legend()

        # 状态文本
        self.status_text = self.ax_status.text(
            0.05, 0.5, '', fontsize=12, verticalalignment='center',
            family='monospace', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5)
        )

    def update(self, frame):
        """更新所有图表"""
        robot = self.monitor.robot_state
        target = self.monitor.target_state

        # 更新云台Yaw
        if len(robot.time_history) > 1:
            times = np.array(robot.time_history) - robot.time_history[0]
            self.line_gimbal_yaw.set_data(times, list(robot.yaw_history))
            self.ax_gimbal_yaw.relim()
            self.ax_gimbal_yaw.autoscale_view()

        # 更新云台Pitch
        if len(robot.time_history) > 1:
            times = np.array(robot.time_history) - robot.time_history[0]
            self.line_gimbal_pitch.set_data(times, list(robot.pitch_history))
            self.ax_gimbal_pitch.relim()
            self.ax_gimbal_pitch.autoscale_view()

        # 更新目标3D位置
        if len(target.pos_x_history) > 1:
            self.line_target_pos.set_data_3d(
                list(target.pos_x_history),
                list(target.pos_y_history),
                list(target.pos_z_history)
            )
            self.ax_target_pos.relim()
            self.ax_target_pos.autoscale_view()

        # 更新目标Yaw和角速度
        if len(target.time_history) > 1:
            times = np.array(target.time_history) - target.time_history[0]
            self.line_target_yaw.set_data(times, list(target.yaw_history))
            self.line_target_yaw_vel.set_data(times, list(target.yaw_vel_history))
            self.ax_target_yaw.relim()
            self.ax_target_yaw.autoscale_view()
            self.ax_target_yaw_vel.relim()
            self.ax_target_yaw_vel.autoscale_view()

        # 更新距离
        if len(target.time_history) > 1:
            times = np.array(target.time_history) - target.time_history[0]
            self.line_distance.set_data(times, list(target.distance_history))
            self.ax_distance.relim()
            self.ax_distance.autoscale_view()

        # 更新状态文本
        status_str = f"""
        ═══════════════════════════════════════════════════════════════════════════════════════
        数据包计数: {self.monitor.packet_count:6d}  |  FPS: {self.monitor.fps:6.1f} Hz
        ═══════════════════════════════════════════════════════════════════════════════════════
        【机器人状态】
          云台Yaw:  {robot.yaw:8.3f} rad  |  云台Pitch: {robot.pitch:8.3f} rad
          射击指令: {'■ 开火' if robot.shoot_command else '□ 停火':8s}  |  控制激活: {'■ 是' if robot.control_active else '□ 否':8s}
        ───────────────────────────────────────────────────────────────────────────────────────
        【目标状态】
          位置: X={target.position_x:7.3f}m  Y={target.position_y:7.3f}m  Z={target.position_z:7.3f}m
          速度: Vx={target.velocity_x:6.3f}m/s  Vy={target.velocity_y:6.3f}m/s  Vz={target.velocity_z:6.3f}m/s
          Yaw角: {target.yaw_angle:7.3f} rad  |  Yaw速度: {target.yaw_velocity:7.3f} rad/s
          距离: {target.distance:7.3f} m  |  装甲板: {target.armor_type} - {target.armor_name}
        ═══════════════════════════════════════════════════════════════════════════════════════
        """
        self.status_text.set_text(status_str)

        return (self.line_gimbal_yaw, self.line_gimbal_pitch, self.line_target_pos,
                self.line_target_yaw, self.line_target_yaw_vel, self.line_distance,
                self.status_text)

    def start(self):
        """启动动画"""
        ani = FuncAnimation(self.fig, self.update, interval=50, blit=False, cache_frame_data=False)
        plt.show()


def main():
    """主函数"""
    print("=" * 80)
    print("RoboMaster 视觉系统实时监控工具")
    print("同济大学SuperPower战队")
    print("=" * 80)
    print()
    print("正在启动监控系统...")
    print()
    print("使用说明:")
    print("  1. 确保C++程序使用tools::Plotter发送数据到UDP端口9870")
    print("  2. 数据格式应为JSON格式")
    print("  3. 关闭窗口即可退出程序")
    print()
    print("-" * 80)

    # 创建监控器
    monitor = VisionMonitor(host='127.0.0.1', port=9870)

    # 启动UDP接收
    monitor.start_receiver()

    # 创建绘图器并启动
    plotter = RealtimePlotter(monitor)

    try:
        plotter.start()
    except KeyboardInterrupt:
        print("\n正在退出...")
    finally:
        monitor.stop()
        print("监控已停止")


if __name__ == '__main__':
    main()
