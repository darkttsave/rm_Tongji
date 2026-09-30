#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据包验证工具 - 完整版本
支持串口通信和模拟模式,用于验证下位机发送的数据包格式
使用说明: python universal_packet_validator.py [config.yaml/config.json]
依赖安装: pip install pyserial PyQt5 pyyaml
"""

import sys
import struct
import time
import json
import yaml
import os
import re
from typing import Optional, List, Tuple, Dict, Any

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTextEdit, QComboBox, QSpinBox,
    QGroupBox, QRadioButton, QTableWidget, QTableWidgetItem,
    QHeaderView, QMessageBox, QTabWidget, QFileDialog, QDialog,
    QDialogButtonBox, QFormLayout, QCheckBox, QLineEdit
)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QColor, QFont

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ==================== CRC校验类 ====================

class CRC8Table:
    """CRC8校验表(与C++代码中的实现一致)"""
    TABLE = [
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
        0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
        0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
        0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
        0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
        0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
        0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
        0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
        0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
        0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
        0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
        0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
        0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
        0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
        0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
        0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
        0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
        0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
        0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
        0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
        0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
        0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
        0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
        0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
        0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
        0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
        0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
        0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
        0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
        0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
        0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35
    ]

    @staticmethod
    def calculate(data: bytes) -> int:
        crc = 0xFF
        for byte in data:
            crc = CRC8Table.TABLE[crc ^ byte]
        return crc

    @staticmethod
    def verify(data: bytes) -> bool:
        if len(data) < 2:
            return False
        return CRC8Table.calculate(data[:-1]) == data[-1]


class CRC16Table:
    """CRC16校验表(完整256项)"""
    TABLE = [
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
    ]

    @staticmethod
    def calculate(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            i = (crc ^ byte) & 0xFF
            crc = (crc >> 8) ^ CRC16Table.TABLE[i]
        return crc

    @staticmethod
    def verify(data: bytes) -> bool:
        if len(data) < 3:
            return False
        calculated = CRC16Table.calculate(data[:-2])
        received = data[-2] | (data[-1] << 8)
        return calculated == received


# ==================== 数据包定义类 ====================

class PacketField:
    """数据包字段定义"""
    def __init__(self, name: str, size: int, field_type: str, description: str = "", validation: Dict = None):
        self.name = name
        self.size = size
        self.field_type = field_type
        self.description = description
        self.validation = validation or {}

    def parse(self, data: bytes) -> Tuple:
        hex_str = ' '.join(f'{b:02X}' for b in data[:self.size])

        if self.field_type == 'uint8':
            value = data[0] if len(data) >= 1 else 0
        elif self.field_type == 'uint16':
            value = struct.unpack('<H', data[:2])[0] if len(data) >= 2 else 0
        elif self.field_type == 'uint32':
            value = struct.unpack('<I', data[:4])[0] if len(data) >= 4 else 0
        elif self.field_type == 'float':
            value = struct.unpack('<f', data[:4])[0] if len(data) >= 4 else 0.0
        elif self.field_type == 'double':
            value = struct.unpack('<d', data[:8])[0] if len(data) >= 8 else 0.0
        else:
            value = data[:self.size]

        return value, hex_str


class PacketProtocol:
    """数据包协议定义"""
    def __init__(self, name: str = "默认协议"):
        self.name = name
        self.fields: List[PacketField] = []
        self.total_size = 0
        self.simulator_rules = {}

    def add_field(self, field: PacketField):
        self.fields.append(field)
        self.total_size += field.size

    def parse(self, data: bytes) -> Tuple:
        if len(data) < self.total_size:
            return False, [], f"数据长度不足: 需要{self.total_size}字节, 实际{len(data)}字节"

        results = []
        offset = 0
        for field in self.fields:
            field_data = data[offset:offset + field.size]
            value, hex_str = field.parse(field_data)
            results.append((field.name, value, hex_str))
            offset += field.size

        return True, results, ""

    def validate_packet(self, data: bytes) -> Tuple[bool, List[str]]:
        """验证数据包，返回(是否有效, 错误列表)"""
        errors = []

        offset = 0
        for field in self.fields:
            if not field.validation:
                offset += field.size
                continue

            val_type = field.validation.get('type')

            if val_type == 'fixed_value':
                expected = field.validation.get('value')
                actual = data[offset] if field.size == 1 else struct.unpack('<H', data[offset:offset+2])[0]
                if actual != expected:
                    errors.append(f"{field.name}错误: 期望{expected}, 实际{actual}")

            elif val_type == 'crc8':
                val_range = field.validation.get('range', [0, offset])
                start, end = val_range
                if end == -1:
                    end = offset
                crc_data = data[start:end]
                calculated_crc = CRC8Table.calculate(crc_data)
                received_crc = data[offset]
                if calculated_crc != received_crc:
                    errors.append(f"CRC8校验失败: 计算值{calculated_crc:02X}, 接收值{received_crc:02X}")

            elif val_type == 'crc16':
                val_range = field.validation.get('range', [0, -1])
                start, end = val_range
                if end == -1:
                    end = offset
                crc_data = data[start:end]
                calculated_crc = CRC16Table.calculate(crc_data)
                received_crc = struct.unpack('<H', data[offset:offset+2])[0]
                if calculated_crc != received_crc:
                    errors.append(f"CRC16校验失败: 计算值{calculated_crc:04X}, 接收值{received_crc:04X}")

            elif val_type == 'checksum':
                val_range = field.validation.get('range', [0, -1])
                start, end = val_range
                if end == -1:
                    end = offset
                checksum_data = data[start:end]
                calculated_sum = sum(checksum_data) & 0xFF
                received_sum = data[offset]
                if calculated_sum != received_sum:
                    errors.append(f"校验和失败: 计算值{calculated_sum:02X}, 接收值{received_sum:02X}")

            offset += field.size

        return len(errors) == 0, errors

    @staticmethod
    def load_from_config(config_path: str) -> 'PacketProtocol':
        """从配置文件加载协议"""
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"配置文件不存在: {config_path}")

        # 判断文件类型
        if config_path.endswith('.yaml') or config_path.endswith('.yml'):
            with open(config_path, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
        elif config_path.endswith('.json'):
            with open(config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
        else:
            raise ValueError("配置文件必须是.yaml, .yml或.json格式")

        protocol = PacketProtocol(config.get('protocol_name', '未命名协议'))

        # 加载字段定义
        for field_config in config.get('fields', []):
            field = PacketField(
                name=field_config['name'],
                size=field_config['size'],
                field_type=field_config['type'],
                description=field_config.get('description', ''),
                validation=field_config.get('validation')
            )
            protocol.add_field(field)

        # 加载模拟器规则
        simulator_config = config.get('simulator', {})
        if simulator_config.get('enabled', False):
            protocol.simulator_rules = simulator_config.get('generate_rules', {})

        return protocol


# ==================== 线程类 ====================

class SerialThread(QThread):
    """串口接收线程"""
    data_received = pyqtSignal(bytes)
    error_occurred = pyqtSignal(str)

    def __init__(self, port: str, baudrate: int):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = False
        self.serial_port = None

    def run(self):
        try:
            self.serial_port = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.1)
            self.running = True
            while self.running:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    self.data_received.emit(data)
                time.sleep(0.01)
        except Exception as e:
            self.error_occurred.emit(f"串口错误: {str(e)}")
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()

    def stop(self):
        self.running = False


class SimulatorThread(QThread):
    """模拟器线程"""
    data_generated = pyqtSignal(bytes)

    def __init__(self, protocol: PacketProtocol, interval: float):
        super().__init__()
        self.protocol = protocol
        self.interval = interval
        self.running = False

    def run(self):
        self.running = True
        counter = 0
        while self.running:
            packet = self.generate_packet(counter)
            self.data_generated.emit(packet)
            counter += 1
            time.sleep(self.interval)

    def parse_rule(self, rule: str, counter: int) -> Any:
        """解析生成规则"""
        if rule.startswith('fixed:'):
            # 固定值: fixed:0xA5 或 fixed:100
            value_str = rule.split(':', 1)[1]
            if value_str.startswith('0x') or value_str.startswith('0X'):
                return int(value_str, 16)
            else:
                return float(value_str) if '.' in value_str else int(value_str)

        elif rule == 'counter':
            # 递增计数器
            return counter

        elif rule.startswith('random:'):
            # 随机范围: random:1-100 或 random:0.5-10.0
            import random
            range_str = rule.split(':', 1)[1]
            min_val, max_val = range_str.split('-')
            min_val = float(min_val)
            max_val = float(max_val)
            if '.' in range_str:
                return random.uniform(min_val, max_val)
            else:
                return random.randint(int(min_val), int(max_val))

        elif rule.startswith('formula:'):
            # 公式计算: formula:counter*0.1
            formula = rule.split(':', 1)[1]
            try:
                import math
                return eval(formula, {"__builtins__": {}}, {"counter": counter, "math": math})
            except:
                return 0

        return 0

    def generate_packet(self, counter: int) -> bytes:
        """生成数据包"""
        packet = bytearray()

        # 第一遍：生成除校验字段外的所有数据
        for field in self.protocol.fields:
            val_type = field.validation.get('type') if field.validation else None

            # 跳过校验字段，稍后计算
            if val_type in ['crc8', 'crc16', 'checksum']:
                # 预留空间
                packet.extend(b'\x00' * field.size)
                continue

            # 查找生成规则
            if field.name in self.protocol.simulator_rules:
                rule = self.protocol.simulator_rules[field.name]
                value = self.parse_rule(rule, counter)
            else:
                # 默认规则
                if val_type == 'fixed_value':
                    value = field.validation.get('value', 0)
                elif field.field_type == 'uint8':
                    value = (counter * 2) % 256
                elif field.field_type == 'uint16':
                    value = (counter * 100) % 65536
                elif field.field_type == 'uint32':
                    value = (counter * 1000) % 4294967296
                elif field.field_type == 'float':
                    value = counter * 0.1
                elif field.field_type == 'double':
                    value = counter * 0.01
                else:
                    value = 0

            # 打包数据
            if field.field_type == 'uint8':
                packet.append(int(value) & 0xFF)
            elif field.field_type == 'uint16':
                packet.extend(struct.pack('<H', int(value) & 0xFFFF))
            elif field.field_type == 'uint32':
                packet.extend(struct.pack('<I', int(value) & 0xFFFFFFFF))
            elif field.field_type == 'float':
                packet.extend(struct.pack('<f', float(value)))
            elif field.field_type == 'double':
                packet.extend(struct.pack('<d', float(value)))
            else:
                packet.extend(b'\x00' * field.size)

        # 第二遍：计算并填充校验字段
        offset = 0
        for field in self.protocol.fields:
            val_type = field.validation.get('type') if field.validation else None

            if val_type == 'crc8':
                # 计算CRC8
                val_range = field.validation.get('range', [0, offset])
                start, end = val_range
                if end == -1:
                    end = offset
                crc_data = packet[start:end]
                crc_value = CRC8Table.calculate(crc_data)
                packet[offset] = crc_value

            elif val_type == 'crc16':
                # 计算CRC16
                val_range = field.validation.get('range', [0, -1])
                start, end = val_range
                if end == -1:
                    end = offset
                crc_data = packet[start:end]
                crc_value = CRC16Table.calculate(crc_data)
                struct.pack_into('<H', packet, offset, crc_value)

            elif val_type == 'checksum':
                # 计算校验和
                val_range = field.validation.get('range', [0, -1])
                start, end = val_range
                if end == -1:
                    end = offset
                checksum_data = packet[start:end]
                checksum_value = sum(checksum_data) & 0xFF
                packet[offset] = checksum_value

            offset += field.size

        return bytes(packet)

    def stop(self):
        self.running = False


# ==================== 主界面类 ====================

class PacketValidatorGUI(QMainWindow):
    """数据包验证工具主界面"""
    def __init__(self, config_path: str = None):
        super().__init__()
        self.setWindowTitle("数据包验证工具 v2.0")
        self.setGeometry(100, 100, 1200, 800)

        # 加载协议配置
        if config_path and os.path.exists(config_path):
            try:
                self.protocol = PacketProtocol.load_from_config(config_path)
                self.config_path = config_path
            except Exception as e:
                QMessageBox.critical(None, "配置加载失败", f"无法加载配置文件:\n{str(e)}")
                self.protocol = self.create_default_protocol()
                self.config_path = None
        else:
            self.protocol = self.create_default_protocol()
            self.config_path = None

        self.serial_thread = None
        self.simulator_thread = None
        self.total_packets = 0
        self.valid_packets = 0
        self.invalid_packets = 0
        self.receive_buffer = bytearray()
        self.init_ui()

    def create_default_protocol(self):
        """创建默认协议(可根据实际需求修改)"""
        protocol = PacketProtocol("RM视觉通信协议")
        protocol.add_field(PacketField("帧头SOF", 1, "uint8", "起始标志(0xA5)", {"type": "fixed_value", "value": 165}))
        protocol.add_field(PacketField("数据长度", 2, "uint16", "数据段长度"))
        protocol.add_field(PacketField("序列号", 1, "uint8", "包序号"))
        protocol.add_field(PacketField("CRC8", 1, "uint8", "包头CRC8校验", {"type": "crc8", "range": [0, 4]}))
        protocol.add_field(PacketField("命令ID", 2, "uint16", "命令类型"))
        protocol.add_field(PacketField("Yaw角度", 4, "float", "偏航角"))
        protocol.add_field(PacketField("Pitch角度", 4, "float", "俯仰角"))
        protocol.add_field(PacketField("距离", 4, "float", "目标距离"))
        protocol.add_field(PacketField("标志位", 1, "uint8", "控制标志"))
        protocol.add_field(PacketField("CRC16", 2, "uint16", "整包CRC16校验", {"type": "crc16", "range": [0, -1]}))

        # 添加模拟器规则
        protocol.simulator_rules = {
            "帧头SOF": "fixed:0xA5",
            "序列号": "counter",
            "Yaw角度": "formula:counter*0.1",
            "Pitch角度": "formula:counter*0.05",
            "距离": "random:0.5-10.0"
        }
        return protocol

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        layout = QVBoxLayout()
        main_widget.setLayout(layout)

        # 创建标签页
        self.tab_widget = QTabWidget()

        # 数据解析标签页
        data_tab = QWidget()
        data_layout = QVBoxLayout()
        data_layout.addWidget(self.create_control_group())
        data_layout.addWidget(self.create_data_group(), 1)
        data_layout.addWidget(self.create_stats_group())
        data_tab.setLayout(data_layout)

        # 配置编辑标签页
        config_tab = self.create_config_editor_tab()

        self.tab_widget.addTab(data_tab, "数据解析")
        self.tab_widget.addTab(config_tab, "配置编辑")

        layout.addWidget(self.tab_widget)

    def create_control_group(self):
        group = QGroupBox("连接设置")
        layout = QVBoxLayout()

        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("工作模式:"))
        self.serial_radio = QRadioButton("串口模式")
        self.simulator_radio = QRadioButton("模拟模式")
        self.serial_radio.setChecked(True)
        mode_layout.addWidget(self.serial_radio)
        mode_layout.addWidget(self.simulator_radio)
        mode_layout.addStretch()
        layout.addLayout(mode_layout)

        serial_layout = QHBoxLayout()
        serial_layout.addWidget(QLabel("串口:"))
        self.port_combo = QComboBox()
        self.refresh_ports()
        serial_layout.addWidget(self.port_combo)
        refresh_btn = QPushButton("刷新")
        refresh_btn.clicked.connect(self.refresh_ports)
        serial_layout.addWidget(refresh_btn)
        serial_layout.addWidget(QLabel("波特率:"))
        self.baudrate_combo = QComboBox()
        self.baudrate_combo.addItems(["115200", "921600", "460800", "230400", "9600"])
        serial_layout.addWidget(self.baudrate_combo)
        serial_layout.addStretch()
        layout.addLayout(serial_layout)

        sim_layout = QHBoxLayout()
        sim_layout.addWidget(QLabel("发送间隔(秒):"))
        self.interval_spin = QSpinBox()
        self.interval_spin.setRange(1, 10)
        self.interval_spin.setValue(1)
        sim_layout.addWidget(self.interval_spin)
        sim_layout.addStretch()
        layout.addLayout(sim_layout)

        btn_layout = QHBoxLayout()
        self.start_btn = QPushButton("启动")
        self.start_btn.clicked.connect(self.start_connection)
        self.start_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        btn_layout.addWidget(self.start_btn)
        self.stop_btn = QPushButton("停止")
        self.stop_btn.clicked.connect(self.stop_connection)
        self.stop_btn.setEnabled(False)
        self.stop_btn.setStyleSheet("background-color: #f44336; color: white; font-weight: bold;")
        btn_layout.addWidget(self.stop_btn)
        clear_btn = QPushButton("清空日志")
        clear_btn.clicked.connect(self.clear_log)
        btn_layout.addWidget(clear_btn)
        btn_layout.addStretch()
        layout.addLayout(btn_layout)

        group.setLayout(layout)
        return group

    def create_data_group(self):
        group = QGroupBox("数据解析")
        layout = QVBoxLayout()

        self.packet_table = QTableWidget()
        self.packet_table.setColumnCount(4)
        self.packet_table.setHorizontalHeaderLabels(["字段名", "值", "十六进制", "描述"])
        header = self.packet_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.Stretch)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        layout.addWidget(self.packet_table)

        layout.addWidget(QLabel("接收日志:"))
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setFont(QFont("Consolas", 9))
        layout.addWidget(self.log_text)

        group.setLayout(layout)
        return group

    def create_stats_group(self):
        group = QGroupBox("统计信息")
        layout = QHBoxLayout()

        self.total_label = QLabel("总数据包: 0")
        self.valid_label = QLabel("有效: 0")
        self.invalid_label = QLabel("无效: 0")
        self.rate_label = QLabel("成功率: 0%")

        font = QFont()
        font.setBold(True)
        font.setPointSize(10)

        for label in [self.total_label, self.valid_label, self.invalid_label, self.rate_label]:
            label.setFont(font)
            layout.addWidget(label)

        layout.addStretch()
        group.setLayout(layout)
        return group

    def refresh_ports(self):
        """刷新虚拟串口列表（排除蓝牙串口）"""
        self.port_combo.clear()
        if not SERIAL_AVAILABLE:
            self.port_combo.addItem("未安装pyserial")
            return
        ports = serial.tools.list_ports.comports()
        # 只显示虚拟串口（通过hwid过滤掉蓝牙串口）
        virtual_ports = []
        for port in ports:
            description = port.description.lower()
            hwid = port.hwid.upper()
            # 过滤掉蓝牙串口：检查hwid和description
            if 'BTHENUM' not in hwid and 'bluetooth' not in description:
                virtual_ports.append(port)

        for port in virtual_ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")

        if self.port_combo.count() == 0:
            self.port_combo.addItem("未找到虚拟串口")

    def start_connection(self):
        if self.serial_radio.isChecked():
            if not SERIAL_AVAILABLE:
                QMessageBox.warning(self, "错误", "串口功能不可用,请安装pyserial库\n\n安装命令: pip install pyserial")
                return
            port_text = self.port_combo.currentText()
            if "未找到" in port_text or "未安装" in port_text:
                QMessageBox.warning(self, "错误", "请选择有效的串口")
                return
            port = port_text.split(" - ")[0]
            baudrate = int(self.baudrate_combo.currentText())
            self.serial_thread = SerialThread(port, baudrate)
            self.serial_thread.data_received.connect(self.on_data_received)
            self.serial_thread.error_occurred.connect(self.on_error)
            self.serial_thread.start()
            self.log_message(f"已连接串口: {port}, 波特率: {baudrate}")
        else:
            interval = self.interval_spin.value()
            self.simulator_thread = SimulatorThread(self.protocol, interval)
            self.simulator_thread.data_generated.connect(self.on_data_received)
            self.simulator_thread.start()
            self.log_message(f"已启动模拟器, 发送间隔: {interval}秒")

        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.serial_radio.setEnabled(False)
        self.simulator_radio.setEnabled(False)

    def stop_connection(self):
        if self.serial_thread:
            self.serial_thread.stop()
            self.serial_thread.wait()
            self.serial_thread = None
        if self.simulator_thread:
            self.simulator_thread.stop()
            self.simulator_thread.wait()
            self.simulator_thread = None

        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.serial_radio.setEnabled(True)
        self.simulator_radio.setEnabled(True)
        self.log_message("已停止连接")

    def on_data_received(self, data):
        self.receive_buffer.extend(data)

        while len(self.receive_buffer) >= self.protocol.total_size:
            # 获取第一个字段作为帧头标识
            if self.protocol.fields and self.protocol.fields[0].validation:
                val_type = self.protocol.fields[0].validation.get('type')
                if val_type == 'fixed_value':
                    header_byte = self.protocol.fields[0].validation.get('value', 0xA5)
                else:
                    header_byte = 0xA5  # 默认值
            else:
                header_byte = 0xA5  # 默认值

            start_idx = self.receive_buffer.find(header_byte)
            if start_idx == -1:
                self.receive_buffer.clear()
                break
            if start_idx > 0:
                self.receive_buffer = self.receive_buffer[start_idx:]
            if len(self.receive_buffer) < self.protocol.total_size:
                break

            packet_data = bytes(self.receive_buffer[:self.protocol.total_size])
            self.receive_buffer = self.receive_buffer[self.protocol.total_size:]
            self.parse_and_validate_packet(packet_data)

    def parse_and_validate_packet(self, packet_data):
        self.total_packets += 1
        success, results, error_msg = self.protocol.parse(packet_data)

        if not success:
            self.invalid_packets += 1
            self.log_message(f"解析失败: {error_msg}", error=True)
            self.update_stats()
            return

        # 使用新的验证方法
        is_valid, validation_errors = self.protocol.validate_packet(packet_data)

        if is_valid:
            self.valid_packets += 1
            self.log_message(f"数据包 #{self.total_packets}: 验证通过 ✓")
        else:
            self.invalid_packets += 1
            self.log_message(f"数据包 #{self.total_packets}: {', '.join(validation_errors)}", error=True)

        self.update_packet_table(results, is_valid)
        self.update_stats()

    def update_packet_table(self, results, is_valid):
        self.packet_table.setRowCount(len(results))

        for i, (name, value, hex_str) in enumerate(results):
            name_item = QTableWidgetItem(name)
            self.packet_table.setItem(i, 0, name_item)

            value_item = QTableWidgetItem(str(value))
            self.packet_table.setItem(i, 1, value_item)

            hex_item = QTableWidgetItem(hex_str)
            hex_item.setFont(QFont("Consolas", 9))
            self.packet_table.setItem(i, 2, hex_item)

            field = self.protocol.fields[i]
            desc_item = QTableWidgetItem(field.description)
            self.packet_table.setItem(i, 3, desc_item)

            color = QColor(200, 255, 200) if is_valid else QColor(255, 200, 200)
            for col in range(4):
                if self.packet_table.item(i, col):
                    self.packet_table.item(i, col).setBackground(color)

    def update_stats(self):
        self.total_label.setText(f"总数据包: {self.total_packets}")
        self.valid_label.setText(f"有效: {self.valid_packets}")
        self.valid_label.setStyleSheet("color: green;")
        self.invalid_label.setText(f"无效: {self.invalid_packets}")
        self.invalid_label.setStyleSheet("color: red;")

        if self.total_packets > 0:
            rate = (self.valid_packets / self.total_packets) * 100
            self.rate_label.setText(f"成功率: {rate:.1f}%")
            if rate >= 90:
                self.rate_label.setStyleSheet("color: green;")
            elif rate >= 70:
                self.rate_label.setStyleSheet("color: orange;")
            else:
                self.rate_label.setStyleSheet("color: red;")

    def log_message(self, message, error=False):
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}"
        if error:
            self.log_text.append(f'<span style="color: red;">{log_entry}</span>')
        else:
            self.log_text.append(log_entry)
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def clear_log(self):
        self.log_text.clear()
        self.total_packets = 0
        self.valid_packets = 0
        self.invalid_packets = 0
        self.update_stats()
        self.log_message("日志已清空")

    def on_error(self, error_msg):
        QMessageBox.critical(self, "错误", error_msg)
        self.stop_connection()

    def create_config_editor_tab(self):
        """创建配置编辑标签页"""
        widget = QWidget()
        layout = QVBoxLayout()
        widget.setLayout(layout)

        # 顶部工具栏
        toolbar_layout = QHBoxLayout()

        load_btn = QPushButton("加载配置")
        load_btn.clicked.connect(self.load_config_file)
        toolbar_layout.addWidget(load_btn)

        new_btn = QPushButton("新建配置")
        new_btn.clicked.connect(self.new_config)
        toolbar_layout.addWidget(new_btn)

        save_btn = QPushButton("保存配置")
        save_btn.clicked.connect(self.save_config_file)
        save_btn.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        toolbar_layout.addWidget(save_btn)

        saveas_btn = QPushButton("另存为")
        saveas_btn.clicked.connect(self.save_config_as)
        toolbar_layout.addWidget(saveas_btn)

        toolbar_layout.addStretch()
        layout.addLayout(toolbar_layout)

        # 基本信息区域
        info_group = QGroupBox("协议基本信息")
        info_layout = QFormLayout()

        self.protocol_name_edit = QLineEdit()
        self.protocol_name_edit.setPlaceholderText("例: RM视觉通信协议")
        info_layout.addRow("协议名称:", self.protocol_name_edit)

        self.protocol_desc_edit = QLineEdit()
        self.protocol_desc_edit.setPlaceholderText("例: 用于机器人视觉系统的通信协议")
        info_layout.addRow("协议描述:", self.protocol_desc_edit)

        byte_order_layout = QHBoxLayout()
        self.little_endian_radio = QRadioButton("小端序 (Little)")
        self.big_endian_radio = QRadioButton("大端序 (Big)")
        self.little_endian_radio.setChecked(True)
        byte_order_layout.addWidget(self.little_endian_radio)
        byte_order_layout.addWidget(self.big_endian_radio)
        byte_order_layout.addStretch()
        info_layout.addRow("字节序:", byte_order_layout)

        self.total_size_label = QLabel("数据包大小: 0 字节")
        self.total_size_label.setStyleSheet("font-weight: bold; color: #2196F3;")
        info_layout.addRow("", self.total_size_label)

        info_group.setLayout(info_layout)
        layout.addWidget(info_group)

        # 字段列表区域
        fields_group = QGroupBox("字段列表")
        fields_layout = QVBoxLayout()

        # 字段表格
        self.fields_table = QTableWidget()
        self.fields_table.setColumnCount(6)
        self.fields_table.setHorizontalHeaderLabels(["字段名", "类型", "大小", "描述", "校验类型", "校验参数"])
        header = self.fields_table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QHeaderView.Stretch)
        header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        header.setSectionResizeMode(5, QHeaderView.Stretch)
        self.fields_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.fields_table.itemSelectionChanged.connect(self.on_field_selected)
        fields_layout.addWidget(self.fields_table)

        # 字段操作按钮
        field_btn_layout = QHBoxLayout()

        add_field_btn = QPushButton("添加字段")
        add_field_btn.clicked.connect(self.add_field)
        field_btn_layout.addWidget(add_field_btn)

        edit_field_btn = QPushButton("编辑字段")
        edit_field_btn.clicked.connect(self.edit_field)
        field_btn_layout.addWidget(edit_field_btn)

        delete_field_btn = QPushButton("删除字段")
        delete_field_btn.clicked.connect(self.delete_field)
        field_btn_layout.addWidget(delete_field_btn)

        move_up_btn = QPushButton("上移")
        move_up_btn.clicked.connect(self.move_field_up)
        field_btn_layout.addWidget(move_up_btn)

        move_down_btn = QPushButton("下移")
        move_down_btn.clicked.connect(self.move_field_down)
        field_btn_layout.addWidget(move_down_btn)

        field_btn_layout.addStretch()
        fields_layout.addLayout(field_btn_layout)

        fields_group.setLayout(fields_layout)
        layout.addWidget(fields_group, 1)

        # 模拟器配置区域
        sim_group = QGroupBox("模拟器配置")
        sim_layout = QVBoxLayout()

        self.sim_enabled_check = QCheckBox("启用模拟器")
        self.sim_enabled_check.setChecked(True)
        sim_layout.addWidget(self.sim_enabled_check)

        sim_rules_label = QLabel("生成规则 (格式: 字段名=规则, 例如: 帧头=fixed:0xA5)")
        sim_layout.addWidget(sim_rules_label)

        self.sim_rules_edit = QTextEdit()
        self.sim_rules_edit.setMaximumHeight(80)
        self.sim_rules_edit.setPlaceholderText("帧头=fixed:0xA5\n序列号=counter\n温度=random:200-300")
        sim_layout.addWidget(self.sim_rules_edit)

        sim_group.setLayout(sim_layout)
        layout.addWidget(sim_group)

        # 加载当前协议到编辑器
        self.load_protocol_to_editor()

        return widget

    def load_protocol_to_editor(self):
        """将当前协议加载到编辑器"""
        self.protocol_name_edit.setText(self.protocol.name)
        self.little_endian_radio.setChecked(True)

        # 加载字段列表
        self.fields_table.setRowCount(0)
        for field in self.protocol.fields:
            self.add_field_to_table(field)

        # 加载模拟器规则
        if self.protocol.simulator_rules:
            rules_text = "\n".join([f"{k}={v}" for k, v in self.protocol.simulator_rules.items()])
            self.sim_rules_edit.setPlainText(rules_text)

        self.update_total_size()

    def add_field_to_table(self, field: PacketField):
        """添加字段到表格"""
        row = self.fields_table.rowCount()
        self.fields_table.insertRow(row)

        self.fields_table.setItem(row, 0, QTableWidgetItem(field.name))
        self.fields_table.setItem(row, 1, QTableWidgetItem(field.field_type))
        self.fields_table.setItem(row, 2, QTableWidgetItem(str(field.size)))
        self.fields_table.setItem(row, 3, QTableWidgetItem(field.description))

        if field.validation:
            val_type = field.validation.get('type', '')
            self.fields_table.setItem(row, 4, QTableWidgetItem(val_type))

            # 格式化校验参数
            params = []
            for k, v in field.validation.items():
                if k != 'type':
                    params.append(f"{k}={v}")
            self.fields_table.setItem(row, 5, QTableWidgetItem(", ".join(params)))
        else:
            self.fields_table.setItem(row, 4, QTableWidgetItem(""))
            self.fields_table.setItem(row, 5, QTableWidgetItem(""))

    def update_total_size(self):
        """更新数据包总大小"""
        total = 0
        for row in range(self.fields_table.rowCount()):
            size_item = self.fields_table.item(row, 2)
            if size_item:
                total += int(size_item.text())
        self.total_size_label.setText(f"数据包大小: {total} 字节")

    def on_field_selected(self):
        """字段选中时的回调"""
        pass

    def add_field(self):
        """添加新字段"""
        dialog = FieldEditorDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            field = dialog.get_field()
            self.add_field_to_table(field)
            self.update_total_size()

    def edit_field(self):
        """编辑选中的字段"""
        selected_rows = self.fields_table.selectionModel().selectedRows()
        if not selected_rows:
            QMessageBox.warning(self, "提示", "请先选择要编辑的字段")
            return

        row = selected_rows[0].row()
        field = self.get_field_from_table(row)

        dialog = FieldEditorDialog(self, field)
        if dialog.exec_() == QDialog.Accepted:
            new_field = dialog.get_field()
            # 更新表格行
            self.fields_table.item(row, 0).setText(new_field.name)
            self.fields_table.item(row, 1).setText(new_field.field_type)
            self.fields_table.item(row, 2).setText(str(new_field.size))
            self.fields_table.item(row, 3).setText(new_field.description)

            if new_field.validation:
                self.fields_table.item(row, 4).setText(new_field.validation.get('type', ''))
                params = []
                for k, v in new_field.validation.items():
                    if k != 'type':
                        params.append(f"{k}={v}")
                self.fields_table.item(row, 5).setText(", ".join(params))
            else:
                self.fields_table.item(row, 4).setText("")
                self.fields_table.item(row, 5).setText("")

            self.update_total_size()

    def delete_field(self):
        """删除选中的字段"""
        selected_rows = self.fields_table.selectionModel().selectedRows()
        if not selected_rows:
            QMessageBox.warning(self, "提示", "请先选择要删除的字段")
            return

        row = selected_rows[0].row()
        field_name = self.fields_table.item(row, 0).text()

        reply = QMessageBox.question(self, "确认删除",
                                     f"确定要删除字段 '{field_name}' 吗？",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.fields_table.removeRow(row)
            self.update_total_size()

    def move_field_up(self):
        """上移字段"""
        selected_rows = self.fields_table.selectionModel().selectedRows()
        if not selected_rows:
            return

        row = selected_rows[0].row()
        if row > 0:
            self.swap_table_rows(row, row - 1)
            self.fields_table.selectRow(row - 1)

    def move_field_down(self):
        """下移字段"""
        selected_rows = self.fields_table.selectionModel().selectedRows()
        if not selected_rows:
            return

        row = selected_rows[0].row()
        if row < self.fields_table.rowCount() - 1:
            self.swap_table_rows(row, row + 1)
            self.fields_table.selectRow(row + 1)

    def swap_table_rows(self, row1, row2):
        """交换表格两行"""
        for col in range(self.fields_table.columnCount()):
            item1 = self.fields_table.takeItem(row1, col)
            item2 = self.fields_table.takeItem(row2, col)
            self.fields_table.setItem(row1, col, item2)
            self.fields_table.setItem(row2, col, item1)

    def get_field_from_table(self, row):
        """从表格行获取字段对象"""
        name = self.fields_table.item(row, 0).text()
        field_type = self.fields_table.item(row, 1).text()
        size = int(self.fields_table.item(row, 2).text())
        description = self.fields_table.item(row, 3).text()

        validation = None
        val_type_item = self.fields_table.item(row, 4)
        if val_type_item and val_type_item.text():
            validation = {'type': val_type_item.text()}
            val_params = self.fields_table.item(row, 5).text()
            if val_params:
                for param in val_params.split(", "):
                    if "=" in param:
                        k, v = param.split("=", 1)
                        # 尝试转换为适当的类型
                        if v.startswith("[") and v.endswith("]"):
                            validation[k] = eval(v)
                        elif v.isdigit():
                            validation[k] = int(v)
                        else:
                            validation[k] = v

        return PacketField(name, size, field_type, description, validation)

    def load_config_file(self):
        """加载配置文件"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "加载配置文件", "",
            "配置文件 (*.yaml *.yml *.json);;所有文件 (*.*)"
        )

        if file_path:
            try:
                self.protocol = PacketProtocol.load_from_config(file_path)
                self.config_path = file_path
                self.load_protocol_to_editor()
                QMessageBox.information(self, "成功", f"已加载配置文件:\n{file_path}")
            except Exception as e:
                QMessageBox.critical(self, "错误", f"加载配置文件失败:\n{str(e)}")

    def new_config(self):
        """新建配置"""
        reply = QMessageBox.question(self, "确认",
                                     "新建配置将清空当前编辑内容，是否继续？",
                                     QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.protocol = PacketProtocol("新协议")
            self.config_path = None
            self.protocol_name_edit.setText("新协议")
            self.protocol_desc_edit.setText("")
            self.fields_table.setRowCount(0)
            self.sim_rules_edit.clear()
            self.update_total_size()

    def save_config_file(self):
        """保存配置文件"""
        if not self.config_path:
            self.save_config_as()
        else:
            self.save_config_to_file(self.config_path)

    def save_config_as(self):
        """另存为配置文件"""
        file_path, _ = QFileDialog.getSaveFileName(
            self, "保存配置文件", "",
            "YAML文件 (*.yaml);;JSON文件 (*.json);;所有文件 (*.*)"
        )

        if file_path:
            self.save_config_to_file(file_path)
            self.config_path = file_path

    def save_config_to_file(self, file_path):
        """保存配置到文件"""
        try:
            # 从编辑器构建配置字典
            config = {
                'protocol_name': self.protocol_name_edit.text() or "未命名协议",
                'byte_order': 'little' if self.little_endian_radio.isChecked() else 'big',
                'fields': []
            }

            # 添加描述（如果有）
            if self.protocol_desc_edit.text():
                config['description'] = self.protocol_desc_edit.text()

            # 添加字段
            for row in range(self.fields_table.rowCount()):
                field = self.get_field_from_table(row)
                field_dict = {
                    'name': field.name,
                    'size': field.size,
                    'type': field.field_type,
                }
                if field.description:
                    field_dict['description'] = field.description
                if field.validation:
                    field_dict['validation'] = field.validation
                config['fields'].append(field_dict)

            # 添加模拟器配置
            if self.sim_enabled_check.isChecked():
                rules_text = self.sim_rules_edit.toPlainText().strip()
                if rules_text:
                    rules = {}
                    for line in rules_text.split('\n'):
                        line = line.strip()
                        if '=' in line:
                            key, value = line.split('=', 1)
                            rules[key.strip()] = value.strip()

                    config['simulator'] = {
                        'enabled': True,
                        'generate_rules': rules
                    }

            # 根据文件扩展名选择格式
            if file_path.endswith('.json'):
                with open(file_path, 'w', encoding='utf-8') as f:
                    json.dump(config, f, ensure_ascii=False, indent=2)
            else:
                with open(file_path, 'w', encoding='utf-8') as f:
                    yaml.dump(config, f, allow_unicode=True,
                             default_flow_style=False, sort_keys=False)

            QMessageBox.information(self, "成功", f"配置已保存到:\n{file_path}")

            # 重新加载到协议对象
            self.protocol = PacketProtocol.load_from_config(file_path)

        except Exception as e:
            QMessageBox.critical(self, "错误", f"保存配置文件失败:\n{str(e)}")

    def closeEvent(self, event):
        self.stop_connection()
        event.accept()


# ==================== 字段编辑对话框 ====================

class FieldEditorDialog(QDialog):
    """字段编辑对话框"""
    def __init__(self, parent=None, field: PacketField = None):
        super().__init__(parent)
        self.setWindowTitle("字段编辑器" if field else "添加字段")
        self.setModal(True)
        self.resize(500, 400)

        self.field = field
        self.init_ui()

        # 如果是编辑模式，加载字段数据
        if field:
            self.load_field_data()

    def init_ui(self):
        layout = QVBoxLayout()
        self.setLayout(layout)

        # 基本信息
        form_layout = QFormLayout()

        self.name_edit = QLineEdit()
        self.name_edit.setPlaceholderText("例: 帧头SOF")
        form_layout.addRow("字段名称*:", self.name_edit)

        self.type_combo = QComboBox()
        self.type_combo.addItems(["uint8", "uint16", "uint32", "int8", "int16", "int32", "float", "double"])
        self.type_combo.currentTextChanged.connect(self.on_type_changed)
        form_layout.addRow("数据类型*:", self.type_combo)

        self.size_spin = QSpinBox()
        self.size_spin.setRange(1, 8)
        self.size_spin.setValue(1)
        form_layout.addRow("字节数*:", self.size_spin)

        self.desc_edit = QLineEdit()
        self.desc_edit.setPlaceholderText("例: 起始标志(0xA5)")
        form_layout.addRow("描述:", self.desc_edit)

        layout.addLayout(form_layout)

        # 校验设置
        validation_group = QGroupBox("校验设置")
        validation_layout = QVBoxLayout()

        self.validation_check = QCheckBox("启用校验")
        self.validation_check.stateChanged.connect(self.on_validation_toggled)
        validation_layout.addWidget(self.validation_check)

        self.validation_type_combo = QComboBox()
        self.validation_type_combo.addItems(["fixed_value", "crc8", "crc16", "checksum"])
        self.validation_type_combo.currentTextChanged.connect(self.on_validation_type_changed)
        validation_layout.addWidget(QLabel("校验类型:"))
        validation_layout.addWidget(self.validation_type_combo)

        # 固定值参数
        self.value_widget = QWidget()
        value_layout = QHBoxLayout()
        value_layout.setContentsMargins(0, 0, 0, 0)
        value_layout.addWidget(QLabel("期望值:"))
        self.value_spin = QSpinBox()
        self.value_spin.setRange(0, 65535)
        value_layout.addWidget(self.value_spin)
        value_layout.addStretch()
        self.value_widget.setLayout(value_layout)
        validation_layout.addWidget(self.value_widget)

        # CRC/校验和范围参数
        self.range_widget = QWidget()
        range_layout = QHBoxLayout()
        range_layout.setContentsMargins(0, 0, 0, 0)
        range_layout.addWidget(QLabel("校验范围:"))
        range_layout.addWidget(QLabel("起始:"))
        self.range_start_spin = QSpinBox()
        self.range_start_spin.setRange(0, 1000)
        self.range_start_spin.setValue(0)
        range_layout.addWidget(self.range_start_spin)
        range_layout.addWidget(QLabel("结束:"))
        self.range_end_spin = QSpinBox()
        self.range_end_spin.setRange(-1, 1000)
        self.range_end_spin.setValue(-1)
        range_layout.addWidget(self.range_end_spin)
        range_layout.addWidget(QLabel("(-1=当前字段之前)"))
        range_layout.addStretch()
        self.range_widget.setLayout(range_layout)
        validation_layout.addWidget(self.range_widget)

        validation_group.setLayout(validation_layout)
        layout.addWidget(validation_group)

        # 按钮
        button_box = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        button_box.accepted.connect(self.validate_and_accept)
        button_box.rejected.connect(self.reject)
        layout.addWidget(button_box)

        # 初始状态
        self.validation_check.setChecked(False)
        self.on_validation_toggled(0)
        self.on_type_changed(self.type_combo.currentText())

    def on_type_changed(self, type_name):
        """类型改变时自动设置字节数"""
        size_map = {
            "uint8": 1, "int8": 1,
            "uint16": 2, "int16": 2,
            "uint32": 4, "int32": 4,
            "float": 4, "double": 8
        }
        if type_name in size_map:
            self.size_spin.setValue(size_map[type_name])

    def on_validation_toggled(self, state):
        """启用/禁用校验时的回调"""
        enabled = state == Qt.Checked
        self.validation_type_combo.setEnabled(enabled)
        self.on_validation_type_changed(self.validation_type_combo.currentText())

    def on_validation_type_changed(self, val_type):
        """校验类型改变时显示对应参数"""
        if not self.validation_check.isChecked():
            self.value_widget.setVisible(False)
            self.range_widget.setVisible(False)
            return

        if val_type == "fixed_value":
            self.value_widget.setVisible(True)
            self.range_widget.setVisible(False)
        elif val_type in ["crc8", "crc16", "checksum"]:
            self.value_widget.setVisible(False)
            self.range_widget.setVisible(True)
        else:
            self.value_widget.setVisible(False)
            self.range_widget.setVisible(False)

    def load_field_data(self):
        """加载字段数据到编辑器"""
        self.name_edit.setText(self.field.name)
        self.type_combo.setCurrentText(self.field.field_type)
        self.size_spin.setValue(self.field.size)
        self.desc_edit.setText(self.field.description or "")

        if self.field.validation:
            self.validation_check.setChecked(True)
            val_type = self.field.validation.get('type', '')
            self.validation_type_combo.setCurrentText(val_type)

            if val_type == 'fixed_value':
                self.value_spin.setValue(self.field.validation.get('value', 0))
            elif val_type in ['crc8', 'crc16', 'checksum']:
                val_range = self.field.validation.get('range', [0, -1])
                self.range_start_spin.setValue(val_range[0])
                self.range_end_spin.setValue(val_range[1])

    def validate_and_accept(self):
        """验证输入并接受"""
        if not self.name_edit.text().strip():
            QMessageBox.warning(self, "验证失败", "字段名称不能为空")
            return

        self.accept()

    def get_field(self) -> PacketField:
        """获取编辑后的字段"""
        name = self.name_edit.text().strip()
        field_type = self.type_combo.currentText()
        size = self.size_spin.value()
        description = self.desc_edit.text().strip()

        validation = None
        if self.validation_check.isChecked():
            val_type = self.validation_type_combo.currentText()
            validation = {'type': val_type}

            if val_type == 'fixed_value':
                validation['value'] = self.value_spin.value()
            elif val_type in ['crc8', 'crc16', 'checksum']:
                validation['range'] = [
                    self.range_start_spin.value(),
                    self.range_end_spin.value()
                ]

        return PacketField(name, size, field_type, description, validation)


# ==================== 主函数 ====================

def main():
    """主函数"""
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    print("=" * 50)
    print("数据包验证工具 v2.0")
    print("=" * 50)

    if not SERIAL_AVAILABLE:
        print("\n提示: 串口功能不可用,只能使用模拟模式")
        print("安装串口支持: pip install pyserial\n")

    # 检查命令行参数
    config_path = None
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
        print(f"加载配置文件: {config_path}")

    window = PacketValidatorGUI(config_path)
    window.show()

    if window.config_path:
        print(f"已加载协议: {window.protocol.name}")
        print(f"数据包大小: {window.protocol.total_size} 字节")
        print(f"字段数量: {len(window.protocol.fields)}")
    else:
        print("使用默认协议")

    print("=" * 50)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
