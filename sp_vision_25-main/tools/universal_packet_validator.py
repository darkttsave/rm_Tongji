#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
通用数据包验证工具 - 配置文件版本
支持JSON配置文件,可适配各种项目的通信协议
使用说明: python universal_packet_validator.py [配置文件路径]
依赖安装: pip install pyserial PyQt5
"""

import sys
import struct
import time
import json
import random
from pathlib import Path
from typing import Optional, List, Tuple, Dict, Any

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTextEdit, QComboBox, QSpinBox, QFileDialog,
    QGroupBox, QRadioButton, QTableWidget, QTableWidgetItem,
    QHeaderView, QMessageBox
)
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QColor, QFont

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ==================== CRC校验类 ====================

class CRC8Table:
    """CRC8校验表"""
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
    """CRC16校验表"""
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


class ChecksumValidator:
    """简单累加校验"""
    @staticmethod
    def calculate(data: bytes) -> int:
        return sum(data) & 0xFF

    @staticmethod
    def verify(data: bytes) -> bool:
        if len(data) < 2:
            return False
        return ChecksumValidator.calculate(data[:-1]) == data[-1]


# ==================== 配置加载器 ====================

class ProtocolConfig:
    """协议配置加载器"""
    def __init__(self, config_path: str):
        self.config_path = config_path
        self.config = self.load_config()

    def load_config(self) -> Dict[str, Any]:
        """加载JSON配置文件"""
        try:
            with open(self.config_path, 'r', encoding='utf-8') as f:
                config = json.load(f)
            return config
        except Exception as e:
            raise Exception(f"加载配置文件失败: {str(e)}")

    def get_protocol_name(self) -> str:
        return self.config.get('protocol_name', '未命名协议')

    def get_description(self) -> str:
        return self.config.get('description', '')

    def get_frame_header(self) -> int:
        """获取帧头值"""
        header_str = self.config.get('frame_header', '0xA5')
        return int(header_str, 16) if header_str.startswith('0x') else int(header_str)

    def get_fields(self) -> List[Dict]:
        return self.config.get('fields', [])

    def get_simulator_config(self) -> Dict:
        return self.config.get('simulator', {})


# ==================== 数据包字段和协议类 ====================

class PacketField:
    """数据包字段定义"""
    def __init__(self, config: Dict):
        self.name = config['name']
        self.size = config['size']
        self.field_type = config['type']
        self.description = config.get('description', '')
        self.validation = config.get('validation', {})

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
    """数据包协议"""
    def __init__(self, config: ProtocolConfig):
        self.config = config
        self.name = config.get_protocol_name()
        self.fields: List[PacketField] = []
        self.total_size = 0
        self._load_fields()

    def _load_fields(self):
        for field_config in self.config.get_fields():
            field = PacketField(field_config)
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
            results.append((field.name, value, hex_str, field))
            offset += field.size

        return True, results, ""

    def validate_packet(self, packet_data: bytes, results: List) -> Tuple[bool, List[str]]:
        """根据配置验证数据包"""
        is_valid = True
        errors = []

        offset = 0
        for i, field in enumerate(self.fields):
            if not field.validation:
                offset += field.size
                continue

            val_type = field.validation.get('type')

            # 固定值验证
            if val_type == 'fixed_value':
                expected = field.validation.get('value')
                actual = results[i][1]
                if actual != expected:
                    is_valid = False
                    errors.append(f"{field.name}值错误(期望:{expected}, 实际:{actual})")

            # CRC8验证
            elif val_type == 'crc8':
                val_range = field.validation.get('range', [0, -1])
                start = val_range[0]
                end = val_range[1] if val_range[1] != -1 else offset + field.size
                if not CRC8Table.verify(packet_data[start:end]):
                    is_valid = False
                    errors.append(f"{field.name}校验失败")

            # CRC16验证
            elif val_type == 'crc16':
                val_range = field.validation.get('range', [0, -1])
                start = val_range[0]
                end = len(packet_data) if val_range[1] == -1 else val_range[1]
                if not CRC16Table.verify(packet_data[start:end]):
                    is_valid = False
                    errors.append(f"{field.name}校验失败")

            # 简单校验和
            elif val_type == 'checksum':
                val_range = field.validation.get('range', [0, -1])
                start = val_range[0]
                end = len(packet_data) if val_range[1] == -1 else val_range[1]
                if not ChecksumValidator.verify(packet_data[start:end]):
                    is_valid = False
                    errors.append(f"{field.name}校验失败")

            offset += field.size

        return is_valid, errors


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
        self.counter = 0
        self.sim_config = protocol.config.get_simulator_config()

    def run(self):
        self.running = True
        while self.running:
            packet = self.generate_packet()
            self.data_generated.emit(packet)
            self.counter += 1
            time.sleep(self.interval)

    def generate_packet(self) -> bytes:
        """根据配置生成模拟数据包"""
        packet = bytearray()
        generate_rules = self.sim_config.get('generate_rules', {})

        for field in self.protocol.fields:
            rule = generate_rules.get(field.name, '')

            if rule.startswith('fixed:'):
                # 固定值
                value_str = rule.split(':')[1]
                value = int(value_str, 16) if value_str.startswith('0x') else int(value_str)
                if field.field_type == 'uint8':
                    packet.append(value)
                elif field.field_type == 'uint16':
                    packet.extend(struct.pack('<H', value))

            elif rule == 'counter':
                # 计数器
                if field.field_type == 'uint8':
                    packet.append(self.counter % 256)
                elif field.field_type == 'uint16':
                    packet.extend(struct.pack('<H', self.counter % 65536))

            elif rule.startswith('formula:'):
                # 公式计算
                formula = rule.split(':', 1)[1]
                try:
                    value = eval(formula.replace('counter', str(self.counter)))
                    if field.field_type == 'float':
                        packet.extend(struct.pack('<f', float(value)))
                    elif field.field_type == 'double':
                        packet.extend(struct.pack('<d', float(value)))
                except:
                    packet.extend(b'\x00' * field.size)

            elif rule.startswith('random:'):
                # 随机值
                range_str = rule.split(':')[1]
                min_val, max_val = map(float, range_str.split('-'))
                if field.field_type == 'uint8':
                    packet.append(random.randint(int(min_val), int(max_val)))
                elif field.field_type == 'uint16':
                    packet.extend(struct.pack('<H', random.randint(int(min_val), int(max_val))))
                elif field.field_type == 'float':
                    packet.extend(struct.pack('<f', random.uniform(min_val, max_val)))

            # 自动生成校验码
            elif 'crc8' in field.name.lower():
                crc = CRC8Table.calculate(packet)
                packet.append(crc)

            elif 'crc16' in field.name.lower():
                crc = CRC16Table.calculate(packet)
                packet.extend(struct.pack('<H', crc))

            elif 'checksum' in field.name.lower() or '校验和' in field.name:
                checksum = ChecksumValidator.calculate(packet)
                packet.append(checksum)

            else:
                # 默认填充
                if field.field_type == 'uint8':
                    packet.append((self.counter * 2) % 256)
                elif field.field_type == 'uint16':
                    packet.extend(struct.pack('<H', (self.counter * 100) % 65536))
                elif field.field_type == 'float':
                    packet.extend(struct.pack('<f', self.counter * 0.1))
                else:
                    packet.extend(b'\x00' * field.size)

        return bytes(packet)

    def stop(self):
        self.running = False


# ==================== 主界面类(后续实现) ====================
# 由于代码较长,主界面类将在下一个文件中实现
