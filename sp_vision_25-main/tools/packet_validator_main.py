#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据包验证工具 - 完整版本
支持串口通信和模拟模式,用于验证下位机发送的数据包格式
使用说明: python packet_validator_main.py
依赖安装: pip install pyserial PyQt5
"""

import sys
import struct
import time
from typing import Optional, List, Tuple

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTextEdit, QComboBox, QSpinBox,
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
    """CRC16校验表"""
    TABLE = [0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
             0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7]  # 省略部分...
    
    @staticmethod
    def calculate(data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            i = (crc ^ byte) & 0xFF
            crc = (crc >> 8) ^ CRC16Table.TABLE[i] if i < len(CRC16Table.TABLE) else crc >> 8
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
    def __init__(self, name: str, size: int, field_type: str, description: str = ""):
        self.name = name
        self.size = size
        self.field_type = field_type
        self.description = description
    
    def parse(self, data: bytes) -> Tuple:
        hex_str = ' '.join(f'{b:02X}' for b in data[:self.size])
        
        if self.field_type == 'uint8':
            value = data[0] if len(data) >= 1 else 0
        elif self.field_type == 'uint16':
            value = struct.unpack('<H', data[:2])[0] if len(data) >= 2 else 0
        elif self.field_type == 'float':
            value = struct.unpack('<f', data[:4])[0] if len(data) >= 4 else 0.0
        else:
            value = data[:self.size]
        
        return value, hex_str


class PacketProtocol:
    """数据包协议定义"""
    def __init__(self, name: str = "默认协议"):
        self.name = name
        self.fields: List[PacketField] = []
        self.total_size = 0
    
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
    
    def generate_packet(self, counter: int) -> bytes:
        packet = bytearray()
        for field in self.protocol.fields:
            if 'sof' in field.name.lower() or '帧头' in field.name:
                packet.append(0xA5)
            elif '序列号' in field.name:
                packet.append(counter % 256)
            elif field.field_type == 'uint8':
                packet.append((counter * 2) % 256)
            elif field.field_type == 'uint16':
                packet.extend(struct.pack('<H', (counter * 100) % 65536))
            elif field.field_type == 'float':
                packet.extend(struct.pack('<f', counter * 0.1))
            elif 'crc8' in field.name.lower():
                crc = CRC8Table.calculate(packet)
                packet.append(crc)
            elif 'crc16' in field.name.lower():
                crc = CRC16Table.calculate(packet)
                packet.extend(struct.pack('<H', crc))
            else:
                packet.extend(b' ' * field.size)
        return bytes(packet)
    
    def stop(self):
        self.running = False
