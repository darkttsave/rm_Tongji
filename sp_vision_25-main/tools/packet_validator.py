#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
数据包验证工具 - 支持串口通信和模拟模式
"""
import sys
import struct
import time
from typing import Optional, List, Tuple
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QPushButton, QTextEdit, 
                             QComboBox, QSpinBox, QGroupBox, QRadioButton, 
                             QTableWidget, QTableWidgetItem, QHeaderView, QMessageBox)
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QColor, QFont

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

print("Data Packet Validator Tool v1.0")
print("==============================")
if not SERIAL_AVAILABLE:
    print("Warning: pyserial not installed. Serial port features unavailable.")
    print("Install with: pip install pyserial")
