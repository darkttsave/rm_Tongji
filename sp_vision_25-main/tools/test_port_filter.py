#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试串口过滤功能
验证蓝牙串口是否被正确过滤
"""

try:
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("错误: 未安装pyserial库")
    print("安装命令: pip install pyserial")
    exit(1)

def test_port_filter():
    """测试串口过滤逻辑"""
    print("=" * 60)
    print("串口过滤测试")
    print("=" * 60)

    ports = serial.tools.list_ports.comports()

    print(f"\n检测到的所有串口 ({len(ports)} 个):")
    print("-" * 60)
    for port in ports:
        print(f"端口: {port.device}")
        print(f"  描述: {port.description}")
        print(f"  HWID: {port.hwid}")
        print(f"  是否蓝牙: {'是' if 'BTHENUM' in port.hwid.upper() else '否'}")
        print()

    # 应用过滤逻辑
    virtual_ports = []
    for port in ports:
        hwid = port.hwid.upper()
        if 'BTHENUM' not in hwid:
            virtual_ports.append(port)

    print("=" * 60)
    print(f"过滤后的虚拟串口 ({len(virtual_ports)} 个):")
    print("-" * 60)
    if len(virtual_ports) == 0:
        print("未找到虚拟串口（所有检测到的串口都是蓝牙串口）")
    else:
        for port in virtual_ports:
            print(f"  {port.device} - {port.description}")

    print("\n" + "=" * 60)
    print("测试完成")
    print("=" * 60)

if __name__ == "__main__":
    test_port_filter()
