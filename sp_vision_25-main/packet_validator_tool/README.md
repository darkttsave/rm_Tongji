# 通用数据包验证工具

一个基于配置驱动的通用串口数据包验证工具，支持YAML/JSON配置，适配任何项目的通信协议。

---

## 📋 文档导航

- **[使用指南.md](使用指南.md)** - 完整使用教程，包含快速开始、界面说明、故障排除
- **[配置参考.md](配置参考.md)** - 配置文件完整说明，包含所有字段、选项和高级用法
- **[更新日志.md](更新日志.md)** - 版本历史、问题修复和技术报告

---

## ✨ 主要特性

- ✅ **配置驱动** - 无需修改代码，通过YAML/JSON配置适配新协议
- ✅ **双模式** - 串口模式（真实硬件）+ 模拟模式（无需硬件测试）
- ✅ **多种校验** - CRC8、CRC16、简单校验和、固定值验证
- ✅ **图形界面** - PyQt5界面，实时显示和统计
- ✅ **灵活模拟器** - 支持固定值、计数器、随机数、公式计算
- ✅ **蓝牙过滤** - 自动过滤蓝牙串口，只显示虚拟串口和硬件串口
- ✅ **100%校验** - v2.0修复了模拟模式CRC校验问题

---

## 🚀 快速开始

### 1. 安装依赖

```bash
pip install PyQt5 pyserial pyyaml
```

### 2. 运行工具

```bash
# 使用YAML配置（推荐）
python universal_packet_validator.py protocol_configs/rm_vision_protocol.yaml

# 使用JSON配置
python universal_packet_validator.py protocol_configs/rm_vision_protocol.json

# 不带参数启动，在GUI中加载配置
python universal_packet_validator.py
```

### 3. 基本使用

1. **选择工作模式**：串口模式或模拟模式
2. **串口模式**：选择COM口和波特率，点击"启动"
3. **模拟模式**：设置发送间隔，点击"启动"
4. **查看结果**：绿色=验证通过，红色=验证失败

---

## 📦 目录结构

```
packet_validator_tool/
├── universal_packet_validator.py       # 主程序
├── protocol_configs/                   # 协议配置文件目录
│   ├── rm_vision_protocol.yaml        # RM视觉协议(YAML)
│   ├── rm_vision_protocol.json        # RM视觉协议(JSON)
│   ├── example_protocol.yaml          # 示例协议
│   └── simple_sensor_protocol.yaml    # 简单传感器协议
├── README.md                          # 本文档
├── 使用指南.md                         # 完整使用教程
├── 配置参考.md                         # 配置文件完整说明
└── 更新日志.md                         # 版本历史和技术报告
```

---

## 📝 最简配置示例

```yaml
protocol_name: 我的协议
byte_order: little

fields:
  - name: 帧头
    size: 1
    type: uint8
    validation:
      type: fixed_value
      value: 165  # 0xA5

  - name: 温度
    size: 2
    type: int16

  - name: 湿度
    size: 2
    type: uint16

  - name: 校验和
    size: 1
    type: uint8
    validation:
      type: checksum
      range: [0, 5]

simulator:
  enabled: true
  generate_rules:
    帧头: "fixed:0xA5"
    温度: "random:200-300"
    湿度: "random:400-600"
```

---

## 🎯 使用场景

### 场景1：验证Arduino通信协议
创建配置文件，描述Arduino发送的数据包格式，工具自动验证每个数据包的正确性。

### 场景2：开发新的通信协议
使用模拟模式快速测试协议设计是否合理，无需编写下位机代码。

### 场景3：跨项目协议验证
在任意项目目录创建YAML配置文件，运行工具即可验证该项目的通信协议。

---

## 🛠️ 支持的功能

### 数据类型
- `uint8`, `uint16`, `uint32` - 无符号整数
- `int8`, `int16`, `int32` - 有符号整数
- `float`, `double` - 浮点数

### 校验算法
- `fixed_value` - 固定值验证（帧头）
- `crc8` - CRC8校验
- `crc16` - CRC16校验
- `checksum` - 简单校验和

### 模拟器规则
- `fixed:0xA5` - 固定值
- `counter` - 递增计数
- `random:0-100` - 随机整数
- `random:0.5-10.0` - 随机浮点数
- `formula:counter*0.1` - 公式计算

---

## ❓ 常见问题

### Q: 如何验证其他项目的协议？
**A**: 在任意位置创建配置文件描述协议格式，使用绝对路径或相对路径运行工具即可。

### Q: YAML还是JSON？
**A**: YAML更易读易写，支持注释，推荐使用。JSON更严格，两者功能完全相同。

### Q: 支持哪些串口？
**A**: 支持虚拟串口和真实USB串口，自动过滤蓝牙串口。

### Q: 模拟模式有什么用？
**A**: 无需硬件即可测试协议配置是否正确，验证工具功能，快速开发调试。

---

## 📚 详细文档

- **[使用指南.md](使用指南.md)** - 详细的使用教程
  - 界面说明
  - 配置文件入门
  - 创建自定义协议
  - 故障排除
  - 高级技巧

- **[配置参考.md](配置参考.md)** - 完整的配置手册
  - 所有配置项说明
  - 验证类型详解
  - 模拟器规则
  - 完整示例
  - FAQ

- **[更新日志.md](更新日志.md)** - 版本历史和技术细节
  - 版本历史
  - CRC校验问题修复报告
  - 串口过滤实现报告
  - 测试指南
  - 性能指标

---

## 🔄 版本信息

**当前版本**: v2.0  
**发布日期**: 2026-09-26  
**状态**: ✅ 生产就绪

### v2.0 主要更新
- ✅ 修复模拟模式CRC校验100%失败问题
- ✅ 添加YAML配置支持
- ✅ 添加蓝牙串口自动过滤
- ✅ 完善文档体系

---

## 📞 获取帮助

遇到问题？
1. 查看 [使用指南.md](使用指南.md) 的故障排除部分
2. 阅读 [配置参考.md](配置参考.md) 的FAQ
3. 查看 [更新日志.md](更新日志.md) 了解已知问题
4. 在项目中提交Issue

---

## 📄 许可

本工具为开源项目，可自由使用和修改。

---

**快速链接**:
- 30秒上手 → [使用指南.md](使用指南.md#快速开始)
- 编写配置 → [配置参考.md](配置参考.md#配置文件结构)
- 问题排查 → [使用指南.md](使用指南.md#故障排除)
