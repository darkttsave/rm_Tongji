# 通用数据包验证工具

一个强大的跨项目通信协议验证工具，支持串口/虚拟串口数据包的实时监听、解析和校验。

## ✨ 特性

- 🔌 **支持虚拟串口** - 自动过滤蓝牙串口，只显示虚拟串口和真实硬件串口
- 📝 **配置驱动** - 使用JSON/YAML配置文件描述协议，无需修改代码
- 🔍 **实时验证** - 支持CRC8/CRC16/校验和等多种校验算法
- 🎨 **图形界面** - 基于PyQt5的直观GUI
- 🧪 **内置模拟器** - 自动生成测试数据包
- 🚀 **跨项目使用** - 可验证任意文件夹内的项目通信协议

## 📦 依赖

```bash
pip install pyserial PyQt5 pyyaml
```

## 🚀 快速开始

### 1. 最简单的方式（30秒上手）

```bash
# 复制示例配置
cp protocol_configs/example_protocol.yaml my_protocol.yaml

# 编辑配置文件，修改为你的协议格式
# 然后运行
python universal_packet_validator.py my_protocol.yaml
```

### 2. 验证其他项目的协议

```bash
# 在你的项目目录创建配置文件
cd /path/to/your/project
# 创建 my_protocol.yaml

# 运行工具（使用绝对路径）
python /path/to/tools/universal_packet_validator.py my_protocol.yaml
```

### 3. 不带参数启动（GUI加载）

```bash
python universal_packet_validator.py
# 在GUI中点击"加载配置"选择配置文件
```

## 📖 文档

| 文档 | 说明 |
|------|------|
| [快速开始.md](快速开始.md) | 30秒快速上手指南 |
| [配置文件使用指南.md](配置文件使用指南.md) | 完整的配置文件编写教程 |
| [CHANGELOG_串口过滤.md](CHANGELOG_串口过滤.md) | 蓝牙串口过滤功能说明 |

## 📄 配置文件示例

### 最简配置（YAML格式）

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

  - name: 数据
    size: 4
    type: float

  - name: CRC校验
    size: 1
    type: uint8
    validation:
      type: crc8
      range: [0, 4]

simulator:
  enabled: true
  generate_rules:
    帧头: "fixed:0xA5"
    数据: "random:0.0-100.0"
```

### 完整示例

查看 [protocol_configs/example_protocol.yaml](protocol_configs/example_protocol.yaml) 了解所有功能。

## 🎯 使用场景

### 场景1：Arduino串口通信验证
```yaml
# arduino_sensor.yaml
protocol_name: Arduino传感器
fields:
  - name: 帧头
    size: 1
    type: uint8
  - name: 温度
    size: 2
    type: int16
  - name: 湿度
    size: 2
    type: uint16
  - name: 校验和
    size: 1
    type: uint8
```

### 场景2：机器人控制协议
```yaml
# robot_control.yaml
protocol_name: 机器人控制
fields:
  - name: 帧头
    size: 1
    type: uint8
  - name: 电机ID
    size: 1
    type: uint8
  - name: 速度
    size: 2
    type: int16
  - name: CRC16
    size: 2
    type: uint16
    validation:
      type: crc16
      range: [0, -1]
```

### 场景3：RoboMaster视觉系统
参考 [protocol_configs/rm_vision_protocol.json](protocol_configs/rm_vision_protocol.json)

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
- `range` - 数值范围验证

### 模拟器规则
- `fixed:0xA5` - 固定值
- `counter` - 递增计数
- `random:0-100` - 随机整数
- `random:0.5-10.0` - 随机浮点数
- `formula:counter*0.1` - 公式计算

## 🔧 工具集

### 主工具
- `universal_packet_validator.py` - 主验证工具
- `packet_validator_complete.py` - 完整版验证工具

### 辅助工具
- `test_port_filter.py` - 串口过滤测试
- `protocol_configs/` - 配置文件目录

## 📊 目录结构

```
tools/
├── universal_packet_validator.py   # 主程序
├── packet_validator_complete.py    # 完整版
├── protocol_configs/               # 配置文件目录
│   ├── rm_vision_protocol.json    # RM视觉协议
│   └── example_protocol.yaml      # 示例配置
├── 快速开始.md                     # 快速入门
├── 配置文件使用指南.md             # 详细文档
├── CHANGELOG_串口过滤.md          # 更新日志
└── test_port_filter.py            # 测试工具
```

## ❓ 常见问题

### Q: 如何验证其他项目的协议？
**A**: 创建配置文件描述协议格式，使用绝对路径运行工具即可。配置文件可以放在任何位置。

### Q: 支持哪些串口？
**A**: 支持虚拟串口和真实USB串口，自动过滤蓝牙串口。

### Q: JSON还是YAML？
**A**: YAML更易读易写，支持注释，推荐使用。JSON更严格，两者功能完全相同。

### Q: 如何调试配置文件？
**A**: 运行 `python -c "import yaml; print(yaml.safe_load(open('your.yaml')))"` 检查语法。

### Q: 支持动态长度的数据包吗？
**A**: 支持，定义"数据长度"字段，工具会自动解析。

## 🔍 测试

### 测试串口过滤
```bash
python test_port_filter.py
```

### 测试配置文件加载
```bash
python -c "
import yaml
with open('protocol_configs/example_protocol.yaml') as f:
    config = yaml.safe_load(f)
    print(f'协议: {config[\"protocol_name\"]}')
    print(f'字段数: {len(config[\"fields\"])}')
"
```

### 测试工具启动
```bash
python universal_packet_validator.py protocol_configs/example_protocol.yaml
```

## 📝 示例配置文件

工具提供了完整的示例配置：

1. **JSON格式**: [rm_vision_protocol.json](protocol_configs/rm_vision_protocol.json)
   - RoboMaster视觉系统协议
   - 包含CRC8和CRC16校验
   - 展示浮点数和整数混合使用

2. **YAML格式**: [example_protocol.yaml](protocol_configs/example_protocol.yaml)
   - 完整功能演示
   - 包含注释说明
   - 展示所有校验类型

## 🚨 注意事项

1. **串口权限**: Windows可能需要管理员权限访问某些串口
2. **波特率设置**: 确保工具和设备使用相同的波特率
3. **字节序**: 大多数嵌入式系统使用小端序（`little`）
4. **CRC算法**: 确认使用的CRC算法与设备端一致

## 📜 更新日志

### 2026-09-26
- ✅ 修复JSON配置文件注释问题
- ✅ 添加蓝牙串口过滤功能（BTHENUM检测）
- ✅ 创建YAML配置示例
- ✅ 完善使用文档

## 🤝 贡献

欢迎提交问题和改进建议！

## 📄 许可

本工具为开源项目，可自由使用和修改。

---

**开始使用**: 阅读 [快速开始.md](快速开始.md) 在30秒内上手！
