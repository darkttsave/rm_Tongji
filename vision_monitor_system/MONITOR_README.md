# RoboMaster 视觉系统实时监控工具

## 📋 功能概述

这是一个为同济大学SuperPower战队sp_vision_25项目开发的实时监控工具，可以在代码运行时以时间线方式可视化显示：

### 监控内容

**机器人自身状态：**
- 云台Yaw角度实时曲线
- 云台Pitch角度实时曲线  
- 射击指令状态（开火/停火）
- 控制激活状态
- 子弹速度

**目标状态数据：**
- 目标3D位置轨迹（X, Y, Z）
- 目标三维速度（Vx, Vy, Vz）
- 目标Yaw角度和角速度
- 目标距离变化
- 装甲板类型和编号
- EKF估计状态

**系统性能：**
- 数据包接收计数
- 实时FPS显示
- 数据更新频率

## 🚀 快速开始

### 1. 安装依赖

```bash
# Ubuntu 22.04
sudo apt update
sudo apt install python3 python3-pip

# 安装Python依赖
pip3 install numpy matplotlib
```

### 2. 运行监控程序

```bash
cd /d/sp_vision_25-main
python3 vision_monitor.py
```

### 3. 修改你的C++代码

在你的C++代码中添加数据发送功能。参考 `monitor_integration_example.cpp` 文件。

**最简单的集成方式：**

```cpp
#include "tools/plotter.hpp"
#include <nlohmann/json.hpp>

// 在main函数中创建plotter
tools::Plotter monitor_plotter("127.0.0.1", 9870);

// 在自瞄主循环中发送数据
while (running) {
    // ... 你的自瞄代码 ...
    
    // 准备监控数据
    nlohmann::json data;
    data["yaw"] = command.yaw;
    data["pitch"] = command.pitch;
    data["shoot"] = command.shoot;
    data["target_x"] = target_position_x;
    data["target_y"] = target_position_y;
    data["target_z"] = target_position_z;
    // ... 添加更多字段 ...
    
    // 发送数据
    monitor_plotter.plot(data);
}
```

### 4. 编译并运行C++程序

```bash
cd /d/sp_vision_25-main/sp_vision_25-main
make -C build/ -j$(nproc)
./build/standard  # 或者你的其他程序
```

## 📊 界面说明

监控界面分为9个区域：

```
┌─────────────────┬─────────────────┬─────────────────┐
│  云台Yaw曲线    │  云台Pitch曲线  │  目标3D轨迹     │
├─────────────────┼─────────────────┼─────────────────┤
│  目标Yaw/速度   │  目标三维速度   │  目标距离       │
├─────────────────┴─────────────────┴─────────────────┤
│              状态信息面板（实时数据）                │
└───────────────────────────────────────────────────────┘
```

**各图表说明：**

1. **云台Yaw曲线**：显示云台yaw轴的角度变化，用于观察云台跟踪效果
2. **云台Pitch曲线**：显示云台pitch轴的角度变化
3. **目标3D轨迹**：在三维空间中显示目标运动轨迹
4. **目标Yaw/速度**：双Y轴图，显示目标yaw角度和角速度
5. **目标三维速度**：显示目标在XYZ三个方向的速度分量
6. **目标距离**：显示目标与机器人的距离变化
7. **状态信息面板**：以表格形式显示所有实时数据

## 🔧 配置说明

### 修改网络配置

如果需要修改UDP端口或IP地址，编辑 `monitor_config.json`：

```json
{
  "network": {
    "host": "127.0.0.1",  // 监听地址
    "port": 9870          // 监听端口
  }
}
```

或者直接在代码中修改：

**Python端：**
```python
monitor = VisionMonitor(host='192.168.1.100', port=9999)
```

**C++端：**
```cpp
tools::Plotter plotter("192.168.1.100", 9999);
```

### 自定义监控字段

在 `monitor_config.json` 中定义你需要监控的数据字段：

```json
{
  "data_fields": {
    "robot_state": [
      "yaw",
      "pitch", 
      "shoot",
      "你的自定义字段"
    ],
    "target_state": [
      "target_x",
      "你的自定义字段"
    ]
  }
}
```

然后在C++代码中发送对应的字段：

```cpp
nlohmann::json data;
data["你的自定义字段"] = 你的数据;
plotter.plot(data);
```

## 💡 使用场景

### 场景1：调试云台跟踪效果

1. 启动监控程序
2. 运行自瞄程序
3. 观察"云台Yaw曲线"和"目标Yaw/速度"的对比
4. 评估云台是否能够跟上目标运动

### 场景2：分析目标运动模式

1. 让敌方机器人做小陀螺运动
2. 观察"目标3D轨迹"和"目标Yaw/速度"
3. 分析角速度是否符合预期
4. 检查EKF估计是否收敛

### 场景3：验证弹道预测

1. 添加瞄准点数据到监控
2. 对比"目标位置"和"瞄准点位置"
3. 评估预测时间是否合理

### 场景4：性能分析

1. 添加各模块处理时间
2. 观察FPS和延迟
3. 找出性能瓶颈

## 📝 常见问题

### Q1: 监控程序启动后没有数据显示？

**A:** 检查以下几点：
1. C++程序是否正在运行？
2. C++程序是否调用了 `plotter.plot(data)`？
3. 网络配置是否正确（IP和端口）？
4. 防火墙是否阻止了UDP通信？

测试方法：
```bash
# 手动发送测试数据
echo '{"yaw": 0.5, "pitch": 0.3}' | nc -u 127.0.0.1 9870
```

### Q2: 图表更新很卡顿？

**A:** 可能的原因：
1. 数据发送频率太高（建议控制在50-100Hz）
2. 历史数据长度太长（修改 `history_length` 参数）
3. 系统性能不足

优化方法：
```python
# 减少历史数据长度
deque(maxlen=200)  # 从500改为200

# 降低更新频率
FuncAnimation(self.fig, self.update, interval=100)  # 从50ms改为100ms
```

### Q3: 如何保存监控数据？

**A:** 有两种方式：

方式1：在C++中直接记录
```cpp
std::ofstream log_file("monitor_log.json", std::ios::app);
log_file << data.dump() << std::endl;
```

方式2：在Python中记录（修改 `_process_packet` 方法）
```python
def _process_packet(self, data: bytes):
    # ... 原有代码 ...
    with open('monitor_log.json', 'a') as f:
        f.write(data.decode('utf-8') + '\n')
```

### Q4: 能否同时监控多台机器人？

**A:** 可以，两种方案：

方案1：不同端口
```cpp
// 机器人1
tools::Plotter plotter1("127.0.0.1", 9870);

// 机器人2  
tools::Plotter plotter2("127.0.0.1", 9871);
```

然后启动两个监控程序实例。

方案2：在数据中添加机器人ID
```cpp
data["robot_id"] = 1;
```

修改监控程序按ID分类显示。

## 🛠️ 进阶功能

### 添加自定义图表

在 `RealtimePlotter` 类中添加新的子图：

```python
def _init_plots(self):
    # ... 原有代码 ...
    
    # 添加新图表
    self.ax_custom = self.fig.add_subplot(gs[2, 2])
    self.ax_custom.set_title('自定义数据')
    self.line_custom, = self.ax_custom.plot([], [], 'orange')

def update(self, frame):
    # ... 原有代码 ...
    
    # 更新自定义图表
    if len(self.custom_data) > 0:
        self.line_custom.set_data(times, self.custom_data)
```

### 添加报警功能

```python
def _process_packet(self, data: bytes):
    # ... 原有代码 ...
    
    # 检查异常情况
    if abs(self.target_state.yaw_velocity) > 20:  # 角速度过高
        print(f"⚠️ 警告：目标角速度异常 {self.target_state.yaw_velocity:.2f} rad/s")
    
    if self.target_state.distance < 1.0:  # 距离过近
        print(f"⚠️ 警告：目标距离过近 {self.target_state.distance:.2f} m")
```

### 数据回放功能

保存数据后可以离线回放：

```python
def replay_from_file(filename):
    """从日志文件回放数据"""
    monitor = VisionMonitor()
    plotter = RealtimePlotter(monitor)
    
    with open(filename, 'r') as f:
        for line in f:
            data = json.loads(line)
            monitor._process_packet(line.encode('utf-8'))
            time.sleep(0.02)  # 50Hz回放
    
    plotter.start()
```

## 📚 数据字段参考

### 完整的JSON数据格式示例

```json
{
  "timestamp": 1234567890.123,
  "frame_count": 12345,
  
  "yaw": 0.523,
  "pitch": -0.174,
  "shoot": true,
  "control": true,
  "bullet_speed": 28.5,
  
  "target_x": 2.35,
  "target_y": 0.15,
  "target_z": 0.45,
  "target_vx": 0.12,
  "target_vy": -0.05,
  "target_vz": 0.0,
  "target_yaw": 1.57,
  "target_yaw_velocity": 12.5,
  
  "armor_type": "small",
  "armor_name": "three",
  "distance": 2.38,
  
  "ekf_converged": true,
  "tracking_state": "locked"
}
```

### EKF状态向量说明

根据你的项目，EKF状态向量可能包含：

```
ekf_state[0]: 目标中心X坐标 (m)
ekf_state[1]: 目标中心Y坐标 (m)  
ekf_state[2]: 目标中心Z坐标 (m)
ekf_state[3]: X方向速度 (m/s)
ekf_state[4]: Y方向速度 (m/s)
ekf_state[5]: Z方向速度 (m/s)
ekf_state[6]: Yaw角度 (rad)
ekf_state[7]: Yaw角速度 (rad/s)
ekf_state[8+]: 各装甲板半径和高度差等
```

具体定义请参考 `tasks/auto_aim/target.hpp`。

## 📞 技术支持

如有问题或建议，请联系：
- 同济大学SuperPower战队算法组
- GitHub: https://github.com/TongjiSuperPower/sp_vision_25

## 📄 许可证

本工具遵循与sp_vision_25项目相同的许可证。

---

**祝调试顺利！🎯**
