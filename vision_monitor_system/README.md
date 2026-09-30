# RoboMaster 视觉系统实时监控工具

## 📁 文件结构

```
vision_monitor_system/
├── README.md                           # 本文件
├── 监控系统使用指南.md                 # 中文快速上手指南 ⭐
├── MONITOR_README.md                   # 详细英文文档
│
├── vision_monitor.py                   # 核心监控程序（Python）⭐
├── test_data_sender.py                 # 测试数据发送器
├── monitor_integration_example.cpp     # C++集成示例代码 ⭐
│
├── monitor_config.json                 # 配置文件
├── install_monitor.sh                  # 自动安装脚本
└── start_monitor_test.sh               # 一键测试脚本
```

**⭐ 标记的文件是最重要的**

## 🚀 三步快速开始

### 第一步：安装依赖

```bash
cd /d/sp_vision_25-main/vision_monitor_system
./install_monitor.sh
```

或手动安装：
```bash
pip3 install numpy matplotlib
```

### 第二步：测试监控系统

**方式1 - 一键测试（推荐）：**
```bash
./start_monitor_test.sh
```

**方式2 - 分步测试：**

终端1 - 启动监控：
```bash
python3 vision_monitor.py
```

终端2 - 发送测试数据：
```bash
python3 test_data_sender.py --scenario normal
```

### 第三步：集成到你的C++代码

参考 `monitor_integration_example.cpp`，在你的代码中添加：

```cpp
#include "tools/plotter.hpp"
#include <nlohmann/json.hpp>

// 在main函数初始化
tools::Plotter monitor("127.0.0.1", 9870);

// 在主循环中发送数据
nlohmann::json data;
data["yaw"] = command.yaw;
data["pitch"] = command.pitch;
data["target_x"] = target_pos_x;
data["target_y"] = target_pos_y;
data["target_z"] = target_pos_z;
// ... 添加更多字段

monitor.plot(data);
```

## 📊 监控内容

### 可视化图表（9个区域）

1. **云台Yaw曲线** - 云台yaw角度实时变化
2. **云台Pitch曲线** - 云台pitch角度实时变化
3. **目标3D轨迹** - 目标在空间中的运动轨迹
4. **目标Yaw/角速度** - 双Y轴显示角度和角速度
5. **目标三维速度** - Vx, Vy, Vz实时曲线
6. **目标距离** - 与目标的距离变化
7. **状态信息面板** - 实时数值显示

### 监控数据字段

**机器人状态：**
- 云台Yaw/Pitch角度
- 射击指令状态
- 控制激活状态
- 子弹速度

**目标状态：**
- 3D位置（X, Y, Z）
- 三维速度（Vx, Vy, Vz）
- Yaw角度和角速度
- 装甲板类型和编号
- 目标距离
- EKF估计状态

## 💡 使用场景

### 场景1：调试云台跟踪效果
观察云台Yaw曲线是否能跟上目标运动

### 场景2：验证目标运动预测
检查目标3D轨迹是否平滑，角速度是否稳定

### 场景3：分析射击决策
对比瞄准误差和开火时机

### 场景4：性能分析
监控各模块处理时间和FPS

## 🔧 配置说明

### 修改网络配置

编辑 `monitor_config.json`：
```json
{
  "network": {
    "host": "127.0.0.1",
    "port": 9870
  }
}
```

或在代码中修改：

**Python端：**
```python
monitor = VisionMonitor(host='192.168.1.100', port=9999)
```

**C++端：**
```cpp
tools::Plotter monitor("192.168.1.100", 9999);
```

## 📝 C++集成建议

### 推荐集成位置

根据你的项目结构，建议在以下文件集成：

- **步兵：** `../sp_vision_25-main/src/standard.cpp`
- **哨兵：** `../sp_vision_25-main/src/sentry.cpp`
- **调试：** `../sp_vision_25-main/src/mt_auto_aim_debug.cpp`

### 集成示例

```cpp
// 1. 在文件开头添加
#include "tools/plotter.hpp"
#include <nlohmann/json.hpp>

// 2. 在main函数中初始化
int main() {
    // ... 原有初始化代码 ...
    
    tools::Plotter monitor_plotter("127.0.0.1", 9870);
    
    // ... 主循环 ...
    while (running) {
        // ... 你的自瞄代码 ...
        
        // 获取目标和指令
        auto targets = tracker.get_targets();
        auto command = aimer.aim(targets, timestamp, bullet_speed);
        
        // 发送监控数据
        if (!targets.empty()) {
            nlohmann::json data;
            const auto& target = targets.front();
            auto ekf_state = target.ekf_x();
            
            // 机器人状态
            data["yaw"] = command.yaw;
            data["pitch"] = command.pitch;
            data["shoot"] = command.shoot;
            data["control"] = command.control;
            
            // 目标状态（根据你的EKF定义调整索引）
            data["target_x"] = ekf_state(0);
            data["target_y"] = ekf_state(1);
            data["target_z"] = ekf_state(2);
            data["target_vx"] = ekf_state(3);
            data["target_vy"] = ekf_state(4);
            data["target_vz"] = ekf_state(5);
            data["target_yaw"] = ekf_state(6);
            data["target_yaw_velocity"] = ekf_state(7);
            
            // 装甲板信息
            data["armor_type"] = auto_aim::ARMOR_TYPES[static_cast<int>(target.armor_type)];
            data["armor_name"] = auto_aim::ARMOR_NAMES[static_cast<int>(target.name)];
            
            // 距离
            double distance = std::sqrt(
                ekf_state(0) * ekf_state(0) +
                ekf_state(1) * ekf_state(1) +
                ekf_state(2) * ekf_state(2)
            );
            data["distance"] = distance;
            
            monitor_plotter.plot(data);
        }
        
        // ... 继续原有代码 ...
    }
}
```

## 🧪 测试命令

### 测试不同场景

```bash
# 正常小陀螺
python3 test_data_sender.py --scenario normal

# 静态目标
python3 test_data_sender.py --scenario static

# 高速小陀螺（18 rad/s）
python3 test_data_sender.py --scenario fast_spin

# 装甲板切换
python3 test_data_sender.py --scenario switch

# 自定义频率和时长
python3 test_data_sender.py --frequency 50 --duration 30
```

### 测试网络连接

```bash
# 检查端口监听
sudo netstat -tulpn | grep 9870

# 手动发送测试数据
echo '{"yaw": 0.5, "pitch": 0.3}' | nc -u 127.0.0.1 9870

# 监控UDP流量
sudo tcpdump -i lo -n port 9870
```

## ❓ 常见问题

### Q1: 监控程序启动后没有数据显示？

检查：
1. C++程序是否在运行？
2. 是否调用了 `monitor.plot(data)`？
3. IP和端口配置是否正确？
4. 防火墙是否阻止UDP通信？

### Q2: 图表更新卡顿？

优化方法：
1. 降低数据发送频率（每3-5帧发送一次）
2. 减少历史数据长度（修改 `maxlen` 参数）
3. 减少监控字段数量

```cpp
// 降低发送频率
static int frame_counter = 0;
if (++frame_counter % 3 == 0) {
    monitor_plotter.plot(data);
}
```

### Q3: EKF状态向量索引不确定？

先打印整个向量看看：
```cpp
auto ekf_state = target.ekf_x();
for (int i = 0; i < ekf_state.size(); ++i) {
    data["ekf_state_" + std::to_string(i)] = ekf_state(i);
}
```

然后在监控界面观察哪个索引对应什么数据。

### Q4: 如何保存监控数据？

**方式1 - C++端保存：**
```cpp
std::ofstream log_file("monitor_log.json", std::ios::app);
log_file << data.dump() << std::endl;
```

**方式2 - Python端保存：**
修改 `vision_monitor.py` 的 `_process_packet` 方法：
```python
with open('monitor_log.json', 'a') as f:
    f.write(data.decode('utf-8') + '\n')
```

## 📚 详细文档

- **中文快速指南：** `监控系统使用指南.md` （推荐先看这个）
- **完整英文文档：** `MONITOR_README.md`
- **C++集成示例：** `monitor_integration_example.cpp`

## 🎯 数据字段参考

### 完整JSON格式示例

```json
{
  "timestamp": 1234567890.123,
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
  "distance": 2.38
}
```

## 📞 技术支持

遇到问题请查看：
1. 本README的常见问题部分
2. `监控系统使用指南.md` 详细说明
3. `monitor_integration_example.cpp` 示例代码

## 📄 许可证

本工具遵循与 sp_vision_25 项目相同的许可证。

---

**同济大学SuperPower战队 算法组**

祝调试顺利！🎯
