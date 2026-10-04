# Track模块学习文档

## 模块概述

Track模块位于 `tasks/auto_aim/tracker.cpp` 和 `tracker.hpp`，是自瞄系统的**核心目标跟踪模块**。

**主要功能**：
- 在连续视频帧中识别和追踪同一个装甲板目标
- 使用状态机管理跟踪状态
- 结合EKF（扩展卡尔曼滤波）进行目标预测和状态更新
- 支持目标切换和优先级管理

---

## 核心类结构

### Tracker类

```cpp
class Tracker {
private:
  Solver & solver_;              // PnP求解器引用
  Color enemy_color_;            // 敌方颜色（红/蓝）
  int detect_count_;             // 连续检测计数
  int temp_lost_count_;          // 临时丢失计数
  int min_detect_count_;         // 进入tracking状态所需的最小检测次数
  int max_temp_lost_count_;      // 允许的最大临时丢失次数
  std::string state_;            // 当前状态
  std::string pre_state_;        // 上一个状态
  Target target_;                // 当前跟踪的目标对象
};
```

---

## 五种跟踪状态

Track模块使用**状态机**管理目标跟踪过程：

### 1. **lost（丢失）**
- **含义**: 未找到任何目标
- **初始状态**: 系统启动时的默认状态
- **行为**: 尝试从装甲板列表中设置新目标

### 2. **detecting（检测中）**
- **含义**: 连续检测到目标，但还未达到稳定跟踪条件
- **计数器**: `detect_count_` 累计检测成功次数
- **转换条件**: 
  - 成功 → `detect_count_ >= min_detect_count_` → 进入 **tracking**
  - 失败 → 重置计数器 → 返回 **lost**

### 3. **tracking（跟踪中）**
- **含义**: 稳定跟踪目标
- **行为**: 
  - 使用EKF预测目标位置
  - 匹配装甲板并更新EKF
- **失败处理**: 丢失1次 → 进入 **temp_lost**

### 4. **temp_lost（临时丢失）**
- **含义**: 短暂丢失目标，但还在容忍范围内
- **计数器**: `temp_lost_count_` 累计丢失次数
- **转换条件**:
  - 重新找到 → 返回 **tracking**
  - `temp_lost_count_ > max_temp_lost_count_` → 进入 **lost**
- **特殊处理**: 前哨站的容忍次数更大（`outpost_max_temp_lost_count_`）

### 5. **switching（切换中）**
- **含义**: 正在切换到更高优先级的目标
- **触发条件**: 全向感知相机检测到更高优先级目标
- **行为**: 等待主相机确认新目标出现

---

## 状态转换图

```
                    ┌──────────┐
                    │   lost   │ (初始状态)
                    └────┬─────┘
                         │ 找到目标
                         ↓
                   ┌──────────┐
                   │detecting │
                   └─┬──────┬─┘
          丢失 ←─────┘      │ 连续检测 >= min_detect_count_
                            ↓
                      ┌──────────┐
              ┌──────→│ tracking │←──────┐
              │       └────┬─────┘       │
              │ 重新找到   │ 丢失1次      │ 重新找到
              │            ↓              │
              │      ┌───────────┐       │
              └──────│temp_lost  │───────┘
                     └─────┬─────┘
                           │ 丢失次数 > max_temp_lost_count_
                           ↓
                     ┌──────────┐
                     │   lost   │
                     └──────────┘
```

---

## 核心函数详解

### 1. `track()` - 主跟踪函数

**普通版本**（用于步兵、英雄、无人机）:
```cpp
std::list<Target> track(
  std::list<Armor> & armors, 
  std::chrono::steady_clock::time_point t,
  bool use_enemy_color = true
);
```

**全向感知版本**（专用于哨兵）:
```cpp
std::tuple<omniperception::DetectionResult, std::list<Target>> track(
  const std::vector<omniperception::DetectionResult> & detection_queue,
  std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t,
  bool use_enemy_color = true
);
```

**处理流程**:
1. 计算时间间隔 `dt`，检测相机是否离线（`dt > 0.1s`）
2. 过滤装甲板：
   - 移除非敌方颜色的装甲板
   - 按图像中心距离排序（优先跟踪靠近中心的）
   - 按优先级排序（高优先级在前）
3. 根据当前状态调用相应函数：
   - `lost` → `set_target()` 设置新目标
   - 其他状态 → `update_target()` 更新现有目标
4. 更新状态机
5. 收敛性检测：
   - 发散检测：`target_.diverged()`
   - NIS检测：检查滤波器收敛质量

---

### 2. `set_target()` - 设置新目标

**功能**: 从装甲板列表中选择优先级最高的装甲板，初始化为跟踪目标

**步骤**:
1. 检查装甲板列表是否为空
2. 取第一个装甲板（已按优先级排序）
3. 调用 `solver_.solve(armor)` 进行PnP求解
4. 根据兵种类型初始化不同的EKF参数

**不同兵种的初始化参数**:

| 兵种 | 半径(m) | 装甲板数量 | 协方差矩阵特点 |
|------|---------|-----------|---------------|
| 平衡步兵（3/4/5大装甲板） | 0.2 | 2 | 标准协方差 |
| 前哨站 | 0.2765 | 3 | 半径几乎确定（1e-4），Z轴不确定性大（81） |
| 基地 | 0.3205 | 3 | 半径几乎确定（1e-4） |
| 普通车辆（1/2号、哨兵、英雄） | 0.2 | 4 | 标准协方差 |

---

### 3. `update_target()` - 更新目标

**功能**: 在当前帧中查找匹配的装甲板，更新EKF状态

**步骤**:

#### 第一阶段：预测
```cpp
target_.predict(t);  // EKF预测下一时刻目标位置
```

#### 第二阶段：匹配统计
```cpp
int found_count = 0;
double min_x = 1e10;  // 记录最左侧装甲板X坐标
for (const auto & armor : armors) {
    // 匹配条件：兵种相同 && 类型相同
    if (armor.name != target_.name || armor.type != target_.armor_type) 
        continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
}
```

#### 第三阶段：EKF更新
```cpp
if (found_count == 0) return false;  // 未找到匹配装甲板

for (auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type)
        continue;
    
    solver_.solve(armor);      // PnP求解
    target_.update(armor);     // EKF更新
}
```

**注意**: 当检测到同一目标的多个装甲板时，会对所有匹配的装甲板进行EKF更新（多次观测提高精度）

---

### 4. `state_machine()` - 状态机更新

```cpp
void state_machine(bool found);
```

根据 `found`（是否找到目标）和当前状态更新到下一个状态。

**关键逻辑**:
- **lost**: 找到目标 → detecting（初始化 `detect_count_ = 1`）
- **detecting**: 
  - 找到 → 计数+1，达到阈值 → tracking
  - 未找到 → 计数归零 → lost
- **tracking**: 未找到 → temp_lost（初始化 `temp_lost_count_ = 1`）
- **temp_lost**: 
  - 找到 → tracking
  - 未找到 → 计数+1，超过阈值 → lost
- **switching**: 找到 → detecting

---

## EKF状态向量与协方差矩阵

### 状态向量（11维）

```cpp
// x  vx  y  vy  z  vz  a  w  r  l  h
```

| 索引 | 变量 | 含义 | 单位 |
|------|------|------|------|
| 0 | x | 旋转中心X坐标 | m |
| 1 | vx | X方向速度 | m/s |
| 2 | y | 旋转中心Y坐标 | m |
| 3 | vy | Y方向速度 | m/s |
| 4 | z | 旋转中心Z坐标 | m |
| 5 | vz | Z方向速度 | m/s |
| 6 | a | Yaw角（旋转角度） | rad |
| 7 | w | 角速度（旋转速度） | rad/s |
| 8 | r | 旋转半径 | m |
| 9 | l | 半径变化量 | m |
| 10 | h | 高度变化量 | m |

### 初始协方差矩阵 P0_dig

**普通车辆**:
```cpp
P0_dig = {1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}
```

**含义**:
- **位置（x, y, z）**: 不确定性小（1），因为PnP求解比较准确
- **速度（vx, vy, vz）**: 不确定性大（64），单帧无法得知速度
- **角度（a）**: 不确定性较小（0.4 ≈ ±36°）
- **角速度（w）**: 不确定性很大（100），不知道车辆转速
- **半径/变化量（r, l, h）**: 不确定性小（1）

**前哨站**:
```cpp
P0_dig = {1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}
```

**特点**:
- **Z轴速度**: 81 > 64，因为前哨站可能上下摆动
- **半径**: 1e-4（几乎确定），前哨站半径固定
- **变化量**: 0（装甲板位置相对固定）

---

## 全向感知增强功能（哨兵专属）

### 目标优先级切换

**触发条件**:
- 状态为 `tracking`（正在跟踪某目标）
- 全向相机检测到更高优先级的目标
- 当前目标已收敛（`target_.convergened()`）

**切换流程**:
```cpp
if (state_ == "tracking" && 
    !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && 
    target_.convergened()) {
    
    state_ = "switching";  // 进入切换状态
    switch_target = temp_target;  // 记录切换目标
    omni_target_priority_ = temp_target.armors.front().priority;
}
```

### switching 状态处理

等待主相机画面中出现目标优先级的装甲板：
```cpp
if (state_ == "switching") {
    // 检查主相机是否看到了目标优先级的装甲板
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
}
```

---

## 鲁棒性设计

### 1. 时间异常检测
```cpp
if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";  // 相机可能离线，重置状态
}
```

### 2. 发散检测
```cpp
if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
}
```

### 3. NIS（归一化新息平方）收敛性检测
```cpp
if (recent_nis_failures >= 0.4 * window_size) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
}
```

检查最近40%的更新是否通过NIS测试，不通过则说明滤波器收敛质量差。

### 4. 前哨站特殊容忍
```cpp
if (target_.name == ArmorName::outpost)
    max_temp_lost_count_ = outpost_max_temp_lost_count_;  // 更大的容忍值
else
    max_temp_lost_count_ = normal_temp_lost_count_;
```

前哨站可能被遮挡或旋转到背面，需要更大的临时丢失容忍度。

---

## 装甲板过滤与排序

### 颜色过滤
```cpp
armors.remove_if([&](const auto_aim::Armor & a) { 
    return a.color != enemy_color_; 
});
```

### 按图像中心距离排序
```cpp
armors.sort([](const Armor & a, const Armor & b) {
    cv::Point2f img_center(1440 / 2, 1080 / 2);
    auto distance_1 = cv::norm(a.center - img_center);
    auto distance_2 = cv::norm(b.center - img_center);
    return distance_1 < distance_2;  // 距离小的在前
});
```

**目的**: 优先跟踪靠近画面中心的目标（云台指向性更好）

### 按优先级排序
```cpp
armors.sort([](const auto_aim::Armor & a, const auto_aim::Armor & b) { 
    return a.priority < b.priority;  // 数字越小优先级越高
});
```

**优先级定义**:
```cpp
enum ArmorPriority {
    first = 1,   // 最高优先级
    second,
    third,
    forth,
    fifth        // 最低优先级
};
```

---

## 与其他模块的交互

```
┌─────────┐      ┌────────┐      ┌─────────┐
│  YOLO   │─────→│ Tracker│─────→│  Aimer  │
│ Detector│      │        │      │         │
└─────────┘      └────┬───┘      └─────────┘
                      │
                      ↓
                 ┌─────────┐
                 │ Solver  │ (PnP求解)
                 └─────────┘
                      ↓
                 ┌─────────┐
                 │ Target  │ (EKF预测更新)
                 └─────────┘
```

### 输入
- **YOLO**: 检测到的装甲板列表 `std::list<Armor>`
- **时间戳**: 当前帧的采集时间
- **全向感知队列**（哨兵）: `detection_queue`

### 输出
- **Target列表**: 跟踪到的目标（通常只有1个）
- **跟踪状态**: `state_`（lost/detecting/tracking/temp_lost/switching）
- **切换目标**（哨兵）: 需要切换到的高优先级目标信息

---

## 函数重载机制（不同兵种适配）

Track模块通过**C++函数重载**实现对不同兵种的适配：

### 普通兵种（步兵/英雄/无人机）
```cpp
auto targets = tracker.track(armors, t);
```

### 哨兵（全向感知）
```cpp
auto detection_queue = perceptron.get_detection_queue();
auto [switch_target, targets] = tracker.track(detection_queue, armors, t);
```

**设计优势**:
- 向下兼容：普通兵种无需修改代码
- 功能扩展：哨兵通过额外参数启用全向感知
- 编译时确定：根据参数类型自动选择正确版本

---

## Lambda表达式应用

### 捕获列表语法
```cpp
[&](const auto_aim::Armor & a) { return a.color != enemy_color_; }
```

**结构**:
- `[&]`: 以引用方式捕获外部变量（这里捕获 `enemy_color_`）
- `(const auto_aim::Armor & a)`: 参数列表
- `{ ... }`: 函数体

**捕获方式对比**:
- `[&]`: 引用捕获所有外部变量（可修改）
- `[=]`: 值捕获所有外部变量（不可修改，副本）
- `[this]`: 捕获当前对象指针
- `[&enemy_color_]`: 只引用捕获特定变量

---

## 配置参数（YAML）

```yaml
enemy_color: "red"              # 敌方颜色
min_detect_count: 3             # 进入tracking所需的最小检测次数
max_temp_lost_count: 10         # 普通目标的最大临时丢失次数
outpost_max_temp_lost_count: 50 # 前哨站的最大临时丢失次数
```

---

## 关键设计亮点

### 1. 状态机鲁棒性
- 使用 `detecting` 状态过滤误检
- 使用 `temp_lost` 状态容忍短暂丢帧
- 前哨站有更大的容忍度

### 2. 优先级系统
- 自动切换到更高优先级目标
- 装甲板排序确保优先处理重要目标

### 3. 收敛性监控
- NIS检测滤波器收敛质量
- 发散检测防止跟踪失效

### 4. 兵种自适应
- 不同兵种使用不同的EKF初始化参数
- 运动模型参数根据兵种调整

### 5. 多观测融合
- 当同时看到同一目标的多个装甲板时，使用所有观测更新EKF
- 提高位置估计精度

---

## 典型应用场景

### 场景1：初次锁定目标
```
lost → detecting (连续3帧) → tracking
```

### 场景2：短暂遮挡
```
tracking → temp_lost → tracking (重新找到)
```

### 场景3：完全丢失
```
tracking → temp_lost (超过阈值) → lost
```

### 场景4：优先级切换（哨兵）
```
tracking (低优先级) → switching → detecting → tracking (高优先级)
```

---

## 调试信息

### 日志输出
```cpp
tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
tools::logger()->debug("[Tracker] Target diverged!");
tools::logger()->debug("[Target] Bad Converge Found!");
tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armor.name]);
```

### 状态查询
```cpp
std::string state() const { return state_; }
```

---

## 总结

Track模块是自瞄系统的**核心大脑**，负责：

✅ **连续帧目标跟踪** - 识别并追踪同一目标  
✅ **状态机管理** - 5种状态应对各种情况  
✅ **EKF预测更新** - 准确估计目标位置和速度  
✅ **鲁棒性设计** - 发散检测、NIS检测、容忍机制  
✅ **兵种自适应** - 不同兵种使用不同参数  
✅ **优先级切换** - 自动锁定高价值目标  
✅ **全向感知支持** - 哨兵专属功能  

通过精心设计的状态机和EKF算法，Track模块实现了稳定、准确、鲁棒的目标跟踪功能！
