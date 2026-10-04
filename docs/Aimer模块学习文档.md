# Aimer 模块学习文档

## 目录
1. [模块概述](#模块概述)
2. [数据结构](#数据结构)
3. [成员变量详解](#成员变量详解)
4. [核心方法](#核心方法)
5. [时间转换机制详解](#时间转换机制详解)
6. [瞄准点选择策略](#瞄准点选择策略)
7. [迭代求解算法](#迭代求解算法)
8. [完整工作流程](#完整工作流程)

---

## 模块概述

**Aimer（瞄准器）** 是自瞄系统的最后一个环节，负责将 Tracker 预测的目标位置转换为云台控制指令（yaw 和 pitch 角度）。

### 核心职责
- 从多个装甲板中选择最优瞄准点
- 计算弹道轨迹（考虑重力下坠）
- 补偿各种时间延迟
- 迭代求解弹丸飞行时间
- 输出精确的云台角度指令

### 文件位置
- 头文件：`tasks/auto_aim/aimer.hpp`
- 实现文件：`tasks/auto_aim/aimer.cpp`

---

## 数据结构

### AimPoint 结构体
**文件位置**：`aimer.hpp:15-19`

```cpp
struct AimPoint {
  bool valid;           // 瞄准点是否有效
  Eigen::Vector4d xyza; // (x, y, z, angle) 瞄准点的位置和角度
};
```

**用途**：
- `valid`: 标记是否找到有效的装甲板（可能所有装甲板都在背面）
- `xyza[0:2]`: 瞄准点的三维坐标 (x, y, z)
- `xyza[3]`: 装甲板的角度信息

---

## 成员变量详解

### 1. 偏移量参数

```cpp
double yaw_offset_;                          // yaw 轴标定偏移（弧度）
std::optional<double> left_yaw_offset_;      // 左发射机构 yaw 偏移
std::optional<double> right_yaw_offset_;     // 右发射机构 yaw 偏移
double pitch_offset_;                        // pitch 轴标定偏移（弧度）
```

**作用**：补偿机械安装误差和相机与云台的坐标系偏差

**来源**：从配置文件（YAML）加载，单位从度转换为弧度（除以 57.3）

---

### 2. 瞄准策略参数

```cpp
double comming_angle_;   // 小陀螺模式：装甲板"进入视野"的角度阈值（弧度）
double leaving_angle_;   // 小陀螺模式：装甲板"离开视野"的角度阈值（弧度）
double lock_id_ = -1;    // 锁定的装甲板 ID（防止频繁切换）
```

**小陀螺瞄准策略**：
- 当目标高速旋转时，选择**正在进入视野**的装甲板
- `comming_angle_`：通常为 60-70 度
- `leaving_angle_`：通常为 30-40 度

---

### 3. 延迟补偿参数

```cpp
double high_speed_delay_time_;   // 高速旋转时的发弹延迟（秒）
double low_speed_delay_time_;    // 低速/静止时的发弹延迟（秒）
double decision_speed_;          // 判断高低速的角速度阈值（rad/s）
```

**延迟时间包括**：
- 摩擦轮加速时间
- 电控指令传输延迟
- 机械响应延迟

**动态选择**：小陀螺高速旋转时延迟更长，需要更多提前量

---

## 核心方法

### 1. `aim()` - 主瞄准函数

**函数签名**：
```cpp
io::Command aim(
  std::list<Target> targets,                    // 目标列表
  std::chrono::steady_clock::time_point timestamp,  // 图像采集时间戳
  double bullet_speed,                          // 弹丸速度（m/s）
  bool to_now = true                           // 是否使用实时模式
);
```

**返回值**：`io::Command` 结构体
```cpp
{
  bool found;      // 是否找到有效目标
  bool fire;       // 是否开火
  double yaw;      // yaw 角度（弧度）
  double pitch;    // pitch 角度（弧度）
}
```

---

### 2. `choose_aim_point()` - 选择瞄准点

**函数签名**：
```cpp
AimPoint choose_aim_point(const Target & target);
```

**返回值**：包含瞄准点坐标和有效性标记的 `AimPoint` 对象

---

## 时间转换机制详解

### 问题：为什么需要时间转换？

在自瞄系统中存在多个时间延迟：
1. **图像处理延迟**：相机采集 → detector 检测 → tracker 跟踪 → aimer 计算
2. **发弹延迟**：云台转动 + 摩擦轮加速 + 机械响应
3. **弹丸飞行时间**：从枪口到目标的飞行时间

**如果不补偿这些延迟，打出的子弹会落在目标的"历史位置"，无法命中！**

---

### 时间轴概念图

```
过去 ←--------------------------------------------→ 未来
     
图像采集时刻          现在（计算时刻）      弹丸发射      弹丸击中
(timestamp)       (steady_clock::now())   (future)   (predict_time)
    |                      |                 |            |
    |<--图像处理延迟 dt1-->|<--发弹延迟 dt2->|<-飞行时间->|
    |<----------总延迟 dt = dt1 + dt2-------->|
```

---

### 代码实现详解

#### 步骤 1: 确定发弹延迟时间

**代码位置**：`aimer.cpp:40-41`

```cpp
double delay_time =
  target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;
```

**逻辑**：
- `ekf_x()[7]`：目标的角速度（rad/s）
- 如果角速度 > `decision_speed_`（如 2 rad/s），使用 `high_speed_delay_time_`（如 0.15s）
- 否则使用 `low_speed_delay_time_`（如 0.1s）

**原因**：高速旋转的小陀螺需要更多提前量

---

#### 步骤 2: 初始化时间基准

**代码位置**：`aimer.cpp:46`

```cpp
auto future = timestamp;  // timestamp 是图像采集的时刻
```

`timestamp` 是 `std::chrono::steady_clock::time_point` 类型，表示相机拍摄这一帧的时间。

---

#### 步骤 3: 计算弹丸发射时刻（两种模式）

##### 模式 A: 实时模式（`to_now = true`）

**代码位置**：`aimer.cpp:47-52`

```cpp
if (to_now) {
  double dt;
  dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
  future += std::chrono::microseconds(int(dt * 1e6));
  target.predict(future);
}
```

**逐行解释**：

1. **计算总延迟时间**：
```cpp
dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
```
- `steady_clock::now()`：**当前时刻**（正在执行 aimer 计算的这一瞬间）
- `timestamp`：**图像采集时刻**（过去的某个时间点）
- `delta_time(now, timestamp)`：计算**从拍照到现在经过了多少秒**
  - 这段时间包括：图像传输 + detector 检测 + tracker 跟踪 + aimer 前置计算
- `+ delay_time`：加上**发弹延迟**（摩擦轮加速 + 指令传输）

**举例**：
- 图像在 `t=0s` 拍摄
- 现在是 `t=0.015s`（图像处理用了 15ms）
- 发弹延迟是 `0.1s`
- **总延迟 dt = 0.015 + 0.1 = 0.115s**

2. **时间单位转换**：
```cpp
future += std::chrono::microseconds(int(dt * 1e6));
```
- `dt * 1e6`：将**秒转换为微秒**（1 秒 = 1,000,000 微秒）
- `int(...)`：转换为整数类型
- `future +=`：将 `future` 时间点向未来推进 `dt` 秒

**为什么用微秒？**
- 保证时间精度，避免浮点数累积误差
- C++ chrono 库使用整数微秒作为内部表示

3. **预测目标位置**：
```cpp
target.predict(future);
```
使用 EKF 预测目标在 `future` 时刻（弹丸发射时刻）的状态

---

##### 模式 B: 固定延迟模式（`to_now = false`）

**代码位置**：`aimer.cpp:54-59`

```cpp
else {
  auto dt = 0.005 + delay_time;  // detector-aimer耗时0.005 + 发弹延时
  future += std::chrono::microseconds(int(dt * 1e6));
  target.predict(future);
}
```

**简化假设**：
- 不动态计算处理时间
- 假设 detector + tracker + aimer 总共耗时 **5ms（0.005s）**
- 总延迟 = 0.005s + `delay_time`（如 0.1s）

**适用场景**：
- 离线测试或回放录像
- 固定频率运行的系统
- 性能足够稳定，处理时间波动小

---

#### 步骤 4: 迭代求解弹丸击中时刻

**代码位置**：`aimer.cpp:86`

```cpp
auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
iteration_target[iter].predict(predict_time);
```

**逻辑**：
- `future`：弹丸发射时刻
- `prev_fly_time`：上一次迭代计算出的弹丸飞行时间（秒）
- `predict_time`：**弹丸击中时刻** = 发射时刻 + 飞行时间

**为什么要迭代？**  
这是一个**相互依赖的问题**：
- 要计算飞行时间，需要知道目标位置
- 要知道目标位置，需要知道飞行时间（目标在移动）

**解决方法**：迭代逼近
1. 第 1 次：假设目标不动，计算初始飞行时间 `fly_time_0`
2. 第 2 次：用 `fly_time_0` 预测目标新位置，计算新飞行时间 `fly_time_1`
3. 第 3 次：用 `fly_time_1` 预测，计算 `fly_time_2`
4. ...
5. 收敛：当 `|fly_time_n - fly_time_{n-1}| < 0.001s`，迭代结束

---

### 完整时间轴示例

假设参数：
- 图像采集时刻：`t = 0s`
- 图像处理延迟：`15ms`
- 发弹延迟：`100ms`
- 弹丸飞行时间：`200ms`（通过迭代求出）

```
t=0s          t=15ms        t=115ms              t=315ms
 |              |             |                    |
图像采集      现在计算      弹丸发射           弹丸击中
(timestamp)  (now)        (future)         (predict_time)
                           
               |<--dt=115ms-->|<--fly_time=200ms-->|
               
               |<----------预测目标在 t=315ms 的位置--------->|
```

**关键思想**：预测目标在弹丸击中时刻的位置，而不是现在的位置！

---

## 瞄准点选择策略

### 方法：`choose_aim_point()`

**代码位置**：`aimer.cpp:144-209`

---

### 策略 1: 装甲板未跳变

**代码位置**：`aimer.cpp:150`

```cpp
if (!target.jumped) return {true, armor_xyza_list[0]};
```

**逻辑**：
- 如果装甲板从未切换过，只有当前装甲板的位置已知
- 直接瞄准第一个（也是唯一）装甲板

---

### 策略 2: 非小陀螺模式（低速旋转）

**代码位置**：`aimer.cpp:163-190`

**判断条件**：
```cpp
if (std::abs(target.ekf_x()[8]) <= 2 && target.name != ArmorName::outpost)
```
- `ekf_x()[8]`：目标的角加速度
- 角加速度绝对值 ≤ 2 rad/s²，判断为非小陀螺

**步骤 1: 筛选可射击装甲板**

```cpp
std::vector<int> id_list;
for (int i = 0; i < armor_num; i++) {
  if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;  // 60度约等于1.047弧度
  id_list.push_back(i);
}
```

- `delta_angle`：装甲板中心与整车中心连线相对于相机的角度
- 只选择 `|delta_angle| ≤ 60°` 的装甲板（正面或侧面，排除背面）

**步骤 2: 锁定机制（防止抖动）**

```cpp
if (id_list.size() > 1) {
  int id0 = id_list[0], id1 = id_list[1];
  
  // 未锁定时，选择角度更小的装甲板
  if (lock_id_ != id0 && lock_id_ != id1)
    lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;
  
  return {true, armor_xyza_list[lock_id_]};
}
```

**锁定机制的作用**：
- 当两个装甲板都可射击时（如车体转到 45 度角），锁定其中一个
- 防止在两个装甲板之间频繁切换导致云台抖动

**退出锁定**：
```cpp
lock_id_ = -1;  // 只有一个装甲板时退出锁定
```

---

### 策略 3: 小陀螺模式（高速旋转）

**代码位置**：`aimer.cpp:192-208`

**特殊参数设置**：
```cpp
if (target.name == ArmorName::outpost) {
  coming_angle = 70 / 57.3;   // 前哨站：70度
  leaving_angle = 30 / 57.3;  // 前哨站：30度
} else {
  coming_angle = comming_angle_;  // 普通目标：使用配置值
  leaving_angle = leaving_angle_;
}
```

**核心逻辑：预测性瞄准**

```cpp
for (int i = 0; i < armor_num; i++) {
  if (std::abs(delta_angle_list[i]) > coming_angle) continue;  // 超过70度则跳过
  
  // 根据旋转方向选择装甲板
  if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle)   // 逆时针旋转
    return {true, armor_xyza_list[i]};
    
  if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle)  // 顺时针旋转
    return {true, armor_xyza_list[i]};
}
```

**原理图解**：

```
       相机视角
          ↓
    ┌─────────┐
    │         │
  ┌─┼─┐   ┌─┼─┐
  │A1│   │A2│      A1: delta_angle = -45° (正在离开)
  └─┼─┘   └─┼─┘      A2: delta_angle = +45° (正在进入)
    │  ╳车体 │
    └─────────┘
         ↻ 逆时针旋转 (ekf_x[7] > 0)
```

**逆时针旋转时**：
- A2 正在进入视野（delta_angle 从 +90° → 0°）
- A1 正在离开视野（delta_angle 从 0° → -90°）
- **应该瞄准 A2（delta_angle > 0 且 < leaving_angle）**

**为什么不打正面的？**
- 小陀螺高速旋转，正面装甲板转瞬即逝
- 打"正在进入"的装甲板，击中概率更高
- 考虑弹丸飞行时间，它会旋转到正面

---

## 迭代求解算法

### 第一步：初始弹道计算

**代码位置**：`aimer.cpp:61-76`

```cpp
auto aim_point0 = choose_aim_point(target);
debug_aim_point = aim_point0;
if (!aim_point0.valid) {
  return {false, false, 0, 0};  // 无有效瞄准点
}

Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);  // 提取 (x, y, z)
auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);  // 水平距离
tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);  // 计算弹道

if (trajectory0.unsolvable) {
  tools::logger()->debug(
    "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", 
    bullet_speed, d0, xyz0[2]);
  debug_aim_point.valid = false;
  return {false, false, 0, 0};  // 弹道无解
}
```

**逐行解释**：

1. **选择瞄准点**：
```cpp
auto aim_point0 = choose_aim_point(target);
```
调用瞄准点选择策略，从多个装甲板中选出最优目标

2. **保存调试信息**：
```cpp
debug_aim_point = aim_point0;
```
存储到成员变量，用于可视化或日志输出

3. **检查有效性**：
```cpp
if (!aim_point0.valid) {
  return {false, false, 0, 0};
}
```
如果没有找到有效装甲板（例如全在背面），返回失败指令

4. **提取三维坐标**：
```cpp
Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
```
`xyza` 是 4 维向量 `(x, y, z, angle)`，`.head(3)` 提取前 3 个元素

5. **计算水平距离**：
```cpp
auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
```
使用勾股定理：`d = √(x² + y²)`，这是弹道计算的关键参数

6. **创建弹道对象**：
```cpp
tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
```
输入：
- `bullet_speed`：弹丸初速度（m/s）
- `d0`：水平距离（m）
- `xyz0[2]`：目标高度 z（m）

输出（`Trajectory` 类会计算）：
- `pitch`：发射仰角
- `fly_time`：飞行时间
- `unsolvable`：是否无解

7. **检查弹道可解性**：
```cpp
if (trajectory0.unsolvable) {
  // 记录日志并返回失败
}
```
可能无解的情况：
- 目标太远，弹速不足
- 目标太高，抛物线无法到达
- 目标在枪口后方

---

### 第二步：迭代精确求解

**代码位置**：`aimer.cpp:78-116`

```cpp
// 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
bool converged = false;
double prev_fly_time = trajectory0.fly_time;
tools::Trajectory current_traj = trajectory0;
std::vector<Target> iteration_target(10, target);  // 创建10个目标副本

for (int iter = 0; iter < 10; ++iter) {
  // 预测目标在 future + prev_fly_time 时刻的位置
  auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
  iteration_target[iter].predict(predict_time);

  // 计算瞄准点
  auto aim_point = choose_aim_point(iteration_target[iter]);
  debug_aim_point = aim_point;
  if (!aim_point.valid) {
    return {false, false, 0, 0};
  }

  // 计算新弹道
  Eigen::Vector3d xyz = aim_point.xyza.head(3);
  double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
  current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

  // 检查弹道是否可解
  if (current_traj.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", 
      iter + 1, bullet_speed, d, xyz.z());
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 检查收敛条件
  if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
    converged = true;
    break;
  }
  prev_fly_time = current_traj.fly_time;
}
```

**迭代逻辑图解**：

```
第 0 次（初始）：
  目标位置 P0 → 计算弹道 → fly_time_0 = 0.2s

第 1 次迭代：
  预测 P1 = P(t + 0.2s) → 计算弹道 → fly_time_1 = 0.21s

第 2 次迭代：
  预测 P2 = P(t + 0.21s) → 计算弹道 → fly_time_2 = 0.209s

第 3 次迭代：
  预测 P3 = P(t + 0.209s) → 计算弹道 → fly_time_3 = 0.2091s

第 4 次迭代：
  预测 P4 = P(t + 0.2091s) → 计算弹道 → fly_time_4 = 0.2091s
  |fly_time_4 - fly_time_3| = 0.0001s < 0.001s → 收敛！
```

**收敛条件**：
```cpp
if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
  converged = true;
  break;
}
```
相邻两次飞行时间差小于 **1ms**，认为收敛

**最多迭代 10 次**：
- 通常 3-5 次就能收敛
- 10 次是保险上限，防止死循环

---

### 第三步：计算最终角度

**代码位置**：`aimer.cpp:118-122`

```cpp
// 计算最终角度
Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
double pitch = -(current_traj.pitch + pitch_offset_);  // 世界坐标系下pitch向上为负
return {true, false, yaw, pitch};
```

**计算 yaw 角**：
```cpp
double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
```
- `atan2(y, x)`：计算 (x, y) 点的极坐标角度（弧度）
- `+ yaw_offset_`：加上标定偏移量

**计算 pitch 角**：
```cpp
double pitch = -(current_traj.pitch + pitch_offset_);
```
- `current_traj.pitch`：弹道计算出的仰角
- `+ pitch_offset_`：加上标定偏移
- 前面的负号：**世界坐标系中 pitch 向上为负**

**返回指令**：
```cpp
return {true, false, yaw, pitch};
```
- `true`：找到有效目标
- `false`：不自动开火（由上层决策）
- `yaw, pitch`：云台控制角度

---

## 完整工作流程

### 流程图

```
┌─────────────────┐
│ 输入：Targets   │
│ timestamp       │
│ bullet_speed    │
└────────┬────────┘
         │
         ▼
┌─────────────────────────┐
│ 1. 选择延迟时间          │
│    根据角速度选择        │
│    high/low_delay_time  │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 2. 计算发射时刻 future  │
│    = timestamp + dt     │
│    (补偿处理+发弹延迟)   │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 3. 预测目标在 future    │
│    时刻的状态            │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 4. 选择瞄准装甲板       │
│    choose_aim_point()   │
└────────┬────────────────┘
         │
         ▼
┌─────────────────────────┐
│ 5. 计算初始弹道         │
│    Trajectory(speed,d,z)│
└────────┬────────────────┘
         │
         ▼
    ╔═══════════════════╗
    ║ 6. 迭代求解循环    ║
    ║ (最多10次)        ║
    ╚═══╤═══════════════╝
        │
        ▼
    ┌─────────────────────┐
    │ 预测 predict_time   │
    │ = future+fly_time   │
    └──────┬──────────────┘
        │
        ▼
    ┌─────────────────────┐
    │ 更新瞄准点          │
    └──────┬──────────────┘
        │
        ▼
    ┌─────────────────────┐
    │ 重新计算弹道        │
    └──────┬──────────────┘
        │
        ▼
    ┌─────────────────────┐
    │ 检查收敛？          │
    │ |Δfly_time|<0.001  │
    └──────┬──────────────┘
        │  否
        └──────┐
               │ 是
               ▼
        ┌─────────────────┐
        │ 7. 计算最终角度  │
        │ yaw = atan2(y,x)│
        │ pitch = -仰角    │
        └──────┬──────────┘
               │
               ▼
        ┌─────────────────┐
        │ 输出：Command   │
        │ {true,false,    │
        │  yaw, pitch}    │
        └─────────────────┘
```

---

### 关键数据流

```
Tracker 输出 → Aimer 输入
├─ Target.ekf_x()         → 目标状态向量
│  ├─ [0:2]: x, y, z      → 位置
│  ├─ [3:6]: vx, vy, vz   → 速度
│  ├─ [7]: vyaw           → 角速度（选择延迟时间）
│  └─ [8]: ayaw           → 角加速度（判断小陀螺）
│
├─ Target.armor_xyza_list() → 所有装甲板位置
│  └─ Vector4d[n]: (x,y,z,angle)
│
└─ timestamp              → 图像采集时间戳

Aimer 内部处理
├─ choose_aim_point()     → 选择最优装甲板
│  └─ 考虑：旋转方向、角度、锁定状态
│
├─ Trajectory()           → 计算弹道
│  ├─ 输入：speed, d, z
│  └─ 输出：pitch, fly_time, unsolvable
│
└─ 迭代求解               → 精确飞行时间
   └─ 目标位置 ⇄ 飞行时间 (相互依赖)

Aimer 输出 → 云台控制
└─ io::Command
   ├─ found: true/false   → 是否找到目标
   ├─ fire: true/false    → 是否开火
   ├─ yaw: double         → 偏航角（弧度）
   └─ pitch: double       → 俯仰角（弧度）
```

---

## 学习问题总结

### 问题 1: Tracker 和 Target 的关系

**问题描述**：
> 所以说先进行 track 对所有的装甲板进行筛选、排序。然后确定当前状态机的状态后调用 target 中的 predict 和 update 进行创建预测和更新吗？

**答案**：
完全正确！工作流程：

1. **Tracker**（高层决策）
   - 对检测到的装甲板按距离和优先级排序
   - 根据状态机状态决定：创建新目标 / 更新现有目标 / 切换目标
   - 调用 `Target` 的方法

2. **Target**（底层跟踪）
   - `predict(t)`：使用 EKF 预测目标在时刻 t 的状态
   - `update(armor)`：用新检测到的装甲板更新 EKF

3. **Aimer**（瞄准控制）
   - 接收 Tracker 输出的 `Target` 列表
   - 选择瞄准点、计算弹道、输出角度指令

**架构分层**：
```
Detector (检测) → Tracker (跟踪决策) → Target (EKF预测) → Aimer (瞄准) → 云台控制
```

---

### 问题 2: 初始弹道计算的逐行解释

**问题描述**：
> 给我逐行解释初始弹道计算部分（aimer.cpp:61-76）

**答案**：
已在 [迭代求解算法 - 第一步](#第一步初始弹道计算) 中详细解释。

核心步骤：
1. 选择瞄准点
2. 检查有效性
3. 提取 3D 坐标
4. 计算水平距离
5. 创建弹道对象
6. 检查可解性

---

### 问题 3: 时间转换看不懂

**问题描述**：
> 这里在进行各种时间的转换时，我看不懂

**答案**：
已在 [时间转换机制详解](#时间转换机制详解) 中详细解释。

关键点：
- **三个时间点**：图像采集 (timestamp) → 弹丸发射 (future) → 弹丸击中 (predict_time)
- **单位转换**：秒 × 1e6 = 微秒（保证精度）
- **两种模式**：实时模式（动态计算延迟）vs 固定延迟模式
- **迭代求解**：飞行时间和目标位置相互依赖，需要迭代收敛

---

## 附录：重要公式

### 1. 水平距离计算
```cpp
d = √(x² + y²)
```

### 2. yaw 角计算
```cpp
yaw = atan2(y, x) + yaw_offset
```

### 3. pitch 角计算
```cpp
pitch = -(弹道仰角 + pitch_offset)
```
注：世界坐标系中 pitch 向上为负

### 4. 收敛条件
```cpp
|fly_time_new - fly_time_old| < 0.001 秒
```

### 5. 时间转换
```cpp
微秒 = 秒 × 1,000,000
future += std::chrono::microseconds(int(dt * 1e6))
```

---

## 总结

Aimer 模块是自瞄系统的"大脑"，它：

1. **选择最优目标**：根据旋转状态智能选择装甲板
2. **补偿多级延迟**：图像处理 + 发弹延迟 + 飞行时间
3. **迭代求解**：精确计算弹丸击中时刻的目标位置
4. **输出精确指令**：yaw 和 pitch 角度控制云台

理解 Aimer 的关键：
- **时间补偿**：打的是未来位置，不是当前位置
- **迭代收敛**：飞行时间和目标位置相互依赖
- **策略分支**：小陀螺打"进入"的，普通模式打"正面"的

---

*文档生成时间：2026-09-28*  
*基于代码版本：sp_vision_25-main*
