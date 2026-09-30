# Solver 模块学习指南

## 一、模块概述

### 功能定位
`Solver` 模块是自瞄系统中负责**空间位姿解算**的核心组件，将装甲板从 2D 图像像素坐标解算到 3D 世界坐标，并进行姿态优化。

### 核心职责
1. 管理多个坐标系之间的转换关系
2. 使用 PnP 算法解算装甲板的 3D 位姿
3. 优化偏航角（yaw）以提高精度

### 文件位置
- 头文件：[tasks/auto_aim/solver.hpp](sp_vision_25-main/tasks/auto_aim/solver.hpp)
- 实现文件：[tasks/auto_aim/solver.cpp](sp_vision_25-main/tasks/auto_aim/solver.cpp)

---

## 二、坐标系体系

### 坐标系转换链

```
装甲板坐标系 ← 相机坐标系 ← 云台坐标系 ← IMU坐标系 ← 世界坐标系
```

### 关键转换参数

#### 1. 相机内参
```cpp
cv::Mat camera_matrix_;      // 相机内参矩阵 (3×3)
cv::Mat distort_coeffs_;     // 畸变系数 (1×5)
```

**用途**：图像去畸变、像素坐标与相机坐标转换

#### 2. 固定外参（标定获得，运行期不变）
```cpp
Eigen::Matrix3d R_camera2gimbal_;    // 相机→云台 旋转矩阵
Eigen::Vector3d t_camera2gimbal_;    // 相机→云台 平移向量
Eigen::Matrix3d R_gimbal2imubody_;   // 云台→IMU 旋转矩阵
```

**来源**：YAML 配置文件，通过手眼标定获得

#### 3. 动态姿态（运行期实时更新）
```cpp
Eigen::Matrix3d R_gimbal2world_;     // 云台→世界 旋转矩阵
```

**更新频率**：每帧更新，来自 IMU 测量数据

### 坐标系关系说明

| 转换矩阵 | 起点 | 终点 | 性质 | 更新方式 |
|---------|------|------|------|----------|
| `R_camera2gimbal_` | 相机 | 云台 | 静态 | 配置文件 |
| `R_gimbal2imubody_` | 云台 | IMU | 静态 | 配置文件 |
| `R_gimbal2world_` | 云台 | 世界 | 动态 | IMU 实时 |

---

## 三、装甲板 3D 模型

### 物理尺寸

```cpp
constexpr double LIGHTBAR_LENGTH = 56e-3;     // 灯条长度: 56mm
constexpr double BIG_ARMOR_WIDTH = 230e-3;    // 大装甲板宽度: 230mm
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  // 小装甲板宽度: 135mm
```

### 角点定义（装甲板坐标系）

装甲板坐标系：X 轴指向前方，Y 轴水平向左，Z 轴垂直向上

**四个角点**（顺时针顺序）：
```cpp
右上: (0,  width/2,  length/2)
左上: (0, -width/2,  length/2)
左下: (0, -width/2, -length/2)
右下: (0,  width/2, -length/2)
```

---

## 四、核心方法详解

### 1. 构造函数：初始化标定参数

**位置**：[solver.cpp:27-44](sp_vision_25-main/tasks/auto_aim/solver.cpp#L27-L44)

```cpp
Solver::Solver(const std::string & config_path)
```

**功能流程**：
1. 加载 YAML 配置文件
2. 读取外参矩阵（云台-IMU、相机-云台）
3. 读取相机内参和畸变系数
4. 数据格式转换（Eigen ↔ OpenCV）

**初始化列表**：
```cpp
: R_gimbal2world_(Eigen::Matrix3d::Identity())
```
将云台到世界的旋转初始化为单位矩阵（假设初始对齐）

---

### 2. set_R_gimbal2world：更新云台姿态

**位置**：[solver.cpp:48-52](sp_vision_25-main/tasks/auto_aim/solver.cpp#L48-L52)

```cpp
void set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ = R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
}
```

**数学原理**：相似变换
```
R_gimbal→world = R_gimbal→IMU^T × R_IMU→world × R_gimbal→IMU
```

**调用时机**：每帧调用，见 [mt_standard.cpp:92](sp_vision_25-main/src/mt_standard.cpp#L92)

**参数说明**：
- `q`：IMU 测量的四元数（IMU 本体→世界坐标系）

---

### 3. solve：PnP 姿态解算（核心方法）

**位置**：[solver.cpp:55-88](sp_vision_25-main/tasks/auto_aim/solver.cpp#L55-L88)

这是**最重要的方法**，完整的位姿解算流程。

**输入**：`Armor` 对象，包含检测到的像素坐标  
**输出**：填充 `Armor` 对象的所有 3D 信息

#### Step 1: 选择装甲板模型
```cpp
const auto & object_points = 
  (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
```

#### Step 2: PnP 求解位姿
```cpp
cv::solvePnP(
  object_points, armor.points, 
  camera_matrix_, distort_coeffs_, 
  rvec, tvec, false, cv::SOLVEPNP_IPPE
);
```

**算法**：IPPE（Infinitesimal Plane-based Pose Estimation）

**输出**：
- `rvec`：旋转向量（罗德里格斯表示）
- `tvec`：平移向量（装甲板在相机坐标系的位置）

#### Step 3: 位置坐标系转换
```cpp
// 相机坐标系
cv::cv2eigen(tvec, xyz_in_camera);

// 云台坐标系
armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;

// 世界坐标系
armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
```

#### Step 4: 姿态坐标系转换
```cpp
// 旋转向量 → 旋转矩阵
cv::Rodrigues(rvec, rmat);
cv::cv2eigen(rmat, R_armor2camera);

// 逐级转换
R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
R_armor2world = R_gimbal2world_ * R_armor2gimbal;
```

#### Step 5: 转换为欧拉角和极坐标
```cpp
// YPR 欧拉角（Yaw, Pitch, Roll）
armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

// 极坐标（Yaw, Pitch, Distance）
armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
```

#### Step 6: 偏航角优化（可选）
```cpp
// 平衡步兵跳过优化
auto is_balance = (armor.type == ArmorType::big) &&
                  (armor.name == ArmorName::three || 
                   armor.name == ArmorName::four || 
                   armor.name == ArmorName::five);
if (is_balance) return;

optimize_yaw(armor);  // 优化 yaw 角度
```

---

### 4. optimize_yaw：偏航角优化

**位置**：[solver.cpp:196-218](sp_vision_25-main/tasks/auto_aim/solver.cpp#L196-L218)

#### 为什么需要优化？

**问题**：PnP 假设装甲板是平面，但实际装甲板可能有倾斜（pitch ≈ ±15°），这会导致 yaw 角估计不准确。

**解决方案**：暴力搜索最优 yaw 角

#### 优化流程

```cpp
constexpr double SEARCH_RANGE = 140;  // 搜索范围: 140度
```

**步骤**：
1. 获取云台当前朝向
2. 在 ±70° 范围内每隔 1° 搜索
3. 对每个候选 yaw 角计算重投影误差
4. 选择误差最小的 yaw 角

**重投影误差计算**：
```cpp
// 将优化后的姿态重新投影回图像
auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);

// 计算投影点与实际检测点的像素距离之和
for (int i = 0; i < 4; i++) 
  error += cv::norm(armor.points[i] - image_points[i]);
```

---

### 5. reproject_armor：重投影验证

**位置**：[solver.cpp:90-127](sp_vision_25-main/tasks/auto_aim/solver.cpp#L90-L127)

```cpp
std::vector<cv::Point2f> reproject_armor(
  const Eigen::Vector3d & xyz_in_world, 
  double yaw, 
  ArmorType type, 
  ArmorName name
) const;
```

**功能**：3D 世界坐标 → 2D 像素坐标（解算的逆过程）

**用途**：
- 验证姿态估计的准确性
- yaw 优化中计算重投影误差

**流程**：
```
世界坐标 → 构造姿态矩阵 → 转相机坐标 → projectPoints → 像素坐标
```

---

### 6. world2pixel：通用坐标转换

**位置**：[solver.cpp:266-292](sp_vision_25-main/tasks/auto_aim/solver.cpp#L266-L292)

```cpp
std::vector<cv::Point2f> world2pixel(
  const std::vector<cv::Point3f> & worldPoints
);
```

**功能**：将世界坐标系中的 3D 点批量投影到图像平面

**特性**：
- 过滤相机后方的点（`camera_point.z() > 0` 检查）
- 只投影可见的点

**应用场景**：
- 可视化调试（绘制预测位置）
- 手眼标定测试
- 轨迹预测可视化

---

## 五、使用示例

### 典型调用流程

**位置**：[mt_standard.cpp:86-98](sp_vision_25-main/src/mt_standard.cpp#L86-L98)

```cpp
// 1. 更新云台姿态
Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
solver.set_R_gimbal2world(q);

// 2. 检测装甲板（由 detector 完成）
auto armors = detector.detect(img);

// 3. 解算每个装甲板的位姿
for (auto & armor : armors) {
  solver.solve(armor);  // 填充 armor 的所有 3D 信息
}

// 4. 使用解算结果进行跟踪
auto targets = tracker.track(armors, t);
```

### 完整数据流

```
像素坐标 (armor.points)
    ↓ [solve()]
相机坐标 (xyz_in_camera)
    ↓ [R_camera2gimbal, t_camera2gimbal]
云台坐标 (xyz_in_gimbal)
    ↓ [R_gimbal2world]
世界坐标 (xyz_in_world)
    ↓ [xyz2ypd()]
极坐标 (ypd_in_world) → 云台控制
```

---

## 六、关键数学概念

### PnP 问题（Perspective-n-Point）

**定义**：已知 n 个 3D 点及其 2D 投影，求相机位姿

**最少需要**：4 个非共面点

**本项目应用**：装甲板的 4 个角点

**算法选择**：IPPE（精度高，适合平面目标）

### 坐标系转换公式

**点的转换**：
```
P_B = R_A→B * P_A + t_A→B
```

**旋转的连续转换**：
```
R_A→C = R_B→C * R_A→B
```

### 罗德里格斯变换

```cpp
cv::Rodrigues(rvec, rmat);  // 旋转向量 ↔ 旋转矩阵
```

**旋转向量**：方向表示旋转轴，长度表示旋转角度  
**旋转矩阵**：3×3 正交矩阵

### 欧拉角与四元数

**欧拉角**（Euler Angles）：
- Yaw（偏航）、Pitch（俯仰）、Roll（横滚）
- 存在万向锁问题

**四元数**（Quaternion）：
- 无万向锁
- IMU 常用四元数表示姿态
- 本项目使用 `Eigen::Quaterniond`

---

## 七、常见问题 FAQ

### Q1: 为什么平衡步兵不优化 yaw？
**A**: 平衡步兵会大幅度倾斜，pitch 假设（±15°）不成立，优化算法的前提条件被破坏。

### Q2: 重投影误差的阈值是多少？
**A**: 代码中没有硬阈值，采用搜索最小误差的策略。通常良好的解算重投影误差 < 2 像素。

### Q3: 如何调试解算结果？
**A**: 
1. 使用 `world2pixel()` 可视化预测位置
2. 检查重投影误差
3. 对比优化前后的 `yaw_raw` 和 `ypr_in_world[0]`

### Q4: 配置文件中的参数如何标定？
**A**:
- 相机内参：使用 OpenCV 标定板标定
- 外参：手眼标定（需要云台运动配合）
- IMU 外参：机械测量或联合标定

### Q5: R_gimbal2imubody 和 R_gimbal2world 的区别？
**A**:
- `R_gimbal2imubody`：静态外参，描述固定安装关系
- `R_gimbal2world`：动态姿态，描述当前空间朝向
- 前者从配置文件加载一次，后者每帧从 IMU 更新

---

## 八、性能优化建议

### 1. yaw 优化搜索范围
```cpp
constexpr double SEARCH_RANGE = 140;  // 可根据实际情况调整
```

**权衡**：
- 范围大：更准确，但耗时长
- 范围小：速度快，但可能错过最优解

**建议**：根据云台转速和检测频率调整

### 2. 坐标转换缓存
当前每次 `solve()` 都重新计算，可以考虑缓存部分矩阵乘法结果。

### 3. 批量处理
如果一帧检测到多个装甲板，可以并行解算（需要确保线程安全）。

---

## 九、相关模块

### 输入依赖
- [armor.hpp](sp_vision_25-main/tasks/auto_aim/armor.hpp) - 装甲板数据结构
- [detector.hpp](sp_vision_25-main/tasks/auto_aim/detector.hpp) - 提供像素坐标

### 输出使用
- [tracker.hpp](sp_vision_25-main/tasks/auto_aim/tracker.hpp) - 使用解算结果进行跟踪
- [aimer.hpp](sp_vision_25-main/tasks/auto_aim/aimer.hpp) - 使用位姿进行瞄准

### 工具依赖
- [math_tools.hpp](sp_vision_25-main/tools/math_tools.hpp) - 欧拉角转换、坐标转换

---

## 十、学习路径建议

### 阶段一：理解坐标系（1-2天）
1. 绘制各坐标系的关系图
2. 理解固定外参和动态姿态的区别
3. 手动计算一个简单的坐标转换

### 阶段二：学习 PnP 原理（2-3天）
1. 学习透视投影的数学原理
2. 理解 PnP 问题的定义和求解方法
3. 实验 OpenCV 的 `solvePnP` 函数

### 阶段三：跟踪数据流（2-3天）
1. 在 `solve()` 中添加调试输出
2. 观察像素→世界的完整转换过程
3. 可视化重投影结果

### 阶段四：深入优化算法（3-5天）
1. 理解为什么需要 yaw 优化
2. 实验不同的搜索范围
3. 对比优化前后的效果

### 推荐学习资源
- OpenCV 官方文档：Camera Calibration and 3D Reconstruction
- 《视觉 SLAM 十四讲》：坐标系转换、PnP 问题
- 《机器人学中的状态估计》：姿态表示、坐标变换

---

## 十一、调试技巧

### 1. 可视化重投影
```cpp
// 在 solve() 后添加
auto reprojected = solver.reproject_armor(
  armor.xyz_in_world, 
  armor.ypr_in_world[0], 
  armor.type, 
  armor.name
);

// 在图像上绘制原始检测点（红色）和重投影点（绿色）
for (int i = 0; i < 4; i++) {
  cv::circle(img, armor.points[i], 3, cv::Scalar(0, 0, 255), -1);
  cv::circle(img, reprojected[i], 3, cv::Scalar(0, 255, 0), -1);
}
```

### 2. 输出关键中间结果
```cpp
tools::logger()->debug("xyz_in_camera: ({}, {}, {})", 
  xyz_in_camera[0], xyz_in_camera[1], xyz_in_camera[2]);
tools::logger()->debug("yaw_raw: {}, yaw_optimized: {}", 
  armor.yaw_raw, armor.ypr_in_world[0]);
```

### 3. 检查标定参数
```cpp
// 检查相机内参是否合理
tools::logger()->info("camera_matrix:\n{}", camera_matrix_);
tools::logger()->info("distort_coeffs:\n{}", distort_coeffs_);
```

---

## 十二、总结

`Solver` 模块是自瞄系统的**核心大脑**，完成了从 2D 像素到 3D 世界坐标的完整映射。

**关键要点**：
1. **多级坐标系转换**：相机→云台→世界
2. **PnP 算法**：从 2D-3D 对应求解位姿
3. **yaw 优化**：提高角度估计精度
4. **固定外参 vs 动态姿态**：理解静态标定和实时更新的区别

掌握此模块需要扎实的**线性代数**、**计算机视觉**和**机器人学**基础，但一旦理解透彻，就能深入理解整个自瞄系统的工作原理。

---

**文档版本**：v1.0  
**最后更新**：2026-09-27  
**维护者**：基于 sp_vision_25 代码库生成
