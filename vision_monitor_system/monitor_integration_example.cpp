/**
 * @file monitor_integration_example.cpp
 * @brief 集成监控系统的示例代码
 *
 * 在你的自瞄代码中添加类似的调用，即可将数据发送到监控系统
 */

#include "tools/plotter.hpp"
#include "tasks/auto_aim/target.hpp"
#include "io/command.hpp"
#include <nlohmann/json.hpp>

// 示例：在自瞄主循环中发送监控数据
void send_monitor_data_example(
    const tools::Plotter& plotter,
    const auto_aim::Target& target,
    const io::Command& command,
    double bullet_speed)
{
    nlohmann::json monitor_data;

    // === 机器人状态数据 ===
    monitor_data["yaw"] = command.yaw;
    monitor_data["pitch"] = command.pitch;
    monitor_data["shoot"] = command.shoot;
    monitor_data["control"] = command.control;
    monitor_data["bullet_speed"] = bullet_speed;

    // === 目标状态数据 ===
    auto ekf_state = target.ekf_x();

    // 目标位置 (EKF状态向量的前3个元素通常是xyz)
    if (ekf_state.size() >= 3) {
        monitor_data["target_x"] = ekf_state(0);
        monitor_data["target_y"] = ekf_state(1);
        monitor_data["target_z"] = ekf_state(2);
    }

    // 目标速度 (根据你的EKF定义调整索引)
    if (ekf_state.size() >= 6) {
        monitor_data["target_vx"] = ekf_state(3);
        monitor_data["target_vy"] = ekf_state(4);
        monitor_data["target_vz"] = ekf_state(5);
    }

    // 目标yaw和角速度 (根据你的EKF定义调整索引)
    if (ekf_state.size() >= 8) {
        monitor_data["target_yaw"] = ekf_state(6);
        monitor_data["target_yaw_velocity"] = ekf_state(7);
    }

    // 装甲板信息
    monitor_data["armor_type"] = auto_aim::ARMOR_TYPES[static_cast<int>(target.armor_type)];
    monitor_data["armor_name"] = auto_aim::ARMOR_NAMES[static_cast<int>(target.name)];

    // 计算距离
    if (ekf_state.size() >= 3) {
        double distance = std::sqrt(
            ekf_state(0) * ekf_state(0) +
            ekf_state(1) * ekf_state(1) +
            ekf_state(2) * ekf_state(2)
        );
        monitor_data["distance"] = distance;
    }

    // 发送数据
    plotter.plot(monitor_data);
}


// 示例：集成到你的主函数中
void integration_example() {
    // 创建Plotter实例（通常在初始化时创建一次）
    tools::Plotter plotter("127.0.0.1", 9870);

    // 在你的主循环中
    while (true) {
        // ... 你的原有代码 ...

        // 假设你已经有了target和command
        // auto_aim::Target target = ...;
        // io::Command command = ...;
        // double bullet_speed = ...;

        // 发送监控数据
        // send_monitor_data_example(plotter, target, command, bullet_speed);

        // ... 继续你的原有代码 ...
    }
}


/**
 * 更详细的监控数据示例
 * 可以根据需要添加更多字段
 */
void send_detailed_monitor_data(
    const tools::Plotter& plotter,
    const std::list<auto_aim::Target>& targets,
    const io::Command& command,
    const std::chrono::steady_clock::time_point& timestamp,
    bool is_tracking,
    int frame_count)
{
    nlohmann::json data;

    // 基础信息
    data["timestamp"] = std::chrono::duration<double>(timestamp.time_since_epoch()).count();
    data["frame_count"] = frame_count;
    data["is_tracking"] = is_tracking;
    data["target_count"] = targets.size();

    // 云台指令
    data["yaw"] = command.yaw;
    data["pitch"] = command.pitch;
    data["shoot"] = command.shoot;
    data["control"] = command.control;

    // 如果有目标
    if (!targets.empty()) {
        const auto& primary_target = targets.front();
        auto ekf_state = primary_target.ekf_x();

        // 完整的目标状态
        for (int i = 0; i < ekf_state.size(); ++i) {
            data["ekf_state_" + std::to_string(i)] = ekf_state(i);
        }

        // 装甲板列表
        auto armor_list = primary_target.armor_xyza_list();
        data["armor_count"] = armor_list.size();

        for (size_t i = 0; i < armor_list.size(); ++i) {
            std::string prefix = "armor_" + std::to_string(i) + "_";
            data[prefix + "x"] = armor_list[i](0);
            data[prefix + "y"] = armor_list[i](1);
            data[prefix + "z"] = armor_list[i](2);
            data[prefix + "a"] = armor_list[i](3);  // angle
        }

        // 收敛状态
        data["target_converged"] = primary_target.convergened();
        data["target_diverged"] = primary_target.diverged();
    }

    plotter.plot(data);
}


/**
 * 性能监控示例
 * 监控各个模块的处理时间
 */
class PerformanceMonitor {
public:
    PerformanceMonitor(tools::Plotter& plotter) : plotter_(plotter) {}

    void send_timing_data(
        double detect_time_ms,
        double track_time_ms,
        double predict_time_ms,
        double total_time_ms,
        double fps)
    {
        nlohmann::json data;
        data["detect_time"] = detect_time_ms;
        data["track_time"] = track_time_ms;
        data["predict_time"] = predict_time_ms;
        data["total_time"] = total_time_ms;
        data["fps"] = fps;
        data["type"] = "performance";  // 标识数据类型

        plotter_.plot(data);
    }

private:
    tools::Plotter& plotter_;
};


/**
 * 建议的集成方式：
 *
 * 1. 在你的main函数开始时创建Plotter:
 *    tools::Plotter monitor_plotter("127.0.0.1", 9870);
 *
 * 2. 在自瞄主循环的关键位置调用send_monitor_data_example:
 *    - 获取到目标后
 *    - 生成控制指令后
 *    - 发送指令到下位机前
 *
 * 3. 运行Python监控程序:
 *    python3 vision_monitor.py
 *
 * 4. 启动你的C++自瞄程序
 *
 * 5. 实时观察监控界面
 */
