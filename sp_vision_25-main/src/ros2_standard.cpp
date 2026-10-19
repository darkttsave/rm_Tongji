#include <fmt/core.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/ros2/ros_camera.hpp"
#include "io/ros2/ros_gimbal.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"

using namespace std::chrono;

const std::string keys =
  "{help h usage ? |      | 输出命令行参数说明}"
  "{@config-path   | configs/daedalus_ros2.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  rclcpp::init(argc, argv);

  int exit_code = 0;
  {
    tools::Exiter exiter;

    io::RosGimbal gimbal(config_path);
    io::RosCamera camera(config_path);

    // Same single-thread algorithm chain as src/standard.cpp. No Shooter is constructed or
    // called, and RosGimbal forces fire_advice=false on every published command.
    auto_aim::YOLO detector(config_path, false);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path, solver);
    auto_aim::Aimer aimer(config_path);

    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point t;

    auto mode = io::Mode::idle;
    auto last_mode = io::Mode::idle;
    auto last_status_log = std::chrono::steady_clock::now();

    while (!exiter.exit()) {
      camera.read(img, t);
      if (img.empty()) continue;

      // Attitude is queried at the image capture stamp, not at receipt time: when the gimbal
      // is moving, receipt-time TF lags the frame and makes the tracker's world observations
      // inconsistent. t (steady clock) is still used by Tracker/Aimer for prediction.
      q = gimbal.imu_at(camera.last_ros_stamp());
      mode = gimbal.mode;

      if (last_mode != mode) {
        tools::logger()->info("Switch to {}", io::MODES[static_cast<size_t>(mode)]);
        last_mode = mode;
      }

      solver.set_R_gimbal2world(q);

      auto armors = detector.detect(img);
      auto targets = tracker.track(armors, t);
      auto command = aimer.aim(targets, t, gimbal.bullet_speed);

      gimbal.send(command);

      const auto now = std::chrono::steady_clock::now();
      if (now - last_status_log > std::chrono::seconds(1)) {
        last_status_log = now;
        tools::logger()->info(
          "[ros2_standard] armors={} targets={} control={} yaw={:.3f}rad pitch={:.3f}rad",
          armors.size(), targets.size(), command.control, command.yaw, command.pitch);
      }
    }

    exit_code = 0;
  }

  rclcpp::shutdown();
  return exit_code;
}
