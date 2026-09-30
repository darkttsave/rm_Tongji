#include <fmt/core.h>
#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/img_tools.hpp"

const std::string keys =
  "{help h usage ? |                     | 输出命令行参数说明 }"
  "{@config-path   | configs/sentry.yaml | yaml配置文件的路径}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);

  tools::Exiter exiter;

  // 初始化模块
  io::Camera camera(config_path);
  auto_aim::YOLO yolo(config_path);
  auto_aim::Solver solver(config_path);

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat img;
    camera.read(img, timestamp);
    if (img.empty()) break;

    auto detect_start = std::chrono::steady_clock::now();

    // 1. 检测装甲板
    auto armors = yolo.detect(img);
    
    auto solve_start = std::chrono::steady_clock::now();

    // 2. 位姿解算（需要设置云台姿态，这里假设云台水平）
    solver.set_R_gimbal2world({1, 0, 0, 0});  // 单位四元数
    
    auto detect_time = tools::delta_time(solve_start, detect_start);
    
    // 3. 显示结果
    cv::Mat drawing = img.clone();
    for (const auto & armor : armors) {
      // 绘制装甲板
      tools::draw_points(drawing, armor.image_points, {0, 255, 0});
      
      // 显示位姿信息
      auto xyz = armor.xyz_in_world;
      auto ypr = armor.ypr_in_world;
      
      tools::draw_text(
        drawing,
        fmt::format("xyz:[{:.2f},{:.2f},{:.2f}]", xyz[0], xyz[1], xyz[2]),
        {armor.center_norm.x * img.cols, armor.center_norm.y * img.rows - 20},
        {0, 255, 0});
      
      tools::draw_text(
        drawing,
        fmt::format("ypr:[{:.1f},{:.1f},{:.1f}]", 
          ypr[0]*57.3, ypr[1]*57.3, ypr[2]*57.3),
        {armor.center_norm.x * img.cols, armor.center_norm.y * img.rows},
        {255, 255, 0});
    }

    auto finish = std::chrono::steady_clock::now();
    auto total_time = tools::delta_time(finish, detect_start);
    
    tools::logger()->info(
      "[detect: {:.1f}ms, total: {:.1f}ms, FPS: {:.1f}] detected {} armors",
      detect_time * 1e3, total_time * 1e3, 1.0 / total_time, armors.size());

    cv::imshow("detect_and_solve", drawing);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  return 0;
}
