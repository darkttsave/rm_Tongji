#include "commandgener.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/math_tools.hpp"

namespace auto_aim
{
namespace multithread
{

CommandGener::CommandGener(
  const std::string & config_path, auto_aim::Shooter & shooter, auto_aim::Aimer & aimer,
  io::Gimbal & gimbal, tools::Plotter & plotter, bool debug)
: shooter_(shooter), aimer_(aimer), gimbal_(gimbal), plotter_(plotter), stop_(false), debug_(debug)
{
  auto yaml = YAML::LoadFile(config_path);
  log_aim_ = yaml["log_aim"].as<bool>(false);
  thread_ = std::thread(&CommandGener::generate_command, this);
}

CommandGener::~CommandGener()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    stop_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void CommandGener::push(
  const std::list<auto_aim::Target> & targets, const std::chrono::steady_clock::time_point & t,
  double bullet_speed, const Eigen::Vector3d & gimbal_pos)
{
  std::lock_guard<std::mutex> lock(mtx_);
  latest_ = {targets, t, bullet_speed, gimbal_pos};
  cv_.notify_one();
}

void CommandGener::generate_command()
{
  auto t0 = std::chrono::steady_clock::now();

  // ---- 终端调试状态（低频输出 + 开火/发弹事件输出）----
  auto last_status_time = t0;
  bool prev_shoot = false;
  bool bullet_init = false;
  uint16_t prev_bullet = 0;

  while (!stop_) {
    std::optional<Input> input;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (latest_ && tools::delta_time(std::chrono::steady_clock::now(), latest_->t) < 0.2) {
        input = latest_;
      } else
        input = std::nullopt;
    }
    if (input) {
      auto command = aimer_.aim(input->targets_, input->t, input->bullet_speed);

      // aimer 原始输出（yaw 归一化前），仅用于调试
      const double aim_yaw_deg = command.yaw * 57.3;
      const double aim_pitch_deg = command.pitch * 57.3;
      const bool aim_valid = aimer_.debug_aim_point.valid;

      if (command.control)
        command.yaw = gimbal_.state().yaw + tools::limit_rad(command.yaw - gimbal_.state().yaw);
      command.shoot = shooter_.shoot(command, aimer_, input->targets_, input->gimbal_pos);
      command.horizon_distance = input->targets_.empty()
                                   ? 0
                                   : std::sqrt(
                                       tools::square(input->targets_.front().ekf_x()[0]) +
                                       tools::square(input->targets_.front().ekf_x()[2]));

      // 将 Command 转换为 Gimbal::send() 所需的参数
      // Gimbal::send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc,
      //              float pitch, float pitch_vel, float pitch_acc)
      gimbal_.send(command.control, command.shoot, command.yaw, 0.0f, 0.0f,
                   command.pitch, 0.0f, 0.0f);

      // ---- 终端调试输出：低频 100ms 一次，开火请求/实际发弹时强制输出 ----
      auto gs = gimbal_.state();
      const double gim_yaw_deg = gs.yaw * 57.3;
      const double pos_yaw_deg = input->gimbal_pos[0] * 57.3;  // shooter 判据用的云台角
      const double err_deg = (command.yaw - gs.yaw) * 57.3;    // 云台还需转动的剩余角
      const int tx_mode = command.control ? (command.shoot ? 2 : 1) : 0;

      double dist = 0.0, spd = 0.0, w_deg = 0.0;
      if (!input->targets_.empty()) {
        const auto & x = input->targets_.front().ekf_x();
        dist = std::sqrt(tools::square(x[0]) + tools::square(x[2]));
        spd = std::sqrt(tools::square(x[1]) + tools::square(x[3]));
        w_deg = x[7] * 57.3;
      }

      const bool fired = bullet_init && (gs.bullet_count != prev_bullet);
      if (!bullet_init) {
        bullet_init = true;
        prev_bullet = gs.bullet_count;
      }
      const bool shoot_rise = command.shoot && !prev_shoot;

      const auto now = std::chrono::steady_clock::now();
      const bool status_due = tools::delta_time(now, last_status_time) >= 0.1;

      if (log_aim_ && (shoot_rise || fired || status_due)) {
        tools::logger()->info(
          "[AIM] d={:.2f}m v={:.2f}m/s w={:.0f}d/s | yaw={:.2f} pitch={:.2f} valid={} | "
          "gim={:.2f} pos={:.2f} err={:.2f} | shoot={} mode={} bullet={}{}{}",
          dist, spd, w_deg, aim_yaw_deg, aim_pitch_deg, aim_valid,
          gim_yaw_deg, pos_yaw_deg, err_deg,
          command.shoot, tx_mode, gs.bullet_count,
          shoot_rise ? " [FIRE]" : "", fired ? " [SHOT]" : "");
        last_status_time = now;
      }
      prev_shoot = command.shoot;
      prev_bullet = gs.bullet_count;

      if (debug_) {
        nlohmann::json data;
        data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);
        data["cmd_yaw"] = command.yaw * 57.3;
        data["cmd_pitch"] = command.pitch * 57.3;
        data["shoot"] = command.shoot;
        data["horizon_distance"] = command.horizon_distance;
        plotter_.plot(data);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));  //approximately 500Hz
  }
}

}  // namespace multithread

}  // namespace auto_aim