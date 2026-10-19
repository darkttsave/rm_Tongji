#include "ros_gimbal.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/exceptions.h>

#include "tools/logger.hpp"

namespace io
{
namespace
{
constexpr double RAD_TO_DEG = 180.0 / M_PI;

Eigen::Quaterniond to_eigen(const geometry_msgs::msg::Quaternion & q)
{
  Eigen::Quaterniond out(q.w, q.x, q.y, q.z);
  out.normalize();
  return out;
}
}  // namespace

RosGimbal::RosGimbal(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);
  if (yaml["bullet_speed"]) bullet_speed = yaml["bullet_speed"].as<double>();
  if (yaml["ros2_yaw_sign"]) yaw_sign_ = yaml["ros2_yaw_sign"].as<double>();
  if (yaml["ros2_pitch_sign"]) pitch_sign_ = yaml["ros2_pitch_sign"].as<double>();
  if (yaml["ros2_parent_frame"]) parent_frame_ = yaml["ros2_parent_frame"].as<std::string>();
  if (yaml["ros2_camera_frame"]) camera_frame_ = yaml["ros2_camera_frame"].as<std::string>();

  node_ = std::make_shared<rclcpp::Node>("sp_vision_ros2_gimbal");
  command_publisher_ = node_->create_publisher<rm_interfaces::msg::GimbalCmd>(
    "/rm_gimbal/cmd", rclcpp::SensorDataQoS());

  // tf2 composes map -> odom -> gimbal_link -> camera_link -> camera_optical_frame and
  // interpolates at the image header stamp, avoiding the attitude lag of receipt-time samples.
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_, false);

  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });

  tools::logger()->info(
    "[RosGimbal] tf2 {} -> {} (bullet_speed={:.1f}, yaw_sign={:.0f}, pitch_sign={:.0f})",
    parent_frame_, camera_frame_, bullet_speed, yaw_sign_, pitch_sign_);
}

RosGimbal::~RosGimbal()
{
  executor_.cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
}

Eigen::Quaterniond RosGimbal::imu_at(const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(parent_frame_, camera_frame_, stamp);
  } catch (const tf2::TransformException & e) {
    // Image stamp can be a little newer than the latest TF on slow frames; fall back to latest
    // instead of feeding an identity attitude into the solver.
    try {
      transform = tf_buffer_->lookupTransform(
        parent_frame_, camera_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e2) {
      static std::chrono::steady_clock::time_point last_warn{};
      const auto now = std::chrono::steady_clock::now();
      if (now - last_warn > std::chrono::seconds(2)) {
        tools::logger()->warn(
          "[RosGimbal] tf2 lookup {} -> {} failed: {}; using last attitude", parent_frame_,
          camera_frame_, e.what());
        last_warn = now;
      }
      std::lock_guard<std::mutex> lock(last_q_mutex_);
      return last_q_;
    }
  }

  Eigen::Quaterniond q = to_eigen(transform.transform.rotation);
  {
    std::lock_guard<std::mutex> lock(last_q_mutex_);
    last_q_ = q;
  }
  return q;
}

void RosGimbal::send(Command command)
{
  rm_interfaces::msg::GimbalCmd msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = parent_frame_;

  // The simulator's /rm_gimbal/cmd uses absolute world yaw/elevation in degrees.
  msg.yaw = yaw_sign_ * command.yaw * RAD_TO_DEG;
  msg.pitch = pitch_sign_ * command.pitch * RAD_TO_DEG;
  msg.yaw_diff = 0.0;
  msg.pitch_diff = 0.0;
  msg.distance = command.control ? 1.0 : -1.0;
  // Hard requirement: never fire from the simulator integration.
  msg.fire_advice = false;

  command_publisher_->publish(msg);

  const auto now = std::chrono::steady_clock::now();
  if (
    !first_command_logged_ || command.control != last_control_ ||
    now - last_command_log_ > std::chrono::seconds(1)) {
    tools::logger()->info(
      "[RosGimbal] cmd control={} yaw={:.2f}deg pitch={:.2f}deg distance={:.1f} fire=false",
      command.control, msg.yaw, msg.pitch, msg.distance);
    first_command_logged_ = true;
    last_control_ = command.control;
    last_command_log_ = now;
  }
}

}  // namespace io
