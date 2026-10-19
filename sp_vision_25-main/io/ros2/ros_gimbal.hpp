#ifndef IO__ROS2__ROS_GIMBAL_HPP
#define IO__ROS2__ROS_GIMBAL_HPP

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rm_interfaces/msg/gimbal_cmd.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "io/cboard.hpp"
#include "io/command.hpp"

namespace io
{
/// ROS2 gimbal adapter for the simulator.
///
/// Replaces CBoard: world attitude comes from /tf (map -> camera_optical_frame), commands are
/// published on /rm_gimbal/cmd. It mirrors the small CBoard API used by src/standard.cpp
/// (mode/bullet_speed/imu_at/send) so the algorithm chain stays untouched.
class RosGimbal
{
public:
  explicit RosGimbal(const std::string & config_path);
  ~RosGimbal();

  // Same public fields as io::CBoard, used by src/standard.cpp.
  double bullet_speed = 25.0;
  io::Mode mode = io::Mode::auto_aim;

  /// World attitude at the image header stamp (tf2 interpolation), not at receipt time.
  Eigen::Quaterniond imu_at(const rclcpp::Time & stamp);

  /// Publish one control message. fire_advice is always false (simulator must not fire).
  void send(Command command);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<rm_interfaces::msg::GimbalCmd>::SharedPtr command_publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex last_q_mutex_;
  Eigen::Quaterniond last_q_ = Eigen::Quaterniond::Identity();

  std::string parent_frame_ = "map";
  std::string camera_frame_ = "camera_optical_frame";

  // The fork's Aimer returns world yaw with the opposite sign to Daedalus (see report).
  double yaw_sign_ = -1.0;
  // Tongji writes pitch upward as negative; Daedalus expects elevation upward as positive.
  double pitch_sign_ = -1.0;

  bool first_command_logged_ = false;
  std::chrono::steady_clock::time_point last_command_log_{};
  bool last_control_ = false;
};

}  // namespace io

#endif  // IO__ROS2__ROS_GIMBAL_HPP
