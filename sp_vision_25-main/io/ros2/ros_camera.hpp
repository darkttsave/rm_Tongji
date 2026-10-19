#ifndef IO__ROS2__ROS_CAMERA_HPP
#define IO__ROS2__ROS_CAMERA_HPP

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/opencv.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "io/camera.hpp"

namespace io
{
/// ROS2 image input for the simulator: /image_raw (sensor_msgs/Image) -> BGR cv::Mat.
///
/// The tracker/aimer need a steady_clock timestamp, so read() also returns the local receipt
/// time. The original sensor header stamp is kept separately and used for tf2 attitude lookup,
/// so a moving gimbal is queried at the exact capture time instead of at receipt time.
class RosCamera : public CameraBase
{
public:
  explicit RosCamera(const std::string & config_path = "");
  ~RosCamera() override;

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

  /// Header stamp of the most recently received image (for tf2 lookup at capture time).
  rclcpp::Time last_ros_stamp();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread spin_thread_;

  std::mutex mutex_;
  std::condition_variable condition_;
  sensor_msgs::msg::Image::SharedPtr latest_;
  std::chrono::steady_clock::time_point latest_stamp_{};
  rclcpp::Time latest_ros_stamp_{0, 0, RCL_ROS_TIME};
  bool has_new_ = false;
  bool first_frame_logged_ = false;
  std::chrono::steady_clock::time_point last_empty_warn_{};
};

}  // namespace io

#endif  // IO__ROS2__ROS_CAMERA_HPP
