#include "ros_camera.hpp"

#include <cstdlib>

#include "tools/logger.hpp"

namespace io
{
RosCamera::RosCamera(const std::string & config_path)
{
  (void)config_path;  // image topic is fixed for the simulator; no camera SDK is opened

  node_ = std::make_shared<rclcpp::Node>("sp_vision_ros2_camera");
  subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", rclcpp::QoS(10),  // reliable: /image_raw carries large 1440x1080 frames
    std::bind(&RosCamera::image_callback, this, std::placeholders::_1));

  executor_.add_node(node_);
  spin_thread_ = std::thread([this]() { executor_.spin(); });

  tools::logger()->info("[RosCamera] waiting for /image_raw ...");
}

RosCamera::~RosCamera()
{
  executor_.cancel();
  if (spin_thread_.joinable()) spin_thread_.join();
}

void RosCamera::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = msg;
  latest_stamp_ = std::chrono::steady_clock::now();
  latest_ros_stamp_ = rclcpp::Time(msg->header.stamp);
  has_new_ = true;
  condition_.notify_one();
}

rclcpp::Time RosCamera::last_ros_stamp()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return latest_ros_stamp_;
}

void RosCamera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  sensor_msgs::msg::Image::SharedPtr msg;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, std::chrono::seconds(1), [this]() { return has_new_; })) {
      img.release();
      timestamp = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (now - last_empty_warn_ > std::chrono::seconds(2)) {
        tools::logger()->warn("[RosCamera] no image received from /image_raw yet");
        last_empty_warn_ = now;
      }
      return;
    }
    msg = latest_;
    timestamp = latest_stamp_;
    has_new_ = false;
  }

  if (!msg || msg->data.empty() || msg->width == 0 || msg->height == 0) {
    img.release();
    return;
  }

  cv::Mat view;
  if (msg->encoding == "rgb8" || msg->encoding == "bgr8") {
    view = cv::Mat(
      static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3,
      const_cast<uint8_t *>(msg->data.data()), static_cast<size_t>(msg->step));
  } else {
    static bool warned = false;
    if (!warned) {
      tools::logger()->warn("[RosCamera] unsupported image encoding '{}'", msg->encoding);
      warned = true;
    }
    img.release();
    return;
  }

  if (msg->encoding == "rgb8") {
    cv::cvtColor(view, img, cv::COLOR_RGB2BGR);
  } else {
    img = view.clone();
  }

  if (!first_frame_logged_) {
    first_frame_logged_ = true;
    tools::logger()->info(
      "[RosCamera] first image: {}x{} {} step={}", msg->width, msg->height, msg->encoding,
      msg->step);

    // Optional one-shot dump for offline detector debugging; disabled by default.
    if (const char * path = std::getenv("ROS2_SIM_SAVE_FRAME")) {
      if (cv::imwrite(path, img)) {
        tools::logger()->info("[RosCamera] saved first frame to {}", path);
      }
    }
  }
}

}  // namespace io
