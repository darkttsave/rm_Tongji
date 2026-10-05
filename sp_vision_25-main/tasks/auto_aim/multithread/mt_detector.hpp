#ifndef AUTO_AIM__MT_DETECTOR_HPP
#define AUTO_AIM__MT_DETECTOR_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#include <thread>
#include <tuple>

#include "tasks/auto_aim/yolos/yolov5.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

namespace auto_aim
{
namespace multithread
{

class MultiThreadDetector
{
public:
  MultiThreadDetector(const std::string & config_path, bool debug = false);
  ~MultiThreadDetector();

  void push(cv::Mat img, std::chrono::steady_clock::time_point t);

  std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> pop();

  std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point> debug_pop();

private:
  void infer_thread();

  ov::Core core_;
  ov::CompiledModel compiled_model_;
  std::string device_;
  YOLO yolo_;
  bool quit_{false};
  bool log_latency_{false};
  bool praysky_mode_{false};
  int input_size_{640};

  // push 线程只传原始帧，infer 线程负责预处理+推理+后处理
  struct RawFrame {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
  };
  tools::ThreadSafeQueue<RawFrame> raw_queue_{4};

  tools::ThreadSafeQueue<
    std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>>
    result_queue_{16};

  std::thread infer_thread_;
};

}  // namespace multithread

}  // namespace auto_aim

#endif  // AUTO_AIM__MT_DETECTOR_HPP
