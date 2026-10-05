#include "mt_detector.hpp"

#include <yaml-cpp/yaml.h>

namespace auto_aim
{
namespace multithread
{

MultiThreadDetector::MultiThreadDetector(const std::string & config_path, bool debug)
: yolo_(config_path, debug)
{
  auto yaml = YAML::LoadFile(config_path);
  auto yolo_name = yaml["yolo_name"].as<std::string>();
  log_latency_ = yaml["log_latency"].as<bool>(false);
  praysky_mode_ = (yolo_name == "praysky");

  if (!praysky_mode_) {
    auto model_path = yaml[yolo_name + "_model_path"].as<std::string>();
    device_ = yaml["device"].as<std::string>();

    auto model = core_.read_model(model_path);

    // 从模型自身读取输入尺寸，避免与模型不匹配（NCHW: [1,3,H,W]）
    auto ps = model->input().get_partial_shape();
    if (ps[2].is_static() && ps[3].is_static())
      input_size_ = static_cast<int>(ps[2].get_length());

    ov::preprocess::PrePostProcessor ppp(model);
    auto & input = ppp.input();

    input.tensor()
      .set_element_type(ov::element::u8)
      .set_shape({1, static_cast<ov::Dimension::value_type>(input_size_), static_cast<ov::Dimension::value_type>(input_size_), 3})
      .set_layout("NHWC")
      .set_color_format(ov::preprocess::ColorFormat::BGR);

    input.model().set_layout("NCHW");

    input.preprocess()
      .convert_element_type(ov::element::f32)
      .convert_color(ov::preprocess::ColorFormat::RGB)
      .scale(255.0);

    model = ppp.build();
    compiled_model_ = core_.compile_model(
      model, device_,
      ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY),
      ov::hint::inference_precision(ov::element::f16));
  }

  infer_thread_ = std::thread(&MultiThreadDetector::infer_thread, this);

  tools::logger()->info("[MultiThreadDetector] initialized !");
}

MultiThreadDetector::~MultiThreadDetector()
{
  quit_ = true;
  if (infer_thread_.joinable()) infer_thread_.join();
}

void MultiThreadDetector::push(cv::Mat img, std::chrono::steady_clock::time_point t)
{
  raw_queue_.push({img.clone(), t});
}

void MultiThreadDetector::infer_thread()
{
  // praysky 模式：ONNX Runtime 推理，直接调 detect()
  if (praysky_mode_) {
    while (!quit_) {
      auto [img, t] = raw_queue_.pop();
      auto t0 = std::chrono::steady_clock::now();
      auto armors = yolo_.detect(img, 0);
      auto t1 = std::chrono::steady_clock::now();
      if (log_latency_)
        tools::logger()->info("[Detector] detect: {:.1f} ms",
          std::chrono::duration<double, std::milli>(t1 - t0).count());
      result_queue_.push({img, std::move(armors), t});
    }
    return;
  }

  // OpenVINO 模式
  auto infer_request = compiled_model_.create_infer_request();
  cv::Mat input_mat(input_size_, input_size_, CV_8UC3, cv::Scalar(0, 0, 0));

  while (!quit_) {
    auto [img, t] = raw_queue_.pop();

    auto x_scale = static_cast<double>(input_size_) / img.rows;
    auto y_scale = static_cast<double>(input_size_) / img.cols;
    auto scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(img.rows * scale);
    auto w = static_cast<int>(img.cols * scale);

    input_mat.setTo(cv::Scalar(0, 0, 0));
    cv::resize(img, input_mat(cv::Rect(0, 0, w, h)), {w, h});

    infer_request.set_input_tensor(
      ov::Tensor(ov::element::u8, {1, (size_t)input_size_, (size_t)input_size_, 3}, input_mat.data));
    auto t0 = std::chrono::steady_clock::now();
    infer_request.infer();
    auto t1 = std::chrono::steady_clock::now();

    auto output_tensor = infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape();
    cv::Mat output(output_shape[1], output_shape[2], CV_32F, output_tensor.data());
    auto armors = yolo_.postprocess(scale, output, img, 0);
    auto t2 = std::chrono::steady_clock::now();

    if (log_latency_)
      tools::logger()->info("[Detector] infer: {:.1f} ms  post: {:.1f} ms",
        std::chrono::duration<double, std::milli>(t1 - t0).count(),
        std::chrono::duration<double, std::milli>(t2 - t1).count());

    result_queue_.push({img, std::move(armors), t});
  }
}

std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point> MultiThreadDetector::pop()
{
  auto [img, armors, t] = result_queue_.pop();
  return {std::move(armors), t};
}

std::tuple<cv::Mat, std::list<Armor>, std::chrono::steady_clock::time_point>
MultiThreadDetector::debug_pop()
{
  return result_queue_.pop();
}

}  // namespace multithread

}  // namespace auto_aim
