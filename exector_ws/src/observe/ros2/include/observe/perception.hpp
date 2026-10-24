#pragma once
#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/tracking.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace observe {
struct Detection {
  uint64_t id=0;
  int class_id=0;
  float confidence=0;
  cv::Rect2f box;
};
float iou(const cv::Rect2f& a, const cv::Rect2f& b);
// Adjacent-frame association only. Reset on stream/session/size changes.
class IdAssigner {
 public:
  void assign(std::vector<Detection>& detections, double stamp, float threshold, double ttl);
  void reset() { previous_.clear(); }
 private:
  uint64_t next_=1;
  double previous_stamp_=0;
  std::vector<Detection> previous_;
};
class Detector {
 public:
  Detector(const std::string& model, const std::string& labels, float confidence, float nms, int max_results);
  std::vector<Detection> detect(const cv::Mat& bgr);
  const std::string& label(int id) const { return labels_.at(id); }
  // OpenCV Zoo YOLOX-S raw output [1,8400,85], 640x640, RGB 0..255.
  std::vector<Detection> decode(const cv::Mat& output, cv::Size original) const;
 private:
  cv::dnn::Net net_;
  std::vector<std::string> labels_;
  float confidence_, nms_;
  int max_results_;
};
class FastTracker {
 public:
  void init(const cv::Mat& image, const cv::Rect2f& box);
  bool update(const cv::Mat& image, cv::Rect2f& box);
 private:
  cv::Ptr<cv::TrackerKCF> tracker_;
  cv::Size size_;
  cv::Rect box_;
};
}
