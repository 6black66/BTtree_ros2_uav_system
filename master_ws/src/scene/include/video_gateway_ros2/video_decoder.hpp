#pragma once
#include <uav_shared_video/frame_ring.hpp>
#include <atomic>
#include <memory>
#include <string>

namespace uav::scene {
struct DecoderConfig {
  std::string input_type = "rtsp";
  std::string source_url;
  std::string input_format;
  std::string rtsp_transport = "tcp";
  int open_timeout_ms = 3000;
  int read_timeout_ms = 2000;
  int max_width = 1920;
  int max_height = 1080;
  video::PixelFormat format = video::PixelFormat::bgr8;
};
struct DecodedFrame {
  video::FrameMetadata metadata;
  std::vector<uint8_t> pixels;
};
class VideoDecoder {
public:
  VideoDecoder(const DecoderConfig& config, std::atomic<bool>& stopping);
  ~VideoDecoder();
  VideoDecoder(const VideoDecoder&) = delete;
  VideoDecoder& operator=(const VideoDecoder&) = delete;
  void open();
  bool next(DecodedFrame& frame);  // False on clean EOF. Throws on IO/decode failure.
  double nominal_fps() const;
private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace uav::scene
