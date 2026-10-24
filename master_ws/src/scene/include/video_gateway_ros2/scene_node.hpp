#pragma once
#include <video_gateway_ros2/video_decoder.hpp>
#include <msgs/msg/video_frame_ref.hpp>
#include <msgs/msg/video_stream_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace uav::scene {
class SceneNode : public rclcpp::Node {
public:
  SceneNode();
  ~SceneNode() override;
private:
  using Clock = std::chrono::steady_clock;
  void run();
  void publish_frame(const video::FrameReference& frame);
  void publish_status();
  bool wait_for(std::chrono::milliseconds duration);
  DecoderConfig config_;
  bool enabled_ = false;
  bool file_loop_ = false;
  int reconnect_delay_ms_ = 1000;
  int frame_timeout_ms_ = 1500;
  double max_publish_fps_ = 0;
  std::string frame_id_;
  std::unique_ptr<video::FrameWriter> writer_;
  rclcpp::Publisher<msgs::msg::VideoFrameRef>::SharedPtr frame_pub_;
  rclcpp::Publisher<msgs::msg::VideoStreamState>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr legacy_pub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::mutex status_mutex_;
  msgs::msg::VideoStreamState status_;
  Clock::time_point last_frame_{}, fps_sample_time_{};
  uint64_t fps_sample_count_ = 0;
  std::atomic<bool> stopping_{false};
  std::mutex wait_mutex_;
  std::condition_variable wake_;
  std::thread worker_;
};
}  // namespace uav::scene
