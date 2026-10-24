#include <video_gateway_ros2/scene_node.hpp>
#include <uav_ros2_support/xml_config.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace uav::scene {
namespace {
std::string default_shm_name(std::string ns) {
  for(char& c : ns) if(!std::isalnum(static_cast<unsigned char>(c))) c = '_';
  return "/uav" + ns + "_scene_frames";
}
}
SceneNode::SceneNode() : Node("scene") {
  using uav_ros2_support::param;
  uav_ros2_support::load_xml_overrides(*this, "scene");
  enabled_ = param<bool>(*this, "enabled", true);
  config_.input_type = param<std::string>(*this, "input_type", "rtsp");
  config_.source_url = param<std::string>(*this, "source_url", "rtsp://192.168.1.160:554/live/track0");
  config_.input_format = param<std::string>(*this, "input_format", "");
  config_.rtsp_transport = param<std::string>(*this, "rtsp_transport", "tcp");
  config_.open_timeout_ms = param<int>(*this, "open_timeout_ms", 3000);
  config_.read_timeout_ms = param<int>(*this, "read_timeout_ms", 2000);
  config_.max_width = param<int>(*this, "max_width", 1920);
  config_.max_height = param<int>(*this, "max_height", 1080);
  config_.format = video::pixel_format(param<std::string>(*this, "output_encoding", "bgr8"));
  const int slots = param<int>(*this, "slot_count", 3);
  auto shm_name = param<std::string>(*this, "shm_name", "");
  if(shm_name.empty()) shm_name = default_shm_name(get_namespace());
  reconnect_delay_ms_ = param<int>(*this, "reconnect_delay_ms", 1000);
  frame_timeout_ms_ = param<int>(*this, "frame_timeout_ms", 1500);
  max_publish_fps_ = param<double>(*this, "max_publish_fps", 0.0);
  file_loop_ = param<bool>(*this, "file_loop", false);
  frame_id_ = param<std::string>(*this, "frame_id", "gimbal_camera");
  const int status_period = param<int>(*this, "status_period_ms", 200);
  const bool legacy = param<bool>(*this, "publish_legacy_frame_stamp", true);
  if(config_.input_type != "rtsp" && config_.input_type != "udp" && config_.input_type != "file")
    throw std::invalid_argument("input_type must be rtsp, udp or file");
  if(enabled_ && config_.source_url.empty()) throw std::invalid_argument("source_url is empty");
  if(config_.input_type == "rtsp" && config_.source_url.rfind("rtsp://",0) != 0 && config_.source_url.rfind("rtsps://",0) != 0)
    throw std::invalid_argument("rtsp source_url must start with rtsp:// or rtsps://");
  if(config_.input_type == "udp" && config_.source_url.rfind("udp://",0) != 0)
    throw std::invalid_argument("udp source_url must start with udp://");
  if(config_.rtsp_transport != "udp" && config_.rtsp_transport != "tcp")
    throw std::invalid_argument("rtsp_transport must be tcp or udp");
  if(config_.max_width < 1 || config_.max_height < 1 || config_.max_width > 16384 || config_.max_height > 16384 ||
     slots < 2 || slots > 32 || config_.open_timeout_ms < 100 || config_.open_timeout_ms > 60000 ||
     config_.read_timeout_ms < 100 || config_.read_timeout_ms > 60000 || reconnect_delay_ms_ < 100 || reconnect_delay_ms_ > 60000 ||
     frame_timeout_ms_ < 100 || frame_timeout_ms_ > 60000 || status_period < 20 || status_period > 60000 ||
     !std::isfinite(max_publish_fps_) || max_publish_fps_ < 0 || max_publish_fps_ > 240)
    throw std::invalid_argument("invalid frame dimensions, slot count, timeout or publish rate");
  frame_pub_ = create_publisher<msgs::msg::VideoFrameRef>(
      param<std::string>(*this, "frame_topic", "scene/frame"), rclcpp::QoS(1).best_effort());
  status_pub_ = create_publisher<msgs::msg::VideoStreamState>(
      param<std::string>(*this, "state_topic", "scene/stream_state"), rclcpp::QoS(1).reliable().transient_local());
  if(legacy) legacy_pub_ = create_publisher<std_msgs::msg::String>(
      param<std::string>(*this, "frame_stamp_topic", "episode/frame_stamp"), 1);
  status_.enabled = enabled_;
  status_.state = enabled_ ? "connecting" : "disabled";
  status_.frame_age_s = -1;
  status_.encoding = video::encoding(config_.format);
  fps_sample_time_ = Clock::now();
  if(enabled_) {
    const uint64_t capacity = uint64_t(config_.max_width) * config_.max_height * video::channels(config_.format);
    if(capacity > std::numeric_limits<uint32_t>::max()) throw std::invalid_argument("frame capacity overflow");
    writer_ = std::make_unique<video::FrameWriter>(shm_name, slots, static_cast<uint32_t>(capacity));
    status_.shm_name = writer_->name(); status_.session_id = writer_->session_id();
  }
  status_timer_ = create_wall_timer(std::chrono::milliseconds(status_period), [this] { publish_status(); });
  if(enabled_) worker_ = std::thread([this] { run(); });
  RCLCPP_INFO(get_logger(), "scene video gateway initialized (%s, %s)", enabled_ ? "enabled" : "disabled", config_.input_type.c_str());
}
SceneNode::~SceneNode() {
  stopping_ = true;
  wake_.notify_all();
  if(worker_.joinable()) worker_.join();
  // Writer unlinks its unique shared memory name only after decode/publish stops.
  writer_.reset();
}
bool SceneNode::wait_for(std::chrono::milliseconds duration) {
  std::unique_lock<std::mutex> lock(wait_mutex_);
  return wake_.wait_for(lock, duration, [this] { return stopping_.load() || !rclcpp::ok(); });
}
void SceneNode::run() {
  bool retry = false;
  Clock::time_point last_publish{};
  while(!stopping_ && rclcpp::ok()) {
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      status_.state = retry ? "reconnecting" : "connecting";
      status_.online = false;
      if(retry) ++status_.reconnects;
    }
    try {
      VideoDecoder decoder(config_, stopping_);
      decoder.open();
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.state = "waiting_frame";
      }
      const double fps = std::clamp(decoder.nominal_fps(), 1.0, 240.0);
      const auto file_period = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / fps));
      auto file_deadline = Clock::now();
      DecodedFrame decoded;
      while(!stopping_ && rclcpp::ok() && decoder.next(decoded)) {
        if(config_.input_type == "file") {
          std::unique_lock<std::mutex> lock(wait_mutex_);
          wake_.wait_until(lock, file_deadline, [this] { return stopping_.load() || !rclcpp::ok(); });
          file_deadline = std::max(file_deadline + file_period, Clock::now());
        }
        if(stopping_ || !rclcpp::ok()) break;
        const auto t = Clock::now();
        {
          std::lock_guard<std::mutex> lock(status_mutex_);
          last_frame_ = t;
          status_.online = true;
          status_.state = "streaming";
          status_.last_error.clear();
          status_.width = decoded.metadata.width; status_.height = decoded.metadata.height;
        }
        if(max_publish_fps_ > 0 && std::chrono::duration<double>(t - last_publish).count() < 1.0 / max_publish_fps_) {
          std::lock_guard<std::mutex> lock(status_mutex_);
          ++status_.dropped_frames;
          continue;
        }
        decoded.metadata.received_stamp_ns = now().nanoseconds();
        auto reference = writer_->write(decoded.pixels.data(), decoded.pixels.size(), decoded.metadata);
        {
          std::lock_guard<std::mutex> lock(status_mutex_);
          // Liveness above follows decoding even when publication is dropped.
          if(reference) ++status_.published_frames; else ++status_.dropped_frames;
        }
        if(reference) { publish_frame(*reference); last_publish = t; }
      }
      if(stopping_ || !rclcpp::ok()) break;
      if(config_.input_type == "file" && !file_loop_) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        status_.state = "ended"; status_.online = false;
        return;
      }
      if(config_.input_type != "file") throw std::runtime_error("video stream ended");
    } catch(const std::exception& error) {
      if(stopping_ || !rclcpp::ok()) break;
      {
        std::lock_guard<std::mutex> lock(status_mutex_);
        ++status_.errors; status_.last_error = error.what(); status_.online = false; status_.state = "reconnecting";
      }
      RCLCPP_WARN(get_logger(), "video input interrupted; retrying: %s", error.what());
    }
    retry = true;
    if(wait_for(std::chrono::milliseconds(reconnect_delay_ms_))) break;
  }
}
void SceneNode::publish_frame(const video::FrameReference& ref) {
  const auto& m = ref.metadata;
  msgs::msg::VideoFrameRef message;
  message.header.stamp = rclcpp::Time(m.received_stamp_ns, get_clock()->get_clock_type());
  message.header.frame_id = frame_id_;
  message.shm_name = ref.shm_name; message.session_id = ref.session_id; message.sequence = m.sequence;
  message.slot = ref.slot; message.width = m.width; message.height = m.height; message.step = m.step;
  message.data_size = m.data_size; message.encoding = video::encoding(m.format);
  message.source_pts_valid = m.source_pts_valid != 0; message.source_pts_ns = m.source_pts_ns;
  frame_pub_->publish(message);
  if(legacy_pub_) {
    std_msgs::msg::String legacy;
    std::ostringstream json;
    json << "{\"seq\":" << m.sequence << ",\"ts_ns\":" << m.received_stamp_ns
         << ",\"session_id\":" << ref.session_id << ",\"slot\":" << ref.slot
         << ",\"shm_name\":\"" << ref.shm_name << "\"}";
    legacy.data = json.str(); legacy_pub_->publish(legacy);
  }
}
void SceneNode::publish_status() {
  msgs::msg::VideoStreamState message;
  const auto t = Clock::now();
  {
    std::lock_guard<std::mutex> lock(status_mutex_);
    message = status_;
    message.frame_age_s = last_frame_ == Clock::time_point{} ? -1 : std::chrono::duration<double>(t - last_frame_).count();
    message.online = status_.online && message.frame_age_s >= 0 && message.frame_age_s * 1000 < frame_timeout_ms_;
    if(status_.state == "streaming" && !message.online) message.state = "stalled";
    const double elapsed = std::chrono::duration<double>(t - fps_sample_time_).count();
    message.fps = elapsed > 0 ? (status_.published_frames - fps_sample_count_) / elapsed : 0;
    fps_sample_count_ = status_.published_frames; fps_sample_time_ = t;
  }
  message.header.stamp = now(); message.header.frame_id = frame_id_;
  status_pub_->publish(message);
}
}  // namespace uav::scene
