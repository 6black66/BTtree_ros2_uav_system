#include <uav_shared_video/frame_ring.hpp>
#include <msgs/msg/video_frame_ref.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <memory>

// Minimal consumer example. Link uav_shared_video in perception nodes to reuse the API.
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("scene_frame_consumer");
  const int max_frames = node->declare_parameter<int>("max_frames", 10);
  const auto topic = node->declare_parameter<std::string>("frame_topic", "scene/frame");
  std::unique_ptr<uav::video::FrameReader> reader;
  std::string current_name;
  uav::video::Frame frame;
  int count = 0;
  auto subscription = node->create_subscription<msgs::msg::VideoFrameRef>(topic, rclcpp::QoS(1).best_effort(),
      [&](msgs::msg::VideoFrameRef::ConstSharedPtr message) {
        try {
          if(!reader || current_name != message->shm_name) {
            reader = std::make_unique<uav::video::FrameReader>(message->shm_name);
            current_name = message->shm_name;
          }
          uav::video::FrameReference ref;
          ref.shm_name = message->shm_name; ref.session_id = message->session_id;
          ref.slot = message->slot; ref.metadata.sequence = message->sequence;
          if(!reader->read(ref, frame)) return;  // Busy/expired references are normal.
          const auto& m = frame.metadata;
          std::cout << "FRAME " << message->session_id << ' ' << m.sequence << ' '
                    << m.width << ' ' << m.height << ' ' << m.data_size << ' '
                    << uav::video::encoding(m.format) << ' ' << int(frame.pixels.at(0)) << ' '
                    << int(frame.pixels.at(1)) << ' ' << int(frame.pixels.at(2)) << std::endl;
          if(max_frames > 0 && ++count >= max_frames) rclcpp::shutdown();
        } catch(const std::exception& error) {
          reader.reset();
          RCLCPP_WARN(node->get_logger(), "frame read skipped: %s", error.what());
        }
      });
  rclcpp::spin(node);
  if(rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
