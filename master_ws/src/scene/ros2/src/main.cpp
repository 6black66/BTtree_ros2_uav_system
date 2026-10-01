/// Episode 场景接口节点(HAL):吊舱 RTSP 视频解码 → 帧共享内存。
/// 骨桩:5Hz 发布帧时间戳(真实实现:解码线程写帧 ring,见 rtsp_shm_consumer)
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("episode");
  auto pub = node->create_publisher<std_msgs::msg::String>("episode/frame_stamp",
                                                           rclcpp::QoS(10));

  uint64_t seq = 0;
  rclcpp::TimerBase::SharedPtr timer = node->create_wall_timer(
      200ms, [&]() {
        std_msgs::msg::String msg;
        msg.data = R"({"seq":)" + std::to_string(seq++) + R"(,"ts":0.0})";
        pub->publish(msg);
      });

  RCLCPP_INFO(node->get_logger(), "episode up (stub)");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
