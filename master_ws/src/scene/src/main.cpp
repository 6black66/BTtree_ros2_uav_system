#include <video_gateway_ros2/scene_node.hpp>
#include <exception>
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<uav::scene::SceneNode>();
    rclcpp::spin(node);
  } catch(const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("scene"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
