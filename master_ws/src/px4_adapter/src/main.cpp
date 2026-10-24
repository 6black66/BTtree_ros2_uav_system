#include <px4_adapter/adapter_node.hpp>
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<uav::px4::AdapterNode>();
    rclcpp::spin(node);
  } catch(const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("px4_adapter"),"%s",error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
