#include "flight_controller_ros2/controller_node.hpp"
#include <exception>
int main(int argc,char** argv) {
  rclcpp::init(argc,argv);
  try {
    auto node=std::make_shared<qlet::ControllerNode>();
    rclcpp::spin(node);
  } catch(const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("controller"),"%s",e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
