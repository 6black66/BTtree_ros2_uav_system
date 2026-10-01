/// Controller 接口节点(HAL):飞控与吊舱的指令/状态网关。
/// 骨桩:
/// - GoTo 服务受理后 4 秒模拟到位(nav_state.arrived=true,模式回 ORBIT)
/// - Orbit/Gimbal 服务直接应答 ok
/// - 1Hz 发布导航状态
/// 真实实现:替换为飞控/吊舱链路,状态写共享内存黑板
#include <msgs/msg/nav_state.hpp>
#include <msgs/srv/go_to.hpp>
#include <msgs/srv/gimbal_cmd.hpp>
#include <msgs/srv/orbit_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("controller");
  node->declare_parameter<int>("goto_delay_ms", 4000);
  node->declare_parameter<bool>("force_failure", false);
  const int goto_delay_ms =
      std::max(0, static_cast<int>(node->get_parameter("goto_delay_ms").as_int()));
  const bool force_failure = node->get_parameter("force_failure").as_bool();

  auto nav_pub = node->create_publisher<msgs::msg::NavState>("controller/nav_state",
                                                             rclcpp::QoS(10));
  auto gimbal_pub = node->create_publisher<std_msgs::msg::String>(
      "controller/gimbal_state", rclcpp::QoS(10));

  msgs::msg::NavState nav;
  nav.mode = "STANDBY";
  nav.arrived = true;
  std::string gimbal_command = "stow";
  double gimbal_yaw = 0.0;
  double gimbal_pitch = 0.0;
  double gimbal_zoom = 1.0;

  rclcpp::TimerBase::SharedPtr arrival_timer;
  auto goto_srv = node->create_service<msgs::srv::GoTo>(
      "controller/goto",
      [&, goto_delay_ms, force_failure](const msgs::srv::GoTo::Request::SharedPtr req,
          msgs::srv::GoTo::Response::SharedPtr res) {
        RCLCPP_INFO(node->get_logger(), "GoTo(%.4f, %.4f) accepted", req->lat,
                    req->lon);
        res->ok = !force_failure;
        res->message = force_failure ? "forced stub failure" : "accepted";
        if(force_failure)
        {
          return;
        }
        nav.mode = "TRANSIT";
        nav.arrived = false;

        if(arrival_timer)
        {
          arrival_timer->cancel();  // 新目标覆盖旧的到位计时
        }
        arrival_timer = node->create_wall_timer(std::chrono::milliseconds(goto_delay_ms), [&]() {
          nav.mode = "ORBIT";
          nav.arrived = true;  // 模拟到位
          RCLCPP_INFO(node->get_logger(), "arrived");
          arrival_timer->cancel();
        });
      });

  auto orbit_srv = node->create_service<msgs::srv::OrbitCmd>(
      "controller/orbit",
      [&, force_failure](const msgs::srv::OrbitCmd::Request::SharedPtr req,
          msgs::srv::OrbitCmd::Response::SharedPtr res) {
        nav.mode = "ORBIT";
        res->ok = !force_failure;
        res->message = force_failure ? "forced stub failure" :
                                       "orbit r=" + std::to_string(req->radius_m);
      });

  auto gimbal_srv = node->create_service<msgs::srv::GimbalCmd>(
      "controller/gimbal",
      [&, force_failure](const msgs::srv::GimbalCmd::Request::SharedPtr req,
          msgs::srv::GimbalCmd::Response::SharedPtr res) {
        res->ok = !force_failure;
        res->message = force_failure ? "forced stub failure" : "gimbal " + req->command;
        if(!force_failure)
        {
          gimbal_command = req->command;
          gimbal_yaw = req->yaw_deg;
          gimbal_pitch = req->pitch_deg;
          gimbal_zoom = req->zoom;
        }
      });

  rclcpp::TimerBase::SharedPtr pub_timer = node->create_wall_timer(
      1s, [&]() {
        nav_pub->publish(nav);
        std_msgs::msg::String gimbal;
        gimbal.data = "{\"command\":\"" + gimbal_command + "\",\"yaw_deg\":" +
                      std::to_string(gimbal_yaw) + ",\"pitch_deg\":" +
                      std::to_string(gimbal_pitch) + ",\"zoom\":" +
                      std::to_string(gimbal_zoom) + "}";
        gimbal_pub->publish(gimbal);
      });

  RCLCPP_INFO(node->get_logger(), "controller up (stub)");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
