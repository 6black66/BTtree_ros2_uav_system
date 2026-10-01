/// 任务管理节点(master 侧):任务分配与阶段指控。
/// 骨桩逻辑:
/// - 收到 executor 事件 on_station → 下发阶段指令 find(开始搜索)
/// - 收到 report_done          → 下发 find(搜索下一个目标)
/// - 真实实现:六机任务分配/航迹规划/任务重分配
#include <mission/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("mission_executor");
  node->declare_parameter<std::string>("target_namespace", "uav1");
  node->declare_parameter<bool>("auto_start", true);
  node->declare_parameter<int>("auto_start_delay_ms", 1000);
  std::string target_ns = node->get_parameter("target_namespace").as_string();
  if(target_ns.empty() || target_ns.front() != '/')
  {
    target_ns.insert(target_ns.begin(), '/');
  }
  while(target_ns.size() > 1 && target_ns.back() == '/')
  {
    target_ns.pop_back();
  }

  auto directive = node->create_publisher<std_msgs::msg::String>(
      target_ns + "/mission/directive", rclcpp::QoS(10));
  auto task = node->create_publisher<std_msgs::msg::String>(
      target_ns + "/mission/task", rclcpp::QoS(1).reliable().transient_local());
  uav::mission::Executor executor;

  rclcpp::TimerBase::SharedPtr auto_start_timer;
  if(node->get_parameter("auto_start").as_bool())
  {
    const auto delay = std::chrono::milliseconds(
        std::max<int64_t>(0, node->get_parameter("auto_start_delay_ms").as_int()));
    auto_start_timer = node->create_wall_timer(delay, [node, task, &auto_start_timer]() {
      std_msgs::msg::String message;
      message.data = R"({"task_id":"demo-1","lat":30.0,"lon":120.0,"alt_m":100.0})";
      task->publish(message);
      RCLCPP_INFO(node->get_logger(), "auto-start task published");
      auto_start_timer->cancel();
    });
  }

  auto events = node->create_subscription<std_msgs::msg::String>(
      target_ns + "/mission/events", rclcpp::QoS(10),
      [&](std_msgs::msg::String::ConstSharedPtr msg) {
        RCLCPP_INFO(node->get_logger(), "event: %s", msg->data.c_str());
        const auto command = executor.directiveForEvent(msg->data);
        if(command)
        {
          std_msgs::msg::String d;
          d.data = uav::mission::phaseName(*command);
          directive->publish(d);
          RCLCPP_INFO(node->get_logger(), "directive -> %s", d.data.c_str());
        }
      });

  RCLCPP_INFO(node->get_logger(), "mission up (stub)");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
