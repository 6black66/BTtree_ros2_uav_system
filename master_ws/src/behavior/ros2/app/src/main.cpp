/// behavior_executor —— 行为执行进程(第三层的心脏)
///
/// 托管 PhaseManager:
/// - 初始进入 standby 阶段
/// - 订阅当前无人机命名空间下的 mission/directive
/// - 10Hz tick:阶段树 SUCCESS/FAILURE → 转移表换树(黑板跨阶段接力)
/// - 进程生命周期由 launch 管理，本节点只负责任务编排
#include <behavior/phase_manager.hpp>
#include <bt_nodes/node_context.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <framework/detail/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <csignal>

using namespace std::chrono_literals;

static std::atomic<bool> g_running{ true };
static void handleSignal(int)
{
  g_running = false;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("behavior_executor");

  // 默认配置从 bringup 包取;参数可覆盖
  const std::string bringup_share =
      ament_index_cpp::get_package_share_directory("bringup");
  node->declare_parameter<std::string>("manifests_dir",
                                       bringup_share + "/config/executors");
  node->declare_parameter<std::string>("transitions",
                                       bringup_share + "/config/transitions/transitions.json");
  node->declare_parameter<std::string>(
      "plugin_dir", ament_index_cpp::get_package_prefix("bt_capability_plugins") + "/lib");
  const std::string manifests_dir =
      node->get_parameter("manifests_dir").as_string();
  const std::string transitions = node->get_parameter("transitions").as_string();
  const std::string plugin_dir = node->get_parameter("plugin_dir").as_string();

  bt_nodes::NodeContext::instance().init(node);

  auto bb = BT::Blackboard::create();
  bb->set("role", std::string("uav"));
  bb->set("candidates", std::string("[]"));
  bb->set("phase_outcome", std::string{});

  const auto logger = node->get_logger();
  auto phase_state_pub = node->create_publisher<std_msgs::msg::String>(
      "behavior/phase_state", rclcpp::QoS(1).reliable().transient_local());
  behavior::PhaseManager phases(
      bb, manifests_dir, transitions, plugin_dir,
      [logger](behavior::LogLevel level, const std::string& message) {
        switch(level)
        {
          case behavior::LogLevel::info:
            RCLCPP_INFO(logger, "[phase] %s", message.c_str());
            break;
          case behavior::LogLevel::warning:
            RCLCPP_WARN(logger, "[phase] %s", message.c_str());
            break;
          case behavior::LogLevel::error:
            RCLCPP_ERROR(logger, "[phase] %s", message.c_str());
            break;
        }
      },
      [phase_state_pub](const behavior::PhaseEvent& event) {
        nlohmann::json state;
        state["phase"] = event.phase;
        state["previous_phase"] = event.previous_phase;
        state["event"] = event.event;
        state["result"] = event.result;
        state["tree"] = event.tree;
        state["plugins"] = event.plugins;
        state["sequence"] = event.sequence;
        state["message"] = event.message;
        std_msgs::msg::String msg;
        msg.data = state.dump();
        phase_state_pub->publish(msg);
      });

  // 命令式阶段指令(mission → executor)
  auto directive_sub = node->create_subscription<std_msgs::msg::String>(
      "mission/directive", rclcpp::QoS(10),
      [&phases](std_msgs::msg::String::ConstSharedPtr msg) {
        RCLCPP_INFO(rclcpp::get_logger("executor"), "[directive] %s",
                    msg->data.c_str());
        phases.commandPhase(msg->data);
      });

  // 初始阶段
  if(!phases.switchTo("standby"))
  {
    RCLCPP_FATAL(node->get_logger(), "cannot enter initial phase: standby");
    return 1;
  }

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  rclcpp::WallRate rate(10.0);  // 阶段树 tick 10Hz

  while(g_running && rclcpp::ok())
  {
    rclcpp::spin_some(node);
    phases.tickOnce();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
