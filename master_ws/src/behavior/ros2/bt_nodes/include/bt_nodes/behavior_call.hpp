#pragma once

#include "bt_nodes/node_context.hpp"

#include <behaviortree_cpp/action_node.h>
#include <msgs/action/task.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <map>
#include <memory>

namespace bt_nodes
{

/// 通用跨进程行为调用(CoroActionNode,非阻塞、可抢占)。
///
/// 端口:
///   id       行为进程名(find/observe/track/refind/report)→ action "behaviors/<id>"
///   goal     输入 json(黑板键)
///   result   输出 json(黑板键)
///   timeout_ms 超时(默认 60s),超时 FAILURE 并取消 goal
///
/// halt() → 取消 goal(行为进程按"安全点取消"约定退出)
/// 通信/超时失败写 phase_outcome=ERROR；track 明确返回业务失败写 LOST。
class BehaviorCall : public BT::CoroActionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("id", "behavior process name"),
             BT::InputPort<std::string>("goal", "{}", "goal json (blackboard key)"),
             BT::OutputPort<std::string>("result", "result json (blackboard key)"),
             BT::InputPort<int>("timeout_ms", 60000, "call timeout") };
  }

  BehaviorCall(const std::string& name, const BT::NodeConfig& config)
    : BT::CoroActionNode(name, config)
  {}

  BT::NodeStatus tick() override;
  void halt() override;

private:
  using Client = rclcpp_action::Client<msgs::action::Task>;

  Client::SharedPtr client(const std::string& id);

  std::string current_id_;
  std::map<std::string, Client::SharedPtr> clients_;

  // 一次调用的中间状态
  Client::GoalHandle::SharedPtr handle_;
  std::chrono::steady_clock::time_point deadline_;
  bool goal_sent_ = false;
  bool cancel_sent_ = false;

  void resetCall();
};

}  // namespace bt_nodes
