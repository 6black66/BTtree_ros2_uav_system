#pragma once

#include "bt_nodes/node_context.hpp"

#include <behaviortree_cpp/action_node.h>
#include <std_msgs/msg/string.hpp>

namespace bt_nodes
{

/// 等待任务指派(待命阶段)。
/// 订阅当前执行器命名空间下的 mission/task。
class WaitForTask : public BT::CoroActionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<std::string>("task", "task json (blackboard key)") };
  }
  WaitForTask(const std::string& name, const BT::NodeConfig& config);
  BT::NodeStatus tick() override;

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  bool received_ = false;
  std::string payload_;
};

/// 向任务管理上报事件(on_station / ready / report_done ...)
class Notify : public BT::SyncActionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("event", "event name") };
  }
  Notify(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}
  BT::NodeStatus tick() override;
};

}  // namespace bt_nodes
