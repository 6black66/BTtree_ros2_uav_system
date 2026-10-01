#pragma once

#include <behaviortree_cpp/condition_node.h>

namespace bt_nodes
{

/// 自检(待命阶段):骨架阶段只打日志
class HealthCheck : public BT::ConditionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return {};
  }
  HealthCheck(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {}
  BT::NodeStatus tick() override;
};

/// 候选存在：支持非空检测框数组或 Observe 返回的 {"ok":true,...} 对象。
class CandidatesFound : public BT::ConditionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("candidates") };
  }
  CandidatesFound(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {}
  BT::NodeStatus tick() override;
};

/// 上行链路正常:黑板 {report_ack} json 的 ok == true
class UplinkOk : public BT::ConditionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("ack") };
  }
  UplinkOk(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config)
  {}
  BT::NodeStatus tick() override;
};

}  // namespace bt_nodes
