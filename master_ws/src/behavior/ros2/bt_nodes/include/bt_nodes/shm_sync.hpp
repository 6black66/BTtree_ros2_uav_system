#pragma once

#include "bt_nodes/node_context.hpp"

#include <behaviortree_cpp/action_node.h>

namespace bt_nodes
{

/// 黑板同步桩(BT Blackboard ↔ IPC 共享内存黑板)。
/// 真实现迁移 shared/ipc/blackboard 后,按映射表双向同步;
/// 骨架阶段:向黑板写入模拟的位姿/吊舱状态,让后续阶段可用。
class ShmSync : public BT::SyncActionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("map", "", "bt<->shm mapping (reserved)") };
  }
  ShmSync(const std::string& name, const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
  {}
  BT::NodeStatus tick() override;
};

}  // namespace bt_nodes
