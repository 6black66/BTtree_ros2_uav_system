#pragma once

#include "bt_nodes/node_context.hpp"

#include <behaviortree_cpp/action_node.h>
#include <msgs/srv/go_to.hpp>
#include <msgs/srv/gimbal_cmd.hpp>
#include <msgs/srv/orbit_cmd.hpp>

#include <chrono>
#include <memory>

namespace bt_nodes
{

/// 转场:下发 GoTo 给 Controller 并等待到位(nav_state.arrived == true)。
/// 端口 task 为 json 字符串(含 lat/lon/alt/radius),也可用 lat/lon 显式端口覆盖。
class GoTo : public BT::CoroActionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("task", "", "task json with dest"),
      BT::InputPort<double>("lat", 0.0, "explicit latitude"),
      BT::InputPort<double>("lon", 0.0, "explicit longitude"),
      BT::InputPort<double>("timeout_s", 300.0, "arrival timeout")
    };
  }
  GoTo(const std::string& name, const BT::NodeConfig& config)
    : BT::CoroActionNode(name, config)
  {}
  BT::NodeStatus tick() override;
  void halt() override;

private:
  rclcpp::Client<msgs::srv::GoTo>::SharedPtr client_;
  bool sent_ = false;
  uint64_t nav_revision_at_send_ = 0;
  std::chrono::steady_clock::time_point deadline_;
};

/// 建立圆盘旋(就位阶段)
class OrbitCommand : public BT::CoroActionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("radius_m", 500.0, "orbit radius"),
             BT::InputPort<double>("timeout_s", 10.0, "service timeout") };
  }
  OrbitCommand(const std::string& name, const BT::NodeConfig& config)
    : BT::CoroActionNode(name, config)
  {}
  BT::NodeStatus tick() override;
  void halt() override
  {
    CoroActionNode::halt();
  }
};

/// 吊舱指令(point / preset_search / zoom / freeze)
class GimbalCommand : public BT::CoroActionNode
{
public:
  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::string>("command", "preset_search"),
             BT::InputPort<double>("yaw_deg", 0.0, ""),
             BT::InputPort<double>("pitch_deg", 0.0, ""),
             BT::InputPort<double>("zoom", 1.0, ""),
             BT::InputPort<double>("timeout_s", 10.0, "") };
  }
  GimbalCommand(const std::string& name, const BT::NodeConfig& config)
    : BT::CoroActionNode(name, config)
  {}
  BT::NodeStatus tick() override;
  void halt() override
  {
    CoroActionNode::halt();
  }
};

}  // namespace bt_nodes
