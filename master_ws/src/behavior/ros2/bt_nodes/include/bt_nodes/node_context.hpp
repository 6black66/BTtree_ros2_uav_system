#pragma once

#include <msgs/msg/nav_state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <string>

namespace bt_nodes
{

/// 行为执行器 ROS 2 适配节点共享的通信上下文。
class NodeContext
{
public:
  static NodeContext& instance();

  /// 宿主进程启动时调用(一次)
  void init(rclcpp::Node::SharedPtr node);

  rclcpp::Node::SharedPtr node() const
  {
    return node_;
  }

  /// 最新导航状态(GoTo/Arrived 等节点消费;init 时订阅刷新)
  void setNavState(const msgs::msg::NavState& st);
  msgs::msg::NavState navState() const;
  bool hasNavState() const;
  uint64_t navRevision() const;

private:
  NodeContext() = default;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<msgs::msg::NavState>::SharedPtr nav_sub_;

  mutable std::mutex mtx_;
  msgs::msg::NavState nav_;
  bool has_nav_ = false;
  uint64_t nav_revision_ = 0;
};

}  // namespace bt_nodes
