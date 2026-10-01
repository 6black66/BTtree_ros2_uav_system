#include "bt_nodes/node_context.hpp"

namespace bt_nodes
{

NodeContext& NodeContext::instance()
{
  static NodeContext ctx;
  return ctx;
}

void NodeContext::init(rclcpp::Node::SharedPtr node)
{
  node_ = node;

  // 导航状态订阅:进入即建,全节点共享
  nav_sub_ = node->create_subscription<msgs::msg::NavState>(
      "controller/nav_state", rclcpp::QoS(10),
      [this](msgs::msg::NavState::ConstSharedPtr msg) { setNavState(*msg); });
}

void NodeContext::setNavState(const msgs::msg::NavState& st)
{
  const std::lock_guard<std::mutex> lock(mtx_);
  nav_ = st;
  has_nav_ = true;
  ++nav_revision_;
}

msgs::msg::NavState NodeContext::navState() const
{
  const std::lock_guard<std::mutex> lock(mtx_);
  return nav_;
}

bool NodeContext::hasNavState() const
{
  const std::lock_guard<std::mutex> lock(mtx_);
  return has_nav_;
}

uint64_t NodeContext::navRevision() const
{
  const std::lock_guard<std::mutex> lock(mtx_);
  return nav_revision_;
}

}  // namespace bt_nodes
