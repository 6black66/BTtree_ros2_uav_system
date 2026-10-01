#include "bt_nodes/mission_nodes.hpp"

#include <mutex>

namespace bt_nodes
{

namespace
{
std::mutex consumed_task_mutex;
std::string consumed_task;
}  // namespace

WaitForTask::WaitForTask(const std::string& name, const BT::NodeConfig& config)
  : BT::CoroActionNode(name, config)
{
  sub_ = NodeContext::instance().node()->create_subscription<std_msgs::msg::String>(
      "mission/task", rclcpp::QoS(1).reliable().transient_local(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(consumed_task_mutex);
        if(!received_ && msg->data != consumed_task)
        {
          payload_ = msg->data;
          received_ = true;
        }
      });
}

BT::NodeStatus WaitForTask::tick()
{
  while(!received_)
  {
    setStatusRunningAndYield();
  }
  {
    const std::lock_guard<std::mutex> lock(consumed_task_mutex);
    consumed_task = payload_;
  }
  setOutput("task", payload_);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus Notify::tick()
{
  std::string event;
  if(!getInput("event", event))
  {
    return BT::NodeStatus::FAILURE;
  }
  auto node = NodeContext::instance().node();
  static auto pub =
      node->create_publisher<std_msgs::msg::String>("mission/events", rclcpp::QoS(10));
  std_msgs::msg::String msg;
  msg.data = event;
  pub->publish(msg);
  RCLCPP_INFO(node->get_logger(), "[event] %s", event.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes
