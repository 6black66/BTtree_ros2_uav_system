#include "bt_nodes/behavior_call.hpp"

#include <chrono>

namespace bt_nodes
{

using msgs::action::Task;

BehaviorCall::Client::SharedPtr BehaviorCall::client(const std::string& id)
{
  auto it = clients_.find(id);
  if(it == clients_.end())
  {
    auto c = rclcpp_action::create_client<Task>(NodeContext::instance().node(),
                                                "behaviors/" + id);
    it = clients_.emplace(id, std::move(c)).first;
  }
  return it->second;
}

BT::NodeStatus BehaviorCall::tick()
{
  std::string id, goal;
  int timeout_ms = 60000;
  if(!getInput("id", id) || !getInput("goal", goal) || !getInput("timeout_ms", timeout_ms))
  {
    config().blackboard->set("phase_outcome", std::string("ERROR"));
    return BT::NodeStatus::FAILURE;
  }

  auto node = NodeContext::instance().node();
  auto cl = client(id);

  // ---- 阶段1:发 goal(仅首次;协程挂起期间不会重入此分支) ----
  if(!goal_sent_)
  {
    config().blackboard->set("phase_outcome", std::string{});
    if(deadline_ == std::chrono::steady_clock::time_point{})
    {
      deadline_ =
          std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    }
    while(!cl->wait_for_action_server(std::chrono::milliseconds(0)))
    {
      if(std::chrono::steady_clock::now() > deadline_)
      {
        RCLCPP_WARN(node->get_logger(), "[bt:%s] action server unavailable",
                    name().c_str());
        config().blackboard->set("phase_outcome", std::string("ERROR"));
        resetCall();
        return BT::NodeStatus::FAILURE;
      }
      setStatusRunningAndYield();  // 行为进程未就绪,等下一 tick
    }

    auto goal_msg = Task::Goal();
    goal_msg.behavior = id;
    goal_msg.goal_json = goal;

    cancel_sent_ = false;
    current_id_ = id;
    goal_sent_ = true;

    auto goal_future = cl->async_send_goal(goal_msg);
    while(goal_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      setStatusRunningAndYield();
    }
    handle_ = goal_future.get();
    if(!handle_)
    {
      RCLCPP_WARN(node->get_logger(), "[bt:%s] goal rejected", name().c_str());
      config().blackboard->set("phase_outcome", std::string("ERROR"));
      resetCall();
      return BT::NodeStatus::FAILURE;
    }
  }

  // ---- 阶段2:等结果(超时取消) ----
  auto result_future = cl->async_get_result(handle_);
  while(result_future.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
  {
    if(std::chrono::steady_clock::now() > deadline_ && !cancel_sent_)
    {
      cl->async_cancel_goal(handle_);
      cancel_sent_ = true;
      config().blackboard->set("phase_outcome", std::string("ERROR"));
      RCLCPP_WARN(node->get_logger(), "[bt:%s] timeout, cancelled", name().c_str());
    }
    setStatusRunningAndYield();
  }

  const auto wrapped = result_future.get();
  resetCall();
  if(wrapped.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    config().blackboard->set("phase_outcome", std::string("ERROR"));
    return BT::NodeStatus::FAILURE;
  }
  setOutput("result", wrapped.result->result_json);
  if(!wrapped.result->ok)
  {
    config().blackboard->set(
        "phase_outcome", std::string(current_id_ == "track" ? "LOST" : "ERROR"));
    return BT::NodeStatus::FAILURE;
  }
  config().blackboard->set("phase_outcome", std::string{});
  return BT::NodeStatus::SUCCESS;
}

void BehaviorCall::halt()
{
  if(handle_ && !cancel_sent_)
  {
    client(current_id_)->async_cancel_goal(handle_);
  }
  resetCall();
  CoroActionNode::halt();
}

void BehaviorCall::resetCall()
{
  handle_ = nullptr;
  goal_sent_ = false;
  cancel_sent_ = false;
  deadline_ = {};
}

}  // namespace bt_nodes
