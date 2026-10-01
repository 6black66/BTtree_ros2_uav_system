#pragma once

/// 行为进程通用骨架(action server 桩)
/// 真实行为:在 work() 里替换为算法调用;取消响应在 cancel 回调里
/// 保证"安全点退出"(如冻结吊舱指向)。

#include <msgs/action/task.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace uav_ros2_support
{

using Task = msgs::action::Task;
using GoalHandle = rclcpp_action::ServerGoalHandle<Task>;

struct Worker
{
  std::shared_ptr<std::atomic<bool>> finished;
  std::thread thread;
};

struct WorkerState
{
  std::atomic<bool> stopping{ false };
  std::mutex mutex;
  std::vector<Worker> workers;
  std::vector<std::shared_ptr<GoalHandle>> deferred_goals;
};

/// result_factory(work_ms) → 进程主循环
inline int runBehaviorServer(int argc, char** argv, const std::string& behavior, int work_ms,
                             std::function<std::string(const std::string& goal)> make_result)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(behavior + "_executor");
  node->declare_parameter<int>("work_ms", work_ms);
  node->declare_parameter<bool>("force_failure", false);
  const int configured_work_ms =
      std::max(0, static_cast<int>(node->get_parameter("work_ms").as_int()));
  const bool force_failure = node->get_parameter("force_failure").as_bool();
  auto worker_state = std::make_shared<WorkerState>();

  auto handle_goal = [](const rclcpp_action::GoalUUID&,
                        const std::shared_ptr<const Task::Goal>) {
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  };
  auto handle_cancel = [node](const std::shared_ptr<GoalHandle>) {
    RCLCPP_WARN(node->get_logger(), "cancel requested (safe point exit)");
    return rclcpp_action::CancelResponse::ACCEPT;
  };
  auto handle_accepted = [node, worker_state, configured_work_ms, force_failure, make_result](
                             const std::shared_ptr<GoalHandle> gh) {
    auto finished = std::make_shared<std::atomic<bool>>(false);
    std::thread worker([node, worker_state, finished, configured_work_ms, force_failure,
                        make_result, gh]() {
      struct CompletionMarker
      {
        std::shared_ptr<std::atomic<bool>> finished;
        ~CompletionMarker()
        {
          *finished = true;
        }
      } completion{ finished };

      const auto defer_goal_destruction = [&]() {
        std::lock_guard<std::mutex> lock(worker_state->mutex);
        worker_state->deferred_goals.push_back(gh);
      };

      try
      {
        auto feedback = std::make_shared<Task::Feedback>();
        auto result = std::make_shared<Task::Result>();

        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::milliseconds(configured_work_ms);
        while(std::chrono::steady_clock::now() < deadline)
        {
          if(worker_state->stopping || !rclcpp::ok())
          {
            defer_goal_destruction();
            return;
          }
          if(gh->is_canceling())
          {
            result->ok = false;
            result->result_json = R"({"cancelled":true})";
            gh->canceled(result);
            return;
          }
          feedback->progress_json = R"({"progress":"working"})";
          gh->publish_feedback(feedback);
          rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        if(worker_state->stopping || !rclcpp::ok())
        {
          defer_goal_destruction();
          return;
        }
        result->ok = !force_failure;
        result->result_json = force_failure ? R"({"stub_failure":true})" :
                                              make_result(gh->get_goal()->goal_json);
        gh->succeed(result);
      }
      catch(const std::exception& e)
      {
        if(worker_state->stopping || !rclcpp::ok())
        {
          defer_goal_destruction();
        }
        else
        {
          RCLCPP_ERROR(node->get_logger(), "action worker failed: %s", e.what());
        }
      }
    });

    std::lock_guard<std::mutex> lock(worker_state->mutex);
    auto it = worker_state->workers.begin();
    while(it != worker_state->workers.end())
    {
      if(*it->finished)
      {
        it->thread.join();
        it = worker_state->workers.erase(it);
      }
      else
      {
        ++it;
      }
    }
    worker_state->workers.push_back({ finished, std::move(worker) });
  };

  auto server = rclcpp_action::create_server<Task>(
      node, "behaviors/" + behavior, handle_goal, handle_cancel, handle_accepted);

  RCLCPP_INFO(node->get_logger(), "behavior '%s' up (stub, work_ms=%d, force_failure=%s)",
              behavior.c_str(), configured_work_ms, force_failure ? "true" : "false");
  rclcpp::spin(node);
  worker_state->stopping = true;
  std::vector<Worker> workers;
  {
    std::lock_guard<std::mutex> lock(worker_state->mutex);
    workers.swap(worker_state->workers);
  }
  for(auto& worker : workers)
  {
    if(worker.thread.joinable())
    {
      worker.thread.join();
    }
  }
  server.reset();
  {
    std::lock_guard<std::mutex> lock(worker_state->mutex);
    worker_state->deferred_goals.clear();
  }
  rclcpp::shutdown();
  return 0;
}

}  // namespace uav_ros2_support
