/// 局部重发现占位进程：根据最后跟踪状态生成局部搜索计划。
#include <refind/executor.hpp>
#include <uav_ros2_support/behavior_server.hpp>

#include <string>

int main(int argc, char** argv)
{
  uav::refind::Executor executor;
  return uav_ros2_support::runBehaviorServer(
      argc, argv, "refind", 800, [&executor](const std::string&) {
        const auto request = executor.execute({});
        const auto& area = request.areas.front();
        return std::string("{\"local_reacquisition\":true,\"lat\":") +
               std::to_string(area.latitude) + ",\"lon\":" +
               std::to_string(area.longitude) + "}";
      });
}
