/// 跟踪行为进程:目标锁定 + 自动跟踪 + 遮挡记忆外推。
/// 骨桩 2s 返回一段 TRACKING 状态。
#include <uav_ros2_support/behavior_server.hpp>
#include <track/executor.hpp>

#include <string>

int main(int argc, char** argv)
{
  uav::track::Executor executor;
  return uav_ros2_support::runBehaviorServer(
      argc, argv, "track", 2000, [&executor](const std::string&) {
        const auto track = executor.execute({});
        return std::string("{\"target_id\":") + std::to_string(track.target_id) +
               ",\"state\":\"TRACKING\",\"lat\":" +
               std::to_string(track.latitude) + ",\"lon\":" +
               std::to_string(track.longitude) + ",\"heading_deg\":" +
               std::to_string(track.heading_deg) + ",\"speed_mps\":" +
               std::to_string(track.speed_mps) + "}";
      });
}
