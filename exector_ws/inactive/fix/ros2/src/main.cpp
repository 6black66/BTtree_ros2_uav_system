/// 定位行为进程:多帧 + 位姿解算经纬度(可选地图匹配/双机协同)。
/// 骨桩 1.5s 返回模拟定位结果。
#include <uav_ros2_support/behavior_server.hpp>
#include <fix/executor.hpp>

#include <string>

int main(int argc, char** argv)
{
  uav::fix::Executor executor;
  return uav_ros2_support::runBehaviorServer(
      argc, argv, "fix", 1500, [&executor](const std::string&) {
        const auto location = executor.execute({});
        return std::string("{\"target_id\":") + std::to_string(location.target_id) +
               ",\"lat\":" + std::to_string(location.latitude) + ",\"lon\":" +
               std::to_string(location.longitude) + ",\"error_m\":" +
               std::to_string(location.error_m) + ",\"frames_used\":" +
               std::to_string(location.frames_used) + "}";
      });
}
