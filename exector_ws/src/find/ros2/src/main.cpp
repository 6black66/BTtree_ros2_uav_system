/// 发现行为进程:搜索模式检测候选。
/// 骨桩 1.5s 返回一个候选;真实实现接帧共享内存 + 检测算法。
#include <uav_ros2_support/behavior_server.hpp>
#include <find/executor.hpp>

#include <string>

int main(int argc, char** argv)
{
  uav::find::Executor executor;
  return uav_ros2_support::runBehaviorServer(
      argc, argv, "find", 1500, [&executor](const std::string&) {
        const auto candidates = executor.execute({});
        const auto& candidate = candidates.front();
        return std::string("[{\"candidate_id\":") +
               std::to_string(candidate.id) + ",\"bbox\":[" +
               std::to_string(candidate.bounding_box[0]) + "," +
               std::to_string(candidate.bounding_box[1]) + "," +
               std::to_string(candidate.bounding_box[2]) + "," +
               std::to_string(candidate.bounding_box[3]) + "],\"confidence\":" +
               std::to_string(candidate.confidence) + "}]";
      });
}
