/// 识别行为进程:吊舱指向候选 + 变焦 + 多帧分类确认。
/// 骨桩 1.2s 返回确认结果(真实实现:yolo/onnxruntime)。
#include <uav_ros2_support/behavior_server.hpp>
#include <observe/executor.hpp>

#include <string>

int main(int argc, char** argv)
{
  uav::observe::Executor executor;
  return uav_ros2_support::runBehaviorServer(
      argc, argv, "observe", 1200, [&executor](const std::string&) {
        const auto target = executor.execute({});
        return std::string("{\"ok\":true,\"confidence\":") +
               std::to_string(target.confidence) + ",\"target_id\":" +
               std::to_string(target.id) + ",\"target_type\":\"" + target.type +
               "\",\"bbox\":[" + std::to_string(target.bounding_box[0]) + "," +
               std::to_string(target.bounding_box[1]) + "," +
               std::to_string(target.bounding_box[2]) + "," +
               std::to_string(target.bounding_box[3]) + "]}";
      });
}
