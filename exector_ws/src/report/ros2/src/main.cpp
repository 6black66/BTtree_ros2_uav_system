/// 上报行为进程:目标切片 + 目标包打包发送(断链时本地缓存回传)。
/// 骨桩 1s 返回上报回执。
#include <uav_ros2_support/behavior_server.hpp>
#include <report/executor.hpp>

#include <string>

int main(int argc, char** argv)
{
  uav::report::Executor executor;
  return uav_ros2_support::runBehaviorServer(
      argc, argv, "report", 1000, [&executor](const std::string&) {
        const auto receipt = executor.execute({});
        return std::string("{\"ok\":") + (receipt.accepted ? "true" : "false") +
               ",\"report_id\":" + std::to_string(receipt.report_id) +
               ",\"channel\":\"" + receipt.channel + "\"}";
      });
}
