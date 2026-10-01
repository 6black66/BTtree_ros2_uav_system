#include "bt_nodes/shm_sync.hpp"

namespace bt_nodes
{

BT::NodeStatus ShmSync::tick()
{
  // TODO: 迁入 shared/ipc/blackboard 后,按映射表在 BT 黑板与
  //       共享内存黑板之间双向同步位姿/吊舱状态/任务指令。
  //       图像帧永远不进黑板,走 Episode 的帧共享内存通道。
  config().blackboard->set("robot_pose", std::string(R"({"lat":30.0,"lon":120.0})"));
  config().blackboard->set("gimbal_state", std::string(R"({"yaw":0,"pitch":-30})"));
  return BT::NodeStatus::SUCCESS;
}

}  // namespace bt_nodes
