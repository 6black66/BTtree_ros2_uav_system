#include <bt_nodes/behavior_call.hpp>
#include <bt_nodes/conditions.hpp>
#include <bt_nodes/mission_nodes.hpp>
#include <bt_nodes/nav_nodes.hpp>
#include <bt_nodes/shm_sync.hpp>

#include <behaviortree_cpp/bt_factory.h>

// 插件按可复用能力拆分，而不是按阶段拆分。同一个库可被多个阶段清单引用，
// PhaseManager 会保证进程内只加载一次。
BT_REGISTER_NODES(factory)
{
#if defined(CAPABILITY_MISSION)
  factory.registerNodeType<bt_nodes::WaitForTask>("MissionWaitForTask");
  factory.registerNodeType<bt_nodes::Notify>("MissionNotify");
#elif defined(CAPABILITY_NAVIGATION)
  factory.registerNodeType<bt_nodes::GoTo>("NavigationGoTo");
#elif defined(CAPABILITY_CONTROLLER)
  factory.registerNodeType<bt_nodes::HealthCheck>("ControllerHealthCheck");
  factory.registerNodeType<bt_nodes::OrbitCommand>("ControllerOrbit");
  factory.registerNodeType<bt_nodes::GimbalCommand>("ControllerGimbal");
#elif defined(CAPABILITY_SCENE)
  factory.registerNodeType<bt_nodes::ShmSync>("SceneSync");
#elif defined(CAPABILITY_FIND)
  factory.registerNodeType<bt_nodes::BehaviorCall>("FindAction");
  factory.registerNodeType<bt_nodes::CandidatesFound>("FindCandidatesFound");
#elif defined(CAPABILITY_OBSERVE)
  factory.registerNodeType<bt_nodes::BehaviorCall>("ObserveAction");
#elif defined(CAPABILITY_TRACK)
  factory.registerNodeType<bt_nodes::BehaviorCall>("TrackAction");
#elif defined(CAPABILITY_REPORT)
  factory.registerNodeType<bt_nodes::BehaviorCall>("ReportAction");
  factory.registerNodeType<bt_nodes::UplinkOk>("ReportUplinkOk");
#elif defined(CAPABILITY_REFIND)
  factory.registerNodeType<bt_nodes::BehaviorCall>("RefindAction");
#else
#error "A capability plugin target must define one CAPABILITY_* macro"
#endif
}
