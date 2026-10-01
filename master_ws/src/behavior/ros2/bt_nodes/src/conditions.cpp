#include "bt_nodes/conditions.hpp"

#include "bt_nodes/node_context.hpp"
#include "framework/detail/json.hpp"

namespace bt_nodes
{

using json = nlohmann::json;

namespace
{
bool parseBb(const BT::TreeNode& node, const std::string& port, json& out)
{
  std::string raw;
  if(!node.getInput(port, raw) || raw.empty())
  {
    return false;
  }
  try
  {
    out = json::parse(raw);
  }
  catch(const json::exception&)
  {
    return false;
  }
  return true;
}
}  // namespace

BT::NodeStatus HealthCheck::tick()
{
  RCLCPP_DEBUG(NodeContext::instance().node()->get_logger(), "health check ok");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CandidatesFound::tick()
{
  json j;
  if(!parseBb(*this, "candidates", j))
  {
    return BT::NodeStatus::FAILURE;
  }
  const bool found = (j.is_array() && !j.empty()) ||
                     (j.is_object() && j.value("ok", false));
  return found ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus UplinkOk::tick()
{
  json j;
  return (parseBb(*this, "ack", j) && j.value("ok", false)) ? BT::NodeStatus::SUCCESS :
                                                              BT::NodeStatus::FAILURE;
}

}  // namespace bt_nodes
