#include "bt_nodes/nav_nodes.hpp"

#include "framework/detail/json.hpp"

namespace bt_nodes
{

using json = nlohmann::json;

BT::NodeStatus GoTo::tick()
{
  std::string task;
  double lat = 0, lon = 0, timeout_s = 300;
  getInput("task", task);
  getInput("lat", lat);
  getInput("lon", lon);
  getInput("timeout_s", timeout_s);

  // 任务 json 里的目的地优先;显式端口其次
  if(!task.empty())
  {
    try
    {
      auto j = json::parse(task);
      lat = j.value("lat", lat);
      lon = j.value("lon", lon);
    }
    catch(const json::exception&)
    {}
  }

  auto node = NodeContext::instance().node();
  if(!client_)
  {
    client_ = node->create_client<msgs::srv::GoTo>("controller/goto");
  }

  // ---- 阶段1:下发 GoTo 服务请求 ----
  if(!sent_)
  {
    const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout_s));
    deadline_ = std::chrono::steady_clock::now() + timeout;
    while(!client_->wait_for_service(std::chrono::milliseconds(0)))
    {
      if(std::chrono::steady_clock::now() > deadline_)
      {
        return BT::NodeStatus::FAILURE;
      }
      setStatusRunningAndYield();
    }
    auto req = std::make_shared<msgs::srv::GoTo::Request>();
    req->lat = lat;
    req->lon = lon;
    req->alt_m = 100.0f;
    req->radius_m = 150.0f;
    nav_revision_at_send_ = NodeContext::instance().navRevision();
    auto fut = client_->async_send_request(req);
    while(fut.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
    {
      setStatusRunningAndYield();
    }
    if(!fut.get()->ok)
    {
      sent_ = false;
      return BT::NodeStatus::FAILURE;
    }
    sent_ = true;
  }

  // ---- 阶段2:等到位 ----
  while(true)
  {
    if(NodeContext::instance().hasNavState() &&
       NodeContext::instance().navRevision() > nav_revision_at_send_ &&
       NodeContext::instance().navState().arrived)
    {
      sent_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    if(std::chrono::steady_clock::now() > deadline_)
    {
      sent_ = false;
      return BT::NodeStatus::FAILURE;
    }
    setStatusRunningAndYield();
  }
}

void GoTo::halt()
{
  sent_ = false;
  CoroActionNode::halt();
}

BT::NodeStatus OrbitCommand::tick()
{
  double radius = 500, timeout_s = 10;
  getInput("radius_m", radius);
  getInput("timeout_s", timeout_s);

  auto node = NodeContext::instance().node();
  static auto client = node->create_client<msgs::srv::OrbitCmd>("controller/orbit");
  const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while(!client->wait_for_service(std::chrono::milliseconds(0)))
  {
    if(std::chrono::steady_clock::now() > deadline)
    {
      return BT::NodeStatus::FAILURE;
    }
    setStatusRunningAndYield();
  }
  auto req = std::make_shared<msgs::srv::OrbitCmd::Request>();
  req->radius_m = static_cast<float>(radius);
  auto fut = client->async_send_request(req);
  while(fut.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    if(std::chrono::steady_clock::now() > deadline)
    {
      return BT::NodeStatus::FAILURE;
    }
    setStatusRunningAndYield();
  }
  return fut.get()->ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus GimbalCommand::tick()
{
  std::string command = "preset_search";
  double yaw = 0, pitch = 0, zoom = 1, timeout_s = 10;
  getInput("command", command);
  getInput("yaw_deg", yaw);
  getInput("pitch_deg", pitch);
  getInput("zoom", zoom);
  getInput("timeout_s", timeout_s);

  auto node = NodeContext::instance().node();
  static auto client = node->create_client<msgs::srv::GimbalCmd>("controller/gimbal");
  const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(timeout_s));
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while(!client->wait_for_service(std::chrono::milliseconds(0)))
  {
    if(std::chrono::steady_clock::now() > deadline)
    {
      return BT::NodeStatus::FAILURE;
    }
    setStatusRunningAndYield();
  }
  auto req = std::make_shared<msgs::srv::GimbalCmd::Request>();
  req->command = command;
  req->yaw_deg = yaw;
  req->pitch_deg = pitch;
  req->zoom = zoom;
  auto fut = client->async_send_request(req);
  while(fut.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    if(std::chrono::steady_clock::now() > deadline)
    {
      return BT::NodeStatus::FAILURE;
    }
    setStatusRunningAndYield();
  }
  return fut.get()->ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

}  // namespace bt_nodes
