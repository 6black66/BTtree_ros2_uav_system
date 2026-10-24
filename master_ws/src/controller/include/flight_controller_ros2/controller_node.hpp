#pragma once
#include "flight_controller_ros2/qlet_protocol.hpp"
#include "flight_controller_ros2/udp_transport.hpp"
#include <rclcpp/rclcpp.hpp>
#include <msgs/msg/gimbal_command.hpp>
#include <msgs/msg/gimbal_state.hpp>
#include <msgs/msg/gimbal_command_result.hpp>
#include <msgs/msg/gimbal_link_state.hpp>
#include <msgs/msg/aircraft_telemetry.hpp>
#include <msgs/msg/nav_state.hpp>
#include <msgs/srv/gimbal_cmd.hpp>
#include <msgs/srv/go_to.hpp>
#include <msgs/srv/orbit_cmd.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <deque>
#include <memory>
#include <optional>

namespace qlet {
// Use the single-threaded executor: all socket/state/queue access is serialized.
class ControllerNode : public rclcpp::Node {
public:
  ControllerNode();
  ~ControllerNode() override;
private:
  using Clock=std::chrono::steady_clock;
  using CommandMsg=msgs::msg::GimbalCommand;
  struct Pending { CommandMsg message; Frame frame; Clock::time_point deadline; };
  bool submit(CommandMsg m, std::string& error);
  void result(const CommandMsg& m,const std::string& status,const std::string& detail);
  void receive();
  void transmit();
  void publish_link();
  void publish_state(const State& state);
  void expire_queue(const std::string& reason);
  void setup_navigation_stub();
  bool online() const;
  void telemetry(const msgs::msg::AircraftTelemetry& m);
  std::unique_ptr<UdpTransport> udp_;
  std::deque<Pending> queue_;
  std::optional<Frame> telemetry_;
  Clock::time_point telemetry_deadline_{}, last_feedback_{}, velocity_deadline_{}, zoom_deadline_{};
  bool have_feedback_=false, velocity_active_=false, zoom_active_=false;
  bool require_online_=true, force_failure_=false, telemetry_enabled_=false;
  int timeout_ms_=500, ttl_ms_=500, max_ttl_ms_=2000, queue_limit_=64;
  std::string owner_, frame_id_;
  Clock::time_point owner_deadline_{}, last_control_send_{};
  uint64_t next_id_=0, valid_=0, invalid_=0, foreign_=0, sent_=0, io_errors_=0;
  rclcpp::Publisher<msgs::msg::GimbalState>::SharedPtr state_pub_;
  rclcpp::Publisher<msgs::msg::GimbalCommandResult>::SharedPtr result_pub_;
  rclcpp::Publisher<msgs::msg::GimbalLinkState>::SharedPtr link_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr legacy_state_pub_;
  rclcpp::Subscription<CommandMsg>::SharedPtr command_sub_;
  rclcpp::Subscription<msgs::msg::AircraftTelemetry>::SharedPtr telemetry_sub_;
  rclcpp::Service<msgs::srv::GimbalCmd>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr rx_timer_, tx_timer_, link_timer_;
  msgs::msg::NavState nav_;
  rclcpp::Publisher<msgs::msg::NavState>::SharedPtr nav_pub_;
  rclcpp::Service<msgs::srv::GoTo>::SharedPtr goto_service_;
  rclcpp::Service<msgs::srv::OrbitCmd>::SharedPtr orbit_service_;
  rclcpp::TimerBase::SharedPtr nav_timer_, arrival_timer_;
};
}
