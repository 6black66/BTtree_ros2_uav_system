#pragma once
#include <px4_adapter/control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <msgs/msg/flight_setpoint.hpp>
#include <msgs/msg/flight_command.hpp>
#include <msgs/msg/flight_command_result.hpp>
#include <msgs/msg/flight_state.hpp>
#include <msgs/msg/flight_link_state.hpp>
#include <msgs/srv/flight_command.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <chrono>
#include <optional>
#include <string>

namespace uav::px4 {
// Run with the single-threaded executor: subscriptions, timers and transactions are serialized.
class AdapterNode : public rclcpp::Node {
public:
  AdapterNode();
private:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;
  using Status = px4_msgs::msg::VehicleStatus;
  using Command = msgs::msg::FlightCommand;
  using NativeCommand = px4_msgs::msg::VehicleCommand;
  struct Lease {
    msgs::msg::FlightSetpoint message;
    Trajectory trajectory;
    Time expires;
    bool reported_sent = false;
  };
  struct Transaction {
    Command request;
    NativeCommand native;
    Time started, expires, last_sent{};
    bool warming = false, acknowledged = false, in_progress = false;
    unsigned attempts = 0;
  };
  void on_status(const Status& status);
  void on_local(const px4_msgs::msg::VehicleLocalPosition& position);
  void on_ack(const px4_msgs::msg::VehicleCommandAck& ack);
  void on_setpoint(msgs::msg::FlightSetpoint message);
  bool submit(Command& request, std::string& detail);
  void tick();
  void publish_state();
  void publish_result(const Command& request, const std::string& state, const std::string& detail,
                      int result = -1, uint32_t command = 0);
  void publish_setpoint_result(const msgs::msg::FlightSetpoint& request, const std::string& state, const std::string& detail);
  void cancel_control(const std::string& reason, const std::string& result = "cancelled");
  void cancel_transaction(const std::string& reason, const std::string& result = "cancelled");
  void fallback_hold();
  bool connected() const;
  bool local_valid(bool velocity) const;
  bool state_matches(const Transaction& transaction) const;
  bool supported_vehicle() const;
  double age(Time time) const;
  double message_age_ms(const builtin_interfaces::msg::Time& stamp) const;
  uint64_t stamp_us() const;
  bool command_enabled_ = true;
  int status_timeout_ms_ = 1000, local_timeout_ms_ = 500, ttl_ms_ = 500, max_ttl_ms_ = 5000;
  int warmup_ms_ = 1500, ack_timeout_ms_ = 500, command_timeout_ms_ = 5000, max_retries_ = 2;
  double max_position_m_ = 10000, max_speed_m_s_ = 20, max_yaw_rate_ = 2;
  int source_system_ = 245, source_component_ = 191, target_system_ = 1, target_component_ = 1;
  std::string frame_id_;
  Status status_{};
  px4_msgs::msg::VehicleLocalPosition local_{};
  px4_msgs::msg::VehicleGlobalPosition global_{};
  px4_msgs::msg::VehicleAttitude attitude_{};
  px4_msgs::msg::VehicleLandDetected land_{};
  Time status_time_{}, local_time_{}, global_time_{}, attitude_time_{}, land_time_{};
  std::optional<Lease> lease_;
  std::optional<Transaction> transaction_;
  bool streaming_ = false, session_active_ = false;
  Time warmup_start_{}, last_stream_{};
  uint64_t last_ack_timestamp_ = 0, next_id_ = 0, received_ = 0, published_ = 0, rejected_ = 0;
  rclcpp::Publisher<msgs::msg::FlightState>::SharedPtr state_pub_;
  rclcpp::Publisher<msgs::msg::FlightLinkState>::SharedPtr link_pub_;
  rclcpp::Publisher<msgs::msg::FlightCommandResult>::SharedPtr result_pub_;
  rclcpp::Publisher<NativeCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Subscription<msgs::msg::FlightSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Subscription<Command>::SharedPtr command_sub_;
  rclcpp::Subscription<Status>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr global_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::Service<msgs::srv::FlightCommand>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr control_timer_, state_timer_;
};
}  // namespace uav::px4
