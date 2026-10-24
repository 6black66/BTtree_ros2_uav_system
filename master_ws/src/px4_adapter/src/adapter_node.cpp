#include <px4_adapter/adapter_node.hpp>
#include <uav_ros2_support/xml_config.hpp>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace uav::px4 {
namespace {
using namespace std::chrono_literals;
bool finite3(float x, float y, float z) { return std::isfinite(x) && std::isfinite(y) && std::isfinite(z); }
std::string mode_name(uint8_t mode) {
  using S = px4_msgs::msg::VehicleStatus;
  switch(mode) {
    case S::NAVIGATION_STATE_MANUAL: return "MANUAL";
    case S::NAVIGATION_STATE_ALTCTL: return "ALTITUDE";
    case S::NAVIGATION_STATE_POSCTL: return "POSITION";
    case S::NAVIGATION_STATE_OFFBOARD: return "OFFBOARD";
    case S::NAVIGATION_STATE_AUTO_LOITER: return "HOLD";
    case S::NAVIGATION_STATE_AUTO_MISSION: return "MISSION";
    case S::NAVIGATION_STATE_AUTO_RTL: return "RTL";
    case S::NAVIGATION_STATE_AUTO_LAND: return "LAND";
    case S::NAVIGATION_STATE_AUTO_TAKEOFF: return "TAKEOFF";
    default: return "OTHER_" + std::to_string(mode);
  }
}
}
AdapterNode::AdapterNode() : Node("px4_adapter") {
  using uav_ros2_support::param;
  uav_ros2_support::load_xml_overrides(*this, "px4_adapter");
  command_enabled_ = param<bool>(*this,"command_enabled",true);
  status_timeout_ms_ = param<int>(*this,"status_timeout_ms",1000);
  local_timeout_ms_ = param<int>(*this,"local_timeout_ms",500);
  ttl_ms_ = param<int>(*this,"setpoint_ttl_ms",500);
  max_ttl_ms_ = param<int>(*this,"max_setpoint_ttl_ms",5000);
  warmup_ms_ = param<int>(*this,"offboard_warmup_ms",1500);
  ack_timeout_ms_ = param<int>(*this,"ack_timeout_ms",500);
  command_timeout_ms_ = param<int>(*this,"command_timeout_ms",5000);
  max_retries_ = param<int>(*this,"command_retries",2);
  max_position_m_ = param<double>(*this,"max_position_m",10000.0);
  max_speed_m_s_ = param<double>(*this,"max_speed_m_s",20.0);
  max_yaw_rate_ = param<double>(*this,"max_yaw_rate_rad_s",2.0);
  source_system_ = param<int>(*this,"source_system",245);
  source_component_ = param<int>(*this,"source_component",191);
  target_system_ = param<int>(*this,"target_system",1);
  target_component_ = param<int>(*this,"target_component",1);
  frame_id_ = param<std::string>(*this,"frame_id","map");
  const int period = param<int>(*this,"control_period_ms",50);
  const int state_period = param<int>(*this,"state_period_ms",100);
  const std::string fmu_ns = param<std::string>(*this,"fmu_namespace","");
  if(period < 10 || period > 100 || state_period < 20 || state_period > 1000 ||
     status_timeout_ms_ < 100 || status_timeout_ms_ > 10000 || local_timeout_ms_ < 100 || local_timeout_ms_ > 10000 ||
     ttl_ms_ < 100 || max_ttl_ms_ < ttl_ms_ || max_ttl_ms_ > 60000 || warmup_ms_ < 1100 || warmup_ms_ > 10000 ||
     ack_timeout_ms_ < 100 || command_timeout_ms_ < ack_timeout_ms_ || command_timeout_ms_ > 60000 ||
     max_retries_ < 0 || max_retries_ > 5 || !std::isfinite(max_position_m_) || max_position_m_ <= 0 || max_position_m_ > 1e7 ||
     !std::isfinite(max_speed_m_s_) || max_speed_m_s_ <= 0 || max_speed_m_s_ > 100 ||
     !std::isfinite(max_yaw_rate_) || max_yaw_rate_ <= 0 || max_yaw_rate_ > 10 ||
     source_system_ < 1 || source_system_ > 255 || source_component_ < 1 || source_component_ > 65535 ||
     target_system_ < 1 || target_system_ > 255 || target_component_ < 1 || target_component_ > 255)
    throw std::invalid_argument("invalid PX4 adapter timing, bounds or system/component identifiers");
  auto topic = [this](const char* key,const char* fallback) { return param<std::string>(*this,key,fallback); };
  state_pub_ = create_publisher<msgs::msg::FlightState>(topic("state_topic","px4/state"),rclcpp::QoS(5).best_effort());
  link_pub_ = create_publisher<msgs::msg::FlightLinkState>(topic("link_topic","px4/link_state"),rclcpp::QoS(1).reliable().transient_local());
  result_pub_ = create_publisher<msgs::msg::FlightCommandResult>(topic("result_topic","px4/command_result"),50);
  auto native = [&fmu_ns](const std::string& direction,const std::string& name,uint32_t version=0) {
    return versioned_topic(fmu_ns,direction,name,version);
  };
  // PX4 native endpoints use best-effort, volatile QoS. No latched control commands.
  const auto native_qos = rclcpp::QoS(10).best_effort().durability_volatile();
  command_pub_ = create_publisher<NativeCommand>(native("in","vehicle_command",NativeCommand::MESSAGE_VERSION),native_qos);
  trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(native("in","trajectory_setpoint",px4_msgs::msg::TrajectorySetpoint::MESSAGE_VERSION),native_qos);
  mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(native("in","offboard_control_mode"),native_qos);
  status_sub_ = create_subscription<Status>(native("out","vehicle_status",Status::MESSAGE_VERSION),native_qos,
      [this](Status::ConstSharedPtr m) { on_status(*m); });
  local_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(native("out","vehicle_local_position",px4_msgs::msg::VehicleLocalPosition::MESSAGE_VERSION),native_qos,
      [this](px4_msgs::msg::VehicleLocalPosition::ConstSharedPtr m) { on_local(*m); });
  global_sub_ = create_subscription<px4_msgs::msg::VehicleGlobalPosition>(native("out","vehicle_global_position",px4_msgs::msg::VehicleGlobalPosition::MESSAGE_VERSION),native_qos,
      [this](px4_msgs::msg::VehicleGlobalPosition::ConstSharedPtr m) { global_=*m; global_time_=Clock::now(); ++received_; });
  attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(native("out","vehicle_attitude",px4_msgs::msg::VehicleAttitude::MESSAGE_VERSION),native_qos,
      [this](px4_msgs::msg::VehicleAttitude::ConstSharedPtr m) { attitude_=*m; attitude_time_=Clock::now(); ++received_; });
  land_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(native("out","vehicle_land_detected",px4_msgs::msg::VehicleLandDetected::MESSAGE_VERSION),native_qos,
      [this](px4_msgs::msg::VehicleLandDetected::ConstSharedPtr m) { land_=*m; land_time_=Clock::now(); ++received_; });
  ack_sub_ = create_subscription<px4_msgs::msg::VehicleCommandAck>(native("out","vehicle_command_ack",px4_msgs::msg::VehicleCommandAck::MESSAGE_VERSION),native_qos,
      [this](px4_msgs::msg::VehicleCommandAck::ConstSharedPtr m) { on_ack(*m); });
  setpoint_sub_ = create_subscription<msgs::msg::FlightSetpoint>(topic("setpoint_topic","px4/setpoint"),rclcpp::QoS(1).reliable().durability_volatile(),
      [this](msgs::msg::FlightSetpoint::ConstSharedPtr m) { on_setpoint(*m); });
  command_sub_ = create_subscription<Command>(topic("command_topic","px4/command"),rclcpp::QoS(10).reliable().durability_volatile(),
      [this](Command::ConstSharedPtr m) { Command request=*m; std::string detail; submit(request,detail); });
  service_ = create_service<msgs::srv::FlightCommand>(topic("command_service","px4/command_service"),
      [this](msgs::srv::FlightCommand::Request::SharedPtr req,msgs::srv::FlightCommand::Response::SharedPtr res) {
        Command command; command.command_id=req->command_id; command.source=req->source; command.command=req->command;
        command.header.stamp=now(); res->accepted=submit(command,res->message); res->command_id=command.command_id;
      });
  control_timer_ = create_wall_timer(std::chrono::milliseconds(period),[this] { tick(); });
  state_timer_ = create_wall_timer(std::chrono::milliseconds(state_period),[this] { publish_state(); });
  RCLCPP_INFO(get_logger(),"PX4 uXRCE-DDS adapter ready; native prefix=%s/fmu; command_enabled=%s; no automatic arming",fmu_ns.c_str(),command_enabled_?"true":"false");
}
double AdapterNode::age(Time time) const {
  return time == Time{} ? -1.0 : std::chrono::duration<double>(Clock::now()-time).count();
}
bool AdapterNode::connected() const { return status_time_ != Time{} && age(status_time_)*1000 < status_timeout_ms_; }
bool AdapterNode::supported_vehicle() const {
  // TrajectorySetpoint controls the multicopter position controller; FW uses another PX4 interface.
  return status_.vehicle_type == Status::VEHICLE_TYPE_ROTARY_WING && !status_.in_transition_mode;
}
bool AdapterNode::local_valid(bool velocity) const {
  if(!connected() || local_time_ == Time{} || age(local_time_)*1000 >= local_timeout_ms_) return false;
  if(velocity) return local_.v_xy_valid && local_.v_z_valid && finite3(local_.vx,local_.vy,local_.vz);
  return local_.xy_valid && local_.z_valid && finite3(local_.x,local_.y,local_.z);
}
double AdapterNode::message_age_ms(const builtin_interfaces::msg::Time& stamp) const {
  if(stamp.sec < 0 || stamp.nanosec >= 1000000000u || (!stamp.sec && !stamp.nanosec))
    throw std::invalid_argument("a valid nonzero ROS header timestamp is required");
  double elapsed=(now()-rclcpp::Time(stamp,get_clock()->get_clock_type())).seconds()*1000;
  if(elapsed < -100) throw std::invalid_argument("request timestamp is in the future");
  return std::max(0.0,elapsed);
}
uint64_t AdapterNode::stamp_us() const { return static_cast<uint64_t>(std::max<int64_t>(0,now().nanoseconds())/1000); }
void AdapterNode::publish_result(const Command& req,const std::string& state,const std::string& detail,int code,uint32_t command) {
  msgs::msg::FlightCommandResult m; m.header.stamp=now(); m.command_id=req.command_id; m.source=req.source;
  m.command=req.command; m.status=state; m.message=detail; m.px4_result=code; m.px4_command=command;
  result_pub_->publish(m);
}
void AdapterNode::publish_setpoint_result(const msgs::msg::FlightSetpoint& req,const std::string& state,const std::string& detail) {
  Command command; command.command_id=req.command_id; command.source=req.source; command.command="setpoint";
  publish_result(command,state,detail);
}
void AdapterNode::cancel_transaction(const std::string& why,const std::string& state) {
  if(transaction_) publish_result(transaction_->request,state,why,-1,transaction_->native.command);
  transaction_.reset();
}
void AdapterNode::cancel_control(const std::string& why,const std::string& state) {
  if(lease_) publish_setpoint_result(lease_->message,state,why);
  lease_.reset(); streaming_=false; session_active_=false; warmup_start_={}; last_stream_={};
}
void AdapterNode::on_status(const Status& m) {
  if(m.system_id != target_system_ || m.component_id != target_component_) return;
  ++received_;
  const bool reboot = status_time_ != Time{} && m.timestamp < status_.timestamp;
  const bool gap = status_time_ != Time{} && !connected();
  if(reboot || gap) {
    cancel_control(reboot?"PX4 timestamp reset":"PX4 status gap"); cancel_transaction("PX4 link reset");
    local_time_={}; global_time_={}; attitude_time_={}; land_time_={}; last_ack_timestamp_=0;
  }
  status_=m; status_time_=Clock::now();
  if((session_active_ && m.nav_state != Status::NAVIGATION_STATE_OFFBOARD) || m.failsafe ||
      (streaming_ && !supported_vehicle())) {
    cancel_control("PX4 mode changed, failsafe active or vehicle left multicopter mode");
    if(transaction_ && (transaction_->request.command=="offboard" || transaction_->request.command=="arm"))
      cancel_transaction("PX4 interrupted Offboard session");
  }
  if(streaming_ && m.nav_state==Status::NAVIGATION_STATE_OFFBOARD) session_active_=true;
}
void AdapterNode::on_local(const px4_msgs::msg::VehicleLocalPosition& m) {
  ++received_;
  if(local_time_ != Time{} && (m.xy_reset_counter != local_.xy_reset_counter || m.z_reset_counter != local_.z_reset_counter ||
      m.heading_reset_counter != local_.heading_reset_counter || m.timestamp < local_.timestamp)) {
    const bool had_session=streaming_;
    cancel_control("PX4 local reference or heading reset");
    if(transaction_ && transaction_->request.command=="offboard") cancel_transaction("PX4 local reference reset");
    if(had_session) fallback_hold();
  }
  local_=m; local_time_=Clock::now();
}
void AdapterNode::on_setpoint(msgs::msg::FlightSetpoint m) {
  if(m.command_id.empty()) m.command_id="setpoint-"+std::to_string(++next_id_);
  if(m.source.empty()) m.source="anonymous";
  auto reject=[&](const std::string& why) { ++rejected_; publish_setpoint_result(m,"rejected",why); };
  try {
    if(!command_enabled_) throw std::invalid_argument("command_enabled=false");
    if(!connected()) throw std::invalid_argument("no fresh PX4 VehicleStatus");
    if(!supported_vehicle()) throw std::invalid_argument("TrajectorySetpoint requires rotary-wing mode; fixed-wing/transition not supported by this interface");
    if(status_.failsafe) throw std::invalid_argument("PX4 reports failsafe");
    const uint32_t ttl=m.ttl_ms?m.ttl_ms:static_cast<uint32_t>(ttl_ms_);
    if(ttl<100 || ttl>static_cast<uint32_t>(max_ttl_ms_)) throw std::invalid_argument("setpoint TTL outside configured bounds");
    const double elapsed=message_age_ms(m.header.stamp);
    if(elapsed>=ttl) throw std::invalid_argument("setpoint expired before receipt");
    if((m.coordinate_frame==0 && !m.header.frame_id.empty() && m.header.frame_id!=frame_id_) ||
       (m.coordinate_frame==1 && !m.header.frame_id.empty() && m.header.frame_id!="map_ned"))
      throw std::invalid_argument("header frame_id does not match local coordinate convention");
    Setpoint p; p.mode=m.mode; p.frame=m.coordinate_frame; p.x=m.x; p.y=m.y; p.z=m.z;
    p.vx=m.vx; p.vy=m.vy; p.vz=m.vz; p.use_yaw=m.use_yaw; p.yaw=m.yaw;
    p.use_yaw_rate=m.use_yaw_rate; p.yaw_rate=m.yaw_rate;
    auto trajectory=convert(p,max_position_m_,max_speed_m_s_,max_yaw_rate_);
    if(!local_valid(m.mode==1)) throw std::invalid_argument("required local position/velocity estimate is invalid or stale");
    if(m.mode==2 && !local_valid(true)) throw std::invalid_argument("velocity estimate required for feedforward");
    if(lease_ && Clock::now()<lease_->expires && lease_->message.source!=m.source)
      throw std::invalid_argument("another source owns the active setpoint lease");
    if(transaction_ && transaction_->request.command!="offboard" && transaction_->request.command!="arm")
      throw std::invalid_argument("mode transition pending");
    if(lease_) {
      if(Clock::now()>=lease_->expires && streaming_) {
        cancel_control("setpoint update arrived after lease expired","expired");
        if(transaction_ && transaction_->request.command=="offboard") cancel_transaction("setpoint lease expired");
        fallback_hold();
        throw std::invalid_argument("session expired; resend setpoint after mode transition");
      }
      if(lease_->message.command_id!=m.command_id) publish_setpoint_result(lease_->message,"superseded","replaced by latest setpoint");
    }
    bool already_reported=lease_ && lease_->message.command_id==m.command_id && lease_->reported_sent;
    lease_=Lease{m,trajectory,Clock::now()+std::chrono::milliseconds(static_cast<int64_t>(ttl-elapsed)),already_reported};
    publish_setpoint_result(m,"accepted","cached; native stream starts only on explicit offboard request");
  } catch(const std::exception& e) { reject(e.what()); }
}
bool AdapterNode::submit(Command& req,std::string& detail) {
  if(req.command_id.empty()) req.command_id="command-"+std::to_string(++next_id_);
  if(req.source.empty()) req.source="anonymous";
  auto reject=[&](const std::string& why) { ++rejected_; detail=why; publish_result(req,"rejected",why); return false; };
  try { if(message_age_ms(req.header.stamp)>1000) return reject("command timestamp is stale"); }
  catch(const std::exception& e) { return reject(e.what()); }
  if(!command_enabled_) return reject("command_enabled=false");
  if(!connected()) return reject("no fresh PX4 VehicleStatus");
  if(transaction_) return reject("another VehicleCommand transaction is pending");
  NativeCommand native{};
  if(req.command=="arm" || req.command=="disarm") {
    native.command=NativeCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    native.param1=req.command=="arm"?1.f:0.f;
  } else if(req.command=="offboard" || req.command=="hold" || req.command=="land" || req.command=="rtl" || req.command=="position_mode") {
    native.command=NativeCommand::VEHICLE_CMD_DO_SET_MODE; native.param1=1;
    native.param2=req.command=="offboard"?6.f:(req.command=="position_mode"?3.f:4.f);
    native.param3=req.command=="hold"?3.f:(req.command=="land"?6.f:(req.command=="rtl"?5.f:0.f));
  } else return reject("unsupported command");
  if(req.command=="offboard") {
    if(!lease_ || Clock::now()>=lease_->expires) return reject("offboard requires a fresh setpoint lease");
    if(req.source!=lease_->message.source) return reject("offboard source must own the setpoint lease");
    if(status_.failsafe || !supported_vehicle() || !local_valid(lease_->message.mode==1))
      return reject("PX4 state does not permit this Offboard interface");
    streaming_=true; session_active_=false; warmup_start_={}; last_stream_={};
  } else if(req.command!="arm") cancel_control("explicit "+req.command+" request");
  native.target_system=target_system_; native.target_component=target_component_;
  native.source_system=source_system_; native.source_component=source_component_; native.from_external=true;
  const auto t=Clock::now();
  transaction_=Transaction{req,native,t,t+std::chrono::milliseconds(command_timeout_ms_+(req.command=="offboard"?warmup_ms_:0))};
  transaction_->warming=req.command=="offboard";
  publish_result(req,"accepted",transaction_->warming?"Offboard warmup started; keep refreshing setpoints":"transaction queued",-1,native.command);
  detail="accepted; observe command_result for ACK and actual state confirmation";
  return true;
}
void AdapterNode::on_ack(const px4_msgs::msg::VehicleCommandAck& ack) {
  ++received_;
  if(!transaction_ || !transaction_->attempts || ack.command!=transaction_->native.command ||
     ack.target_system!=source_system_ || ack.target_component!=source_component_ || !ack.from_external ||
     ack.timestamp<=last_ack_timestamp_) return;
  last_ack_timestamp_=ack.timestamp;
  auto& tx=*transaction_;
  if(ack.result==px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS) {
    tx.in_progress=true; publish_result(tx.request,"in_progress","PX4 reports progress",ack.result,ack.command);
  } else if(ack.result==px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
    tx.acknowledged=true; publish_result(tx.request,"acknowledged","PX4 accepted; awaiting matching VehicleStatus",ack.result,ack.command);
  } else {
    const bool offboard=tx.request.command=="offboard";
    publish_result(tx.request,"rejected","PX4 rejected VehicleCommand",ack.result,ack.command);
    transaction_.reset();
    if(offboard) cancel_control("PX4 rejected Offboard");
  }
}
bool AdapterNode::state_matches(const Transaction& tx) const {
  if(!connected() || status_time_<tx.started) return false;
  const auto& name=tx.request.command;
  if(name=="arm") return status_.arming_state==Status::ARMING_STATE_ARMED;
  if(name=="disarm") return status_.arming_state==Status::ARMING_STATE_DISARMED;
  if(name=="offboard") return status_.nav_state==Status::NAVIGATION_STATE_OFFBOARD;
  if(name=="hold") return status_.nav_state==Status::NAVIGATION_STATE_AUTO_LOITER;
  if(name=="land") return status_.nav_state==Status::NAVIGATION_STATE_AUTO_LAND;
  if(name=="rtl") return status_.nav_state==Status::NAVIGATION_STATE_AUTO_RTL;
  return name=="position_mode" && status_.nav_state==Status::NAVIGATION_STATE_POSCTL;
}
void AdapterNode::fallback_hold() {
  if(!command_enabled_ || !connected() || status_.nav_state!=Status::NAVIGATION_STATE_OFFBOARD || transaction_) return;
  Command command; command.header.stamp=now(); command.source="adapter_watchdog"; command.command="hold";
  std::string detail; submit(command,detail);
}
void AdapterNode::tick() {
  const auto t=Clock::now();
  if(!connected()) {
    cancel_control("PX4 status timeout"); cancel_transaction("PX4 status timeout","timeout");
    return;
  }
  if(lease_ && (t>=lease_->expires || !local_valid(lease_->message.mode==1) ||
      (lease_->message.mode==2 && !local_valid(true)) || status_.failsafe || !supported_vehicle())) {
    const bool was_streaming=streaming_;
    cancel_control("setpoint expired or required estimator feedback became invalid","expired");
    if(transaction_ && transaction_->request.command=="offboard") cancel_transaction("Offboard input expired","timeout");
    if(was_streaming) fallback_hold();
  }
  if(streaming_ && lease_) {
    if(last_stream_!=Time{} && t-last_stream_>200ms) {
      if(session_active_) {
        cancel_control("Offboard stream scheduler gap");
        if(transaction_ && transaction_->request.command=="offboard") cancel_transaction("Offboard stream interrupted");
        fallback_hold(); return;
      }
      warmup_start_={};
    }
    if(warmup_start_==Time{}) warmup_start_=t;
    const float nan=std::numeric_limits<float>::quiet_NaN();
    px4_msgs::msg::TrajectorySetpoint trajectory{};
    trajectory.timestamp=stamp_us(); trajectory.position=lease_->trajectory.position; trajectory.velocity=lease_->trajectory.velocity;
    trajectory.acceleration.fill(nan); trajectory.jerk.fill(nan);
    trajectory.yaw=lease_->trajectory.yaw; trajectory.yawspeed=lease_->trajectory.yaw_rate;
    px4_msgs::msg::OffboardControlMode mode{};
    mode.timestamp=trajectory.timestamp; mode.position=lease_->trajectory.position_control; mode.velocity=!mode.position;
    // Warm up only the Offboard heartbeat. Inject trajectories into the PX4
    // controller only after actual VehicleStatus reports Offboard, avoiding
    // conflicts with the internal controllers in manual/mission modes.
    mode_pub_->publish(mode); ++published_; last_stream_=t;
    if(status_.nav_state==Status::NAVIGATION_STATE_OFFBOARD) {
      trajectory_pub_->publish(trajectory); ++published_;
      if(!lease_->reported_sent) {
        publish_setpoint_result(lease_->message,"sent","native trajectory published in confirmed Offboard mode");
        lease_->reported_sent=true;
      }
    }
  }
  if(!transaction_) return;
  auto& tx=*transaction_;
  if(t>=tx.expires) {
    const bool offboard=tx.request.command=="offboard";
    cancel_transaction("deadline elapsed before ACK plus actual state confirmation","timeout");
    if(offboard) { cancel_control("Offboard request timed out"); fallback_hold(); }
    return;
  }
  if(tx.warming) {
    if(!streaming_ || !lease_ || warmup_start_==Time{} || t-warmup_start_<std::chrono::milliseconds(warmup_ms_)) return;
    tx.warming=false;
  }
  if(tx.acknowledged && state_matches(tx)) {
    publish_result(tx.request,"confirmed","ACK accepted and matching fresh VehicleStatus observed",0,tx.native.command);
    transaction_.reset(); return;
  }
  if(tx.acknowledged || tx.in_progress) return;
  if(!tx.attempts || (t-tx.last_sent>=std::chrono::milliseconds(ack_timeout_ms_) && tx.attempts<=static_cast<unsigned>(max_retries_))) {
    tx.native.timestamp=stamp_us(); tx.native.confirmation=static_cast<uint8_t>(tx.attempts);
    command_pub_->publish(tx.native); ++published_; ++tx.attempts; tx.last_sent=t;
    publish_result(tx.request,"sent",tx.attempts==1?"VehicleCommand published":"VehicleCommand retried after missing ACK",-1,tx.native.command);
  }
}
void AdapterNode::publish_state() {
  const double nan=std::numeric_limits<double>::quiet_NaN();
  msgs::msg::FlightState state; state.header.stamp=now(); state.header.frame_id=frame_id_;
  state.connected=connected(); state.armed=state.connected && status_.arming_state==Status::ARMING_STATE_ARMED;
  state.offboard=state.connected && status_.nav_state==Status::NAVIGATION_STATE_OFFBOARD;
  state.failsafe=status_.failsafe; state.vehicle_type=status_.vehicle_type; state.nav_state=status_.nav_state;
  state.is_vtol=status_.is_vtol; state.in_transition=status_.in_transition_mode;
  state.mode=state.connected?mode_name(status_.nav_state):"DISCONNECTED";
  state.local_position_valid=local_valid(false); state.local_velocity_valid=local_valid(true);
  state.x=state.local_position_valid?local_.y:nan; state.y=state.local_position_valid?local_.x:nan; state.z=state.local_position_valid?-local_.z:nan;
  state.vx=state.local_velocity_valid?local_.vy:nan; state.vy=state.local_velocity_valid?local_.vx:nan; state.vz=state.local_velocity_valid?-local_.vz:nan;
  state.local_age_s=age(local_time_); state.attitude_age_s=age(attitude_time_); state.global_age_s=age(global_time_);
  state.roll=state.pitch=state.yaw=nan;
  if(state.connected && attitude_time_!=Time{} && state.attitude_age_s*1000<local_timeout_ms_) {
    try { const auto angles=euler_enu(attitude_.q); state.roll=angles[0]; state.pitch=angles[1]; state.yaw=angles[2]; state.attitude_valid=true; }
    catch(const std::exception&) {}
  }
  state.global_position_valid=state.connected && global_time_!=Time{} && state.global_age_s*1000<status_timeout_ms_ &&
      global_.lat_lon_valid && global_.alt_valid && std::isfinite(global_.lat) && std::isfinite(global_.lon) && std::isfinite(global_.alt) &&
      std::abs(global_.lat)<=90 && std::abs(global_.lon)<=180;
  state.latitude_deg=state.global_position_valid?global_.lat:nan; state.longitude_deg=state.global_position_valid?global_.lon:nan;
  state.altitude_amsl_m=state.global_position_valid?global_.alt:nan;
  state.local_reference_valid=state.local_position_valid && local_.xy_global && local_.z_global &&
      std::isfinite(local_.ref_lat) && std::isfinite(local_.ref_lon) && std::isfinite(local_.ref_alt);
  state.reference_latitude_deg=state.local_reference_valid?local_.ref_lat:nan;
  state.reference_longitude_deg=state.local_reference_valid?local_.ref_lon:nan;
  state.reference_altitude_amsl_m=state.local_reference_valid?local_.ref_alt:nan;
  state.landed_valid=state.connected && land_time_!=Time{} && age(land_time_)*1000<status_timeout_ms_;
  state.landed=state.landed_valid && land_.landed;
  state_pub_->publish(state);
  msgs::msg::FlightLinkState link; link.header=state.header; link.connected=state.connected; link.status_age_s=age(status_time_);
  link.streaming_setpoints=streaming_;
  link.offboard_session=streaming_?(session_active_?"active":(transaction_ && transaction_->warming?"warming":"requesting")):"idle";
  if(lease_) { link.active_source=lease_->message.source; link.active_setpoint_id=lease_->message.command_id; }
  link.received_messages=received_; link.published_messages=published_; link.rejected_requests=rejected_;
  link_pub_->publish(link);
}
}  // namespace uav::px4
