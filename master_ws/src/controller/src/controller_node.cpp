#include "flight_controller_ros2/controller_node.hpp"
#include "flight_controller_ros2/xml_config.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <thread>

namespace qlet {
using namespace std::chrono_literals;
ControllerNode::ControllerNode() : Node("controller") {
  load_xml_overrides(*this);
  auto local_ip=param<std::string>(*this,"local_ip","0.0.0.0");
  auto remote_ip=param<std::string>(*this,"gimbal_ip","192.168.1.160");
  int local_port=param<int>(*this,"local_port",10000), remote_port=param<int>(*this,"gimbal_port",10000);
  timeout_ms_=param<int>(*this,"feedback_timeout_ms",500);
  ttl_ms_=param<int>(*this,"command_ttl_ms",500);
  max_ttl_ms_=param<int>(*this,"max_command_ttl_ms",2000);
  queue_limit_=param<int>(*this,"queue_limit",64);
  int period=param<int>(*this,"send_period_ms",20);
  require_online_=param<bool>(*this,"require_online",true);
  force_failure_=param<bool>(*this,"force_failure",false);
  telemetry_enabled_=param<bool>(*this,"telemetry_enabled",false);
  frame_id_=param<std::string>(*this,"frame_id","gimbal");
  if(timeout_ms_<20 || ttl_ms_<20 || max_ttl_ms_<ttl_ms_ || max_ttl_ms_>60000 || queue_limit_<1 || queue_limit_>4096 || period<20 || period>1000)
    throw std::invalid_argument("invalid timeout, TTL, queue limit or send period");
  if(telemetry_enabled_ && period!=20) throw std::invalid_argument("telemetry_enabled requires send_period_ms=20");
  auto topic=[this](const char* key,const char* def){ return param<std::string>(*this,key,def); };
  state_pub_=create_publisher<msgs::msg::GimbalState>(topic("state_topic","controller/gimbal/state"),rclcpp::QoS(5).best_effort());
  result_pub_=create_publisher<msgs::msg::GimbalCommandResult>(topic("result_topic","controller/gimbal/command_result"),20);
  link_pub_=create_publisher<msgs::msg::GimbalLinkState>(topic("link_topic","controller/gimbal/link_state"),rclcpp::QoS(1).reliable().transient_local());
  legacy_state_pub_=create_publisher<std_msgs::msg::String>("controller/gimbal_state",10);
  command_sub_=create_subscription<CommandMsg>(topic("command_topic","controller/gimbal/command"),rclcpp::QoS(static_cast<size_t>(queue_limit_)).reliable(),
    [this](CommandMsg::ConstSharedPtr m){ std::string error; submit(*m,error); });
  if(telemetry_enabled_) telemetry_sub_=create_subscription<msgs::msg::AircraftTelemetry>(
    topic("telemetry_topic","controller/aircraft/telemetry"),rclcpp::QoS(1).best_effort(),
    [this](msgs::msg::AircraftTelemetry::ConstSharedPtr m){ telemetry(*m); });
  service_=create_service<msgs::srv::GimbalCmd>("controller/gimbal",
    [this](const msgs::srv::GimbalCmd::Request::SharedPtr req,msgs::srv::GimbalCmd::Response::SharedPtr res){
      CommandMsg m; m.source="legacy_service"; m.command=req->command; m.yaw_deg=req->yaw_deg; m.pitch_deg=req->pitch_deg;
      if(m.command=="zoom") {
        res->ok=false; res->message="legacy zoom ratio has no universal QLET mapping; use zoom_direction or focal_length topic fields";
        result(m,"rejected",res->message); return;
      }
      res->ok=submit(m,res->message);
      if(res->ok) res->message="accepted into send queue; not device completion";
    });
  udp_=std::make_unique<UdpTransport>(local_ip,local_port,remote_ip,remote_port);
  rx_timer_=create_wall_timer(5ms,[this]{receive();});
  tx_timer_=create_wall_timer(std::chrono::milliseconds(period),[this]{transmit();});
  link_timer_=create_wall_timer(100ms,[this]{publish_link();});
  setup_navigation_stub();
  RCLCPP_INFO(get_logger(),"QLET gateway %s:%d -> %s:%d (navigation remains simulated)",local_ip.c_str(),local_port,remote_ip.c_str(),remote_port);
}
ControllerNode::~ControllerNode() {
  // Normal shutdown stops only continuous actions this gateway started.
  // Abrupt process termination cannot execute this cleanup.
  auto stop=[this](uint16_t command) {
    std::this_thread::sleep_until(last_control_send_+20ms);
    try {
      udp_->send(encode(command,Bytes(4,0)));
      last_control_send_=Clock::now();
    } catch(const std::exception& e) { RCLCPP_ERROR(get_logger(),"shutdown stop: %s",e.what()); }
  };
  if(udp_ && velocity_active_) stop(0x82);
  if(udp_ && zoom_active_) stop(0x45);
}
bool ControllerNode::online() const {
  return have_feedback_ && Clock::now()-last_feedback_<std::chrono::milliseconds(timeout_ms_);
}
void ControllerNode::result(const CommandMsg& m,const std::string& status,const std::string& detail) {
  msgs::msg::GimbalCommandResult r; r.header.stamp=now(); r.command_id=m.command_id; r.source=m.source;
  r.command=m.command; r.status=status; r.message=detail; result_pub_->publish(r);
}
bool ControllerNode::submit(CommandMsg m,std::string& error) {
  if(m.command_id.empty()) m.command_id="controller-"+std::to_string(++next_id_);
  if(m.source.empty()) m.source="anonymous";
  auto reject=[&](const std::string& why){ error=why; result(m,"rejected",why); return false; };
  if(force_failure_) return reject("forced failure configured");
  Frame frame;
  try {
    Command c; c.command=m.command; c.yaw_deg=m.yaw_deg; c.pitch_deg=m.pitch_deg;
    c.yaw_rate_deg_s=m.yaw_rate_deg_s; c.pitch_rate_deg_s=m.pitch_rate_deg_s; c.focal_length_mm=m.focal_length_mm;
    c.zoom_direction=m.zoom_direction; c.zoom_speed=m.zoom_speed; c.display_mode=m.display_mode;
    c.target_x=m.target_x; c.target_y=m.target_y; frame=make_command(c);
  } catch(const std::exception& e) { return reject(e.what()); }
  const auto t=Clock::now();
  const auto ttl=m.ttl_ms ? m.ttl_ms : static_cast<uint32_t>(ttl_ms_);
  if(ttl<20 || ttl>static_cast<uint32_t>(max_ttl_ms_)) return reject("ttl_ms outside configured bounds");
  double age_ms=0;
  if(m.header.stamp.sec || m.header.stamp.nanosec) {
    try {
      if(m.header.stamp.sec<0 || m.header.stamp.nanosec>=1000000000u)
        return reject("invalid command timestamp");
      age_ms=(now()-rclcpp::Time(m.header.stamp,get_clock()->get_clock_type())).seconds()*1000;
    } catch(const std::exception& e) { return reject(e.what()); }
    if(age_ms < -100 || age_ms>=ttl) return reject("command timestamp is stale or in the future");
    age_ms=std::max(0.0,age_ms);
  }
  bool stop=m.command=="stop", zoom_stop=m.command=="zoom" && m.zoom_direction==0;
  if(require_online_ && !online() && !stop && !zoom_stop) return reject("no fresh gimbal feedback");
  if(!stop && !zoom_stop && !owner_.empty() && owner_!=m.source && t<owner_deadline_)
    return reject("control source busy: "+owner_);
  auto deadline=t+std::chrono::milliseconds(static_cast<int64_t>(ttl-age_ms));
  bool continuous=m.command=="velocity" || m.command=="point" || m.command=="attitude_point" || m.command=="zoom";
  if(stop) expire_queue("cleared by stop");
  else if(continuous || zoom_stop) {
    for(auto it=queue_.begin();it!=queue_.end();) {
      bool replace=it->message.command==m.command;
      if(replace) {result(it->message,"superseded","replaced by latest command"); it=queue_.erase(it);} else ++it;
    }
  }
  if(queue_.size()>=static_cast<size_t>(queue_limit_)) return reject("send queue full");
  Pending p{m,std::move(frame),deadline};
  if(stop || zoom_stop) queue_.push_front(std::move(p)); else queue_.push_back(std::move(p));
  if(stop) { owner_.clear(); owner_deadline_=t; }
  else if(!zoom_stop) {
    owner_deadline_ = owner_==m.source ? std::max(owner_deadline_,deadline) : deadline;
    owner_=m.source;
  }
  result(m,"accepted","queued; device completion must be observed in state"); error.clear(); return true;
}
void ControllerNode::expire_queue(const std::string& why) {
  for(const auto& p:queue_) result(p.message,"expired",why);
  queue_.clear();
}
void ControllerNode::transmit() {
  const auto t=Clock::now();
  if(have_feedback_ && !online()) {
    expire_queue("feedback timed out"); owner_.clear();
    // Force stops when feedback is lost; never replay queued movement after recovery.
    if(velocity_active_) velocity_deadline_=t;
    if(zoom_active_) zoom_deadline_=t;
    telemetry_.reset();
  }
  auto send_frame=[this](const Frame& f) {
    try {
      udp_->send(encode(f.command,f.payload)); ++sent_;
      if(f.command!=0x0402) last_control_send_=Clock::now();
      return true;
    }
    catch(const std::exception& e) { ++io_errors_; RCLCPP_ERROR(get_logger(),"%s",e.what()); return false; }
  };
  // Check actual send times too: timer jitter must not violate the wire limit.
  if(t-last_control_send_<20ms) return;
  // At most one control frame per tick, preserving >=20ms separation.
  if(velocity_active_ && t>=velocity_deadline_) {
    if(send_frame({0x82,Bytes(4,0)})) velocity_active_=false;
    return;
  }
  if(zoom_active_ && t>=zoom_deadline_) {
    if(send_frame({0x45,Bytes(4,0)})) zoom_active_=false;
    return;
  }
  while(!queue_.empty() && queue_.front().deadline<=t) {
    result(queue_.front().message,"expired","TTL elapsed before send"); queue_.pop_front();
  }
  if(!queue_.empty()) {
    auto p=std::move(queue_.front()); queue_.pop_front();
    if(send_frame(p.frame)) {
      result(p.message,"sent","local UDP send completed; not a device acknowledgement");
      if(p.message.command=="velocity") {
        velocity_active_=p.message.yaw_rate_deg_s!=0 || p.message.pitch_rate_deg_s!=0; velocity_deadline_=p.deadline;
      } else if(p.message.command=="stop") {
        velocity_active_=false;
        // A global stop also schedules a lens stop at the next permitted interval.
        if(zoom_active_) zoom_deadline_=t;
      } else if(p.frame.command>=0x71 && p.frame.command<=0x7C) velocity_active_=false;
      if(p.message.command=="zoom") { zoom_active_=p.message.zoom_direction!=0; zoom_deadline_=p.deadline; }
    } else {
      result(p.message,"send_failed","UDP send failed; discrete commands are not retried");
      if(p.message.command=="velocity") {velocity_active_=true; velocity_deadline_=t;}
      if(p.message.command=="zoom") {zoom_active_=true; zoom_deadline_=t;}
    }
  }
  if(telemetry_ && t<telemetry_deadline_) send_frame(*telemetry_);
}
void ControllerNode::receive() {
  try {
    Bytes bytes;
    // Bounded work keeps ROS callbacks responsive even under packet floods.
    for(int i=0;i<64 && udp_->receive(bytes,foreign_);++i) {
      for(const auto& f:decode_datagram(bytes,invalid_)) {
        if(f.command!=0x0014) continue;
        try {
          auto state=decode_state(f.payload);
          if(have_feedback_ && !online()) {
            expire_queue("feedback gap exceeded timeout"); owner_.clear();
            if(velocity_active_) velocity_deadline_=Clock::now();
            if(zoom_active_) zoom_deadline_=Clock::now();
          }
          last_feedback_=Clock::now(); have_feedback_=true; ++valid_; publish_state(state);
        } catch(const std::exception&) { ++invalid_; }
      }
    }
  } catch(const std::exception& e) { ++io_errors_; RCLCPP_ERROR(get_logger(),"%s",e.what()); }
}
void ControllerNode::publish_link() {
  msgs::msg::GimbalLinkState m; m.header.stamp=now(); m.online=online();
  m.feedback_age_s=have_feedback_? std::chrono::duration<double>(Clock::now()-last_feedback_).count():-1;
  m.valid_state_frames=valid_; m.invalid_frames=invalid_; m.foreign_datagrams=foreign_; m.sent_frames=sent_;
  m.io_errors=io_errors_; m.queued_commands=queue_.size(); link_pub_->publish(m);
}
void ControllerNode::publish_state(const State& s) {
  msgs::msg::GimbalState m; m.header.stamp=now(); m.header.frame_id=frame_id_;
#define COPY(field) m.field=s.field
  COPY(payload_type); COPY(self_test); COPY(status1); COPY(status2); COPY(servo_mode);
  COPY(yaw_deg); COPY(pitch_deg); COPY(roll_deg); COPY(attitude_yaw_deg); COPY(attitude_pitch_deg); COPY(attitude_roll_deg);
  COPY(ir_fov_deg); COPY(visible_fov_deg); COPY(ir_focal_length_mm); COPY(visible_focal_length_mm);
  COPY(range_m); COPY(target_lon_deg); COPY(target_lat_deg); COPY(target_alt_m); COPY(temperature_c); COPY(max_temperature_c);
  COPY(storage_remaining_percent); COPY(storage_total_gb); COPY(display_mode); COPY(last_command); COPY(attitude_indication);
  COPY(target_x); COPY(target_y); COPY(target_width); COPY(target_height); COPY(target_type); COPY(tracking_state);
#undef COPY
  state_pub_->publish(m);
  std_msgs::msg::String legacy;
  std::ostringstream json;
  json<<"{\"yaw_deg\":"<<s.yaw_deg<<",\"pitch_deg\":"<<s.pitch_deg<<",\"servo_mode\":"<<int(s.servo_mode)<<"}";
  legacy.data=json.str(); legacy_state_pub_->publish(legacy);
}
void ControllerNode::telemetry(const msgs::msg::AircraftTelemetry& m) {
  try {
    double age=(now()-rclcpp::Time(m.header.stamp,get_clock()->get_clock_type())).seconds();
    if(age< -0.1 || age>0.5 || (m.header.stamp.sec==0 && m.header.stamp.nanosec==0))
      throw std::invalid_argument("telemetry requires fresh header stamp (<=500ms)");
    Bytes p(30,0);
    auto put=[&](size_t offset,double value,double lo,double hi,double scale,int bytes) {
      if(!std::isfinite(value) || value<lo || value>hi) throw std::invalid_argument("telemetry parameter out of range");
      uint32_t raw=static_cast<uint32_t>(static_cast<int64_t>(std::llround(value*scale)));
      for(int i=0;i<bytes;++i) p.at(offset+i)=static_cast<uint8_t>((raw>>(8*i))&255);
    };
    put(0,m.pitch_deg,-90,90,100,2); put(2,m.roll_deg,-180,180,100,2); put(4,m.heading_deg,0,360,100,2);
    if(m.gps_year>99 || m.gps_month<1 || m.gps_month>12 || m.gps_day<1 || m.gps_day>31 || m.gps_hour>23 || m.gps_minute>59 || m.gps_second>59 || m.gps_centisecond>99 || m.satellites>24)
      throw std::invalid_argument("invalid GPS time or satellite count");
    p[6]=m.gps_year; p[7]=m.gps_month; p[8]=m.gps_day; p[9]=m.gps_hour; p[10]=m.gps_minute; p[11]=m.gps_second; p[12]=m.gps_centisecond;
    put(13,m.longitude_deg,-180,180,1e7,4); put(17,m.latitude_deg,-90,90,1e7,4); p[21]=m.satellites;
    put(22,m.altitude_m,-500,10000,1,2); put(24,m.velocity_north_m_s,-327.68,327.67,100,2);
    put(26,m.velocity_east_m_s,-327.68,327.67,100,2); put(28,m.velocity_down_m_s,-327.68,327.67,100,2);
    telemetry_=Frame{0x0402,std::move(p)};
    telemetry_deadline_=Clock::now()+std::chrono::milliseconds(static_cast<int64_t>((0.5-std::max(0.0,age))*1000));
  } catch(const std::exception& e) {telemetry_.reset(); RCLCPP_WARN(get_logger(),"telemetry rejected: %s",e.what());}
}
void ControllerNode::setup_navigation_stub() {
  const int delay=param<int>(*this,"goto_delay_ms",4000);
  if(delay<1) throw std::invalid_argument("goto_delay_ms must be positive");
  nav_.mode="STANDBY"; nav_.arrived=true;
  nav_pub_=create_publisher<msgs::msg::NavState>("controller/nav_state",10);
  goto_service_=create_service<msgs::srv::GoTo>("controller/goto",
    [this,delay](const msgs::srv::GoTo::Request::SharedPtr,msgs::srv::GoTo::Response::SharedPtr res){
      res->ok=!force_failure_; res->message=force_failure_?"forced stub failure":"accepted (simulated navigation)";
      if(!res->ok) return;
      nav_.mode="TRANSIT"; nav_.arrived=false;
      if(arrival_timer_) arrival_timer_->cancel();
      arrival_timer_=create_wall_timer(std::chrono::milliseconds(delay),[this]{nav_.mode="ORBIT"; nav_.arrived=true; arrival_timer_->cancel();});
    });
  orbit_service_=create_service<msgs::srv::OrbitCmd>("controller/orbit",
    [this](const msgs::srv::OrbitCmd::Request::SharedPtr,msgs::srv::OrbitCmd::Response::SharedPtr res){
      res->ok=!force_failure_; res->message=force_failure_?"forced stub failure":"orbit (simulated navigation)";
      if(res->ok) nav_.mode="ORBIT";
    });
  nav_timer_=create_wall_timer(1s,[this]{nav_pub_->publish(nav_);});
}
}
