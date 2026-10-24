#include "flight_controller_ros2/qlet_protocol.hpp"
#include <cmath>
#include <stdexcept>
#include <limits>

namespace qlet {
namespace {
uint16_t u16(const Bytes& b, size_t p) { return uint16_t(b.at(p)) | (uint16_t(b.at(p+1)) << 8); }
int16_t i16(const Bytes& b, size_t p) {
  const auto v = u16(b,p); return static_cast<int16_t>(v < 0x8000 ? int(v) : int(v)-65536);
}
int32_t i32(const Bytes& b, size_t p) {
  uint32_t v = uint32_t(u16(b,p)) | (uint32_t(u16(b,p+2)) << 16);
  return static_cast<int32_t>(v <= INT32_MAX ? int64_t(v) : int64_t(v)-4294967296LL);
}
void put16(Bytes& b, size_t p, int value) {
  auto v = static_cast<uint16_t>(value); b.at(p) = v & 255; b.at(p+1) = v >> 8;
}
int scaled(double value, double lo, double hi, double scale) {
  if(!std::isfinite(value) || value < lo || value > hi) throw std::invalid_argument("parameter outside protocol range");
  return static_cast<int>(std::lround(value * scale));
}
}
Bytes encode(uint16_t command, const Bytes& payload) {
  if(payload.size() > 253) throw std::invalid_argument("payload exceeds 253 bytes");
  Bytes b{0xFB,0x2C,0xAA,static_cast<uint8_t>(payload.size()+2),uint8_t(command&255),uint8_t(command>>8)};
  b.insert(b.end(),payload.begin(),payload.end());
  uint8_t sum=0; for(size_t i=3;i<b.size();++i) sum ^= b[i]; b.push_back(sum); return b;
}
Frame make_command(const Command& c) {
  Frame f{0, Bytes(4,0)};
  if(c.command == "point" || c.command == "attitude_point") {
    f.command = c.command == "point" ? 0x72 : 0x7C;
    put16(f.payload,0,scaled(c.yaw_deg,-179,179,100));
    put16(f.payload,2,scaled(c.pitch_deg,-120,90,100));
  } else if(c.command == "velocity") {
    f.command=0x82;
    put16(f.payload,0,scaled(c.yaw_rate_deg_s,-50,50,100));
    put16(f.payload,2,scaled(c.pitch_rate_deg_s,-50,50,100));
  } else if(c.command == "stop") f.command=0x82;
  else if(c.command == "center") f.command=0x71;
  else if(c.command == "stow") f.command=0x74;
  else if(c.command == "down") f.command=0x73;
  else if(c.command == "lock" || c.command == "freeze") f.command=0x7A;
  else if(c.command == "follow") f.command=0x7B;
  else if(c.command == "scan" || c.command == "preset_search") f.command=0x79;
  else if(c.command == "zoom") {
    if(c.zoom_direction < -1 || c.zoom_direction > 1 || c.zoom_speed < 0 || c.zoom_speed > 7)
      throw std::invalid_argument("zoom_direction must be -1/0/1; zoom_speed must be 0..7");
    f.command=0x45; f.payload[0]=c.zoom_direction==0 ? 0 : (c.zoom_direction>0 ? 1 : 2); f.payload[1]=c.zoom_speed;
  } else if(c.command == "focal_length") {
    f.command=0x4C; put16(f.payload,0,scaled(c.focal_length_mm,0.1,6500,10));
  } else if(c.command == "display") {
    if(c.display_mode<0 || c.display_mode>5) throw std::invalid_argument("display_mode must be 0..5");
    f.command=0x25; f.payload[0]=c.display_mode;
  } else if(c.command == "track") {
    if(c.target_x < -960 || c.target_x>960 || c.target_y < -540 || c.target_y>540)
      throw std::invalid_argument("tracking coordinates exceed image-center bounds");
    f.command=0x3A; put16(f.payload,0,c.target_x); put16(f.payload,2,c.target_y);
  } else if(c.command == "cancel_track") f.command=0x3B;
  else if(c.command == "photo") f.command=0x32;
  else if(c.command == "record_start") { f.command=0x33; f.payload[0]=1; }
  else if(c.command == "record_stop") { f.command=0x33; f.payload[0]=2; }
  else if(c.command == "range_once") f.command=0x3D;
  else if(c.command == "range_start") { f.command=0x3E; f.payload[0]=1; }
  else if(c.command == "range_stop") f.command=0x3F;
  else throw std::invalid_argument("unsupported command: " + c.command);
  return f;
}
std::vector<Frame> decode_datagram(const Bytes& b, uint64_t& invalid) {
  std::vector<Frame> frames;
  size_t p=0;
  while(p<b.size()) {
    if(b.size()-p<7 || b[p]!=0xFC || b[p+1]!=0x2C || b[p+2]!=0x55) { ++invalid; ++p; continue; }
    const size_t length=b[p+3], total=length+5;
    if(length<2 || total>b.size()-p) { ++invalid; ++p; continue; }
    uint8_t sum=0; for(size_t i=p+3;i<p+total-1;++i) sum ^= b[i];
    if(sum!=b[p+total-1]) { ++invalid; ++p; continue; }
    frames.push_back({u16(b,p+4),Bytes(b.begin()+p+6,b.begin()+p+total-1)}); p+=total;
  }
  return frames;
}
State decode_state(const Bytes& p) {
  if(p.size()<60) throw std::invalid_argument("state payload shorter than table 14 (60 bytes)");
  State s; s.payload_type=p[0]; s.self_test=p[1]; s.status1=u16(p,2); s.status2=u16(p,4); s.servo_mode=p[6];
  s.yaw_deg=i16(p,7)*0.01; s.pitch_deg=i16(p,9)*0.01; s.roll_deg=i16(p,11)*0.01;
  s.ir_fov_deg=u16(p,14)*0.1; s.storage_remaining_percent=p[16]; s.visible_fov_deg=u16(p,17)*0.1;
  s.storage_total_gb=u16(p,19)*0.1; s.ir_focal_length_mm=u16(p,21)*0.1; s.visible_focal_length_mm=u16(p,23)*0.1;
  s.range_m=u16(p,25)*0.1; s.target_lon_deg=i32(p,27)*1e-7; s.target_lat_deg=i32(p,31)*1e-7;
  s.target_alt_m=i16(p,35); s.temperature_c=i16(p,37)*0.1;
  s.attitude_yaw_deg=i16(p,39)*0.01; s.attitude_pitch_deg=i16(p,41)*0.01; s.attitude_roll_deg=i16(p,43)*0.01;
  s.display_mode=p[45]; s.last_command=p[46]; s.attitude_indication=p[47];
  s.target_x=i16(p,48); s.target_y=i16(p,50); s.target_width=u16(p,52); s.target_height=u16(p,54);
  s.target_type=p[56]; s.tracking_state=p[57]; s.max_temperature_c=i16(p,58)*0.1; return s;
}
}  // namespace qlet
