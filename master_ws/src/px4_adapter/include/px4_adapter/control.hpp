#pragma once
#include <array>
#include <cstdint>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace uav::px4 {
constexpr double pi = 3.14159265358979323846;
inline double wrap(double angle) { return std::remainder(angle, 2.0 * pi); }
inline std::array<float, 3> enu_to_ned(double x, double y, double z) {
  return {static_cast<float>(y), static_cast<float>(x), static_cast<float>(-z)};
}
struct Setpoint {
  uint8_t mode = 0, frame = 0;
  double x = 0, y = 0, z = 0, vx = 0, vy = 0, vz = 0;
  bool use_yaw = false, use_yaw_rate = false;
  double yaw = 0, yaw_rate = 0;
};
struct Trajectory {
  bool position_control = false;
  std::array<float, 3> position, velocity;
  float yaw, yaw_rate;
};
inline Trajectory convert(const Setpoint& input, double max_position_m, double max_speed_m_s, double max_yaw_rate) {
  if(input.mode > 2 || input.frame > 1) throw std::invalid_argument("invalid setpoint mode or coordinate frame");
  auto validate = [](double value, double limit) {
    if(!std::isfinite(value) || std::abs(value) > limit) throw std::invalid_argument("setpoint exceeds configured bounds or is nonfinite");
  };
  const float nan = std::numeric_limits<float>::quiet_NaN();
  Trajectory output{input.mode != 1, {nan,nan,nan}, {nan,nan,nan}, nan, nan};
  if(input.mode != 1) {
    validate(input.x,max_position_m); validate(input.y,max_position_m); validate(input.z,max_position_m);
    output.position = input.frame == 0 ? enu_to_ned(input.x,input.y,input.z) :
      std::array<float,3>{float(input.x),float(input.y),float(input.z)};
  }
  if(input.mode != 0) {
    validate(input.vx,max_speed_m_s); validate(input.vy,max_speed_m_s); validate(input.vz,max_speed_m_s);
    if(std::hypot(std::hypot(input.vx,input.vy),input.vz) > max_speed_m_s)
      throw std::invalid_argument("velocity magnitude exceeds configured limit");
    output.velocity = input.frame == 0 ? enu_to_ned(input.vx,input.vy,input.vz) :
      std::array<float,3>{float(input.vx),float(input.vy),float(input.vz)};
  }
  if(input.use_yaw) {
    validate(input.yaw,1e6);
    output.yaw = static_cast<float>(wrap(input.frame == 0 ? pi/2-input.yaw : input.yaw));
  }
  if(input.use_yaw_rate) {
    validate(input.yaw_rate,max_yaw_rate);
    output.yaw_rate = static_cast<float>(input.frame == 0 ? -input.yaw_rate : input.yaw_rate);
  }
  return output;
}
inline std::array<double,3> euler_enu(const std::array<float,4>& q) {
  double norm = 0;
  for(auto v:q) { if(!std::isfinite(v)) throw std::invalid_argument("invalid quaternion"); norm += double(v)*v; }
  if(norm < 1e-8) throw std::invalid_argument("zero quaternion");
  const double length = std::sqrt(norm);
  const double w=q[0]/length, x=q[1]/length, y=q[2]/length, z=q[3]/length;
  const double pitch_term=2*(w*y-z*x);
  const double pitch=std::asin(std::fmax(-1.,std::fmin(1.,pitch_term)));
  const double yaw=std::atan2(2*(w*z+x*y),1-2*(y*y+z*z));
  return {std::atan2(2*(w*x+y*z),1-2*(x*x+y*y)), -pitch, wrap(pi/2-yaw)};
}
inline std::string versioned_topic(const std::string& prefix, const std::string& direction,
                                 const std::string& base, uint32_t version) {
  if(!prefix.empty() && prefix.front() != '/') throw std::invalid_argument("fmu_namespace must be absolute or empty");
  std::string ns=prefix;
  while(!ns.empty() && ns.back()=='/') ns.pop_back();
  return ns+"/fmu/"+direction+"/"+base+(version ? "_v"+std::to_string(version) : "");
}
}  // namespace uav::px4
