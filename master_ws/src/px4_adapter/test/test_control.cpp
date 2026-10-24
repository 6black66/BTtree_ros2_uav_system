#include <px4_adapter/control.hpp>
#include <iostream>
#include <functional>
using namespace uav::px4;
void check(bool ok,const char* why) { if(!ok) throw std::runtime_error(why); }
void rejects(const std::function<void()>& action) {
  try { action(); } catch(const std::invalid_argument&) { return; }
  throw std::runtime_error("expected invalid input rejection");
}
int main() {
  try {
    Setpoint input; input.x=1; input.y=2; input.z=3; input.use_yaw=true; input.yaw=0;
    auto p=convert(input,10000,20,2);
    check(p.position==std::array<float,3>{2,1,-3},"ENU position to NED");
    check(p.position_control && std::isnan(p.velocity[0]) && std::isnan(p.yaw_rate),"ignored fields are NaN");
    check(std::abs(p.yaw-pi/2)<1e-6,"east ENU yaw is east NED yaw");
    input.mode=1; input.vx=3; input.vy=4; input.vz=5; input.use_yaw_rate=true; input.yaw_rate=.3;
    p=convert(input,10000,20,2);
    check(!p.position_control && std::isnan(p.position[0]),"velocity must not contain position zeros");
    check(p.velocity==std::array<float,3>{4,3,-5} && std::abs(p.yaw_rate+.3)<1e-6,"velocity and yaw rate sign");
    input.mode=2; input.frame=1; p=convert(input,10000,20,2);
    check(p.position==std::array<float,3>{1,2,3} && p.velocity==std::array<float,3>{3,4,5},"NED passthrough and feedforward");
    input.vx=15; input.vy=15; input.vz=0; rejects([&]{convert(input,10000,20,2);});
    input.mode=0; input.x=std::numeric_limits<double>::quiet_NaN(); rejects([&]{convert(input,10000,20,2);});
    input.x=0; input.frame=2; rejects([&]{convert(input,10000,20,2);});
    const auto angles=euler_enu({1,0,0,0});
    check(std::abs(angles[2]-pi/2)<1e-8 && angles[0]==0 && angles[1]==0,"quaternion frame conversion");
    rejects([]{euler_enu({0,0,0,0});});
    check(versioned_topic("","out","vehicle_status",1)=="/fmu/out/vehicle_status_v1","versioned PX4 topic");
    check(versioned_topic("/px4_1/","in","trajectory_setpoint",0)=="/px4_1/fmu/in/trajectory_setpoint","namespace and unversioned topic");
    rejects([]{versioned_topic("relative","in","x",0);});
    std::cout<<"PX4 control conversion checks passed\n";
  } catch(const std::exception& error) { std::cerr<<error.what()<<'\n'; return 1; }
}
