#include "flight_controller_ros2/qlet_protocol.hpp"
#include <iostream>
#include <stdexcept>
#include <functional>
#include <limits>
#include <cmath>
using namespace qlet;
void check(bool condition,const char* message) {if(!condition) throw std::runtime_error(message);}
void rejects(const std::function<void()>& fn) {try {fn();} catch(const std::invalid_argument&) {return;} throw std::runtime_error("expected rejection");}
Bytes feedback(uint16_t command,Bytes payload) {
  auto b=encode(command,payload); b[0]=0xFC; b[2]=0x55; return b;
}
int main() {
  try {
    Command c; c.command="center";
    auto f=make_command(c);
    check(encode(f.command,f.payload)==Bytes({0xFB,0x2C,0xAA,6,0x71,0,0,0,0,0,0x77}),"PDF center vector");
    c.command="point"; c.yaw_deg=5; c.pitch_deg=5; f=make_command(c);
    check(encode(f.command,f.payload)==Bytes({0xFB,0x2C,0xAA,6,0x72,0,0xF4,1,0xF4,1,0x74}),"PDF point vector");
    check(encode(0x70,{0xF6,0xFF,0,0})==Bytes({0xFB,0x2C,0xAA,6,0x70,0,0xF6,0xFF,0,0,0x7F}),"PDF velocity vector");
    c.command="velocity"; c.yaw_rate_deg_s=-10.25; f=make_command(c);
    check(f.command==0x82 && f.payload[0]==0xFF && f.payload[1]==0xFB,"fine signed velocity");
    c.yaw_rate_deg_s=51; rejects([&]{make_command(c);});
    c.command="point"; c.yaw_deg=std::numeric_limits<double>::quiet_NaN(); rejects([&]{make_command(c);});
    c.yaw_deg=180; rejects([&]{make_command(c);});
    c.command="unknown"; rejects([&]{make_command(c);});
    rejects([]{encode(1,Bytes(254));});
    uint64_t invalid=0;
    // PDF table 15 SN response establishes length and XOR semantics for feedback.
    Bytes sn{0xFC,0x2C,0x55,0x10,0x10,0,2,0x50,0x30,0x32,0x38,0x32,0x30,0x32,0x34,0x30,0x36,0x30,0x30,0x33,0x59};
    auto frames=decode_datagram(sn,invalid);
    check(frames.size()==1 && invalid==0 && frames[0].payload.size()==14,"PDF SN feedback vector");
    Bytes p(60,0); p[0]=3; p[1]=0xFF; p[7]=0xF4; p[8]=1; p[9]=0x48; p[10]=0xF4;
    p[27]=0; p[28]=0x74; p[29]=0x79; p[30]=0xB8; // -120 degrees
    p[39]=0x18; p[40]=0xFC; p[48]=0xFF; p[49]=0xFF; p[57]=3;
    auto packet=feedback(0x14,p); auto both=packet; both.insert(both.end(),packet.begin(),packet.end());
    frames=decode_datagram(both,invalid); check(frames.size()==2 && invalid==0,"multiple UDP frames");
    auto s=decode_state(frames[0].payload);
    check(s.yaw_deg==5 && s.pitch_deg==-30 && s.attitude_yaw_deg==-10 && s.target_x==-1 && s.tracking_state==3,"state signed angles and tracking");
    check(std::abs(s.target_lon_deg+120)<1e-7,"signed 32 bit longitude");
    packet.back()^=1; check(decode_datagram(packet,invalid).empty() && invalid>0,"checksum rejection");
    invalid=0; packet=feedback(0x14,p); packet.pop_back(); check(decode_datagram(packet,invalid).empty() && invalid>0,"truncated datagram rejection");
    rejects([]{decode_state(Bytes(59));});
    packet={0,0,0}; auto valid=feedback(0x14,p); packet.insert(packet.end(),valid.begin(),valid.end());
    check(decode_datagram(packet,invalid).size()==1,"resync after noise");
    std::cout<<"QLET protocol checks passed\n";
  } catch(const std::exception& e) {std::cerr<<e.what()<<'\n'; return 1;}
}
