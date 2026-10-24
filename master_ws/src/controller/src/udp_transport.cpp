#include "flight_controller_ros2/udp_transport.hpp"
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <system_error>
#include <stdexcept>
namespace qlet {
namespace {
sockaddr_in address(const std::string& ip,int port) {
  sockaddr_in a{}; a.sin_family=AF_INET;
  if(port<1 || port>65535 || inet_pton(AF_INET,ip.c_str(),&a.sin_addr)!=1)
    throw std::invalid_argument("invalid IPv4 address or UDP port: "+ip);
  a.sin_port=htons(static_cast<uint16_t>(port)); return a;
}
}
UdpTransport::UdpTransport(const std::string& ip,int port,const std::string& remote,int remote_port)
: remote_(address(remote,remote_port)) {
  auto local=address(ip,port);
  fd_=socket(AF_INET,SOCK_DGRAM|SOCK_NONBLOCK|SOCK_CLOEXEC,0);
  if(fd_<0) throw std::system_error(errno,std::generic_category(),"UDP socket");
  if(bind(fd_,reinterpret_cast<sockaddr*>(&local),sizeof(local))<0) {
    int e=errno; close(fd_); fd_=-1; throw std::system_error(e,std::generic_category(),"UDP bind");
  }
}
UdpTransport::~UdpTransport() { if(fd_>=0) close(fd_); }
void UdpTransport::send(const Bytes& data) {
  auto n=sendto(fd_,data.data(),data.size(),0,reinterpret_cast<const sockaddr*>(&remote_),sizeof(remote_));
  if(n<0) throw std::system_error(errno,std::generic_category(),"UDP sendto");
  if(static_cast<size_t>(n)!=data.size()) throw std::runtime_error("short UDP send");
}
bool UdpTransport::receive(Bytes& data,uint64_t& foreign) {
  uint8_t buffer[65536]; sockaddr_in from{}; socklen_t len=sizeof(from);
  auto n=recvfrom(fd_,buffer,sizeof(buffer),0,reinterpret_cast<sockaddr*>(&from),&len);
  if(n<0) {
    if(errno==EAGAIN || errno==EWOULDBLOCK || errno==EINTR) return false;
    throw std::system_error(errno,std::generic_category(),"UDP recvfrom");
  }
  data.clear();
  if(from.sin_addr.s_addr!=remote_.sin_addr.s_addr || from.sin_port!=remote_.sin_port) { ++foreign; return true; }
  data.assign(buffer,buffer+n); return true;
}
}
