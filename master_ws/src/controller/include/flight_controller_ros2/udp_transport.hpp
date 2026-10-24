#pragma once
#include "flight_controller_ros2/qlet_protocol.hpp"
#include <string>
#include <netinet/in.h>
namespace qlet {
class UdpTransport {
public:
  UdpTransport(const std::string& local_ip, int local_port, const std::string& remote_ip, int remote_port);
  ~UdpTransport();
  UdpTransport(const UdpTransport&)=delete;
  UdpTransport& operator=(const UdpTransport&)=delete;
  void send(const Bytes& data);
  // Nonblocking; foreign-source datagrams are consumed and counted separately.
  bool receive(Bytes& data, uint64_t& foreign);
private:
  int fd_=-1;
  sockaddr_in remote_{};
};
}
