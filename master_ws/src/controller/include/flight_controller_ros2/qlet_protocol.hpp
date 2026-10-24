#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace qlet {
using Bytes = std::vector<uint8_t>;
struct Frame { uint16_t command; Bytes payload; };
struct Command {
  std::string command;
  double yaw_deg = 0, pitch_deg = 0, yaw_rate_deg_s = 0, pitch_rate_deg_s = 0;
  double focal_length_mm = 0;
  int zoom_direction = 0, zoom_speed = 0, display_mode = 0;
  int target_x = 0, target_y = 0;
};
struct State {
  uint8_t payload_type = 0, self_test = 0, servo_mode = 0, display_mode = 0;
  uint16_t status1 = 0, status2 = 0;
  double yaw_deg = 0, pitch_deg = 0, roll_deg = 0;
  double attitude_yaw_deg = 0, attitude_pitch_deg = 0, attitude_roll_deg = 0;
  double ir_fov_deg = 0, visible_fov_deg = 0, ir_focal_length_mm = 0, visible_focal_length_mm = 0;
  double range_m = 0, target_lon_deg = 0, target_lat_deg = 0, target_alt_m = 0;
  double temperature_c = 0, max_temperature_c = 0, storage_total_gb = 0;
  uint8_t storage_remaining_percent = 0, last_command = 0, attitude_indication = 0;
  int16_t target_x = 0, target_y = 0;
  uint16_t target_width = 0, target_height = 0;
  uint8_t target_type = 0, tracking_state = 0;
};
Bytes encode(uint16_t command, const Bytes& payload);
Frame make_command(const Command& command);
// A UDP datagram may contain several complete frames. Never join separate datagrams.
std::vector<Frame> decode_datagram(const Bytes& bytes, uint64_t& invalid_frames);
State decode_state(const Bytes& payload);
}  // namespace qlet
