# Local positions/velocities in meters and m/s. Yaw/yaw rate in radians and rad/s.
# Header stamp is required; refresh before TTL expires, including during Offboard warmup.
std_msgs/Header header
string command_id
string source
uint8 POSITION=0
uint8 VELOCITY=1
uint8 POSITION_VELOCITY=2
uint8 mode
uint8 ENU=0
uint8 NED=1
uint8 coordinate_frame
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
bool use_yaw
float64 yaw
bool use_yaw_rate
float64 yaw_rate
# 0 uses adapter default. Position/velocity setpoints are both leased, not latched forever.
uint32 ttl_ms
