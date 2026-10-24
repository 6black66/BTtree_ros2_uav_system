# Local positions and velocities are ENU; body convention is FLU; radians and SI units.
# Header is adapter publication time. Ages use the adapter monotonic receipt clock.
std_msgs/Header header
bool connected
bool armed
bool offboard
bool failsafe
bool is_vtol
bool in_transition
uint8 vehicle_type
uint8 nav_state
string mode
bool local_position_valid
bool local_velocity_valid
float64 x
float64 y
float64 z
float64 vx
float64 vy
float64 vz
bool attitude_valid
float64 roll
float64 pitch
float64 yaw
bool global_position_valid
float64 latitude_deg
float64 longitude_deg
float64 altitude_amsl_m
# Local origin is the estimator origin, not necessarily home or takeoff height.
bool local_reference_valid
float64 reference_latitude_deg
float64 reference_longitude_deg
float64 reference_altitude_amsl_m
bool landed_valid
bool landed
float64 local_age_s
float64 attitude_age_s
float64 global_age_s
