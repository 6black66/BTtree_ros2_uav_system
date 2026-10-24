# Supported commands and their active fields are documented in docs/controller.md.
# Angles are degrees in the QLET convention; positive yaw right, positive pitch up.
std_msgs/Header header
string command_id
string source
string command
# Zero uses the controller default TTL. Continuous commands must be refreshed.
uint32 ttl_ms
float64 yaw_deg
float64 pitch_deg
float64 yaw_rate_deg_s
float64 pitch_rate_deg_s
float64 focal_length_mm
int8 zoom_direction
uint8 zoom_speed
uint8 display_mode
# Pixel offsets relative to image center, QLET convention.
int16 target_x
int16 target_y
