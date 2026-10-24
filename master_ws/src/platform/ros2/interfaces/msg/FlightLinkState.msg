std_msgs/Header header
bool connected
float64 status_age_s
# Active session heartbeat; trajectories publish only after actual Offboard entry.
bool streaming_setpoints
# idle / warming / requesting / active
string offboard_session
string active_source
string active_setpoint_id
uint64 received_messages
uint64 published_messages
uint64 rejected_requests
