std_msgs/Header header
bool online
# -1 until first valid 0x0014 state frame. Other frame types do not refresh this.
float64 feedback_age_s
uint64 valid_state_frames
uint64 invalid_frames
uint64 foreign_datagrams
uint64 sent_frames
uint64 io_errors
uint32 queued_commands
