std_msgs/Header header
string command_id
string source
string command
# accepted / sent / acknowledged / in_progress / confirmed / rejected / timeout /
# superseded / expired / cancelled
string status
# -1 if not associated with VehicleCommandAck. ACK alone is not state confirmation.
int16 px4_result
uint32 px4_command
string message
