# Pixels stay in local POSIX shared memory. A reference can expire as slots rotate.
# header.stamp is producer receive/decode time; source_pts_ns is stream-relative.
std_msgs/Header header
string shm_name
uint64 session_id
uint64 sequence
uint32 slot
uint32 width
uint32 height
uint32 step
uint32 data_size
string encoding
bool source_pts_valid
int64 source_pts_ns
