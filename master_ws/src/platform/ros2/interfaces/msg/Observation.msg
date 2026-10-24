# Source frame timestamp, not inference completion time.
std_msgs/Header header
uint64 session_id
uint64 frame_sequence
uint32 image_width
uint32 image_height
# Identifies the slow snapshot; fast results retain the selected snapshot's ID.
uint64 observation_id
string request_id
# slow: ok / no_frame / stale / error
# fast: idle / initialized / tracking / lost / rejected / stopped
string status
string detail
float32 processing_ms
# Slow: detector scores. Fast: confidence=-1 (KCF has no calibrated class score).
ObjectDetection[] detections
