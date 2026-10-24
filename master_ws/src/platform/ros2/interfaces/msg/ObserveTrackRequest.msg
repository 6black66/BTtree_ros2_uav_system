# Exactly one active target per observe node. A new start replaces it only after
# validation. Use all three IDs from a recent slow Observation.
string request_id
string command
# command: start / stop. IDs ignored for stop.
uint64 session_id
uint64 observation_id
uint64 target_id
