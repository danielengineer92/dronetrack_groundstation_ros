# High-level motion intent from the mission executor to the low-level control node.
# The control node still owns safety checks and converts this intent to ControlCommand.
builtin_interfaces/Time stamp

# Mission motion mode:
# IDLE, TRACK_CENTER, FLY_FORWARD, APPROACH_TARGET, ORBIT_TARGET, ORBIT_FIXED, HOLD
string mode

# False means the control node must publish IDLE/zero output.
bool active

# Scripted body-frame velocity request for FLY_FORWARD/HOLD/debug modes.
float32 velocity_forward
float32 velocity_right
float32 velocity_down
float32 yaw_rate

# Vision-based approach/orbit settings.
float32 desired_distance_m
float32 orbit_speed_m_s
float32 orbit_radius_m

# Fixed-center orbit (mode == ORBIT_FIXED): the executor freezes a local-NED
# center estimated from vision, then the control node orbits it on pure
# geometry with NO live vision requirement (survives lock loss).
bool orbit_center_valid
float32 orbit_center_north
float32 orbit_center_east
float32 orbit_center_down

# Debug/status text from mission executor.
uint32 step_index
string step_name
string status
