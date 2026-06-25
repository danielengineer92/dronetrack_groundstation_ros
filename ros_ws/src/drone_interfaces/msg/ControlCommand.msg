# Movement command output
builtin_interfaces/Time stamp

# Command type
string command_type  # VELOCITY, POSITION, IDLE

# Velocity commands (m/s, body frame) and yaw-rate command (rad/s)
# Used for legacy/debug velocity mode and as diagnostics for POSITION mode.
float32 velocity_forward
float32 velocity_right
float32 velocity_down
float32 yaw_rate

# Position commands (local NED frame; meters, yaw in degrees)
# position_valid must be true when command_type == POSITION.
bool position_valid
float32 position_north
float32 position_east
float32 position_down
float32 yaw_deg

# Whether the control node approved this command for the MAVSDK bridge.
# For POSITION hold commands, executed=true is the approval bit and
# execution_status may describe the hold reason, such as MISSION_HOLD or
# NO_TARGET_DATA. Legacy VELOCITY commands should use SENT when approved.
bool executed
string execution_status  # SENT, BLOCKED_*, IDLE, SIMULATED

# Source tracking error that generated this command
float32 source_error_x
float32 source_error_y
