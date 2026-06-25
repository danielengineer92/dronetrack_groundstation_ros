# One-shot MAVSDK Action command request.
# Telemetry node owns the single MAVSDK connection and may reject these unless its action gate allows them.
builtin_interfaces/Time stamp
uint32 command_id

# Action: TAKEOFF, LAND, RETURN_TO_LAUNCH, DO_ORBIT, HOLD
string action

# Extra hard gate from the sender. If false, telemetry must ignore the command.
bool execute

# TAKEOFF settings
float32 takeoff_altitude_m

# DO_ORBIT settings
float32 radius_m
float32 velocity_m_s
float32 orbit_revolutions
string yaw_behavior

# Orbit center. Use NaN for current vehicle position/altitude behavior where supported by MAVSDK/PX4.
float64 latitude_deg
float64 longitude_deg
float64 absolute_altitude_m

string note
