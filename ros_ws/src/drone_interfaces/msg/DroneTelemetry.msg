# PX4 drone telemetry data
builtin_interfaces/Time stamp

# Connection status
bool connected
string connection_status

# Battery
float32 battery_voltage
float32 battery_remaining_percent

# GPS/global position
float64 latitude
float64 longitude
float32 absolute_altitude
float32 relative_altitude
int32 gps_num_satellites
int32 gps_fix_type

# Local position (PX4 NED frame, meters). Valid when local_position_valid is true.
bool local_position_valid
float32 local_position_north
float32 local_position_east
float32 local_position_down

# Attitude (radians)
float32 roll
float32 pitch
float32 yaw

# Velocity (m/s NED)
float32 velocity_north
float32 velocity_east
float32 velocity_down

# State
bool armed
string flight_mode
string landed_state

# Health
bool health_all_ok
bool health_accelerometer_ok
bool health_gyroscope_ok
bool health_magnetometer_ok
bool health_gps_ok
