# World-frame target state from the constant-velocity Kalman estimator
# (target_state_estimator_node). Frame: PX4 local NED, meters / m/s.
builtin_interfaces/Time stamp

# True once the filter has converged (enough updates) and measurements are
# fresh. coasting=true means we are predicting through a target-loss gap;
# valid drops to false when the coast exceeds the timeout.
bool valid
bool coasting

# Filtered target state (horizontal; altitude is not estimated)
float32 position_north
float32 position_east
float32 velocity_north
float32 velocity_east
float32 speed                 # sqrt(vN^2 + vE^2)
float32 heading_rad           # direction of MOTION, NED (0 = north, +CW to east)

# Position extrapolated prediction_horizon_s into the future (lead point /
# latency compensation).
float32 predicted_north
float32 predicted_east
float32 prediction_horizon_s

# 1-sigma confidence (sqrt of mean diagonal covariance)
float32 position_std_m
float32 velocity_std_m_s

float32 measurement_age_s     # seconds since the last accepted measurement
uint32 updates                # accepted measurements since last reset
