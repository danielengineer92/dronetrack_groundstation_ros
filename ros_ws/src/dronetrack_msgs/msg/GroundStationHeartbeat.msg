# Liveness beacon published by the laptop ground station.
# The Pi watchdog uses this to decide whether the ground station link is alive.
# This is intentionally independent of detection rate: the heartbeat keeps
# beating even when YOLO produces no detections, so "link up" and "target seen"
# are separate signals.
builtin_interfaces/Time stamp

# Monotonic counter, increments every beat. Lets the Pi detect a frozen/replayed
# publisher (counter stops advancing) even if a stale message keeps arriving.
uint32 sequence

# Ground-station identity and software build, for logging/debug only.
string station_id
string software_version

# Self-reported ground-station health. Advisory only; the Pi never trusts these
# blindly, but they are useful on the dashboard.
float32 perception_fps        # measured YOLO inference FPS, 0.0 if unknown
float32 cpu_percent           # ground-station CPU load, -1.0 if unknown
bool perception_ok            # ground station believes its YOLO pipeline is healthy
