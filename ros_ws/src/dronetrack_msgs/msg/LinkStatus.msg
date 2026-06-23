# Pi-side verdict on the ground-station link. Published by the Pi watchdog so the
# rest of the stack (and the dashboard) can see one authoritative link state.
builtin_interfaces/Time stamp

# Overall link verdict. When link_ok is false the Pi must already be in a safe
# state on its own; this flag exists for observability and to de-assert
# operator/perception permissions, not to *cause* safety (the Pi fails safe
# regardless of what this says).
bool link_ok

# Component freshness, seconds since last accepted message. -1.0 = never seen.
float32 heartbeat_age_s
float32 detection_age_s

# Estimated one-way latency from ground-station stamp to Pi receipt, seconds.
# Requires clock sync (NTP/chrony) between Pi and laptop to be meaningful.
# NaN if not computable.
float32 estimated_latency_s

# Last heartbeat sequence number the Pi accepted.
uint32 last_heartbeat_sequence

# Human-readable reason for the current verdict, e.g.
# "OK", "HEARTBEAT_STALE", "DETECTIONS_STALE", "NO_HEARTBEAT".
string reason
