"""Pure control-math helpers for the control node.

Kept free of any rclpy/ROS imports so the command-generation logic can be unit
tested with plain Python (see test/test_control_math.py). These functions only
compute setpoints; the control node still owns every safety gate and decides
whether to publish the result.
"""

from __future__ import annotations

import math


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(value, hi))


# Equatorial Earth radius (WGS-84). A flat/equirectangular projection is used
# because target offsets are at most a few tens of metres — far below where
# earth curvature matters.
_EARTH_RADIUS_M = 6378137.0


def project_target_global(
    *,
    lat_deg: float,
    lon_deg: float,
    yaw_rad: float,
    bearing_x_rad: float,
    bearing_y_rad: float,
    slant_distance_m: float,
) -> tuple[float, float]:
    """Project a monocular target detection to a global (lat, lon).

    ``slant_distance_m`` is the straight-line (line-of-sight) range from the
    pinhole estimator. The horizontal ground distance is the slant range
    foreshortened by the camera elevation angle ``bearing_y_rad`` (the vertical
    offset of the target from the optical axis):

        horizontal_m = slant_distance_m * cos(bearing_y_rad)

    Omitting this ``cos`` term places the orbit centre too far out — by a factor
    of ``1 / cos(elevation)`` — and, for a target directly below the vehicle,
    would emit the full slant distance horizontally instead of ~0. The heading
    to the target is the vehicle yaw plus the horizontal bearing ``bearing_x_rad``
    (NED: 0 = north, positive clockwise toward east), so:

        north_m = horizontal_m * cos(yaw + bearing_x)
        east_m  = horizontal_m * sin(yaw + bearing_x)

    Returns ``(out_lat_deg, out_lon_deg)`` via an equirectangular offset.
    """
    horizontal_m = float(slant_distance_m) * math.cos(float(bearing_y_rad))
    global_bearing = float(yaw_rad) + float(bearing_x_rad)
    north_m = horizontal_m * math.cos(global_bearing)
    east_m = horizontal_m * math.sin(global_bearing)

    lat_rad = math.radians(float(lat_deg))
    out_lat = float(lat_deg) + math.degrees(north_m / _EARTH_RADIUS_M)
    out_lon = float(lon_deg) + math.degrees(
        east_m / (_EARTH_RADIUS_M * max(math.cos(lat_rad), 1e-6))
    )
    return out_lat, out_lon


def approach_forward_velocity(
    *,
    distance_valid: bool,
    distance_m: float,
    desired_distance_m: float,
    gain: float,
    max_speed: float,
    deadband_m: float = 0.0,
    target_locked: bool = True,
) -> float:
    """Forward velocity (m/s, body-frame +X) to close to ``desired_distance_m``.

    Returns ``0.0`` (i.e. hold / no forward motion) whenever the target data is
    not usable — not locked, distance invalid, non-finite, or non-positive — so a
    stale or lost target can never produce a forward command. This is the same
    fail-safe stance the rest of the control node takes.

    When valid, the sign follows the distance error: positive (move forward) when
    the target is farther than desired, negative (back off) when closer. The
    result is clamped to ``[-max_speed, max_speed]`` and zeroed inside
    ``deadband_m`` of the goal to avoid hunting.
    """
    if not target_locked or not distance_valid:
        return 0.0
    if not math.isfinite(distance_m) or distance_m <= 0.0:
        return 0.0
    if not math.isfinite(desired_distance_m) or desired_distance_m < 0.0:
        return 0.0

    distance_error_m = float(distance_m) - float(desired_distance_m)
    if abs(distance_error_m) <= max(0.0, deadband_m):
        return 0.0

    velocity = float(gain) * distance_error_m
    max_speed = abs(float(max_speed))
    return clamp(velocity, -max_speed, max_speed)
