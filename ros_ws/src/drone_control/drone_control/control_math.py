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
