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

    return offset_global(lat_deg, lon_deg, north_m, east_m)


def offset_global(
    lat_deg: float, lon_deg: float, north_m: float, east_m: float
) -> tuple[float, float]:
    """Offset a global (lat, lon) by metres north/east (equirectangular)."""
    lat_rad = math.radians(float(lat_deg))
    out_lat = float(lat_deg) + math.degrees(float(north_m) / _EARTH_RADIUS_M)
    out_lon = float(lon_deg) + math.degrees(
        float(east_m) / (_EARTH_RADIUS_M * max(math.cos(lat_rad), 1e-6))
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


def yaw_pid_step(
    *,
    error: float,
    dt: float,
    kp: float,
    ki: float,
    kd: float,
    integral: float,
    filtered_derivative: float,
    prev_error: float,
    first_sample: bool,
    output_limit: float,
    integral_limit: float,
    derivative_alpha: float,
) -> tuple[float, float, float]:
    """One step of the yaw-tracking PID. Pure; the caller owns the state.

    ``error`` is the deadbanded normalized image error in [-1, 1] (setpoint is
    always 0 = target centred, so derivative-of-error == derivative-of-
    measurement and there is no setpoint kick). Returns
    ``(output_rad_s, new_integral, new_filtered_derivative)``; the caller
    stores the last two plus ``error`` for the next call.

    Design notes, matched to a noisy vision error signal:

    - **D is low-pass filtered**: ``derivative_alpha`` in (0, 1] blends the raw
      slope into the previous filtered value (1 = unfiltered). The raw
      derivative of YOLO pixel error is jittery; kd on the raw slope would
      amplify it (why the original controller was P-only).
    - **First sample after a reset produces no D** (no meaningful slope yet).
    - **I has two guards**: the integral is clamped so its contribution
      ``|ki * integral|`` never exceeds ``integral_limit`` (rad/s), and it does
      not accumulate while the unsaturated output already exceeds
      ``output_limit`` in the direction of the error (conditional
      anti-windup — a saturated yaw command means more integral cannot help,
      only overshoot later).
    - **ki == 0 zeroes the integral** so no stale windup is replayed when I is
      enabled live via ``ros2 param set``.

    The output is NOT clamped here: the control node's ``limit_motion`` owns
    saturation (``max_yaw_rate``) and slew (``max_yaw_accel``); this function
    only needs ``output_limit`` to know when to freeze the integrator.
    """
    dt = max(float(dt), 1e-6)
    error = float(error)

    if kd > 0.0 and not first_sample:
        alpha = clamp(float(derivative_alpha), 1e-3, 1.0)
        raw_derivative = (error - float(prev_error)) / dt
        new_derivative = alpha * raw_derivative + (1.0 - alpha) * float(filtered_derivative)
    else:
        new_derivative = 0.0

    if ki > 0.0:
        new_integral = float(integral) + error * dt
        max_integral = abs(float(integral_limit)) / ki
        new_integral = clamp(new_integral, -max_integral, max_integral)
    else:
        new_integral = 0.0

    output = kp * error + ki * new_integral + kd * new_derivative

    # Conditional anti-windup: if we are saturated and the error keeps pushing
    # the same way, back the integral out so it does not wind up.
    if abs(output) > abs(output_limit) and error * output > 0.0:
        new_integral = float(integral) if ki > 0.0 else 0.0
        output = kp * error + ki * new_integral + kd * new_derivative

    return output, new_integral, new_derivative


def yaw_feedforward_step(
    *,
    los_angle_rad: float,
    prev_los_angle_rad: float,
    dt: float,
    prev_ff_rad_s: float,
    first_sample: bool,
    lpf_alpha: float,
    limit_rad_s: float,
) -> float:
    """Estimate the target's inertial LOS rate for yaw feed-forward.

    A pure error controller must keep the target OFF-centre to generate any
    yaw rate at all — it trails a moving target by design. Feed-forward
    commands the rate the target is actually moving at, so feedback only
    cleans up residuals.

    ``los_angle_rad`` is the target's inertial line-of-sight angle: the
    vehicle's MEASURED yaw (telemetry) plus the camera bearing
    (``TargetError.bearing_x_rad``). Both inputs are measurements — the
    command never appears in this estimate. That matters: a first attempt
    used ``d(bearing)/dt + commanded_yaw_rate``, and because PX4 does not
    achieve the command instantly, the commanded-vs-actual mismatch fed back
    through the estimator and self-excited (measured: mean error 0.105 ->
    0.42, 31 sign-flips/min). With the measured LOS angle, a stationary
    target gives a constant angle regardless of our own rotation, so the
    feed-forward is structurally zero for a still ball.

    The angle delta is wrap-aware; the rate is low-passed (``lpf_alpha`` in
    (0, 1]) and clamped to ``±limit_rad_s``. Returns the new filtered
    feed-forward (also the state for the next call).
    """
    if first_sample:
        return 0.0
    dt = max(float(dt), 1e-6)
    delta = float(los_angle_rad) - float(prev_los_angle_rad)
    delta = math.atan2(math.sin(delta), math.cos(delta))  # wrap to [-pi, pi]
    raw = delta / dt
    alpha = clamp(float(lpf_alpha), 1e-3, 1.0)
    ff = alpha * raw + (1.0 - alpha) * float(prev_ff_rad_s)
    lim = abs(float(limit_rad_s))
    return clamp(ff, -lim, lim)


def los_rate_from_state(
    *,
    rel_north_m: float,
    rel_east_m: float,
    rel_velocity_north_m_s: float,
    rel_velocity_east_m_s: float,
    min_range_m: float = 0.5,
) -> float:
    """Inertial LOS rate (rad/s, NED +CW) from relative position & velocity.

    For target at r = (rN, rE) relative to us, moving at relative velocity
    v = (vN, vE), the bearing lambda = atan2(rE, rN) rotates at

        d(lambda)/dt = (rN*vE - rE*vN) / |r|^2

    (the 2D cross product picks the tangential velocity component; radial
    motion contributes nothing). This replaces differentiating the delayed
    camera angle: fed from the target state estimator's PREDICTED state, the
    feed-forward carries no vision-pipeline latency. Returns 0 inside
    ``min_range_m`` (the rate blows up as 1/r at point-blank range and the
    estimate is least reliable exactly there).
    """
    rn, re = float(rel_north_m), float(rel_east_m)
    r2 = rn * rn + re * re
    if r2 < float(min_range_m) ** 2:
        return 0.0
    return (rn * float(rel_velocity_east_m_s) - re * float(rel_velocity_north_m_s)) / r2
