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
    north_m, east_m = project_target_local_offset(
        yaw_rad=yaw_rad,
        bearing_x_rad=bearing_x_rad,
        bearing_y_rad=bearing_y_rad,
        slant_distance_m=slant_distance_m,
    )
    return offset_global(lat_deg, lon_deg, north_m, east_m)


def project_target_local_offset(
    *,
    yaw_rad: float,
    bearing_x_rad: float,
    bearing_y_rad: float,
    slant_distance_m: float,
) -> tuple[float, float]:
    """Project a monocular detection to a (north_m, east_m) offset from the drone.

    Same slant-range foreshortening and yaw+bearing heading as
    ``project_target_global`` (see its docstring), but stops at the local-NED
    offset so callers working in PX4 local coordinates (ORBIT_FIXED) don't have
    to round-trip through lat/lon.
    """
    horizontal_m = float(slant_distance_m) * math.cos(float(bearing_y_rad))
    global_bearing = float(yaw_rad) + float(bearing_x_rad)
    return (
        horizontal_m * math.cos(global_bearing),
        horizontal_m * math.sin(global_bearing),
    )


def orbit_fixed_setpoint(
    *,
    drone_north: float,
    drone_east: float,
    center_north: float,
    center_east: float,
    radius_m: float,
    speed_m_s: float,
    lead_s: float,
    clockwise: bool = True,
) -> tuple[float, float, float]:
    """Carrot position setpoint for a fixed-center orbit (no vision input).

    The setpoint sits ON the circle, leading the drone's current angular
    position by ``(speed / radius) * lead_s`` radians. Because the lead is
    measured from where the drone actually is, the carrot self-regulates: if
    the drone lags (wind, PX4 speed limits) the carrot waits instead of running
    away, and if the drone starts off-circle the nearest-point projection pulls
    it radially onto the ring. Angle convention matches atan2(east, north)
    (NED: 0 = north, positive toward east => positive lead = clockwise from
    above).

    Returns ``(north_sp, east_sp, yaw_rad)`` with yaw facing the center so the
    camera keeps filming the target even though vision is not consumed.
    """
    radius_m = max(float(radius_m), 0.1)
    dn = float(drone_north) - float(center_north)
    de = float(drone_east) - float(center_east)
    if math.hypot(dn, de) < 1e-6:
        # Degenerate: drone exactly at the center. Pick due north arbitrarily;
        # the next tick has geometry again.
        theta = 0.0
    else:
        theta = math.atan2(de, dn)

    lead = (float(speed_m_s) / radius_m) * max(float(lead_s), 0.0)
    theta_sp = theta + lead if clockwise else theta - lead

    north_sp = float(center_north) + radius_m * math.cos(theta_sp)
    east_sp = float(center_east) + radius_m * math.sin(theta_sp)
    yaw_rad = math.atan2(float(center_east) - east_sp, float(center_north) - north_sp)
    return north_sp, east_sp, yaw_rad


def settle_gate(
    streak_start_age: "float | None",
    step_age: float,
    centered: bool,
    settle_s: float,
) -> tuple["float | None", bool]:
    """Track a continuous target-centered streak; report when sampling may start.

    smart_orbit only trusts center fixes taken from a settled hover with the
    nose on the ball. The caller feeds this every tick with the step age and
    whether the target is centered *right now*; it returns the (possibly reset)
    streak start and whether the streak has lasted ``settle_s``. Any un-centered
    tick breaks the streak — the timer restarts from the next centered tick.

    ``ready`` is only ever True on a tick where ``centered`` is True, so a
    caller gating fixes on ``ready`` also gets per-fix centering for free.
    """
    if not centered:
        return None, False
    if streak_start_age is None:
        streak_start_age = float(step_age)
    return streak_start_age, (float(step_age) - streak_start_age) >= max(float(settle_s), 0.0)


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
    delay_s: float = 0.0,
    decel_m_s2: float = 0.0,
    error_x: float = 0.0,
    align_error_limit: float = 0.0,
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

    Two professional-approach refinements, each opt-in via its parameter:

    - **Delay-aware braking profile** (``decel_m_s2 > 0``): the speed is capped
      by ``braking_speed_limit`` over the remaining distance error, so a far
      target is closed on at ``max_speed`` but the vehicle decelerates INTO the
      standoff instead of lunging past it (the vision distance is ``delay_s``
      stale; a pure P law cannot brake in time once saturated).
    - **Alignment gating** (``align_error_limit > 0``): the CLOSING (positive)
      speed is scaled by ``alignment_scale`` of the normalized image error
      ``error_x``, so the vehicle never charges forward while the target is
      still far off-centre (forward is simply the wrong direction then).
      Backing off (negative) is never alignment-gated — increasing separation
      is safe regardless of centring.
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

    if decel_m_s2 > 0.0:
        cap = braking_speed_limit(
            remaining=distance_error_m,
            delay_s=delay_s,
            decel=decel_m_s2,
            max_speed=max_speed,
        )
        velocity = clamp(velocity, -cap, cap)

    if velocity > 0.0:
        velocity *= alignment_scale(error_x=error_x, error_limit=align_error_limit)

    return clamp(velocity, -max_speed, max_speed)


def braking_speed_limit(
    *,
    remaining: float,
    delay_s: float,
    decel: float,
    max_speed: float,
    min_speed: float = 0.0,
) -> float:
    """Largest speed MAGNITUDE that can still stop within ``remaining``, given
    a measurement dead time. Dimension-agnostic: rad & rad/s & rad/s^2 for the
    yaw axis, m & m/s & m/s^2 for translation.

    A far-away target saturates a proportional controller, so it closes at the
    configured max rate. But the vision measurement is delayed by ~``delay_s``
    (YOLO + smoothing + pipeline), so a saturated controller keeps commanding
    "go" for ``delay_s`` after it has actually arrived — it sails past the goal
    and swings back (yaw), or lunges past the standoff distance (approach).

    This returns the speed cap of a trapezoidal motion profile: the fastest
    speed from which the ``remaining`` error can be arrested, allowing a
    ``delay_s`` dead time during which we keep moving, then decelerating at
    ``decel``. Solving ``|remaining| = v*delay + v^2 / (2*a)`` for ``v``:

        v = -a*delay + sqrt((a*delay)^2 + 2*a*|remaining|)     (a = decel)

    which tends to ``|remaining| / delay`` for small errors (dead-time limited)
    and to ``sqrt(2*a*|remaining|)`` for large errors (decel limited). The
    result is a non-negative magnitude clamped to ``[min_speed, max_speed]``.
    The caller applies it as a symmetric clamp on its command, so it only ever
    *reduces* an over-eager rate; near the goal the underlying command is
    already ~0 so nothing changes. ``min_speed`` keeps a little authority for
    fine corrections so the cap never fully strangles the loop just outside
    the deadband.
    """
    err = abs(float(remaining))
    a = max(float(decel), 1e-6)
    delay = max(float(delay_s), 0.0)
    hi = abs(float(max_speed))
    lo = clamp(abs(float(min_speed)), 0.0, hi)

    at = a * delay
    speed = -at + math.sqrt(at * at + 2.0 * a * err)
    return clamp(speed, lo, hi)


def alignment_scale(*, error_x: float, error_limit: float, inner: float = 0.0) -> float:
    """Forward-speed scale [0, 1] from how far off-centre the target is.

    Translating toward a target whose bearing is still converging drives the
    vehicle in the WRONG direction — the classic "launch at the target"
    failure is full forward speed commanded while the yaw loop is still
    swinging. This ramps the allowed closing speed linearly from 1 (target
    centred) to 0 at ``|error_x| >= error_limit`` (normalized image error),
    so the vehicle centres first, then closes.

    ``inner`` (optional) flattens the gate to exactly 1.0 for
    ``|error_x| <= inner`` and ramps only between ``inner`` and
    ``error_limit``. Rationale: a sloped gate MODULATES its term with the
    error oscillation during steady tracking — for the yaw feed-forward that
    coupling acts as extra proportional gain that switches sign at every
    centre crossing. A small flat zone removes the in-band modulation while
    keeping the acquisition gating above it. ``inner = 0`` reproduces the
    original pure ramp; ``inner >= error_limit`` degrades to a hard on/off
    gate at ``error_limit``.

    ``error_limit <= 0`` disables the gate (returns 1.0).
    """
    limit = float(error_limit)
    if limit <= 0.0:
        return 1.0
    knee = clamp(float(inner), 0.0, limit)
    e = abs(float(error_x))
    if e <= knee:
        return 1.0
    if limit - knee <= 1e-9:
        return 0.0
    return clamp(1.0 - (e - knee) / (limit - knee), 0.0, 1.0)


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
    derivative_error: float | None = None,
    prev_derivative_error: float = 0.0,
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
    - **D can run on a separate error signal**: pass ``derivative_error`` (and
      ``prev_derivative_error``) to differentiate the RAW pre-deadband error
      while P and I keep the deadbanded one. The deadband rescale zeroes the
      slope inside the band and kinks it at the boundary — exactly where a
      centred tracking loop lives — so D on the deadbanded error produces
      phantom slope discontinuities at every band crossing. Omitting
      ``derivative_error`` keeps the original single-error behaviour.
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
        if derivative_error is not None:
            raw_derivative = (float(derivative_error) - float(prev_derivative_error)) / dt
        else:
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


def curved_lead_offset(
    *,
    velocity_north_m_s: float,
    velocity_east_m_s: float,
    turn_rate_rad_s: float,
    lead_s: float,
    max_turn_rate_rad_s: float = 2.0,
) -> tuple[float, float]:
    """Where a target goes in ``lead_s``, on a constant-turn-rate arc.

    Straight extrapolation ``v * lead`` points along the CURRENT velocity, but
    a turning target's true displacement is the chord of its arc — rotated by
    half the turn angle and slightly shorter. For a target of speed ``s`` and
    heading ``theta`` turning at ``omega``:

        dN = (s/omega) * ( sin(theta + omega*lead) - sin(theta) )
        dE = (s/omega) * (-cos(theta + omega*lead) + cos(theta) )

    which reduces to the straight ``s*lead*(cos theta, sin theta)`` as
    omega -> 0. Measured on the orbiting ball this cut the lead-vector
    direction error from ~23 deg (straight) to ~9 deg. ``turn_rate`` is clamped
    to ``max_turn_rate_rad_s`` (a noisy estimate must not fold the arc back on
    itself).
    """
    s = math.hypot(float(velocity_north_m_s), float(velocity_east_m_s))
    lead = float(lead_s)
    w = clamp(float(turn_rate_rad_s), -abs(max_turn_rate_rad_s), abs(max_turn_rate_rad_s))
    theta = math.atan2(float(velocity_east_m_s), float(velocity_north_m_s))
    if abs(w) < 1e-3:
        return s * lead * math.cos(theta), s * lead * math.sin(theta)
    dN = (s / w) * (math.sin(theta + w * lead) - math.sin(theta))
    dE = (s / w) * (-math.cos(theta + w * lead) + math.cos(theta))
    return dN, dE


def estimate_turn_rate(times_s, vel_north, vel_east) -> float:
    """Turn rate (rad/s) = slope of the velocity heading over a short history.

    ``times_s``/``vel_north``/``vel_east`` are equal-length recent samples.
    Returns 0 with fewer than 3 samples or near-zero speed (heading undefined).
    """
    n = len(times_s)
    if n < 3:
        return 0.0
    angles = [math.atan2(ve, vn) for vn, ve in zip(vel_north, vel_east)]
    # unwrap
    unwrapped = [angles[0]]
    for a in angles[1:]:
        prev = unwrapped[-1]
        while a - prev > math.pi:
            a -= 2 * math.pi
        while a - prev < -math.pi:
            a += 2 * math.pi
        unwrapped.append(a)
    t0 = times_s[0]
    xs = [t - t0 for t in times_s]
    mx = sum(xs) / n
    my = sum(unwrapped) / n
    denom = sum((x - mx) ** 2 for x in xs)
    if denom < 1e-9:
        return 0.0
    return sum((x - mx) * (y - my) for x, y in zip(xs, unwrapped)) / denom
