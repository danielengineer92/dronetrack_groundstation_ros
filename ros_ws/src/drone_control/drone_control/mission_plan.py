"""
Declarative mission plan model and loader for the mission executor.

A mission plan is an ordered list of step "verbs" (takeoff, prime_offboard,
scan, track_center, approach, orbit, rtl, land, hold). The mission executor walks
the plan instead of a hardcoded state chain, so a flight script can be reordered
or swapped between runs by editing/selecting a YAML file instead of editing code.

This module is intentionally free of any rclpy/ROS imports so it can be unit
tested with plain Python and validated offline.

Plan file format (YAML)::

    mission:
      name: orbit_red_ball
      steps:
        - {type: takeoff, altitude_m: 3.0}
        - {type: prime_offboard, hold_s: 1.5}
        - {type: track_center, until: centered, timeout_s: 15}
        - {type: orbit, radius_m: 2.0, speed_m_s: 0.4, revolutions: 1}
        - {type: land}

Per-step params override the executor's existing parameter defaults
(takeoff_altitude_m, orbit_radius_m, ...); anything omitted falls back to those.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional


# Step verb -> MissionState name (the executor resolves this to its own enum).
# Keeping the mapping as plain strings avoids importing the rclpy-based node here.
STEP_TYPE_TO_STATE: dict[str, str] = {
    "takeoff": "TAKEOFF",
    "prime_offboard": "PRIME_OFFBOARD",
    "scan": "SCAN",
    "track_center": "TRACK_CENTER",
    "approach": "APPROACH_TARGET",
    "orbit": "DO_ORBIT",
    "rtl": "RETURN_TO_LAUNCH",
    "land": "LAND",
    "hold": "HOLD",
    "complete": "COMPLETE",
}

VALID_STEP_TYPES: frozenset[str] = frozenset(STEP_TYPE_TO_STATE)

# Exit predicates the executor knows how to evaluate. "none" means the step never
# auto-advances on a condition (it only advances on its timeout, or never if no
# timeout is set) which reproduces today's track_center_timeout_s=0 behavior.
VALID_UNTIL: frozenset[str] = frozenset(
    {"airborne", "centered", "locked", "approach_done", "none"}
)

# Step verbs that command vehicle motion / Offboard. Used only for lint warnings:
# these should normally come after a prime_offboard step. `scan` yaw-sweeps in
# Offboard (position-held), so it belongs here too.
MOTION_STEP_TYPES: frozenset[str] = frozenset(
    {"scan", "track_center", "approach", "orbit"}
)

# Verbs that run open-loop or search for a bounded time. A plan author should give
# each one a timeout so a lost/never-seen target cannot wedge the mission forever.
# (track_center is deliberately excluded: until=none + no timeout is its supported
# "hold and yaw-track indefinitely" mode.)
TIMEOUT_RECOMMENDED_STEP_TYPES: frozenset[str] = frozenset(
    {"scan", "approach", "orbit"}
)

# Allowed sweep directions for a `scan` step (counter-clockwise / clockwise yaw).
VALID_SCAN_DIRECTIONS: frozenset[str] = frozenset({"ccw", "cw"})

# Known numeric step parameters and their validity ranges, checked at parse time.
# Mission plans can now arrive from the dashboard (untrusted operator input), so
# every number a step handler will later read with get_float() must already be a
# finite float in a sane range — a bad value must fail at load, never mid-flight.
# Format: key -> (min_exclusive, max_inclusive). None disables that bound.
NUMERIC_STEP_PARAM_RANGES: dict[str, tuple[Optional[float], Optional[float]]] = {
    "altitude_m": (0.0, 120.0),
    "hold_s": (0.0, 600.0),
    "yaw_deg": (0.0, 100_000.0),
    "yaw_rate_deg_s": (0.0, 180.0),
    "distance_m": (0.0, 500.0),
    "radius_m": (0.0, 500.0),
    "speed_m_s": (0.0, 20.0),
    "revolutions": (0.0, 100.0),
}


@dataclass
class MissionStep:
    """One step in a mission plan: a verb plus its parameter overrides."""

    type: str
    params: dict = field(default_factory=dict)

    @property
    def until(self) -> Optional[str]:
        value = self.params.get("until")
        return None if value is None else str(value)

    @property
    def timeout_s(self) -> Optional[float]:
        value = self.params.get("timeout_s")
        if value is None:
            return None
        try:
            timeout = float(value)
        except (TypeError, ValueError):
            return None
        return timeout if math.isfinite(timeout) and timeout >= 0.0 else None

    @property
    def state_name(self) -> str:
        return STEP_TYPE_TO_STATE[self.type]

    def get_float(self, key: str, default: float) -> float:
        # Validated at parse time; the fallback keeps a handler tick from ever
        # raising inside the executor timer if a plan was built programmatically.
        value = self.params.get(key)
        if value is None:
            return float(default)
        try:
            result = float(value)
        except (TypeError, ValueError):
            return float(default)
        return result if math.isfinite(result) else float(default)

    @property
    def scan_direction(self) -> str:
        """Sweep direction for a scan step ('ccw' or 'cw'); defaults to 'ccw'."""
        return str(self.params.get("direction", "ccw")).lower()


@dataclass
class MissionPlan:
    name: str
    steps: tuple[MissionStep, ...]


class MissionPlanError(ValueError):
    """Raised when a mission plan is structurally invalid."""


def build_default_plan(run_full_orbit: bool = False) -> MissionPlan:
    """Built-in plan used when no mission_plan_file is provided.

    This reproduces the executor's original hardcoded behavior exactly so that
    existing launches/configs are unchanged:
      - run_full_orbit=False: takeoff (if needed) -> prime_offboard -> track_center
        (holds forever, matching track_center_timeout_s=0).
      - run_full_orbit=True: append approach -> orbit -> rtl -> land, and let
        track_center advance once the target is locked + centered.
    """
    if run_full_orbit:
        steps = [
            MissionStep("takeoff"),
            MissionStep("prime_offboard"),
            MissionStep("track_center", {"until": "centered"}),
            MissionStep("approach"),
            MissionStep("orbit"),
            MissionStep("rtl"),
            MissionStep("land"),
        ]
    else:
        steps = [
            MissionStep("takeoff"),
            MissionStep("prime_offboard"),
            MissionStep("track_center", {"until": "none"}),
        ]
    return MissionPlan("default", tuple(steps))


def parse_mission_plan(data: object) -> MissionPlan:
    """Parse a loaded mapping (e.g. from yaml.safe_load) into a MissionPlan.

    Raises MissionPlanError with a clear message on any structural problem.
    Pure function with no file IO so it can be tested directly with a dict.
    """
    if not isinstance(data, dict):
        raise MissionPlanError("mission plan must be a mapping with a top-level 'mission' key")
    mission = data.get("mission")
    if not isinstance(mission, dict):
        raise MissionPlanError("mission plan must contain a 'mission' mapping")

    name = str(mission.get("name", "unnamed"))
    raw_steps = mission.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise MissionPlanError(f"mission '{name}' must contain a non-empty 'steps' list")

    steps: list[MissionStep] = []
    for index, raw in enumerate(raw_steps):
        steps.append(_parse_step(raw, index, name))

    return MissionPlan(name, tuple(steps))


def _parse_step(raw: object, index: int, mission_name: str) -> MissionStep:
    where = f"mission '{mission_name}' step {index}"

    # Allow the bare-string shorthand "land" as well as {type: land, ...}.
    if isinstance(raw, str):
        raw = {"type": raw}
    if not isinstance(raw, dict):
        raise MissionPlanError(f"{where} must be a step name or a mapping with a 'type'")

    params = dict(raw)
    step_type = params.pop("type", None)
    if step_type is None:
        raise MissionPlanError(f"{where} is missing required 'type'")
    step_type = str(step_type)
    if step_type not in VALID_STEP_TYPES:
        valid = ", ".join(sorted(VALID_STEP_TYPES))
        raise MissionPlanError(f"{where} has unknown type '{step_type}'. Valid types: {valid}")

    until = params.get("until")
    if until is not None and str(until) not in VALID_UNTIL:
        valid = ", ".join(sorted(VALID_UNTIL))
        raise MissionPlanError(
            f"{where} (type '{step_type}') has unknown until '{until}'. Valid: {valid}"
        )

    if "timeout_s" in params:
        try:
            timeout_val = float(params["timeout_s"])
        except (TypeError, ValueError):
            raise MissionPlanError(f"{where} timeout_s must be a number, got {params['timeout_s']!r}")
        if not math.isfinite(timeout_val) or timeout_val < 0.0:
            raise MissionPlanError(f"{where} timeout_s must be a finite number >= 0, got {timeout_val}")

    for key, (lo, hi) in NUMERIC_STEP_PARAM_RANGES.items():
        if key not in params:
            continue
        try:
            value = float(params[key])
        except (TypeError, ValueError):
            raise MissionPlanError(f"{where} {key} must be a number, got {params[key]!r}")
        if not math.isfinite(value):
            raise MissionPlanError(f"{where} {key} must be finite, got {value}")
        if lo is not None and value <= lo:
            raise MissionPlanError(f"{where} {key} must be > {lo:g}, got {value:g}")
        if hi is not None and value > hi:
            raise MissionPlanError(f"{where} {key} must be <= {hi:g}, got {value:g}")

    if step_type == "scan":
        _validate_scan_params(params, where)

    return MissionStep(step_type, params)


def _validate_scan_params(params: dict, where: str) -> None:
    """Validate the scan-specific keys: direction, yaw_deg, yaw_rate_deg_s.

    These describe a bounded yaw sweep (hold position, rotate up to yaw_deg at
    yaw_rate_deg_s). Bad values are rejected at load time so a malformed scan can
    never be dispatched to the vehicle.
    """
    direction = str(params.get("direction", "ccw")).lower()
    if direction not in VALID_SCAN_DIRECTIONS:
        valid = ", ".join(sorted(VALID_SCAN_DIRECTIONS))
        raise MissionPlanError(
            f"{where} (type 'scan') has unknown direction '{params.get('direction')}'. Valid: {valid}"
        )

    for key, must_be_positive in (("yaw_deg", True), ("yaw_rate_deg_s", True)):
        if key not in params:
            continue
        try:
            value = float(params[key])
        except (TypeError, ValueError):
            raise MissionPlanError(f"{where} (type 'scan') {key} must be a number, got {params[key]!r}")
        if not math.isfinite(value):
            raise MissionPlanError(f"{where} (type 'scan') {key} must be finite, got {value}")
        if must_be_positive and value <= 0.0:
            raise MissionPlanError(f"{where} (type 'scan') {key} must be > 0, got {value}")


def lint_plan(plan: MissionPlan) -> list[str]:
    """Return non-fatal warnings about a plan (does not raise).

    The executor logs these on load. Catches the common ordering mistake of a
    motion/Offboard step appearing before Offboard has been primed.
    """
    warnings: list[str] = []
    primed = False
    has_motion = False
    has_prime = any(s.type == "prime_offboard" for s in plan.steps)
    for index, step in enumerate(plan.steps):
        if step.type == "prime_offboard":
            primed = True
        elif step.type in MOTION_STEP_TYPES:
            has_motion = True
            if not primed:
                warnings.append(
                    f"step {index} '{step.type}' runs before any 'prime_offboard'; "
                    "Offboard may not be active yet"
                )

        # Bounded/search steps should carry a timeout so a lost or never-seen
        # target cannot wedge the mission forever.
        if step.type in TIMEOUT_RECOMMENDED_STEP_TYPES and step.timeout_s is None:
            warnings.append(
                f"step {index} '{step.type}' has no timeout_s; add one so the step "
                "cannot run indefinitely if its exit condition is never met"
            )

        # A scan that never auto-advances and has no timeout would sweep forever.
        if step.type == "scan" and step.until in (None, "none") and step.timeout_s is None:
            warnings.append(
                f"step {index} 'scan' has neither 'until' nor 'timeout_s'; "
                "it should exit on 'until: locked' and/or a timeout_s"
            )

    if has_motion and not has_prime:
        warnings.append(
            "plan contains motion steps but no 'prime_offboard' step; "
            "Offboard is never primed"
        )

    return warnings


def parse_mission_plan_text(text: str, source: str = "<inline>") -> MissionPlan:
    """Parse a YAML/JSON mission plan document. Raises MissionPlanError on failure.

    Used for plans that arrive over a topic (e.g. uploaded from the dashboard);
    JSON is a subset of YAML so both serializations go through the same parser
    and the same validation.
    """
    import yaml  # local import: PyYAML is always available in a ROS 2 environment

    max_plan_bytes = 64 * 1024
    if len(text.encode("utf-8", errors="replace")) > max_plan_bytes:
        raise MissionPlanError(f"mission plan from {source} exceeds {max_plan_bytes} bytes")

    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise MissionPlanError(f"mission plan from {source} is not valid YAML: {exc}") from exc

    return parse_mission_plan(data)


def load_mission_plan(path: str) -> MissionPlan:
    """Load and parse a YAML mission plan file. Raises MissionPlanError on failure."""
    plan_path = Path(path).expanduser()
    try:
        text = plan_path.read_text(encoding="utf-8")
    except OSError as exc:
        raise MissionPlanError(f"could not read mission plan file '{plan_path}': {exc}") from exc

    return parse_mission_plan_text(text, source=f"file '{plan_path}'")
