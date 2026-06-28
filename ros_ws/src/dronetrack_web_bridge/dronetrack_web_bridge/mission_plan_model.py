"""
Pure-Python mission plan model for the web dashboard (no rclpy, no ROS imports).
Mirrors drone_control/mission_plan.py but lives with the web bridge so the
dashboard can validate, serialise, and lint mission plans without importing
the Pi-side package.
"""

import math
import re
from typing import Any

# ── Step schema: all 9 verbs with per-verb parameter definitions ──────────

STEP_SCHEMA: dict[str, dict[str, Any]] = {
    "takeoff": {
        "label": "Take Off",
        "description": "Command takeoff if not already airborne",
        "params": {
            "altitude_m": {
                "type": "float", "default": 3.0, "min": 0.5, "step": 0.5,
                "label": "Altitude (m)",
            },
        },
        "category": "action",
    },
    "prime_offboard": {
        "label": "Prime Offboard",
        "description": "Enable offboard mode and hold for a stabilisation period",
        "params": {
            "hold_s": {
                "type": "float", "default": 1.5, "min": 0.0, "step": 0.5,
                "label": "Hold Time (s)",
            },
        },
        "category": "preflight",
    },
    "scan": {
        "label": "Scan / Seek",
        "description": "Yaw sweep to find the target in place",
        "params": {
            "direction": {
                "type": "enum", "default": "ccw", "options": ["ccw", "cw"],
                "label": "Sweep Direction",
            },
            "yaw_deg": {
                "type": "float", "default": 180.0, "min": 5.0, "max": 360.0,
                "step": 15.0, "label": "Sweep Angle (deg)",
            },
            "yaw_rate_deg_s": {
                "type": "float", "default": 20.0, "min": 1.0, "max": 90.0,
                "step": 5.0, "label": "Sweep Rate (deg/s)",
            },
            "until": {
                "type": "enum", "default": "locked",
                "options": ["locked", "none"],
                "label": "Exit Condition",
            },
            "timeout_s": {
                "type": "float", "default": 12.0, "min": 0.0, "step": 1.0,
                "label": "Timeout (s)",
            },
        },
        "category": "motion",
    },
    "track_center": {
        "label": "Track Center",
        "description": "Maintain yaw-on-target and fly toward the detected object",
        "params": {
            "distance_m": {
                "type": "float", "default": 3.0, "min": 0.5, "step": 0.5,
                "label": "Target Distance (m)",
            },
            "until": {
                "type": "enum", "default": "centered",
                "options": ["centered", "approach_done", "none"],
                "label": "Exit Condition",
            },
            "timeout_s": {
                "type": "float", "default": 15.0, "min": 0.0, "step": 1.0,
                "label": "Timeout (s)",
            },
        },
        "category": "motion",
    },
    "approach": {
        "label": "Approach Target",
        "description": "Fly to within the specified distance of the target",
        "params": {
            "distance_m": {
                "type": "float", "default": 2.0, "min": 0.3, "step": 0.5,
                "label": "Approach Distance (m)",
            },
            "timeout_s": {
                "type": "float", "default": 20.0, "min": 0.0, "step": 1.0,
                "label": "Timeout (s)",
            },
        },
        "category": "motion",
    },
    "orbit": {
        "label": "Orbit",
        "description": "Orbit the target at a fixed radius and speed",
        "params": {
            "radius_m": {
                "type": "float", "default": 2.0, "min": 0.5, "step": 0.5,
                "label": "Orbit Radius (m)",
            },
            "speed_m_s": {
                "type": "float", "default": 0.4, "min": 0.1, "step": 0.1,
                "label": "Orbit Speed (m/s)",
            },
            "revolutions": {
                "type": "float", "default": 1.0, "min": 0.25, "step": 0.25,
                "label": "Revolutions",
            },
            "timeout_s": {
                "type": "float", "default": 45.0, "min": 0.0, "step": 1.0,
                "label": "Timeout (s)",
            },
        },
        "category": "motion",
    },
    "rtl": {
        "label": "Return to Launch",
        "description": "Command RTL (land at home position)",
        "params": {
            "timeout_s": {
                "type": "float", "default": 15.0, "min": 0.0, "step": 1.0,
                "label": "Timeout (s)",
            },
        },
        "category": "action",
    },
    "land": {
        "label": "Land",
        "description": "Command immediate landing",
        "params": {
            "timeout_s": {
                "type": "float", "default": 10.0, "min": 0.0, "step": 1.0,
                "label": "Timeout (s)",
            },
        },
        "category": "action",
    },
    "hold": {
        "label": "Hold",
        "description": "Hold position (offboard hover)",
        "params": {
            "status": {
                "type": "str", "default": "holding position",
                "label": "Status",
            },
            "timeout_s": {
                "type": "float", "default": 0.0, "min": 0.0, "step": 1.0,
                "label": "Timeout (s, 0=forever)",
            },
        },
        "category": "preflight",
    },
    "goto_relative": {
        "label": "Go To (Relative)",
        "description": "Fly a NED offset from the current position",
        "params": {
            "north_m": {
                "type": "float", "default": 0.0, "min": -50.0, "max": 50.0, "step": 0.5,
                "label": "North (m)",
            },
            "east_m": {
                "type": "float", "default": 0.0, "min": -50.0, "max": 50.0, "step": 0.5,
                "label": "East (m)",
            },
            "down_m": {
                "type": "float", "default": 0.0, "min": -20.0, "max": 10.0, "step": 0.5,
                "label": "Down (m, negative=up)",
            },
            "timeout_s": {
                "type": "float", "default": 15.0, "min": 0.0, "step": 1.0,
                "label": "Timeout (s)",
            },
            "acceptance_m": {
                "type": "float", "default": 0.5, "min": 0.1, "step": 0.1,
                "label": "Acceptance Radius (m)",
            },
        },
        "category": "motion",
    },
    "goto_absolute": {
        "label": "Go To (Absolute)",
        "description": "Fly to an absolute local NED position (relative to home)",
        "params": {
            "north_m": {
                "type": "float", "default": 0.0, "min": -50.0, "max": 50.0, "step": 0.5,
                "label": "North (m)",
            },
            "east_m": {
                "type": "float", "default": 0.0, "min": -50.0, "max": 50.0, "step": 0.5,
                "label": "East (m)",
            },
            "down_m": {
                "type": "float", "default": -3.0, "min": -50.0, "max": 0.0, "step": 0.5,
                "label": "Down (m, negative=up)",
            },
            "timeout_s": {
                "type": "float", "default": 15.0, "min": 0.0, "step": 1.0,
                "label": "Timeout (s)",
            },
            "acceptance_m": {
                "type": "float", "default": 0.5, "min": 0.1, "step": 0.1,
                "label": "Acceptance Radius (m)",
            },
        },
        "category": "motion",
    },
}

CATEGORY_COLORS: dict[str, str] = {
    "preflight": "blue",
    "action": "green",
    "motion": "orange",
}

# ── Public API ────────────────────────────────────────────────────────────

def get_step_schema() -> dict:
    """Return the full step schema for the UI."""
    return STEP_SCHEMA


def create_default_step(verb: str) -> dict:
    """Return a dict {"type": verb, "params": {...defaults...}}."""
    if verb not in STEP_SCHEMA:
        raise ValueError(f"Unknown verb: {verb}")
    schema = STEP_SCHEMA[verb]
    params = {}
    for key, spec in schema.get("params", {}).items():
        params[key] = spec["default"]
    return {"type": verb, "params": params}


def validate_step(step: dict) -> list[str]:
    """Validate a single step dict against the schema. Returns list of error strings (empty = valid)."""
    errors: list[str] = []

    if not isinstance(step, dict):
        return ["step is not a dict"]

    verb = step.get("type")
    if not isinstance(verb, str) or verb not in STEP_SCHEMA:
        return [f"unknown step type: {verb!r}"]

    schema = STEP_SCHEMA[verb]
    params = step.get("params", {})
    if not isinstance(params, dict):
        return ["step params is not a dict"]

    for key, spec in schema.get("params", {}).items():
        val = params.get(key)
        if val is None:
            errors.append(f"missing param '{key}' for {verb}")
            continue

        ptype = spec["type"]
        if ptype == "float":
            if not isinstance(val, (int, float)):
                errors.append(f"param '{key}' must be a number, got {type(val).__name__}")
                continue
            val = float(val)
            if not math.isfinite(val):
                errors.append(f"param '{key}' must be finite, got {val}")
                continue
            if "min" in spec and val < spec["min"]:
                errors.append(f"param '{key}'={val} below minimum {spec['min']}")
            if "max" in spec and val > spec["max"]:
                errors.append(f"param '{key}'={val} above maximum {spec['max']}")
        elif ptype in ("enum", "str"):
            if not isinstance(val, str):
                errors.append(f"param '{key}' must be a string, got {type(val).__name__}")
                continue
            if ptype == "enum" and val not in spec.get("options", []):
                errors.append(f"param '{key}'={val!r} not in options {spec['options']}")

    return errors


def steps_to_yaml(plan_name: str, steps: list[dict]) -> str:
    """Serialize a plan to valid mission YAML."""
    import yaml

    data: dict[str, Any] = {
        "mission": {
            "name": plan_name.strip() or "untitled",
            "steps": [],
        }
    }
    for step in steps:
        verb = step.get("type")
        if not isinstance(verb, str) or verb not in STEP_SCHEMA:
            raise ValueError(f"Unknown or missing step type: {verb!r}")
        entry: dict[str, Any] = {"type": verb}
        for key, val in step.get("params", {}).items():
            entry[key] = val
        data["mission"]["steps"].append(entry)

    return yaml.dump(data, default_flow_style=False, sort_keys=False, allow_unicode=True)


def lint_steps(steps: list[dict]) -> list[str]:
    """Plan-level lint warnings (motion-before-prime, missing timeouts, etc.)."""
    warnings: list[str] = []

    if not steps:
        warnings.append("Plan is empty")
        return warnings

    has_prime = any(s.get("type") == "prime_offboard" for s in steps)
    has_motion = any(
        s.get("type") in {"scan", "track_center", "approach", "orbit", "goto_relative", "goto_absolute"}
        for s in steps
    )

    seen_prime = False
    for i, step in enumerate(steps):
        verb = step.get("type", "")
        is_motion = verb in {"scan", "track_center", "approach", "orbit", "goto_relative", "goto_absolute"}

        if is_motion and not seen_prime:
            warnings.append(
                f"Step {i+1} ({verb}): motion step before prime_offboard — "
                "add a prime_offboard step first"
            )

        if verb == "prime_offboard":
            seen_prime = True

        timeout = step.get("params", {}).get("timeout_s")
        if verb in {"scan", "approach", "orbit"} and timeout is None:
            warnings.append(
                f"Step {i+1} ({verb}): no timeout set — step may run indefinitely"
            )

        if verb == "scan":
            until = step.get("params", {}).get("until")
            if until in (None, "none") and timeout is None:
                warnings.append(
                    f"Step {i+1} (scan): has neither 'until' nor 'timeout_s' — "
                    "the step could run indefinitely"
                )

    if has_motion and not has_prime:
        warnings.append(
            "Plan contains motion steps but no prime_offboard — "
            "offboard mode will not be enabled"
        )

    return warnings


def sanitize_filename(name: str) -> str:
    """Lowercase, underscores, strip special characters."""
    s = name.strip().lower()
    s = re.sub(r"[^a-z0-9]+", "_", s)
    s = s.strip("_")
    return s or "untitled_mission"
