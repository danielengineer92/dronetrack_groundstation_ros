"""
Extensions for mission_preview.py:
- steps_to_yaml() serialization from mission_plan_model
- plan_from_steps() factory from in-memory plan

These are thin wrappers that delegate to mission_plan_model.
"""

from .mission_plan_model import steps_to_yaml as _steps_to_yaml, lint_steps as _lint_steps


def steps_to_yaml(plan_name: str, steps: list) -> str:
    """Serialize a plan (list of step dicts) to valid mission YAML."""
    return _steps_to_yaml(plan_name, steps)


def plan_from_steps(plan_name: str, steps: list) -> dict:
    """Factory: build a plan dict with name, steps, and lint warnings."""
    return {
        "name": plan_name,
        "steps": steps,
        "warnings": _lint_steps(steps),
    }
