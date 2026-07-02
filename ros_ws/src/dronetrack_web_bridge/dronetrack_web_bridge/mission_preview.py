"""Mission-plan preview helpers for the laptop dashboard.

This module is deliberately display-only. It loads local YAML mission files so
the operator can inspect them from the dashboard, but it never changes the
Pi-side mission executor or publishes any mission-selection command.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Any


VALID_STEP_TYPES = {
    "takeoff",
    "prime_offboard",
    "scan",
    "track_center",
    "approach",
    "orbit",
    "rtl",
    "land",
    "hold",
    "complete",
}

MOTION_STEP_TYPES = {"scan", "track_center", "approach", "orbit"}
TIMEOUT_RECOMMENDED_STEP_TYPES = {"scan", "approach", "orbit"}


def discover_mission_paths(module_file: str | None = None) -> list[Path]:
    """Return likely local mission YAML directories/files for preview.

    Source checkouts can preview the sibling ``drone_control/missions`` folder.
    Installed systems can preview the ``drone_control`` share directory when that
    package is present. Missing candidates are ignored.
    """
    candidates: list[Path] = []

    if module_file:
        current = Path(module_file).resolve()
        for parent in current.parents:
            sibling = parent / "drone_control" / "missions"
            if sibling.is_dir():
                candidates.append(sibling)

    try:
        from ament_index_python.packages import get_package_share_directory

        share_missions = Path(get_package_share_directory("drone_control")) / "missions"
        if share_missions.is_dir():
            candidates.append(share_missions)
    except Exception:  # noqa: BLE001 - preview still works without ament index.
        pass

    return _dedupe_paths(candidates)


def load_mission_catalog(configured_paths: Any = "", module_file: str | None = None) -> list[dict[str, Any]]:
    """Load preview records for configured/default mission YAML files."""
    paths = _coerce_path_list(configured_paths)
    if not paths:
        paths = discover_mission_paths(module_file)

    records: list[dict[str, Any]] = []
    seen: set[Path] = set()
    for path in paths:
        for mission_file in _expand_mission_path(path):
            resolved = mission_file.resolve()
            if resolved in seen:
                continue
            seen.add(resolved)
            records.append(preview_mission_file(resolved, module_file=module_file))

    records.sort(key=lambda r: (not bool(r.get("valid")), str(r.get("name") or r.get("filename"))))
    return records


def preview_mission_data(data: Any, module_file: str | None = None) -> dict[str, Any]:
    """Validate an in-memory mission mapping and return a JSON-safe record.

    Used both for file previews and for dashboard mission-builder validation.
    Prefers the real drone_control parser (full validation, identical to what
    the Pi-side executor enforces) and falls back to the local lightweight
    checks when drone_control is not importable.
    """
    record: dict[str, Any] = {
        "name": "",
        "valid": False,
        "error": "",
        "warnings": [],
        "steps": [],
    }

    # Prefer the real drone_control parser: it enforces exactly what the Pi-side
    # executor enforces. The lightweight fallback is ONLY for environments where
    # drone_control is not importable — it must never mask a strict-parser
    # rejection, or the dashboard would call a plan valid that the executor
    # (correctly) refuses.
    try:
        _ensure_drone_control_importable(module_file)
        from drone_control.mission_plan import parse_mission_plan  # noqa: F401

        parser_available = True
    except Exception:  # noqa: BLE001
        parser_available = False

    try:
        if parser_available:
            preview = _preview_with_drone_control(data, module_file)
        else:
            preview = _preview_with_fallback(data)
    except Exception as exc:  # noqa: BLE001
        record["error"] = str(exc)
        return record

    record.update(preview)
    record["valid"] = True
    return record


def preview_mission_file(path: Path, module_file: str | None = None) -> dict[str, Any]:
    """Return a JSON-safe preview record for one mission YAML file."""
    record: dict[str, Any] = {
        "filename": path.name,
        "path": str(path),
        "name": path.stem,
        "valid": False,
        "error": "",
        "warnings": [],
        "steps": [],
        "pi_param_hint": (
            "Set mission_executor_node.mission_plan_file on the Pi to the matching "
            f"Pi-side YAML path. Bundled example install path: "
            f"install/drone_control/share/drone_control/missions/{path.name}"
        ),
    }

    try:
        data = _load_yaml_data(path)
    except Exception as exc:  # noqa: BLE001
        record["error"] = f"could not read mission YAML: {exc}"
        return record

    data_record = preview_mission_data(data, module_file=module_file)
    name = data_record.pop("name", "") or record["name"]
    record.update(data_record)
    record["name"] = name
    return record


def _preview_with_drone_control(data: Any, module_file: str | None) -> dict[str, Any]:
    _ensure_drone_control_importable(module_file)
    from drone_control.mission_plan import lint_plan, parse_mission_plan

    plan = parse_mission_plan(data)
    return {
        "name": plan.name,
        "warnings": lint_plan(plan),
        "steps": [
            {
                "index": index,
                "type": step.type,
                "params": dict(step.params),
                "label": _step_label(step.type, step.params),
            }
            for index, step in enumerate(plan.steps)
        ],
    }


def _preview_with_fallback(data: Any) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ValueError("mission plan must be a mapping with a top-level 'mission' key")
    mission = data.get("mission")
    if not isinstance(mission, dict):
        raise ValueError("mission plan must contain a 'mission' mapping")
    raw_steps = mission.get("steps")
    if not isinstance(raw_steps, list) or not raw_steps:
        raise ValueError("mission must contain a non-empty 'steps' list")

    steps: list[dict[str, Any]] = []
    for index, raw in enumerate(raw_steps):
        if isinstance(raw, str):
            raw = {"type": raw}
        if not isinstance(raw, dict):
            raise ValueError(f"step {index} must be a step name or mapping")
        params = dict(raw)
        step_type = str(params.pop("type", ""))
        if step_type not in VALID_STEP_TYPES:
            raise ValueError(f"step {index} has unknown type '{step_type}'")
        steps.append(
            {
                "index": index,
                "type": step_type,
                "params": params,
                "label": _step_label(step_type, params),
            }
        )

    warnings = _fallback_lint(steps)
    return {"name": str(mission.get("name", "unnamed")), "warnings": warnings, "steps": steps}


def _load_yaml_data(path: Path) -> Any:
    text = path.read_text(encoding="utf-8")
    try:
        import yaml

        return yaml.safe_load(text)
    except ImportError:
        return _parse_basic_mission_yaml(text)


def _parse_basic_mission_yaml(text: str) -> dict[str, Any]:
    """Parse the small mission YAML subset used by bundled example plans.

    This is only a PyYAML-free fallback for dashboard previews. It supports:
    ``mission:``, ``name:``, ``steps:``, bare string steps, and one-line inline
    step mappings such as ``- {type: scan, timeout_s: 12}``.
    """
    mission: dict[str, Any] = {}
    steps: list[Any] = []
    in_mission = False
    in_steps = False

    for raw_line in text.splitlines():
        line = _strip_yaml_comment(raw_line).rstrip()
        if not line.strip():
            continue

        stripped = line.strip()
        indent = len(line) - len(line.lstrip(" "))
        if indent == 0 and stripped == "mission:":
            in_mission = True
            in_steps = False
            continue
        if not in_mission:
            continue
        if indent <= 2 and stripped.startswith("name:"):
            mission["name"] = _parse_scalar(stripped.split(":", 1)[1].strip())
            continue
        if indent <= 2 and stripped == "steps:":
            in_steps = True
            continue
        if in_steps and stripped.startswith("- "):
            steps.append(_parse_step_line(stripped[2:].strip()))

    if not in_mission:
        raise ValueError("mission plan must contain a top-level 'mission' key")
    mission["steps"] = steps
    return {"mission": mission}


def _parse_step_line(text: str) -> Any:
    if text.startswith("{") and text.endswith("}"):
        body = text[1:-1].strip()
        out: dict[str, Any] = {}
        if not body:
            return out
        for part in body.split(","):
            if ":" not in part:
                raise ValueError(f"bad inline step mapping item: {part!r}")
            key, value = part.split(":", 1)
            out[key.strip()] = _parse_scalar(value.strip())
        return out
    return _parse_scalar(text)


def _parse_scalar(text: str) -> Any:
    value = text.strip().strip('"').strip("'")
    if value.lower() in ("true", "false"):
        return value.lower() == "true"
    try:
        if any(ch in value for ch in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _strip_yaml_comment(line: str) -> str:
    in_single = False
    in_double = False
    for index, char in enumerate(line):
        if char == "'" and not in_double:
            in_single = not in_single
        elif char == '"' and not in_single:
            in_double = not in_double
        elif char == "#" and not in_single and not in_double:
            return line[:index]
    return line


def _fallback_lint(steps: list[dict[str, Any]]) -> list[str]:
    warnings: list[str] = []
    primed = False
    has_motion = False
    has_prime = any(step["type"] == "prime_offboard" for step in steps)
    for step in steps:
        index = int(step["index"])
        step_type = str(step["type"])
        params = dict(step.get("params") or {})
        if step_type == "prime_offboard":
            primed = True
        elif step_type in MOTION_STEP_TYPES:
            has_motion = True
            if not primed:
                warnings.append(
                    f"step {index} '{step_type}' runs before any 'prime_offboard'; Offboard may not be active yet"
                )
        if step_type in TIMEOUT_RECOMMENDED_STEP_TYPES and "timeout_s" not in params:
            warnings.append(
                f"step {index} '{step_type}' has no timeout_s; add one so the step cannot run indefinitely"
            )
        if step_type == "scan" and params.get("until") in (None, "none") and "timeout_s" not in params:
            warnings.append(
                f"step {index} 'scan' has neither 'until' nor 'timeout_s'; it should have a bounded exit"
            )
    if has_motion and not has_prime:
        warnings.append("plan contains motion steps but no 'prime_offboard' step; Offboard is never primed")
    return warnings


def _ensure_drone_control_importable(module_file: str | None) -> None:
    try:
        import drone_control.mission_plan  # noqa: F401

        return
    except ImportError:
        pass

    if not module_file:
        return
    current = Path(module_file).resolve()
    for parent in current.parents:
        package_root = parent / "drone_control"
        mission_plan = package_root / "drone_control" / "mission_plan.py"
        if mission_plan.is_file():
            sys.path.insert(0, str(package_root))
            return


def _coerce_path_list(raw: Any) -> list[Path]:
    if raw is None:
        return []
    if isinstance(raw, (list, tuple)):
        return [Path(str(item)).expanduser() for item in raw if str(item).strip()]
    text = str(raw).strip()
    if not text:
        return []
    pieces = []
    for chunk in text.replace(";", os.pathsep).split(os.pathsep):
        chunk = chunk.strip()
        if chunk:
            pieces.append(Path(chunk).expanduser())
    return pieces


def _expand_mission_path(path: Path) -> list[Path]:
    if path.is_file() and path.suffix.lower() in (".yaml", ".yml"):
        return [path]
    if path.is_dir():
        return sorted([p for p in path.glob("*.y*ml") if p.is_file()])
    return []


def _dedupe_paths(paths: list[Path]) -> list[Path]:
    out: list[Path] = []
    seen: set[Path] = set()
    for path in paths:
        resolved = path.resolve()
        if resolved not in seen:
            seen.add(resolved)
            out.append(path)
    return out


def _step_label(step_type: str, params: dict[str, Any]) -> str:
    if step_type == "takeoff":
        return f"takeoff to {params.get('altitude_m', 'default')} m"
    if step_type == "prime_offboard":
        return f"prime offboard for {params.get('hold_s', 'default')} s"
    if step_type == "scan":
        return (
            f"scan {params.get('direction', 'ccw')} "
            f"{params.get('yaw_deg', 'default')} deg @ {params.get('yaw_rate_deg_s', 'default')} deg/s"
        )
    if step_type == "track_center":
        return f"track center until {params.get('until', 'none')}"
    if step_type == "approach":
        return f"approach to {params.get('distance_m', 'default')} m"
    if step_type == "orbit":
        return (
            f"orbit r={params.get('radius_m', 'default')} m, "
            f"speed={params.get('speed_m_s', 'default')} m/s"
        )
    if step_type == "rtl":
        return "return to launch"
    if step_type == "land":
        return "land"
    if step_type == "hold":
        return str(params.get("status", "hold position"))
    return step_type
