"""Pure tests for dashboard mission-plan preview helpers."""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

try:
    from dronetrack_web_bridge.mission_preview import (
        load_mission_catalog,
        preview_mission_data,
        preview_mission_file,
    )
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from dronetrack_web_bridge.mission_preview import (  # noqa: E402
        load_mission_catalog,
        preview_mission_data,
        preview_mission_file,
    )


def test_preview_valid_scan_plan():
    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "scan.yaml"
        path.write_text(
            """
mission:
  name: scan_preview
  steps:
    - {type: takeoff, altitude_m: 3.0}
    - {type: prime_offboard, hold_s: 1.5}
    - {type: scan, direction: ccw, yaw_deg: 180, yaw_rate_deg_s: 20, until: locked, timeout_s: 12}
    - {type: land}
""",
            encoding="utf-8",
        )

        preview = preview_mission_file(path, module_file=__file__)

    assert preview["valid"] is True
    assert preview["name"] == "scan_preview"
    assert [step["type"] for step in preview["steps"]] == ["takeoff", "prime_offboard", "scan", "land"]
    assert preview["warnings"] == []


def test_catalog_ignores_non_yaml_and_reports_invalid_yaml():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        (root / "ignore.txt").write_text("not yaml", encoding="utf-8")
        (root / "bad.yaml").write_text("mission: [", encoding="utf-8")
        (root / "ok.yaml").write_text(
            """
mission:
  name: ok
  steps:
    - takeoff
""",
            encoding="utf-8",
        )

        catalog = load_mission_catalog(str(root), module_file=__file__)

    assert [item["filename"] for item in catalog] == ["ok.yaml", "bad.yaml"]
    assert catalog[0]["valid"] is True
    assert catalog[1]["valid"] is False
    assert "could not read mission YAML" in catalog[1]["error"]


def test_preview_mission_data_valid_builder_payload():
    # Exactly the shape the dashboard mission builder POSTs to /api/mission_plan/*.
    data = {
        "mission": {
            "name": "builder_mission",
            "steps": [
                {"type": "takeoff", "altitude_m": 3.0},
                {"type": "prime_offboard", "hold_s": 1.5},
                {"type": "track_center", "until": "centered", "timeout_s": 15},
                {"type": "land"},
            ],
        }
    }
    record = preview_mission_data(data, module_file=__file__)
    assert record["valid"] is True
    assert record["name"] == "builder_mission"
    assert [step["type"] for step in record["steps"]] == [
        "takeoff", "prime_offboard", "track_center", "land",
    ]


def test_preview_mission_data_rejects_bad_step():
    record = preview_mission_data(
        {"mission": {"name": "bad", "steps": [{"type": "barrel_roll"}]}},
        module_file=__file__,
    )
    assert record["valid"] is False
    assert "barrel_roll" in record["error"]


def test_preview_mission_data_strict_parser_rejection_not_masked():
    # When drone_control is importable, its strict checks (numeric ranges) must
    # surface as invalid — the lightweight fallback must not paper over them.
    try:
        import drone_control.mission_plan  # noqa: F401
    except ImportError:
        return  # fallback-only environment; strict ranges are the executor's job
    record = preview_mission_data(
        {"mission": {"name": "bad", "steps": [{"type": "takeoff", "altitude_m": -5}]}},
        module_file=__file__,
    )
    assert record["valid"] is False
    assert "altitude_m" in record["error"]


def _run_all():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    failures = 0
    for test in tests:
        try:
            test()
            print(f"PASS {test.__name__}")
        except Exception as exc:  # noqa: BLE001
            failures += 1
            print(f"FAIL {test.__name__}: {exc}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_run_all())
