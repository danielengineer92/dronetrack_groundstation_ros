"""Pure tests for dashboard mission-plan preview helpers."""

from __future__ import annotations

import os
import sys
import tempfile
from pathlib import Path

try:
    from dronetrack_web_bridge.mission_preview import load_mission_catalog, preview_mission_file
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from dronetrack_web_bridge.mission_preview import load_mission_catalog, preview_mission_file  # noqa: E402


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
