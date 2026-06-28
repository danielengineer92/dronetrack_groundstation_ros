"""Unit tests for mission_plan_model (round-trip YAML, validation, linting)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "dronetrack_web_bridge"))

from mission_plan_model import (
    get_step_schema,
    create_default_step,
    validate_step,
    steps_to_yaml,
    lint_steps,
    sanitize_filename,
    STEP_SCHEMA,
    CATEGORY_COLORS,
)


def _plan_from_yaml(yaml_str: str) -> tuple[str, list[dict]]:
    import yaml
    data = yaml.safe_load(yaml_str)
    mission = data["mission"]
    steps = []
    for raw in mission["steps"]:
        tp = raw["type"]
        params = {k: v for k, v in raw.items() if k != "type"}
        steps.append({"type": tp, "params": params})
    return mission["name"], steps


class TestGetStepSchema:
    def test_returns_all_verbs(self):
        schema = get_step_schema()
        assert "takeoff" in schema
        assert "scan" in schema
        assert "approach" in schema
        assert "orbit" in schema
        assert "land" in schema
        assert "goto_relative" in schema
        assert "goto_absolute" in schema
        assert len(schema) >= 11

    def test_every_verb_has_label_description_category_params(self):
        for verb, spec in STEP_SCHEMA.items():
            assert isinstance(spec["label"], str)
            assert isinstance(spec["description"], str)
            assert spec["category"] in CATEGORY_COLORS
            assert isinstance(spec["params"], dict)


class TestCreateDefaultStep:
    def test_takeoff_defaults(self):
        step = create_default_step("takeoff")
        assert step["type"] == "takeoff"
        assert step["params"]["altitude_m"] == 3.0

    def test_scan_defaults(self):
        step = create_default_step("scan")
        assert step["params"]["direction"] == "ccw"
        assert step["params"]["yaw_deg"] == 180.0
        assert step["params"]["until"] == "locked"
        assert step["params"]["timeout_s"] == 12.0

    def test_unknown_verb_raises(self):
        try:
            create_default_step("not_a_verb")
            assert False, "should have raised"
        except ValueError:
            pass


class TestValidateStep:
    def test_valid_takeoff(self):
        step = create_default_step("takeoff")
        assert validate_step(step) == []

    def test_valid_scan(self):
        step = create_default_step("scan")
        assert validate_step(step) == []

    def test_missing_type(self):
        assert len(validate_step({"params": {}})) > 0

    def test_unknown_type(self):
        assert len(validate_step({"type": "bogus", "params": {}})) > 0

    def test_bad_altitude(self):
        step = {"type": "takeoff", "params": {"altitude_m": -1.0}}
        errors = validate_step(step)
        assert any("below minimum" in e for e in errors)

    def test_bad_scan_direction(self):
        step = {"type": "scan", "params": {
            "direction": "left", "yaw_deg": 180, "yaw_rate_deg_s": 20,
            "until": "locked", "timeout_s": 12,
        }}
        errors = validate_step(step)
        assert any("not in options" in e for e in errors)

    def test_str_type_params(self):
        step = create_default_step("hold")
        assert validate_step(step) == []


class TestStepsToYaml:
    def test_round_trip_basic_plan(self):
        name = "test_mission"
        steps = [
            create_default_step("takeoff"),
            create_default_step("prime_offboard"),
            create_default_step("scan"),
            create_default_step("track_center"),
            create_default_step("approach"),
            create_default_step("orbit"),
            create_default_step("rtl"),
            create_default_step("land"),
        ]
        yaml_str = steps_to_yaml(name, steps)
        parsed_name, parsed_steps = _plan_from_yaml(yaml_str)
        assert parsed_name == "test_mission"
        assert len(parsed_steps) == 8
        assert parsed_steps[0]["type"] == "takeoff"

    def test_custom_params_preserved(self):
        steps = [{"type": "takeoff", "params": {"altitude_m": 7.5}}]
        yaml_str = steps_to_yaml("custom", steps)
        parsed_name, parsed_steps = _plan_from_yaml(yaml_str)
        assert parsed_steps[0]["params"]["altitude_m"] == 7.5

    def test_scan_cw_preserved(self):
        step = create_default_step("scan")
        step["params"]["direction"] = "cw"
        yaml_str = steps_to_yaml("cw_scan", [step])
        _, parsed = _plan_from_yaml(yaml_str)
        assert parsed[0]["params"]["direction"] == "cw"

    def test_unknown_step_type_raises(self):
        try:
            steps_to_yaml("bad", [{"type": "nope", "params": {}}])
            assert False, "should have raised"
        except ValueError:
            pass


class TestLintSteps:
    def test_clean_plan(self):
        steps = [
            create_default_step("takeoff"),
            create_default_step("prime_offboard"),
            create_default_step("scan"),
        ]
        warnings = lint_steps(steps)
        assert all("motion step before prime" not in w for w in warnings)

    def test_motion_before_prime(self):
        steps = [
            create_default_step("takeoff"),
            create_default_step("scan"),
        ]
        warnings = lint_steps(steps)
        assert any("before prime_offboard" in w for w in warnings)

    def test_scan_without_exit(self):
        step = create_default_step("scan")
        step["params"]["until"] = "none"
        step["params"]["timeout_s"] = None
        warnings = lint_steps([step])
        assert any("until" in w or "indefinitely" in w for w in warnings)

    def test_empty_plan(self):
        warnings = lint_steps([])
        assert any("empty" in w.lower() for w in warnings)

    def test_motion_without_prime_warns(self):
        steps = [
            create_default_step("takeoff"),
            create_default_step("scan"),
            create_default_step("orbit"),
        ]
        warnings = lint_steps(steps)
        assert any("no prime_offboard" in w for w in warnings)

    def test_goto_relative_is_motion(self):
        steps = [
            create_default_step("takeoff"),
            create_default_step("goto_relative"),
        ]
        warnings = lint_steps(steps)
        assert any("before prime_offboard" in w for w in warnings)


class TestSanitizeFilename:
    def test_spaces_to_underscores(self):
        assert sanitize_filename("My Mission Plan") == "my_mission_plan"

    def test_special_chars_removed(self):
        assert sanitize_filename("test!@#plan") == "test_plan"

    def test_empty_yields_default(self):
        assert sanitize_filename("") == "untitled_mission"

    def test_leading_trailing_underscores_removed(self):
        assert sanitize_filename("  hello world  ") == "hello_world"


class TestSchemaCompleteness:
    def test_all_categories_have_colors(self):
        for verb, spec in STEP_SCHEMA.items():
            assert spec["category"] in CATEGORY_COLORS, f"{verb} category '{spec['category']}' missing color"

    def test_motion_verbs_have_timeout(self):
        motion_verbs = {"scan", "track_center", "approach", "orbit", "goto_relative", "goto_absolute"}
        for verb in motion_verbs:
            assert verb in STEP_SCHEMA
            params = STEP_SCHEMA[verb]["params"]
            if verb not in ("track_center",):
                assert "timeout_s" in params, f"{verb} missing timeout_s"

    def test_enum_params_have_options(self):
        for verb, spec in STEP_SCHEMA.items():
            for pname, pspec in spec["params"].items():
                if pspec["type"] == "enum":
                    assert "options" in pspec, f"{verb}.{pname} missing options"
                    assert len(pspec["options"]) >= 1
