# Mission Step Schema Reference

> **Source:** `mission_plan_model.py::STEP_SCHEMA` — the single canonical definition
> served by `GET /api/mission-step-schema` (handler at `web_dashboard_node.py:602`).
> The Pi-side executor model (`drone_control/mission_plan.py`) recognises one
> additional verb (`complete`) not exposed by the dashboard schema.

## Endpoint

```
GET /api/mission-step-schema
```

### Response shape

```json
{
  "verbs": {
    "<verb_name>": {
      "label": "<human-readable label>",
      "description": "<one-line description>",
      "params": {
        "<param_name>": {
          "type": "float|enum|str",
          "default": <default_value>,
          "label": "<param label>",
          "min": <number>,       // float only
          "max": <number>,       // float only (optional)
          "step": <number>,      // float only
          "options": ["<v1>", "<v2>"]  // enum only
        }
      },
      "category": "action|preflight|motion"
    }
  }
}
```

## Verb Taxonomy

| # | Verb | Category | Badge Color | Description |
|---|------|----------|-------------|-------------|
| 1 | `takeoff` | `action` | green | Command takeoff if not already airborne |
| 2 | `prime_offboard` | `preflight` | blue | Enable offboard mode and hold for a stabilisation period |
| 3 | `scan` | `motion` | orange | Yaw sweep to find the target in place |
| 4 | `track_center` | `motion` | orange | Maintain yaw-on-target and fly toward the detected object |
| 5 | `approach` | `motion` | orange | Fly to within the specified distance of the target |
| 6 | `orbit` | `motion` | orange | Orbit the target at a fixed radius and speed |
| 7 | `rtl` | `action` | green | Command RTL (land at home position) |
| 8 | `land` | `action` | green | Command immediate landing |
| 9 | `hold` | `preflight` | blue | Hold position (offboard hover) |

**Color mapping** (from `CATEGORY_COLORS` at `mission_plan_model.py:162`):
- `preflight` → `"blue"`
- `action` → `"green"`
- `motion` → `"orange"`

## Per-Verb Parameter Definitions

### 1. `takeoff` — action (green)

| Param | Type | Default | Constraints | Label |
|-------|------|---------|-------------|-------|
| `altitude_m` | **float** | `3.0` | min `0.5`, step `0.5` | Altitude (m) |

### 2. `prime_offboard` — preflight (blue)

| Param | Type | Default | Constraints | Label |
|-------|------|---------|-------------|-------|
| `hold_s` | **float** | `1.5` | min `0.0`, step `0.5` | Hold Time (s) |

### 3. `scan` — motion (orange)

| Param | Type | Default | Constraints | Label |
|-------|------|---------|-------------|-------|
| `direction` | **enum** | `"ccw"` | options: `["ccw", "cw"]` | Sweep Direction |
| `yaw_deg` | **float** | `180.0` | min `5.0`, max `360.0`, step `15.0` | Sweep Angle (deg) |
| `yaw_rate_deg_s` | **float** | `20.0` | min `1.0`, max `90.0`, step `5.0` | Sweep Rate (deg/s) |
| `until` | **enum** | `"locked"` | options: `["locked", "none"]` | Exit Condition |
| `timeout_s` | **float** | `12.0` | min `0.0`, step `1.0` | Timeout (s) |

### 4. `track_center` — motion (orange)

| Param | Type | Default | Constraints | Label |
|-------|------|---------|-------------|-------|
| `distance_m` | **float** | `3.0` | min `0.5`, step `0.5` | Target Distance (m) |
| `until` | **enum** | `"centered"` | options: `["centered", "approach_done", "none"]` | Exit Condition |
| `timeout_s` | **float** | `15.0` | min `0.0`, step `1.0` | Timeout (s) |

### 5. `approach` — motion (orange)

| Param | Type | Default | Constraints | Label |
|-------|------|---------|-------------|-------|
| `distance_m` | **float** | `2.0` | min `0.3`, step `0.5` | Approach Distance (m) |
| `timeout_s` | **float** | `20.0` | min `0.0`, step `1.0` | Timeout (s) |

> **Note:** The Pi-side model (`mission_plan.py`) supports an `until` field on
> `approach` (valid values: `airborne`, `centered`, `locked`, `approach_done`,
> `none`), and `docs/missions.md` documents it as a recognised key. The web bridge
> schema currently omits `until` from approach; if the UI needs it, add it to
> `STEP_SCHEMA["approach"]["params"]` in `mission_plan_model.py`.

### 6. `orbit` — motion (orange)

| Param | Type | Default | Constraints | Label |
|-------|------|---------|-------------|-------|
| `radius_m` | **float** | `2.0` | min `0.5`, step `0.5` | Orbit Radius (m) |
| `speed_m_s` | **float** | `0.4` | min `0.1`, step `0.1` | Orbit Speed (m/s) |
| `revolutions` | **float** | `1.0` | min `0.25`, step `0.25` | Revolutions |
| `timeout_s` | **float** | `45.0` | min `0.0`, step `1.0` | Timeout (s) |

### 7. `rtl` — action (green)

| Param | Type | Default | Constraints | Label |
|-------|------|---------|-------------|-------|
| `timeout_s` | **float** | `15.0` | min `0.0`, step `1.0` | Timeout (s) |

### 8. `land` — action (green)

| Param | Type | Default | Constraints | Label |
|-------|------|---------|-------------|-------|
| `timeout_s` | **float** | `10.0` | min `0.0`, step `1.0` | Timeout (s) |

### 9. `hold` — preflight (blue)

| Param | Type | Default | Constraints | Label |
|-------|------|---------|-------------|-------|
| `status` | **str** | `"holding position"` | — | Status |
| `timeout_s` | **float** | `0.0` | min `0.0`, step `1.0` | Timeout (s, 0=forever) |

## `until` Enum Values

The full set of `until` predicates recognised by the Pi-side executor
(`drone_control/mission_plan.py:55-57`):

| Value | Meaning |
|-------|---------|
| `airborne` | Advance once airborne (takeoff complete) |
| `centered` | Advance when the target is yaw-centered |
| `locked` | Advance when the target is locked by the tracker |
| `approach_done` | Advance when within approach-distance tolerance |
| `none` | Never auto-advance on a condition (exit only on `timeout_s`) |

## `direction` Enum Values (scan only)

| Value | Meaning |
|-------|---------|
| `ccw` | Counter-clockwise yaw sweep (default) |
| `cw` | Clockwise yaw sweep |

## Field Type Quick Reference

- **float** — all numeric parameters. Always have `default`, `min`, and `step`.
  May optionally have `max`.
- **enum** — string values restricted to a fixed `options` list. `direction`,
  `until`.
- **str** — free-form string. Only `hold.status`.

## Validation Rules (from `validate_step`)

1. `type` must be one of the 9 recognised verbs.
2. All schema-defined `params` keys must be present (not `None`).
3. **float** params must be numeric (int or float), finite, and within `[min, max]`
   (if `max` is present).
4. **enum** and **str** params must be strings.
5. **enum** params must be one of the values in `options`.

## Lint Warnings (from `lint_steps`)

1. A motion step (`scan`/`track_center`/`approach`/`orbit`) before any
   `prime_offboard` step.
2. A plan with motion steps but no `prime_offboard` at all.
3. `scan`/`approach`/`orbit` with no `timeout_s`.
4. A `scan` with neither `until` nor `timeout_s` (would sweep forever).
5. An empty plan (no steps).

## Pi-Side `complete` Verb

The Pi-side executor model (`drone_control/mission_plan.py:47`) recognises
`"complete"` as a valid step type that maps to the `COMPLETE` mission state. This
verb is **not** in the dashboard's `STEP_SCHEMA` (it is a terminal state the
executor auto-appends, not a user-selectable step). The dashboard preview parser
(`mission_preview.py:16-27`) accepts it as a valid type for display purposes.
