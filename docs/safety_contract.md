# Safety Contract

This is the binding agreement between the laptop ground station and the Pi. It
defines exactly what the laptop may publish, what it may never publish, and how
the Pi validates everything it receives.

## Principle

> The laptop is an **untrusted perception + advisory** peer. The Pi treats every
> laptop message as suspect, validates it, and remains safe — holding position —
> if the laptop sends nothing, stale data, garbage, or disappears entirely.

Nothing the laptop does can arm the vehicle, send a control setpoint, open
Offboard, or bypass a Pi safety gate.

## What the laptop is ALLOWED to publish

| Topic | Type | Meaning | How the Pi treats it |
|---|---|---|---|
| `/groundstation/vision/detections` | `drone_interfaces/DetectionArray` | YOLO perception output | Validated by `detection_gate_node`, then republished to the internal `/drone/vision/detections`. Never trusted raw. |
| `/groundstation/heartbeat` | `dronetrack_msgs/GroundStationHeartbeat` | Link liveness beacon | Drives `ground_station_watchdog_node`. Frozen/replayed beats (non-advancing `sequence`) are ignored. |
| `/drone/mission/request` | `std_msgs/Bool` | Operator: start/stop mission | Advisory request. `mission_executor_node` still runs all preflight/arm/altitude gates. |
| `/drone/autonomy/request` | `std_msgs/Bool` | Operator: ask for autonomy | Advisory. `autonomy_manager_node` decides; only it asserts `enabled`. |
| `/drone/mavsdk/offboard_request` | `std_msgs/Bool` | Operator: ask for Offboard | Advisory. Manager decides `offboard_enable`. |
| `/drone/mavsdk/action_command` | `drone_interfaces/MavsdkActionCommand` | Operator: HOLD/LAND/RTL/TAKEOFF | Passes only if `telemetry_node.allow_mavsdk_actions` is true **and** the action gate's own checks pass. Blocked by default. |

These four request topics are exactly the topics an operator could already drive
from a terminal in `dronetrack_pi_ros`; the dashboard is just a convenient
publisher of the same advisory requests.

## What the laptop must NEVER publish

| Topic | Type | Owner (Pi only) |
|---|---|---|
| `/drone/autonomy/enabled` | `std_msgs/Bool` | `autonomy_manager_node` |
| `/drone/mavsdk/offboard_enable` | `std_msgs/Bool` | `autonomy_manager_node` |
| `/drone/control/command` | `drone_interfaces/ControlCommand` | `control_node` |
| `/drone/telemetry` | `drone_interfaces/DroneTelemetry` | `telemetry_node` |

If a laptop process ever publishes one of these, that is a contract violation and
a bug. (The provided laptop nodes never create these publishers.)

## How the Pi validates detections (`detection_gate_node`)

Every inbound `DetectionArray` must pass, or it is dropped whole:

1. **Rate limit** — no more than `max_message_rate_hz` accepted (flood/DoS cap).
2. **Link alive** — a fresh heartbeat within `max_heartbeat_age_s` must exist
   (`require_heartbeat`), else all detections are dropped.
3. **Freshness** — `now - stamp <= max_detection_age_s`; older frames dropped.
4. **No future stamps** — `stamp` more than `clock_skew_tolerance_s` in the
   future is rejected (bad clock or spoofed frame).
5. **Monotonic** — `stamp` must be strictly newer than the last accepted one;
   out-of-order/replayed frames dropped.

Then each individual `Detection` must pass, or that detection is dropped:

6. **Confidence** ≥ `min_confidence`, finite.
7. **Normalized geometry** — `center_x/y`, `width`, `height` finite and in `[0,1]`.
8. **Pixel bounds** — `pixel_center_x/y` within the reported image size.
9. **Count cap** — at most `max_detections` kept.

The gate can only **reduce or pass through** perception. It never invents a
target and never touches control/MAVSDK. Defaults are in `configs/pi.yaml`.

## Link-loss reaction (`ground_station_watchdog_node`)

On the UP→DOWN edge (heartbeat older than `max_heartbeat_age_s`, or never seen):

- Publishes `/drone/autonomy/request = false` and
  `/drone/mavsdk/offboard_request = false` (`deassert_on_link_loss`, default on).
  De-asserting a request can only make the Pi more conservative.
- Optionally publishes a `HOLD` `MavsdkActionCommand`
  (`request_hold_on_link_loss`, default **off**; needs `allow_mavsdk_actions`).

It also publishes `/drone/groundstation/link_status` and `/drone/groundstation/ok`
continuously for observability. The reaction fires **once per edge**, so an
operator can deliberately recover once the link returns.

Independent of all of the above, the existing control stack already fails safe:
stale `TargetError` ⇒ `control_node` holds the local-NED anchor and stops yawing.
The watchdog is defense-in-depth, not the sole safety mechanism.

## Staleness budget (defaults)

| Signal | Param | Default | Rationale |
|---|---|---|---|
| Heartbeat rate | `groundstation_heartbeat_node.rate_hz` | 5 Hz | 5× margin over the 1 s timeout |
| Heartbeat timeout | `*.max_heartbeat_age_s` | 1.0 s | tolerate brief Wi-Fi gaps, react within ~1 s |
| Detection age | `detection_gate_node.max_detection_age_s` | 0.5 s | stale vision must not steer yaw |
| Watchdog tick | `ground_station_watchdog_node.check_rate_hz` | 10 Hz | sub-100 ms detection of the edge |

Tighten these for closer/faster flight; loosen them on a poor link, but never set
the heartbeat timeout longer than you are willing to keep yawing on old data.
