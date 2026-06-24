# Architecture — Split Ground-Station DroneTrack

This repo derives a **two-machine** architecture from `dronetrack_pi_ros`. The
single design rule:

> **All safety-critical behavior stays on the Pi.** Arming, failsafes, flight
> control, the MAVSDK/PX4 link, mission sequencing, watchdogs, and emergency
> stop never move to the laptop. The laptop only contributes perception and
> *advisory* operator requests, and the Pi remains safe if the laptop vanishes.

## Where the network boundary sits

In `dronetrack_pi_ros` the whole pipeline ran on the Pi:

```
camera -> yolo -> tracker -> control -> telemetry(MAVSDK) -> PX4
```

YOLO is the only compute-heavy, non-safety stage in that chain, so the boundary
is drawn at **detections**:

```
   PI (drone)                         LAPTOP (ground station)
 ┌───────────────────────────┐      ┌─────────────────────────────┐
 │ camera_node               │      │                             │
 │   /drone/camera/image_raw │      │                             │
 │        │ (compress)       │      │                             │
 │        ▼                  │  Wi-Fi  ┌──────────────┐           │
 │  image_raw/compressed ───────────►│ yolo_node    │           │
 │                           │      │  (perception) │           │
 │                           │◄───────── /groundstation/vision/detections
 │  detection_gate_node      │      │            │                │
 │   validate + republish    │      │  heartbeat_node ──► /groundstation/heartbeat
 │        │                  │      │            │                │
 │        ▼ /drone/vision/detections │  web_dashboard_node        │
 │  tracker_node             │      │   (FPS/latency/link/status) │
 │        ▼                  │      └─────────────────────────────┘
 │  mission/autonomy/control │
 │        ▼                  │   ground_station_watchdog_node
 │  telemetry_node (MAVSDK) ─────► PX4 / Pixhawk
 │  + action gate            │
 └───────────────────────────┘
```

The laptop's YOLO publishes onto an **untrusted** topic
(`/groundstation/vision/detections`). The Pi's **detection gate** validates each
message and republishes the survivors onto the **exact topic the existing
tracker already consumes** (`/drone/vision/detections`). Because the gate reuses
the original topic name, **no existing Pi node changes** — `tracker_node`,
`control_node`, `mission_executor_node`, `autonomy_manager_node`, and
`telemetry_node` are reused verbatim from `dronetrack_pi_ros`.

## Node ownership

| Responsibility | Node | Package | Machine | Source |
|---|---|---|---|---|
| Camera capture | `camera_node` | drone_camera | Pi | reused |
| Compress for Wi-Fi | `camera_compressor_node` | dronetrack_pi | Pi | new wiring |
| **Detection gate / validator** | `detection_gate_node` | dronetrack_pi | Pi | **new** |
| **Ground-station watchdog** | `ground_station_watchdog_node` | dronetrack_pi | Pi | **new** |
| Target selection / lock | `tracker_node` | drone_tracker | Pi | reused |
| Mission sequencing | `mission_executor_node` | drone_control | Pi | reused |
| Safety gates | `autonomy_manager_node` | drone_control | Pi | reused |
| Low-level control | `control_node` | drone_control | Pi | reused |
| MAVSDK/PX4 bridge + action gate | `telemetry_node` | drone_telemetry | Pi | reused |
| Health monitor | `health_monitor_node` | drone_diagnostics | Pi | reused |
| **YOLO inference** | `yolo_node` | dronetrack_perception | Laptop | **adapted** |
| **Heartbeat beacon** | `groundstation_heartbeat_node` | dronetrack_groundstation | Laptop | **new** |
| **Web dashboard** | `web_dashboard_node` | dronetrack_web_bridge | Laptop | **adapted** |

## Why this is safe when the laptop disconnects

The existing flight behavior is **position-hold + yaw-only target centering**.
Losing the laptop means fresh detections stop arriving. That has two effects,
both safe:

1. The tracker stops getting fresh detections → `TargetError` goes stale →
   `control_node` already holds the captured local-NED anchor and stops yawing.
   The drone **holds position on its own**, exactly as it does today when the
   target leaves frame.
2. The new `ground_station_watchdog_node` detects the missing heartbeat and
   **de-asserts** `/drone/autonomy/request` and `/drone/mavsdk/offboard_request`
   (publishes `false`). This is strictly conservative: de-asserting a *request*
   can only make the system hold/stop, never move. Optionally it can request a
   `HOLD` through the existing Pi-owned MAVSDK action gate.

The watchdog never publishes `/drone/autonomy/enabled`,
`/drone/mavsdk/offboard_enable`, or `ControlCommand` — those remain owned by
`autonomy_manager_node` and `control_node`. See [safety_contract.md](safety_contract.md).

## Heartbeat / staleness strategy (summary)

- **Heartbeat** (`/groundstation/heartbeat`, 5 Hz) is the link-liveness signal.
  It is independent of detections, with a monotonically increasing `sequence`
  so the Pi can spot a frozen/replayed publisher.
- **Detections** carry the original Pi camera-frame stamp. The gate drops any
  detection older than `max_detection_age_s`, out-of-order, or arriving while
  the heartbeat is stale.
- The Pi publishes one authoritative `LinkStatus` (`link_ok`, ages, estimated
  latency, reason) for the dashboard and logs.

Full details and tuning live in [safety_contract.md](safety_contract.md) and the
config files under `configs/`.
