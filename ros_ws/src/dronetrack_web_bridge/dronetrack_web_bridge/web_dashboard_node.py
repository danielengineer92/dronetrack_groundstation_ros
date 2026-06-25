"""Web dashboard / web bridge (runs ON THE LAPTOP ground station).

Adapted from dronetrack_pi_ros/src/drone_dashboard. Same philosophy: Python
stdlib HTTP server only (no Flask/Node), serving an HTML page plus a JSON status
endpoint. Differences for the split architecture:

  - Runs on the laptop, not the Pi.
  - Surfaces the GROUND-STATION-relevant signals the user asked for:
    connection state, perception FPS, estimated latency, detection confidence,
    and drone status (from telemetry).
  - Operator buttons publish only *request* topics that the Pi re-validates. The
    dashboard never publishes control, enable, or arming topics.

Endpoints:
    GET  /                 dashboard HTML
    GET  /api/status       latest status JSON
    GET  /stream.mjpg      live camera stream with YOLO detection overlays
    POST /api/mission_request   {"enabled": true|false}
    POST /api/autonomy_request  {"enabled": true|false}
    POST /api/abort_hold        {"confirm": true}
    POST /api/land              {"confirm": true}
"""

from __future__ import annotations

import json
import math
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    import cv2
    import numpy as np
except ImportError:  # The raw MJPEG stream still works without overlay support.
    cv2 = None
    np = None

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from drone_interfaces.msg import DetectionArray, DroneTelemetry, MavsdkActionCommand
from dronetrack_msgs.msg import GroundStationHeartbeat, LinkStatus


DASHBOARD_HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>DroneTrack Ground Station</title>
<style>
  :root { color-scheme: dark; }
  body { margin:0; font-family: system-ui, sans-serif; background:#06101e; color:#e7f4ff; }
  .wrap { max-width: 1240px; margin: 0 auto; padding: 20px; }
  h1 { font-size: 22px; margin:0; }
  .grid { display:grid; grid-template-columns: repeat(auto-fit,minmax(200px,1fr)); gap:14px; }
  .card { background:#0a1528; border:1px solid rgba(100,160,220,.18); border-radius:14px; padding:14px; }
  .camera { margin-top:14px; background:#071326; border:1px solid rgba(100,160,220,.2); border-radius:14px; overflow:hidden; }
  .camera-head { display:flex; justify-content:space-between; gap:12px; align-items:flex-end; padding:14px; }
  .camera-title { font-size:20px; font-weight:800; margin-top:3px; }
  .frame { background:#02060c; aspect-ratio:16/9; display:grid; place-items:center; }
  .frame img { width:100%; height:100%; object-fit:contain; display:block; }
  .target-row { display:flex; justify-content:space-between; gap:10px; flex-wrap:wrap; padding:10px 14px 14px; border-top:1px solid rgba(100,160,220,.12); }
  .target-list { color:#cfe6ff; }
  .stale { color:#ffcc66; }
  .k { color:#83a3bd; font-size:11px; text-transform:uppercase; letter-spacing:.12em; }
  .v { font-size:24px; font-weight:800; margin-top:4px; }
  .ok { color:#50fa7b; } .warn { color:#ffcc66; } .bad { color:#ff5f75; }
  .row { display:flex; gap:10px; flex-wrap:wrap; margin-top:16px; }
  button { font:inherit; font-weight:700; border:0; border-radius:10px; padding:12px 16px; cursor:pointer; }
  .start { background:#1f6feb; color:#fff; } .ready { background:#236; color:#cfe; }
  .hold { background:#7a5; color:#012; } .land { background:#ff5f75; color:#210; }
  small { color:#83a3bd; }
</style></head>
<body><div class="wrap">
  <h1>DroneTrack &mdash; Ground Station</h1>
  <small id="updated">connecting...</small>
  <section class="camera">
    <div class="camera-head">
      <div><div class="k">Camera</div><div class="camera-title">YOLO Targets</div></div>
      <small id="stream_state">waiting for image...</small>
    </div>
    <div class="frame"><img id="stream" src="/stream.mjpg" alt="Live camera feed with YOLO target boxes"/></div>
    <div class="target-row"><small class="target-list" id="targets">No targets</small><small id="frame_age"></small></div>
  </section>
  <div class="grid" style="margin-top:14px">
    <div class="card"><div class="k">Link</div><div class="v" id="link">--</div><small id="link_reason"></small></div>
    <div class="card"><div class="k">Latency</div><div class="v" id="latency">--</div></div>
    <div class="card"><div class="k">Perception FPS</div><div class="v" id="fps">--</div></div>
    <div class="card"><div class="k">Detections</div><div class="v" id="det">--</div><small id="conf"></small></div>
    <div class="card"><div class="k">Drone Link</div><div class="v" id="px4">--</div><small id="mode"></small></div>
    <div class="card"><div class="k">Battery</div><div class="v" id="batt">--</div></div>
    <div class="card"><div class="k">Rel. Altitude</div><div class="v" id="alt">--</div></div>
    <div class="card"><div class="k">Armed</div><div class="v" id="armed">--</div></div>
  </div>
  <div class="row">
    <button class="ready" onclick="post('autonomy_request',{enabled:true})">System Ready</button>
    <button class="start" onclick="post('mission_request',{enabled:true})">Start Mission</button>
    <button class="hold" onclick="post('abort_hold',{confirm:true})">Abort / Hold</button>
    <button class="land" onclick="post('land',{confirm:true})">Land</button>
  </div>
  <p><small>Operator buttons publish request topics only. The Pi re-validates and
  may ignore them. This page cannot arm, send control, or bypass Pi safety gates.</small></p>
</div>
<script>
function cls(el,c){el.className='v '+c;}
async function tick(){
  try{
    const r = await fetch('/api/status'); const s = await r.json();
    const link = document.getElementById('link');
    link.textContent = s.link_ok ? 'UP' : 'DOWN'; cls(link, s.link_ok?'ok':'bad');
    document.getElementById('link_reason').textContent = s.link_reason || '';
    document.getElementById('latency').textContent =
      (s.latency_s>=0 && s.latency_s===s.latency_s) ? (s.latency_s*1000).toFixed(0)+' ms' : '--';
    document.getElementById('fps').textContent = s.perception_fps>=0 ? s.perception_fps.toFixed(1) : '--';
    document.getElementById('det').textContent = s.detection_count;
    document.getElementById('conf').textContent = s.max_confidence>0 ? ('max conf '+s.max_confidence.toFixed(2)) : '';
    document.getElementById('targets').textContent = s.target_summary || 'No targets';
    const hasFrame = s.camera_frame_age_s >= 0;
    document.getElementById('stream_state').textContent = hasFrame
      ? (s.overlay_available ? 'overlay live' : 'camera live') : 'waiting for image...';
    const frameAge = document.getElementById('frame_age');
    frameAge.textContent = hasFrame ? ('frame age '+s.camera_frame_age_s.toFixed(1)+' s') : '';
    frameAge.className = s.camera_frame_age_s > 1.5 ? 'stale' : '';
    const px4 = document.getElementById('px4');
    px4.textContent = s.px4_connected ? 'CONNECTED' : 'NO LINK'; cls(px4, s.px4_connected?'ok':'bad');
    document.getElementById('mode').textContent = s.flight_mode || '';
    document.getElementById('batt').textContent = s.battery_percent>=0 ? s.battery_percent.toFixed(0)+'%' : '--';
    document.getElementById('alt').textContent = s.rel_altitude_m.toFixed(1)+' m';
    const armed = document.getElementById('armed');
    armed.textContent = s.armed ? 'ARMED' : 'disarmed'; cls(armed, s.armed?'warn':'ok');
    document.getElementById('updated').textContent = 'updated '+new Date().toLocaleTimeString();
  }catch(e){ document.getElementById('updated').textContent='status fetch failed'; }
}
async function post(path, body){
  await fetch('/api/'+path, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  tick();
}
setInterval(tick, 500); tick();
</script>
</body></html>"""


class _DualStackServer(ThreadingHTTPServer):
    """Threaded HTTP server that accepts both IPv4 and IPv6 connections."""

    address_family = socket.AF_INET6

    def server_bind(self):
        # Clear IPV6_V6ONLY so the ::-bound socket also serves IPv4 (127.0.0.1).
        try:
            self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        except (AttributeError, OSError):
            pass
        super().server_bind()


class WebDashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("web_dashboard_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("link_status_topic", "/drone/groundstation/link_status")
        self.declare_parameter("heartbeat_topic", "/groundstation/heartbeat")
        self.declare_parameter("detections_topic", "/groundstation/vision/detections")
        self.declare_parameter("camera_topic", "/drone/camera/image_raw/compressed")
        self.declare_parameter("telemetry_topic", "/drone/telemetry")
        self.declare_parameter("mission_request_topic", "/drone/mission/request")
        self.declare_parameter("autonomy_request_topic", "/drone/autonomy/request")
        self.declare_parameter("offboard_request_topic", "/drone/mavsdk/offboard_request")
        self.declare_parameter("mavsdk_action_topic", "/drone/mavsdk/action_command")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)

        self._lock = threading.Lock()
        self._state = {
            "link_ok": False, "link_reason": "no data", "latency_s": float("nan"),
            "perception_fps": -1.0, "detection_count": 0, "max_confidence": 0.0,
            "px4_connected": False, "flight_mode": "", "battery_percent": -1.0,
            "rel_altitude_m": 0.0, "armed": False, "camera_frame_age_s": -1.0,
            "target_summary": "", "overlay_available": cv2 is not None and np is not None,
        }
        self._latest_jpeg: bytes | None = None
        self._latest_frame_time = 0.0
        self._latest_detections: list[dict] = []
        self._latest_detection_size = (0, 0)
        self._latest_detection_time = 0.0
        self._action_id = 0

        lossy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                           history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST, depth=5)

        self.create_subscription(LinkStatus, str(self.get_parameter("link_status_topic").value),
                                 self._on_link, reliable)
        self.create_subscription(GroundStationHeartbeat, str(self.get_parameter("heartbeat_topic").value),
                                 self._on_heartbeat, lossy)
        self.create_subscription(DetectionArray, str(self.get_parameter("detections_topic").value),
                                 self._on_detections, lossy)
        self.create_subscription(CompressedImage, str(self.get_parameter("camera_topic").value),
                                 self._on_camera, lossy)
        self.create_subscription(DroneTelemetry, str(self.get_parameter("telemetry_topic").value),
                                 self._on_telemetry, lossy)

        self.mission_pub = self.create_publisher(Bool, str(self.get_parameter("mission_request_topic").value), reliable)
        self.autonomy_pub = self.create_publisher(Bool, str(self.get_parameter("autonomy_request_topic").value), reliable)
        self.offboard_pub = self.create_publisher(Bool, str(self.get_parameter("offboard_request_topic").value), reliable)
        self.action_pub = self.create_publisher(MavsdkActionCommand, str(self.get_parameter("mavsdk_action_topic").value), reliable)

        self._start_server()
        self.get_logger().info(f"Dashboard serving on http://{self.host}:{self.port}/")

    # ---- subscriptions ---------------------------------------------------
    def _on_link(self, msg: LinkStatus) -> None:
        with self._lock:
            self._state["link_ok"] = bool(msg.link_ok)
            self._state["link_reason"] = msg.reason
            self._state["latency_s"] = float(msg.estimated_latency_s)

    def _on_heartbeat(self, msg: GroundStationHeartbeat) -> None:
        with self._lock:
            self._state["perception_fps"] = float(msg.perception_fps)

    def _on_detections(self, msg: DetectionArray) -> None:
        max_conf = max((d.confidence for d in msg.detections), default=0.0)
        detections = [
            {
                "class_name": d.class_name,
                "confidence": float(d.confidence),
                "pixel_center_x": int(d.pixel_center_x),
                "pixel_center_y": int(d.pixel_center_y),
                "pixel_width": int(d.pixel_width),
                "pixel_height": int(d.pixel_height),
            }
            for d in msg.detections
        ]
        target_summary = ", ".join(
            f"{d['class_name']} {d['confidence']:.2f}" for d in detections[:3]
        )
        if len(detections) > 3:
            target_summary += f", +{len(detections) - 3} more"
        with self._lock:
            self._state["detection_count"] = int(msg.count)
            self._state["max_confidence"] = float(max_conf)
            self._state["target_summary"] = target_summary
            self._latest_detections = detections
            self._latest_detection_size = (int(msg.image_width), int(msg.image_height))
            self._latest_detection_time = time.monotonic()

    def _on_camera(self, msg: CompressedImage) -> None:
        with self._lock:
            self._latest_jpeg = bytes(msg.data)
            self._latest_frame_time = time.monotonic()

    def _on_telemetry(self, msg: DroneTelemetry) -> None:
        with self._lock:
            self._state["px4_connected"] = bool(msg.connected)
            self._state["flight_mode"] = msg.flight_mode
            self._state["battery_percent"] = float(msg.battery_remaining_percent)
            self._state["rel_altitude_m"] = float(msg.relative_altitude)
            self._state["armed"] = bool(msg.armed)

    # ---- operator actions ------------------------------------------------
    def publish_mission_request(self, enabled: bool) -> None:
        self.mission_pub.publish(Bool(data=enabled))

    def publish_autonomy_request(self, enabled: bool) -> None:
        self.autonomy_pub.publish(Bool(data=enabled))

    def _send_action(self, action: str, note: str) -> None:
        # Called from HTTP handler threads; guard the counter so concurrent
        # button presses can't collide on a command_id.
        with self._lock:
            self._action_id += 1
            action_id = self._action_id
        cmd = MavsdkActionCommand()
        cmd.stamp = self.get_clock().now().to_msg()
        cmd.command_id = action_id
        cmd.action = action
        cmd.execute = True
        cmd.note = note
        self.action_pub.publish(cmd)

    def abort_hold(self) -> None:
        # De-assert requests, then ask the Pi-owned action gate for HOLD.
        self.autonomy_pub.publish(Bool(data=False))
        self.offboard_pub.publish(Bool(data=False))
        self.mission_pub.publish(Bool(data=False))
        self._send_action("HOLD", "dashboard abort/hold")

    def land(self) -> None:
        self.autonomy_pub.publish(Bool(data=False))
        self.offboard_pub.publish(Bool(data=False))
        self.mission_pub.publish(Bool(data=False))
        self._send_action("LAND", "dashboard land")

    # ---- http server -----------------------------------------------------
    def snapshot(self) -> dict:
        with self._lock:
            snap = dict(self._state)
            if self._latest_frame_time > 0.0:
                snap["camera_frame_age_s"] = time.monotonic() - self._latest_frame_time
        # JSON has no NaN/Infinity literals, and the browser's JSON.parse rejects
        # them — which would make every /api/status fetch throw (latency_s is NaN
        # until the first LinkStatus arrives from the Pi). Emit null instead; the
        # dashboard JS already renders a missing value as '--'.
        return {
            k: (None if isinstance(v, float) and not math.isfinite(v) else v)
            for k, v in snap.items()
        }

    def _latest_stream_frame(self) -> bytes | None:
        now = time.monotonic()
        with self._lock:
            jpeg = self._latest_jpeg
            detections = list(self._latest_detections)
            detection_size = self._latest_detection_size
            detections_fresh = self._latest_detection_time > 0.0 and now - self._latest_detection_time <= 1.0
        if jpeg is None:
            return None
        if cv2 is None or np is None or not detections or not detections_fresh:
            return jpeg
        return self._draw_overlay(jpeg, detections, detection_size)

    def _draw_overlay(self, jpeg: bytes, detections: list[dict], detection_size: tuple[int, int]) -> bytes:
        frame = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return jpeg

        frame_h, frame_w = frame.shape[:2]
        det_w, det_h = detection_size
        scale_x = frame_w / det_w if det_w > 0 else 1.0
        scale_y = frame_h / det_h if det_h > 0 else 1.0

        for det in detections:
            cx = det["pixel_center_x"] * scale_x
            cy = det["pixel_center_y"] * scale_y
            bw = det["pixel_width"] * scale_x
            bh = det["pixel_height"] * scale_y
            x1 = max(0, min(frame_w - 1, int(cx - bw / 2)))
            y1 = max(0, min(frame_h - 1, int(cy - bh / 2)))
            x2 = max(0, min(frame_w - 1, int(cx + bw / 2)))
            y2 = max(0, min(frame_h - 1, int(cy + bh / 2)))
            if x2 <= x1 or y2 <= y1:
                continue

            color = (80, 255, 120)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{det['class_name']} {det['confidence']:.2f}"
            (label_w, label_h), baseline = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            label_y = max(label_h + baseline + 4, y1)
            cv2.rectangle(
                frame,
                (x1, label_y - label_h - baseline - 6),
                (min(frame_w - 1, x1 + label_w + 8), label_y + 2),
                color,
                -1,
            )
            cv2.putText(
                frame,
                label,
                (x1 + 4, label_y - baseline - 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (1, 6, 12),
                2,
                cv2.LINE_AA,
            )

        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 82])
        return encoded.tobytes() if ok else jpeg

    def stream_mjpeg(self, handler: BaseHTTPRequestHandler) -> None:
        handler.send_response(200)
        handler.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        handler.send_header("Pragma", "no-cache")
        handler.send_header("Connection", "close")
        handler.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        handler.end_headers()

        while True:
            frame = self._latest_stream_frame()
            if frame is None:
                time.sleep(0.1)
                continue
            try:
                handler.wfile.write(b"--frame\r\n")
                handler.wfile.write(b"Content-Type: image/jpeg\r\n")
                handler.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii"))
                handler.wfile.write(frame)
                handler.wfile.write(b"\r\n")
                handler.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                return
            time.sleep(1.0 / 12.0)

    def _start_server(self) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):  # silence default logging
                pass

            def _send(self, code, body, ctype="application/json"):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._send(200, DASHBOARD_HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif self.path == "/stream.mjpg":
                    node.stream_mjpeg(self)
                elif self.path == "/api/status":
                    self._send(200, json.dumps(node.snapshot()).encode("utf-8"))
                else:
                    self._send(404, b'{"error":"not found"}')

            def do_POST(self):
                length = int(self.headers.get("Content-Length", 0) or 0)
                raw = self.rfile.read(length) if length else b"{}"
                try:
                    payload = json.loads(raw or b"{}")
                except json.JSONDecodeError:
                    self._send(400, b'{"error":"bad json"}')
                    return
                try:
                    if self.path == "/api/mission_request":
                        node.publish_mission_request(bool(payload.get("enabled", False)))
                    elif self.path == "/api/autonomy_request":
                        node.publish_autonomy_request(bool(payload.get("enabled", False)))
                    elif self.path == "/api/abort_hold":
                        if payload.get("confirm"):
                            node.abort_hold()
                    elif self.path == "/api/land":
                        if payload.get("confirm"):
                            node.land()
                    else:
                        self._send(404, b'{"error":"not found"}')
                        return
                    self._send(200, b'{"ok":true}')
                except Exception as exc:  # noqa: BLE001
                    self._send(500, json.dumps({"error": str(exc)}).encode("utf-8"))

        # Bind dual-stack (IPv4+IPv6) when listening on all interfaces, so the
        # page is reachable as both http://127.0.0.1 and http://localhost. Under
        # WSL2 mirrored networking, Windows resolves "localhost" to IPv6 ::1
        # first; an IPv4-only listener makes "localhost" time out. Falls back to
        # plain IPv4 if dual-stack binding is unavailable.
        if self.host in ("0.0.0.0", "::", ""):
            try:
                self._httpd = _DualStackServer(("::", self.port), Handler)
            except OSError:
                self._httpd = ThreadingHTTPServer(("0.0.0.0", self.port), Handler)
        else:
            self._httpd = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()

    def destroy_node(self) -> None:
        try:
            self._httpd.shutdown()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WebDashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
