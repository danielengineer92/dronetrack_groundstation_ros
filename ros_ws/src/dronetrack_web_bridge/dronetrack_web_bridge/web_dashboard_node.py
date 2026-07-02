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
    GET  /api/missions     local mission plan previews
    GET  /stream.mjpg      live camera stream with YOLO detection overlays
    POST /api/mission_request       {"enabled": true|false}
    POST /api/autonomy_request      {"enabled": true|false}
    POST /api/abort_hold            {"confirm": true}
    POST /api/land                  {"confirm": true}
    POST /api/mission_plan/validate {"mission": {...}}
    POST /api/mission_plan/save     {"mission": {...}, "filename": "name"}
    POST /api/mission_plan/send     {"mission": {...}} or {"catalog_index": N}

Mission builder trust boundary: /api/mission_plan/send publishes the plan text on
a request topic. The Pi-side mission executor is the authority: it re-validates
the plan with the full parser, refuses uploads while a mission is active, and
answers on an ack topic. Uploading a plan never starts a mission, never arms,
and never bypasses the Pi safety gates.
"""

from __future__ import annotations

import json
import math
import os
import re
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

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

from drone_interfaces.msg import DetectionArray, DroneTelemetry, MavsdkActionCommand, MissionCommand
from dronetrack_msgs.msg import GroundStationHeartbeat, LinkStatus
from dronetrack_web_bridge.mission_preview import (
    discover_mission_paths,
    load_mission_catalog,
    preview_mission_data,
)


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
  .card { background:#0a1528; border:1px solid rgba(100,160,220,.18); border-radius:8px; padding:14px; }
  .camera, .planner { margin-top:14px; background:#071326; border:1px solid rgba(100,160,220,.2); border-radius:8px; overflow:hidden; }
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
  button { font:inherit; font-weight:700; border:0; border-radius:8px; padding:12px 16px; cursor:pointer; }
  .start { background:#1f6feb; color:#fff; } .ready { background:#236; color:#cfe; }
  .hold { background:#7a5; color:#012; } .land { background:#ff5f75; color:#210; }
  select { font:inherit; color:#e7f4ff; background:#09182d; border:1px solid rgba(100,160,220,.28); border-radius:8px; padding:9px 10px; min-width:230px; }
  input { font:inherit; color:#e7f4ff; background:#09182d; border:1px solid rgba(100,160,220,.28); border-radius:8px; padding:9px 10px; }
  .step select, .step input { min-width:0; padding:5px 7px; font-size:13px; }
  .step-params { display:flex; gap:6px; flex-wrap:wrap; margin-top:6px; }
  .step-params label { display:flex; flex-direction:column; gap:2px; color:#83a3bd; font-size:11px; }
  .step-tools { display:flex; gap:4px; margin-top:6px; }
  .step-tools button { padding:4px 9px; font-size:12px; background:#123; color:#cfe6ff; }
  .ack-ok { color:#50fa7b; } .ack-bad { color:#ff5f75; } .ack-wait { color:#ffcc66; }
  .planner-head { display:flex; justify-content:space-between; gap:12px; flex-wrap:wrap; padding:14px; align-items:flex-end; }
  .plan-body { display:grid; grid-template-columns:minmax(220px,320px) 1fr; gap:14px; padding:0 14px 14px; }
  .plan-meta { color:#cfe6ff; word-break:break-word; }
  .steps { display:grid; gap:8px; }
  .step { display:grid; grid-template-columns:34px minmax(0,1fr); gap:10px; align-items:start; padding:9px; background:#09182d; border:1px solid rgba(100,160,220,.14); border-radius:8px; }
  .step-num { color:#83a3bd; font-weight:800; }
  .step-type { font-weight:800; }
  .warnings { color:#ffcc66; margin-top:10px; display:grid; gap:4px; }
  @media (max-width: 760px) { .plan-body { grid-template-columns:1fr; } }
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
    <div class="card"><div class="k">Mission</div><div class="v" id="mission">--</div><small id="mission_detail"></small></div>
    <div class="card"><div class="k">Step</div><div class="v" id="mission_step">--</div><small id="mission_step_detail"></small></div>
    <div class="card"><div class="k">Autonomy</div><div class="v" id="autonomy">--</div><small id="autonomy_detail"></small></div>
  </div>
  <section class="planner">
    <div class="planner-head">
      <div><div class="k">Mission Plans</div><div class="camera-title">Catalog</div></div>
      <div style="display:flex;gap:8px;align-items:flex-end;flex-wrap:wrap">
        <select id="mission_select" onchange="renderMissionPreview()"></select>
        <button class="ready" onclick="sendCatalogMission()">Send to Drone</button>
      </div>
    </div>
    <div class="plan-body">
      <div>
        <div class="k">Selected</div>
        <div class="plan-meta" id="plan_meta">No mission plans found</div>
        <div class="warnings" id="plan_warnings"></div>
      </div>
      <div class="steps" id="plan_steps"></div>
    </div>
  </section>
  <section class="planner">
    <div class="planner-head">
      <div><div class="k">Mission Builder</div><div class="camera-title">Create Mission</div></div>
      <div style="display:flex;gap:8px;align-items:flex-end;flex-wrap:wrap">
        <input id="builder_name" placeholder="mission name" value="custom_mission"/>
        <select id="builder_step_type"></select>
        <button class="ready" onclick="builderAddStep()">+ Add Step</button>
      </div>
    </div>
    <div class="plan-body">
      <div>
        <div class="k">Plan Status</div>
        <div class="plan-meta" id="builder_result">Add steps, then Validate / Save / Send.</div>
        <div class="warnings" id="builder_warnings"></div>
        <div class="k" style="margin-top:12px">Executor Ack</div>
        <div class="plan-meta" id="plan_ack">--</div>
        <div class="row" style="margin-top:12px">
          <button class="ready" onclick="builderValidate()">Validate</button>
          <button class="ready" onclick="builderSave()">Save to Catalog</button>
          <button class="start" onclick="builderSend()">Send to Drone</button>
        </div>
      </div>
      <div class="steps" id="builder_steps"></div>
    </div>
  </section>
  <div class="row">
    <button class="ready" onclick="post('autonomy_request',{enabled:true})">System Ready</button>
    <button class="start" onclick="post('mission_request',{enabled:true})">Start Mission</button>
    <button class="hold" onclick="post('abort_hold',{confirm:true})">Abort / Hold</button>
    <button class="land" onclick="post('land',{confirm:true})">Land</button>
  </div>
  <p><small>Operator buttons publish request topics only. The Pi re-validates and
  may ignore them. Sent mission plans are re-validated by the mission executor and
  refused while a mission is active. This page cannot arm, send control, or bypass
  Pi safety gates.</small></p>
</div>
<script>
function cls(el,c){el.className='v '+c;}
let missionCatalog = [];
function finite(n){ return typeof n === 'number' && Number.isFinite(n); }
function missionMain(text){
  const s = (text || '').trim();
  const idx = s.indexOf(':');
  return idx >= 0 ? s.slice(0, idx).trim() : s;
}
function missionDetail(text){
  const s = (text || '').trim();
  const idx = s.indexOf(':');
  return idx >= 0 ? s.slice(idx + 1).trim() : '';
}
async function tick(){
  try{
    const r = await fetch('/api/status'); const s = await r.json();
    const link = document.getElementById('link');
    link.textContent = s.link_ok ? 'UP' : 'DOWN'; cls(link, s.link_ok?'ok':'bad');
    document.getElementById('link_reason').textContent = s.link_reason || '';
    document.getElementById('latency').textContent =
      (finite(s.latency_s) && s.latency_s>=0) ? (s.latency_s*1000).toFixed(0)+' ms' : '--';
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
    const mission = document.getElementById('mission');
    const rawMission = s.mission_state || (s.mission_requested ? 'REQUESTED' : 'IDLE');
    const ms = missionMain(rawMission);
    mission.textContent = ms; cls(mission, ms==='IDLE'||ms==='COMPLETE'||ms==='DISABLED'?'ok':ms==='ABORTED'?'bad':'warn');
    document.getElementById('mission_detail').textContent = missionDetail(rawMission) || (s.mission_requested?'operator requested':'');
    const step = document.getElementById('mission_step');
    const stepName = s.mission_step_name || '--';
    step.textContent = stepName; cls(step, stepName==='--'?'ok':'warn');
    document.getElementById('mission_step_detail').textContent =
      s.mission_command_mode ? (s.mission_command_mode + ' | ' + (s.mission_command_status || '')) : '';
    const auton = document.getElementById('autonomy');
    const ae = s.autonomy_effective || (s.autonomy_requested ? 'REQUESTED' : 'OFF');
    auton.textContent = ae; cls(auton, ae==='OFF'||ae==='DISABLED'?'ok':ae==='ENABLED'||ae==='READY'?'warn':'ok');
    document.getElementById('autonomy_detail').textContent = s.autonomy_requested?'operator requested':'';
    const ack = document.getElementById('plan_ack');
    ack.textContent = s.plan_ack || '--';
    ack.className = 'plan-meta ' + (
      (s.plan_ack||'').startsWith('ACCEPTED') ? 'ack-ok' :
      (s.plan_ack||'').startsWith('REJECTED') ? 'ack-bad' :
      (s.plan_ack||'').startsWith('PENDING') ? 'ack-wait' : '');
    document.getElementById('updated').textContent = 'updated '+new Date().toLocaleTimeString();
  }catch(e){ document.getElementById('updated').textContent='status fetch failed'; }
}
async function loadMissions(){
  try{
    const r = await fetch('/api/missions'); const data = await r.json();
    missionCatalog = data.missions || [];
    const sel = document.getElementById('mission_select');
    sel.innerHTML = '';
    missionCatalog.forEach((m, i) => {
      const opt = document.createElement('option');
      opt.value = String(i);
      opt.textContent = (m.valid ? '' : '! ') + (m.name || m.filename);
      sel.appendChild(opt);
    });
    renderMissionPreview();
  }catch(e){
    document.getElementById('plan_meta').textContent = 'Mission preview unavailable';
  }
}
function renderMissionPreview(){
  const sel = document.getElementById('mission_select');
  const m = missionCatalog[Number(sel.value || 0)];
  const meta = document.getElementById('plan_meta');
  const warnings = document.getElementById('plan_warnings');
  const steps = document.getElementById('plan_steps');
  warnings.innerHTML = ''; steps.innerHTML = '';
  if(!m){
    meta.textContent = 'No mission plans found';
    return;
  }
  meta.textContent = m.valid
    ? ((m.path || m.filename) + ' | ' + (m.pi_param_hint || 'set mission_plan_file on the Pi before Start Mission'))
    : ((m.path || m.filename) + ' | ' + (m.error || 'invalid mission'));
  (m.warnings || []).forEach(w => {
    const div = document.createElement('small');
    div.textContent = w;
    warnings.appendChild(div);
  });
  (m.steps || []).forEach(st => {
    const item = document.createElement('div'); item.className = 'step';
    const num = document.createElement('div'); num.className = 'step-num'; num.textContent = String((st.index || 0) + 1);
    const body = document.createElement('div');
    const typ = document.createElement('div'); typ.className = 'step-type'; typ.textContent = st.type || '';
    const label = document.createElement('small'); label.textContent = st.label || '';
    body.appendChild(typ); body.appendChild(label);
    item.appendChild(num); item.appendChild(body); steps.appendChild(item);
  });
}
async function post(path, body){
  await fetch('/api/'+path, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  tick();
}
async function postJson(path, body){
  const r = await fetch('/api/'+path, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  return await r.json();
}

// ---- mission builder ----
// Per-step editable parameters: [key, placeholder, kind]. kind: num | until | dir | text
const STEP_PARAM_DEFS = {
  takeoff:        [['altitude_m','3.0','num']],
  prime_offboard: [['hold_s','1.5','num']],
  scan:           [['direction','ccw','dir'],['yaw_deg','180','num'],['yaw_rate_deg_s','20','num'],['until','locked','until'],['timeout_s','12','num']],
  track_center:   [['until','','until'],['timeout_s','','num']],
  approach:       [['distance_m','2.0','num'],['until','','until'],['timeout_s','20','num']],
  orbit:          [['radius_m','2.0','num'],['speed_m_s','0.4','num'],['revolutions','1','num'],['timeout_s','','num']],
  rtl:            [['timeout_s','15','num']],
  land:           [['timeout_s','','num']],
  hold:           [['status','holding position','text'],['timeout_s','','num']],
};
const UNTIL_OPTIONS = ['', 'locked', 'centered', 'approach_done', 'airborne', 'none'];
let builderSteps = [];

function builderInit(){
  const sel = document.getElementById('builder_step_type');
  Object.keys(STEP_PARAM_DEFS).forEach(t => {
    const o = document.createElement('option'); o.value = t; o.textContent = t; sel.appendChild(o);
  });
  builderSteps = [
    {type:'takeoff', params:{altitude_m:'3.0'}},
    {type:'prime_offboard', params:{hold_s:'1.5'}},
    {type:'track_center', params:{}},
    {type:'land', params:{}},
  ];
  renderBuilder();
}
function builderAddStep(){
  const t = document.getElementById('builder_step_type').value;
  builderSteps.push({type:t, params:{}});
  renderBuilder();
}
function builderRemove(i){ builderSteps.splice(i,1); renderBuilder(); }
function builderMove(i,d){
  const j = i+d;
  if(j<0 || j>=builderSteps.length) return;
  [builderSteps[i], builderSteps[j]] = [builderSteps[j], builderSteps[i]];
  renderBuilder();
}
function builderSetParam(i,k,v){ builderSteps[i].params[k]=v; }
function renderBuilder(){
  const root = document.getElementById('builder_steps');
  root.innerHTML = '';
  builderSteps.forEach((st,i) => {
    const item = document.createElement('div'); item.className='step';
    const num = document.createElement('div'); num.className='step-num'; num.textContent=String(i+1);
    const body = document.createElement('div');
    const typ = document.createElement('div'); typ.className='step-type'; typ.textContent=st.type;
    const params = document.createElement('div'); params.className='step-params';
    (STEP_PARAM_DEFS[st.type]||[]).forEach(([key, placeholder, kind]) => {
      const lab = document.createElement('label');
      lab.appendChild(document.createTextNode(key));
      let inp;
      if(kind==='until' || kind==='dir'){
        inp = document.createElement('select');
        (kind==='dir' ? ['ccw','cw'] : UNTIL_OPTIONS).forEach(v => {
          const o=document.createElement('option'); o.value=v; o.textContent=v===''?'(default)':v; inp.appendChild(o);
        });
        inp.value = st.params[key] ?? '';
      }else{
        inp = document.createElement('input');
        inp.placeholder = placeholder;
        inp.size = Math.max(6, placeholder.length);
        inp.value = st.params[key] ?? '';
      }
      inp.addEventListener('input', e => builderSetParam(i, key, e.target.value));
      inp.addEventListener('change', e => builderSetParam(i, key, e.target.value));
      lab.appendChild(inp);
      params.appendChild(lab);
    });
    const tools = document.createElement('div'); tools.className='step-tools';
    [['\\u2191',()=>builderMove(i,-1)],['\\u2193',()=>builderMove(i,1)],['remove',()=>builderRemove(i)]].forEach(([txt,fn])=>{
      const b=document.createElement('button'); b.textContent=txt; b.onclick=fn; tools.appendChild(b);
    });
    body.appendChild(typ); body.appendChild(params); body.appendChild(tools);
    item.appendChild(num); item.appendChild(body);
    root.appendChild(item);
  });
}
function buildMission(){
  const steps = builderSteps.map(st => {
    const out = {type: st.type};
    Object.entries(st.params).forEach(([k,v]) => {
      const s = String(v).trim();
      if(s === '') return;
      const n = Number(s);
      out[k] = Number.isFinite(n) && /^[-+0-9.eE]+$/.test(s) ? n : s;
    });
    return out;
  });
  return {name: (document.getElementById('builder_name').value.trim() || 'custom_mission'), steps};
}
function showBuilderResult(text, ok, warnings){
  const res = document.getElementById('builder_result');
  res.textContent = text;
  res.className = 'plan-meta ' + (ok ? 'ack-ok' : 'ack-bad');
  const w = document.getElementById('builder_warnings');
  w.innerHTML = '';
  (warnings||[]).forEach(t => { const d=document.createElement('small'); d.textContent=t; w.appendChild(d); });
}
async function builderValidate(){
  try{
    const r = await postJson('mission_plan/validate', {mission: buildMission()});
    if(r.valid) showBuilderResult('Valid: '+r.steps.length+' steps', true, r.warnings);
    else showBuilderResult('Invalid: '+(r.error||'unknown error'), false, r.warnings);
  }catch(e){ showBuilderResult('Validate failed: '+e, false); }
}
async function builderSave(){
  try{
    const m = buildMission();
    const r = await postJson('mission_plan/save', {mission: m, filename: m.name});
    if(r.ok){ showBuilderResult('Saved: '+r.path, true, (r.record||{}).warnings); loadMissions(); }
    else showBuilderResult('Save failed: '+(r.error||'unknown error'), false, (r.record||{}).warnings);
  }catch(e){ showBuilderResult('Save failed: '+e, false); }
}
async function builderSend(){
  try{
    const r = await postJson('mission_plan/send', {mission: buildMission()});
    if(r.ok) showBuilderResult('Sent \\''+r.sent+'\\' to the mission executor \\u2014 watch Executor Ack.', true, (r.record||{}).warnings);
    else showBuilderResult('Send failed: '+(r.error||'unknown error'), false, (r.record||{}).warnings);
    tick();
  }catch(e){ showBuilderResult('Send failed: '+e, false); }
}
async function sendCatalogMission(){
  const sel = document.getElementById('mission_select');
  if(sel.value === '') return;
  try{
    const r = await postJson('mission_plan/send', {catalog_index: Number(sel.value)});
    const meta = document.getElementById('plan_meta');
    meta.textContent = r.ok ? ('Sent \\''+r.sent+'\\' to the mission executor \\u2014 watch Executor Ack.')
                            : ('Send failed: '+(r.error||'unknown error'));
    tick();
  }catch(e){ document.getElementById('plan_meta').textContent = 'Send failed: '+e; }
}
setInterval(tick, 500); tick(); loadMissions(); builderInit();
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
        self.declare_parameter("mission_state_topic", "/drone/mission/state")
        self.declare_parameter("mission_command_topic", "/drone/mission/command")
        self.declare_parameter("autonomy_state_topic", "/drone/autonomy/state")
        self.declare_parameter("mission_catalog_paths", "")
        self.declare_parameter("mission_plan_topic", "/drone/mission/plan_request")
        self.declare_parameter("mission_plan_ack_topic", "/drone/mission/plan_ack")
        # Where dashboard-built missions are saved. Empty = ~/dronetrack_missions.
        self.declare_parameter("mission_save_dir", "")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)

        save_dir = str(self.get_parameter("mission_save_dir").value).strip()
        self.mission_save_dir = Path(save_dir).expanduser() if save_dir else Path.home() / "dronetrack_missions"
        self._catalog_lock = threading.Lock()
        self.mission_catalog: list[dict] = []
        self._reload_mission_catalog()

        self._lock = threading.Lock()
        self._shutdown = threading.Event()
        self._state = {
            "link_ok": False, "link_reason": "no data", "latency_s": float("nan"),
            "perception_fps": -1.0, "detection_count": 0, "max_confidence": 0.0,
            "px4_connected": False, "flight_mode": "", "battery_percent": -1.0,
            "rel_altitude_m": 0.0, "armed": False, "camera_frame_age_s": -1.0,
            "target_summary": "", "overlay_available": cv2 is not None and np is not None,
            "autonomy_requested": False, "mission_requested": False,
            "autonomy_effective": "", "mission_state": "",
            "mission_command_mode": "", "mission_step_index": -1,
            "mission_step_name": "", "mission_command_status": "",
            "plan_ack": "",
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
        self.create_subscription(String, str(self.get_parameter("mission_state_topic").value),
                                 self._on_mission_state, reliable)
        self.create_subscription(MissionCommand, str(self.get_parameter("mission_command_topic").value),
                                 self._on_mission_command, reliable)
        self.create_subscription(String, str(self.get_parameter("autonomy_state_topic").value),
                                 self._on_autonomy_state, reliable)
        self.create_subscription(String, str(self.get_parameter("mission_plan_ack_topic").value),
                                 self._on_plan_ack, reliable)

        self.mission_pub = self.create_publisher(Bool, str(self.get_parameter("mission_request_topic").value), reliable)
        self.plan_pub = self.create_publisher(String, str(self.get_parameter("mission_plan_topic").value), reliable)
        self.autonomy_pub = self.create_publisher(Bool, str(self.get_parameter("autonomy_request_topic").value), reliable)
        self.offboard_pub = self.create_publisher(Bool, str(self.get_parameter("offboard_request_topic").value), reliable)
        self.action_pub = self.create_publisher(MavsdkActionCommand, str(self.get_parameter("mavsdk_action_topic").value), reliable)

        self._start_server()
        self.get_logger().info(
            f"Dashboard serving on http://{self.host}:{self.port}/ | "
            f"mission previews={len(self.mission_catalog)}"
        )

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

    def _on_mission_state(self, msg: String) -> None:
        with self._lock:
            self._state["mission_state"] = msg.data

    def _on_mission_command(self, msg: MissionCommand) -> None:
        with self._lock:
            self._state["mission_command_mode"] = str(msg.mode)
            self._state["mission_step_index"] = int(msg.step_index)
            self._state["mission_step_name"] = str(msg.step_name)
            self._state["mission_command_status"] = str(msg.status)

    def _on_autonomy_state(self, msg: String) -> None:
        with self._lock:
            self._state["autonomy_effective"] = msg.data

    def _on_plan_ack(self, msg: String) -> None:
        with self._lock:
            self._state["plan_ack"] = msg.data

    # ---- mission builder ---------------------------------------------------
    def _reload_mission_catalog(self) -> None:
        configured = str(self.get_parameter("mission_catalog_paths").value or "").strip()
        if configured:
            paths = [p for p in configured.replace(";", os.pathsep).split(os.pathsep) if p.strip()]
        else:
            paths = [str(p) for p in discover_mission_paths(__file__)]
        paths.append(str(self.mission_save_dir))  # dashboard-built missions join the catalog
        catalog = load_mission_catalog(paths, module_file=__file__)
        with self._catalog_lock:
            self.mission_catalog = catalog

    @staticmethod
    def _extract_mission_data(payload: dict) -> dict:
        mission = payload.get("mission")
        if not isinstance(mission, dict):
            raise ValueError("payload must contain a 'mission' object")
        return {"mission": mission}

    def validate_mission_payload(self, payload: dict) -> dict:
        data = self._extract_mission_data(payload)
        return preview_mission_data(data, module_file=__file__)

    def save_mission_payload(self, payload: dict) -> dict:
        data = self._extract_mission_data(payload)
        record = preview_mission_data(data, module_file=__file__)
        if not record.get("valid"):
            return {"ok": False, "error": record.get("error") or "invalid mission", "record": record}

        requested = str(payload.get("filename") or record.get("name") or "custom_mission")
        stem = re.sub(r"[^A-Za-z0-9_-]+", "_", requested.removesuffix(".yaml").removesuffix(".yml")).strip("_")
        if not stem:
            return {"ok": False, "error": "filename resolves to nothing after sanitizing"}

        import yaml

        try:
            self.mission_save_dir.mkdir(parents=True, exist_ok=True)
            path = self.mission_save_dir / f"{stem}.yaml"
            path.write_text(
                yaml.safe_dump(data, sort_keys=False, default_flow_style=False),
                encoding="utf-8",
            )
        except OSError as exc:
            return {"ok": False, "error": f"could not write mission file: {exc}"}

        self._reload_mission_catalog()
        self.get_logger().info(f"Mission builder saved plan '{record.get('name')}' to {path}")
        return {"ok": True, "path": str(path), "record": record}

    def send_mission_payload(self, payload: dict) -> dict:
        if "catalog_index" in payload:
            with self._catalog_lock:
                catalog = list(self.mission_catalog)
            try:
                record = catalog[int(payload["catalog_index"])]
            except (ValueError, TypeError, IndexError):
                return {"ok": False, "error": "catalog_index out of range"}
            if not record.get("valid"):
                return {"ok": False, "error": f"catalog mission invalid: {record.get('error')}"}
            try:
                text = Path(str(record["path"])).read_text(encoding="utf-8")
            except OSError as exc:
                return {"ok": False, "error": f"could not read mission file: {exc}"}
            plan_name = str(record.get("name") or record.get("filename"))
        else:
            data = self._extract_mission_data(payload)
            record = preview_mission_data(data, module_file=__file__)
            if not record.get("valid"):
                return {"ok": False, "error": record.get("error") or "invalid mission", "record": record}
            text = json.dumps(data)
            plan_name = str(record.get("name"))

        with self._lock:
            self._state["plan_ack"] = "PENDING: waiting for mission executor ack..."
        self.plan_pub.publish(String(data=text))
        self.get_logger().info(f"Mission builder sent plan '{plan_name}' to the mission executor")
        return {"ok": True, "sent": plan_name, "record": record}

    # ---- operator actions ------------------------------------------------
    def publish_mission_request(self, enabled: bool) -> None:
        self.mission_pub.publish(Bool(data=enabled))
        with self._lock:
            self._state["mission_requested"] = enabled

    def publish_autonomy_request(self, enabled: bool) -> None:
        self.autonomy_pub.publish(Bool(data=enabled))
        with self._lock:
            self._state["autonomy_requested"] = enabled

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
        self.autonomy_pub.publish(Bool(data=False))
        self.offboard_pub.publish(Bool(data=False))
        self.mission_pub.publish(Bool(data=False))
        with self._lock:
            self._state["autonomy_requested"] = False
            self._state["mission_requested"] = False
        self._send_action("HOLD", "dashboard abort/hold")

    def land(self) -> None:
        self.autonomy_pub.publish(Bool(data=False))
        self.offboard_pub.publish(Bool(data=False))
        self.mission_pub.publish(Bool(data=False))
        with self._lock:
            self._state["autonomy_requested"] = False
            self._state["mission_requested"] = False
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

    def mission_catalog_snapshot(self) -> dict:
        with self._catalog_lock:
            missions = list(self.mission_catalog)
        return {
            "mode": "preview_and_upload",
            "trust_boundary": (
                "Uploaded plans are re-validated by the Pi-side mission executor, which "
                "refuses uploads while a mission is active. Uploading never starts a "
                "mission, never arms, and never bypasses Pi safety gates."
            ),
            "save_dir": str(self.mission_save_dir),
            "missions": missions,
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

        while not self._shutdown.is_set():
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
                elif self.path == "/api/missions":
                    self._send(200, json.dumps(node.mission_catalog_snapshot()).encode("utf-8"))
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
                if not isinstance(payload, dict):
                    self._send(400, b'{"error":"payload must be a JSON object"}')
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
                    elif self.path == "/api/mission_plan/validate":
                        self._send(200, json.dumps(node.validate_mission_payload(payload)).encode("utf-8"))
                        return
                    elif self.path == "/api/mission_plan/save":
                        self._send(200, json.dumps(node.save_mission_payload(payload)).encode("utf-8"))
                        return
                    elif self.path == "/api/mission_plan/send":
                        self._send(200, json.dumps(node.send_mission_payload(payload)).encode("utf-8"))
                        return
                    else:
                        self._send(404, b'{"error":"not found"}')
                        return
                    self._send(200, b'{"ok":true}')
                except ValueError as exc:
                    self._send(400, json.dumps({"error": str(exc)}).encode("utf-8"))
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
        self._shutdown.set()  # unblock any MJPEG stream loops before stopping the server
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
