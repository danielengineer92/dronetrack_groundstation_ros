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
    GET    /                            dashboard HTML
    GET    /api/status                  latest status JSON
    GET    /api/missions                local mission plan previews (display only)
    GET    /stream.mjpg                 live camera stream with YOLO detection overlays
    GET    /api/mission-plans           list saved plans + templates
    GET    /api/mission-plans/<file>    load a plan by filename
    POST   /api/mission-plans/save      save a plan to disk  (JSON body: {name, steps, overwrite?})
    POST   /api/mission-plans/send      validate & publish a plan (JSON body: {name, steps})
    DELETE /api/mission-plans/<file>    delete a saved plan
    GET    /api/mission-step-schema     step verb + parameter definitions for the UI
    POST   /api/mission_request         {"enabled": true|false}
    POST   /api/autonomy_request        {"enabled": true|false}
    POST   /api/abort_hold              {"confirm": true}
    POST   /api/land                    {"confirm": true}
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

from drone_interfaces.msg import DetectionArray, DroneTelemetry, MavsdkActionCommand, MissionCommand
from dronetrack_msgs.msg import GroundStationHeartbeat, LinkStatus
from dronetrack_web_bridge.mission_preview import load_mission_catalog
from dronetrack_web_bridge.mission_plan_model import (
    get_step_schema, create_default_step, validate_step,
    steps_to_yaml, lint_steps, sanitize_filename, CATEGORY_COLORS, STEP_SCHEMA,
)
from dronetrack_web_bridge.mission_preview_ext import plan_from_steps


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
  .danger { background:#ff5f75; color:#fff; }
  .badge { font-size:11px; font-weight:800; border-radius:4px; padding:2px 7px; text-transform:uppercase; letter-spacing:.05em; white-space:nowrap; }
  .badge-action { background:#2ea043; color:#fff; }
  .badge-preflight { background:#1f6feb; color:#fff; }
  .badge-motion { background:#c06a00; color:#fff; }
  .step-card { background:#09182d; border:1px solid rgba(100,160,220,.14); border-radius:8px; padding:10px 12px; cursor:pointer; transition:border-color .15s; }
  .step-card.selected { border-color:#1f6feb; }
  .step-card-head { display:flex; gap:8px; align-items:center; }
  .step-summary { flex:1; font-size:13px; color:#cfe6ff; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .step-actions { display:flex; gap:4px; flex-shrink:0; }
  .step-actions button { padding:4px 8px; font-size:12px; border-radius:5px; }
  .param-editor { margin-top:8px; display:grid; grid-template-columns:repeat(auto-fill,minmax(140px,1fr)); gap:8px; padding-top:8px; border-top:1px solid rgba(100,160,220,.1); }
  .param-editor label { display:grid; gap:3px; font-size:12px; color:#83a3bd; }
  .param-editor input, .param-editor select { font:inherit; color:#e7f4ff; background:#06101e; border:1px solid rgba(100,160,220,.28); border-radius:5px; padding:5px 7px; width:100%; box-sizing:border-box; }
  .add-bar { display:flex; gap:6px; flex-wrap:wrap; }
  .add-bar button { font-size:12px; padding:6px 10px; background:#09182d; color:#cfe6ff; border:1px solid rgba(100,160,220,.28); border-radius:6px; }
  .add-bar button:hover { background:#1f6feb; color:#fff; border-color:#1f6feb; }
  .planner-toolbar { display:flex; gap:8px; flex-wrap:wrap; align-items:center; padding:14px; border-bottom:1px solid rgba(100,160,220,.1); }
  .planner-toolbar input[type=text] { font:inherit; color:#e7f4ff; background:#09182d; border:1px solid rgba(100,160,220,.28); border-radius:8px; padding:9px 10px; min-width:150px; }
  .status-bar { display:flex; gap:14px; font-size:12px; color:#83a3bd; padding:8px 14px; border-top:1px solid rgba(100,160,220,.1); }
  .status-ok { color:#50fa7b; } .status-bad { color:#ff5f75; }
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
    <div class="card"><div class="k">Local NED</div><div class="v" id="ned">--</div></div>
    <div class="card"><div class="k">Armed</div><div class="v" id="armed">--</div></div>
    <div class="card"><div class="k">Mission</div><div class="v" id="mission">--</div><small id="mission_detail"></small></div>
    <div class="card"><div class="k">Step</div><div class="v" id="mission_step">--</div><small id="mission_step_detail"></small></div>
    <div class="card"><div class="k">Autonomy</div><div class="v" id="autonomy">--</div><small id="autonomy_detail"></small></div>
  </div>
  <section class="planner">
    <div class="planner-toolbar">
      <div><div class="k">Mission Planner</div><div class="camera-title" style="font-size:16px">Builder</div></div>
      <input id="plan_name" type="text" placeholder="Plan name..." value="untitled"/>
      <button class="start" onclick="savePlan()">Save</button>
      <select id="load_select"><option value="">-- Load plan --</option></select>
      <button class="ready" onclick="loadSelectedPlan()">Load</button>
      <button onclick="clearPlan()" style="background:#09182d;color:#cfe6ff;border:1px solid rgba(100,160,220,.28);border-radius:8px;padding:12px 16px">Clear</button>
      <button class="start" onclick="sendPlan()">Send to Drone</button>
    </div>
    <div style="padding:14px">
      <div class="k" style="margin-bottom:6px">Add Step</div>
      <div class="add-bar" id="add_bar"></div>
      <div id="step_list" style="margin-top:10px;display:grid;gap:6px"></div>
      <div class="warnings" id="plan_warnings" style="margin-top:8px"></div>
    </div>
    <div class="status-bar" id="status_bar"><span>0 steps</span></div>
  </section>
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
function finite(n){ return typeof n === 'number' && Number.isFinite(n); }
let planSteps = [], selectedStep = null, stepSchema = {};
const VERB_CAT = {
  prime_offboard:'preflight', takeoff:'action', land:'action', rtl:'action',
  hold:'action', scan:'motion', approach:'motion', orbit:'motion', track_center:'motion',
  goto_relative:'motion', goto_absolute:'motion'
};
const ADD_VERBS = ['takeoff','prime_offboard','scan','track_center','approach','orbit','goto_relative','goto_absolute','rtl','land','hold'];
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
    const nedEl = document.getElementById('ned');
    if (s.local_ned && s.local_ned.length === 3) {
      nedEl.textContent = s.local_ned[0].toFixed(1)+', '+s.local_ned[1].toFixed(1)+', '+s.local_ned[2].toFixed(1);
    } else { nedEl.textContent = '--'; }
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
    document.getElementById('updated').textContent = 'updated '+new Date().toLocaleTimeString();
  }catch(e){ document.getElementById('updated').textContent='status fetch failed'; }
}
async function initPlanner() {
  try {
    const r = await fetch('/api/mission-step-schema');
    const d = await r.json();
    stepSchema = d.verbs || {};
  } catch(e) {}
  const bar = document.getElementById('add_bar');
  ADD_VERBS.forEach(v => {
    const btn = document.createElement('button');
    btn.textContent = '+ ' + v;
    btn.onclick = () => addStep(v);
    bar.appendChild(btn);
  });
  await refreshLoadDropdown();
  const saved = localStorage.getItem('dronetrack_plan');
  if (saved) {
    try {
      const d = JSON.parse(saved);
      planSteps = d.steps || [];
      document.getElementById('plan_name').value = d.name || 'untitled';
    } catch(e) {}
  }
  renderSteps();
}

async function refreshLoadDropdown() {
  try {
    const r = await fetch('/api/mission-plans');
    const d = await r.json();
    const sel = document.getElementById('load_select');
    sel.innerHTML = '<option value="">-- Load plan --</option>';
    (d.plans || []).forEach(p => {
      const o = document.createElement('option');
      o.value = p.filename;
      o.textContent = p.name + (p.is_template ? ' [template]' : '') + ' (' + (p.step_count||0) + ')';
      sel.appendChild(o);
    });
  } catch(e) {}
}

function addStep(verb) {
  const sc = stepSchema[verb] || {};
  const params = {};
  for (const [k, spec] of Object.entries(sc.params || {})) {
    if (spec.default !== undefined) params[k] = spec.default;
  }
  planSteps.push({type: verb, params});
  selectedStep = planSteps.length - 1;
  autosave(); renderSteps();
}

function deleteStep(i) {
  planSteps.splice(i, 1);
  if (selectedStep === i) selectedStep = null;
  else if (selectedStep !== null && selectedStep > i) selectedStep--;
  autosave(); renderSteps();
}

function moveStep(i, dir) {
  const j = i + dir;
  if (j < 0 || j >= planSteps.length) return;
  [planSteps[i], planSteps[j]] = [planSteps[j], planSteps[i]];
  if (selectedStep === i) selectedStep = j;
  else if (selectedStep === j) selectedStep = i;
  autosave(); renderSteps();
}

function selectStep(i) {
  selectedStep = selectedStep === i ? null : i;
  renderSteps();
}

function verbCat(verb) { return VERB_CAT[verb] || 'action'; }

function stepSummary(step) {
  const p = step.params || {};
  const kv = Object.entries(p).slice(0,3).map(([k,v]) => k+'='+v);
  return kv.length ? kv.join(', ') : '';
}

function renderSteps() {
  const list = document.getElementById('step_list');
  list.innerHTML = '';
  planSteps.forEach((step, i) => {
    const card = document.createElement('div');
    card.className = 'step-card' + (selectedStep === i ? ' selected' : '');
    const head = document.createElement('div');
    head.className = 'step-card-head';
    head.onclick = () => selectStep(i);
    const num = document.createElement('span');
    num.style.cssText = 'color:#83a3bd;font-weight:800;font-size:13px;min-width:22px;flex-shrink:0';
    num.textContent = (i+1)+'.';
    const badge = document.createElement('span');
    badge.className = 'badge badge-' + verbCat(step.type);
    badge.textContent = step.type;
    const sumEl = document.createElement('span');
    sumEl.className = 'step-summary';
    sumEl.textContent = stepSummary(step);
    const acts = document.createElement('div');
    acts.className = 'step-actions';
    acts.onclick = e => e.stopPropagation();
    const upB = document.createElement('button');
    upB.textContent = '▲'; upB.title = 'Move up';
    upB.style.cssText = 'background:#09182d;color:#cfe6ff;border:1px solid rgba(100,160,220,.28)';
    upB.onclick = e => { e.stopPropagation(); moveStep(i,-1); };
    const dnB = document.createElement('button');
    dnB.textContent = '▼'; dnB.title = 'Move down';
    dnB.style.cssText = 'background:#09182d;color:#cfe6ff;border:1px solid rgba(100,160,220,.28)';
    dnB.onclick = e => { e.stopPropagation(); moveStep(i,1); };
    const delB = document.createElement('button');
    delB.textContent = '✕'; delB.title = 'Delete'; delB.className = 'danger';
    delB.onclick = e => { e.stopPropagation(); deleteStep(i); };
    acts.append(upB, dnB, delB);
    head.append(num, badge, sumEl, acts);
    card.appendChild(head);
    if (selectedStep === i) {
      const sc = stepSchema[step.type] || {};
      const pEntries = Object.entries(sc.params || {});
      if (pEntries.length) {
        const ed = document.createElement('div');
        ed.className = 'param-editor';
        pEntries.forEach(([pName, spec]) => {
          const lbl = document.createElement('label');
          lbl.textContent = spec.label || pName;
          let inp;
          if (spec.type === 'enum' && spec.options) {
            inp = document.createElement('select');
            spec.options.forEach(c => {
              const o = document.createElement('option');
              o.value = c; o.textContent = c;
              const cur = step.params[pName];
              if (cur === c || (cur === undefined && spec.default === c)) o.selected = true;
              inp.appendChild(o);
            });
          } else if (spec.type === 'str') {
            inp = document.createElement('input');
            inp.type = 'text';
            const cur = step.params[pName];
            inp.value = cur !== undefined ? cur : (spec.default !== undefined ? spec.default : '');
          } else {
            inp = document.createElement('input');
            inp.type = 'number';
            if (spec.step) inp.step = spec.step;
            if (spec.min !== undefined) inp.min = spec.min;
            if (spec.max !== undefined) inp.max = spec.max;
            const cur = step.params[pName];
            inp.value = cur !== undefined ? cur : (spec.default !== undefined ? spec.default : '');
          }
          inp.onchange = () => {
            planSteps[i].params[pName] = inp.tagName === 'SELECT' || spec.type === 'str' ? inp.value : parseFloat(inp.value);
            sumEl.textContent = stepSummary(planSteps[i]);
            autosave();
          };
          lbl.appendChild(inp);
          ed.appendChild(lbl);
        });
        card.appendChild(ed);
      }
    }
    list.appendChild(card);
  });
  updateWarnings();
  updateStatusBar();
}

function updateWarnings() {
  const w = document.getElementById('plan_warnings');
  w.innerHTML = '';
  planSteps.forEach((s, i) => {
    if (!s.type) {
      const d = document.createElement('small');
      d.textContent = '⚠ Step '+(i+1)+': missing type';
      w.appendChild(d);
    }
  });
}

function updateStatusBar() {
  const bar = document.getElementById('status_bar');
  const n = planSteps.length;
  bar.innerHTML = '<span>'+n+' step'+(n===1?'':'s')+'</span><span class="'+(n>0?'status-ok':'status-bad')+'">'+(n>0?'ready to send':'plan empty')+'</span>';
}

function autosave() {
  const name = document.getElementById('plan_name').value || 'untitled';
  localStorage.setItem('dronetrack_plan', JSON.stringify({name, steps: planSteps}));
}

function clearPlan() {
  planSteps = []; selectedStep = null;
  document.getElementById('plan_name').value = 'untitled';
  autosave(); renderSteps();
}

async function savePlan() {
  const name = document.getElementById('plan_name').value || 'untitled';
  const doSave = async (overwrite) => {
    const r = await fetch('/api/mission-plans/save', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({name, steps:planSteps, overwrite})});
    return r.json();
  };
  let d = await doSave(false);
  if (d.conflict) {
    if (!confirm('Plan "'+name+'" already exists. Overwrite?')) return;
    d = await doSave(true);
  }
  if (d.ok) { await refreshLoadDropdown(); }
  else { alert('Save failed: '+(d.errors||[]).join(', ')); }
}

async function loadSelectedPlan() {
  const filename = document.getElementById('load_select').value;
  if (!filename) return;
  const r = await fetch('/api/mission-plans/'+encodeURIComponent(filename));
  if (!r.ok) { alert('Load failed'); return; }
  const d = await r.json();
  planSteps = d.steps || [];
  document.getElementById('plan_name').value = d.name || filename.replace('.yaml','');
  selectedStep = null;
  autosave(); renderSteps();
}

async function sendPlan() {
  const name = document.getElementById('plan_name').value || 'untitled';
  if (!planSteps.length) { alert('Plan is empty'); return; }
  const r = await fetch('/api/mission-plans/send', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify({name, steps:planSteps})});
  const d = await r.json();
  if (!d.ok) alert('Send failed: '+(d.errors||[]).join(', '));
}
async function post(path, body){
  await fetch('/api/'+path, {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  tick();
}
setInterval(tick, 500); tick(); initPlanner();
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
        self.declare_parameter("mission_plans_dir", "~/drone_mission_plans/")
        self.declare_parameter("mission_plan_topic", "/drone/mission/plan")

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)
        self.mission_catalog = load_mission_catalog(
            self.get_parameter("mission_catalog_paths").value,
            module_file=__file__,
        )

        self._lock = threading.Lock()
        self._state = {
            "link_ok": False, "link_reason": "no data", "latency_s": float("nan"),
            "perception_fps": -1.0, "detection_count": 0, "max_confidence": 0.0,
            "px4_connected": False, "flight_mode": "", "battery_percent": -1.0,
            "rel_altitude_m": 0.0, "armed": False, "landed_state": "",
            "local_ned": None, "camera_frame_age_s": -1.0,
            "target_summary": "", "overlay_available": cv2 is not None and np is not None,
            "autonomy_requested": False, "mission_requested": False,
            "autonomy_effective": "", "mission_state": "",
            "mission_command_mode": "", "mission_step_index": -1,
            "mission_step_name": "", "mission_command_status": "",
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

        self.mission_pub = self.create_publisher(Bool, str(self.get_parameter("mission_request_topic").value), reliable)
        self.autonomy_pub = self.create_publisher(Bool, str(self.get_parameter("autonomy_request_topic").value), reliable)
        self.offboard_pub = self.create_publisher(Bool, str(self.get_parameter("offboard_request_topic").value), reliable)
        self.action_pub = self.create_publisher(MavsdkActionCommand, str(self.get_parameter("mavsdk_action_topic").value), reliable)
        self.plan_pub = self.create_publisher(String, str(self.get_parameter("mission_plan_topic").value), reliable)

        self.mission_plans_dir = str(self.get_parameter("mission_plans_dir").value)

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
            self._state["landed_state"] = msg.landed_state
            if bool(msg.local_position_valid):
                self._state["local_ned"] = [
                    round(float(msg.local_position_north), 2),
                    round(float(msg.local_position_east), 2),
                    round(float(msg.local_position_down), 2),
                ]

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
        return {
            "mode": "preview_only",
            "trust_boundary": (
                "Mission selection stays on the Pi: set mission_executor_node.mission_plan_file "
                "there before starting a mission. The dashboard only previews local YAML."
            ),
            "missions": self.mission_catalog,
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
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?", 1)[0]  # strip query params
                if path in ("/", "/index.html"):
                    self._send(200, DASHBOARD_HTML.encode("utf-8"), "text/html; charset=utf-8")
                elif path == "/stream.mjpg":
                    node.stream_mjpeg(self)
                elif path == "/api/status":
                    self._send(200, json.dumps(node.snapshot()).encode("utf-8"))
                elif path == "/api/missions":
                    self._send(200, json.dumps(node.mission_catalog_snapshot()).encode("utf-8"))
                elif path == "/api/mission-step-schema":
                    self._send(200, json.dumps({"verbs": get_step_schema()}).encode("utf-8"))
                elif path == "/api/mission-plans":
                    self._send(200, json.dumps(node._list_mission_plans()).encode("utf-8"))
                elif path.startswith("/api/mission-plans/"):
                    from urllib.parse import unquote
                    filename = unquote(path[len("/api/mission-plans/"):])
                    if not filename or not node._safe_filename(filename):
                        self._send(400, b'{"error":"invalid filename"}')
                        return
                    result = node._load_mission_plan(filename)
                    if result is None:
                        self._send(404, b'{"error":"plan not found"}')
                    else:
                        self._send(200, json.dumps(result).encode("utf-8"))
                else:
                    self._send(404, b'{"error":"not found"}')

            def do_DELETE(self):
                path = self.path.split("?", 1)[0]
                if path.startswith("/api/mission-plans/"):
                    from urllib.parse import unquote
                    filename = unquote(path[len("/api/mission-plans/"):])
                    if not filename or not node._safe_filename(filename):
                        self._send(400, b'{"error":"invalid filename"}')
                        return
                    result = node._delete_mission_plan(filename)
                    if result is None:
                        self._send(404, b'{"error":"plan not found"}')
                    else:
                        self._send(200, json.dumps(result).encode("utf-8"))
                else:
                    self._send(404, b'{"error":"not found"}')

            def do_OPTIONS(self):
                self.send_response(200)
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, DELETE, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")
                self.end_headers()

            def do_POST(self):
                path = self.path.split("?", 1)[0]
                length = int(self.headers.get("Content-Length", 0) or 0)
                if length > 1_000_000:
                    self._send(413, b'{"error":"payload too large"}')
                    return
                raw = self.rfile.read(length) if length else b"{}"
                try:
                    payload = json.loads(raw or b"{}")
                except json.JSONDecodeError:
                    self._send(400, b'{"error":"bad json"}')
                    return
                try:
                    if path == "/api/mission_request":
                        node.publish_mission_request(bool(payload.get("enabled", False)))
                    elif path == "/api/autonomy_request":
                        node.publish_autonomy_request(bool(payload.get("enabled", False)))
                    elif path == "/api/abort_hold":
                        if payload.get("confirm"):
                            node.abort_hold()
                    elif path == "/api/land":
                        if payload.get("confirm"):
                            node.land()
                    elif path == "/api/mission-plans/save":
                        result = node._save_mission_plan(payload)
                        if result.get("conflict"):
                            self._send(409, json.dumps(result).encode("utf-8"))
                            return
                        if "errors" in result and result["errors"]:
                            self._send(400, json.dumps(result).encode("utf-8"))
                            return
                        self._send(200, json.dumps(result).encode("utf-8"))
                    elif path == "/api/mission-plans/send":
                        result = node._send_mission_plan(payload)
                        if "errors" in result and result["errors"]:
                            self._send(400, json.dumps(result).encode("utf-8"))
                            return
                        self._send(200, json.dumps(result).encode("utf-8"))
                    else:
                        self._send(404, b'{"error":"not found"}')
                        return
                except Exception:
                    node.get_logger().exception("POST handler error")
                    self._send(500, b'{"error":"internal error"}')

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

    @staticmethod
    def _safe_filename(filename: str) -> bool:
        """Reject path-traversal and empty filenames before passing to Path."""
        if not filename or ".." in filename:
            return False
        if filename.startswith("/") or "\\" in filename:
            return False
        return True

    def _list_mission_plans(self) -> dict:
        """List saved plans + templates for the Load dropdown."""
        import os as _os
        from pathlib import Path as _Path
        plans_dir = _Path(self.mission_plans_dir).expanduser()
        plans = []
        if plans_dir.exists():
            for f in sorted(plans_dir.glob("*.yaml")):
                try:
                    import yaml
                    data = yaml.safe_load(f.read_text())
                    mission = data.get("mission", data)
                    steps = mission.get("steps", [])
                    plans.append({
                        "name": mission.get("name", f.stem),
                        "filename": f.name,
                        "modified": _os.path.getmtime(str(f)),
                        "step_count": len(steps),
                        "valid": True,
                        "is_template": False,
                    })
                except Exception:
                    plans.append({"name": f.stem, "filename": f.name, "modified": 0, "step_count": 0, "valid": False, "is_template": False})
        # Templates from drone_control/missions
        templates = []
        mission_src = _Path(__file__).resolve().parent.parent.parent / "drone_control" / "missions"
        if mission_src.exists():
            for f in sorted(mission_src.glob("*.yaml")):
                templates.append({
                    "name": f.stem, "filename": f.name, "is_template": True,
                    "step_count": 0, "valid": True, "modified": 0,
                })
        return {"plans": plans + templates}

    def _load_mission_plan(self, filename: str) -> dict | None:
        """Load a specific plan by filename."""
        if not self._safe_filename(filename):
            return None
        from pathlib import Path as _Path
        plans_dir = _Path(self.mission_plans_dir).expanduser()
        resolved_plans_dir = plans_dir.resolve()
        paths = [plans_dir / filename]
        mission_src = _Path(__file__).resolve().parent.parent.parent / "drone_control" / "missions"
        if mission_src.exists():
            resolved_mission_src = mission_src.resolve()
            paths.append(mission_src / filename)
        for fp in paths:
            if fp.exists():
                # Extra defence: confirm the resolved path stays within its base
                try:
                    resolved = fp.resolve()
                except OSError:
                    continue
                bases = [resolved_plans_dir]
                if mission_src.exists():
                    bases.append(resolved_mission_src)
                if not any(resolved.is_relative_to(base) for base in bases):
                    continue
                try:
                    import yaml
                    data = yaml.safe_load(fp.read_text())
                    mission = data.get("mission", data)
                    steps = mission.get("steps", [])
                    named_steps = []
                    for s in steps:
                        if isinstance(s, str):
                            tp = s
                            raw_params = {}
                        elif isinstance(s, dict):
                            tp = s.get("type", "")
                            raw_params = {k: v for k, v in s.items() if k != "type"}
                        else:
                            continue
                        defaults = {}
                        if tp in STEP_SCHEMA:
                            for k, spec in STEP_SCHEMA[tp].get("params", {}).items():
                                defaults[k] = spec["default"]
                        defaults.update(raw_params)
                        named_steps.append({"type": tp, "params": defaults})
                    return {
                        "name": mission.get("name", filename),
                        "steps": named_steps,
                        "warnings": lint_steps(named_steps),
                        "filename": filename,
                    }
                except Exception as e:
                    return {"name": filename, "steps": [], "warnings": [str(e)], "filename": filename}
        return None

    def _save_mission_plan(self, payload: dict) -> dict:
        """Save a plan to disk. Returns {ok, filename, warnings} or {conflict: True}."""
        name = payload.get("name", "untitled")
        steps = payload.get("steps", [])
        overwrite = payload.get("overwrite", False)
        errors = []
        for s in steps:
            errors.extend(validate_step(s))
        if errors:
            return {"ok": False, "errors": errors}
        filename = sanitize_filename(name) + ".yaml"
        from pathlib import Path as _Path
        plans_dir = _Path(self.mission_plans_dir).expanduser()
        plans_dir.mkdir(parents=True, exist_ok=True)
        fp = plans_dir / filename
        if fp.exists() and not overwrite:
            return {"ok": False, "conflict": True, "filename": filename}
        yaml_str = steps_to_yaml(name, steps)
        fp.write_text(yaml_str)
        return {"ok": True, "filename": filename, "warnings": lint_steps(steps)}

    def _delete_mission_plan(self, filename: str) -> dict | None:
        """Delete a saved plan. Returns {ok: True} or None if not found."""
        if not self._safe_filename(filename):
            return None
        from pathlib import Path as _Path
        fp = _Path(self.mission_plans_dir).expanduser() / filename
        try:
            if not fp.resolve().is_relative_to(_Path(self.mission_plans_dir).expanduser().resolve()):
                return None
        except OSError:
            return None
        if fp.exists():
            fp.unlink()
            return {"ok": True}
        return None

    def _send_mission_plan(self, payload: dict) -> dict:
        """Validate, serialize, and publish a plan. Returns {ok, yaml, warnings} or {errors}."""
        name = payload.get("name", "untitled")
        steps = payload.get("steps", [])
        errors = []
        for s in steps:
            errors.extend(validate_step(s))
        if errors:
            return {"ok": False, "errors": errors}
        yaml_str = steps_to_yaml(name, steps)
        msg = String(data=yaml_str)
        self.plan_pub.publish(msg)
        self.get_logger().info(f"Published mission plan '{name}' ({len(steps)} steps) to {self.get_parameter('mission_plan_topic').value}")
        return {"ok": True, "yaml": yaml_str, "warnings": lint_steps(steps)}

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
