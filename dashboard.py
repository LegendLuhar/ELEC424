#!/usr/bin/env python3
"""
dashboard.py  –  ELEC 424 Car Dashboard  (camera + lane-keeping control)
Run:  sudo python3 dashboard.py
Open: http://168.5.172.32:8080
"""

import cv2
import numpy as np
import threading
import json
import math
import time
import lgpio
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlparse, parse_qs

# ── GPIO / PWM constants ──────────────────────────────────────────────────────
GPIOCHIP      = 4
THROTTLE_PIN  = 18
STEERING_PIN  = 19
PWM_FREQ      = 50
DUTY_NEUTRAL  = 7.5
DUTY_FORWARD  = 8.1
DUTY_MAX      = 8.3
DUTY_LEFT     = 6.0
DUTY_RIGHT    = 9.0
Kp_steer      = 0.8
Kd_steer      = 0.2

# ── Red stop-box constants ────────────────────────────────────────────────────
RED_LOWER1 = np.array([0,   120,  70], dtype='uint8')
RED_UPPER1 = np.array([10,  255, 255], dtype='uint8')
RED_LOWER2 = np.array([160, 120,  70], dtype='uint8')
RED_UPPER2 = np.array([180, 255, 255], dtype='uint8')
STOP_AREA_THRESHOLD = 0.25
STOP_HOLD_FRAMES    = 30
STOP_COUNT_MAX      = 2

# ── Shared camera state ───────────────────────────────────────────────────────
_frame_lock   = threading.Lock()
_latest_frame = None

# ── Shared HSV tuning ─────────────────────────────────────────────────────────
_hsv_lock = threading.Lock()
_hsv = {'hmin': 80, 'hmax': 140, 'smin': 10, 'smax': 255, 'vmin': 40, 'vmax': 255}

# ── Shared telemetry (written by lane-keeping thread, read by /api/status) ───
_tele_lock = threading.Lock()
_tele = {
    'running': False,
    'blue_px': 0,
    'steer_angle': 90,
    'error': 0,
    'throttle_pct': DUTY_NEUTRAL,
    'stop_count': 0,
    'is_stopped': False,
    'frame': 0,
}

# ── Lane-keeping thread control ───────────────────────────────────────────────
_lk_thread  = None
_lk_stop_ev = threading.Event()


def _angle_to_duty(angle):
    offset = (angle - 90) * (1.5 / 45.0)
    return max(DUTY_LEFT, min(DUTY_RIGHT, DUTY_NEUTRAL + offset))


def _lane_keeping_loop():
    global _tele
    _lk_stop_ev.clear()

    h_gpio = lgpio.gpiochip_open(GPIOCHIP)
    lgpio.gpio_claim_output(h_gpio, THROTTLE_PIN, 0)
    lgpio.gpio_claim_output(h_gpio, STEERING_PIN, 0)
    lgpio.tx_pwm(h_gpio, THROTTLE_PIN, PWM_FREQ, DUTY_NEUTRAL)
    lgpio.tx_pwm(h_gpio, STEERING_PIN, PWM_FREQ, DUTY_NEUTRAL)
    print("ESC arming – 2 s…")
    time.sleep(2)

    last_error     = 0
    last_time      = time.time()
    stop_count     = 0
    stopped_frames = 0
    is_stopped     = False
    frame_num      = 0

    with _tele_lock:
        _tele['running'] = True

    try:
        while not _lk_stop_ev.is_set():
            with _frame_lock:
                frame = _latest_frame
            if frame is None:
                time.sleep(0.02)
                continue

            # ── Stop-box check every 3rd frame ───────────────────────────────
            if frame_num % 3 == 0:
                roi_start = frame.shape[0] // 2
                roi       = frame[roi_start:, :]
                hsv_r     = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                red_mask  = cv2.bitwise_or(
                    cv2.inRange(hsv_r, RED_LOWER1, RED_UPPER1),
                    cv2.inRange(hsv_r, RED_LOWER2, RED_UPPER2),
                )
                red_seen = cv2.countNonZero(red_mask) / (roi.shape[0] * roi.shape[1]) > STOP_AREA_THRESHOLD

                if red_seen and not is_stopped:
                    stop_count    += 1
                    is_stopped     = True
                    stopped_frames = 0
                    lgpio.tx_pwm(h_gpio, THROTTLE_PIN, PWM_FREQ, DUTY_NEUTRAL)
                    lgpio.tx_pwm(h_gpio, STEERING_PIN, PWM_FREQ, DUTY_NEUTRAL)
                    print(f"Stop #{stop_count}")
                    if stop_count >= STOP_COUNT_MAX:
                        print("Final stop – parked.")
                        break
                elif not red_seen and is_stopped:
                    stopped_frames += 1
                    if stopped_frames >= STOP_HOLD_FRAMES:
                        is_stopped = False

            if is_stopped:
                with _tele_lock:
                    _tele.update({'is_stopped': True, 'stop_count': stop_count, 'frame': frame_num})
                time.sleep(0.02)
                frame_num += 1
                continue

            # ── Blue centroid steering ────────────────────────────────────────
            with _hsv_lock:
                lo = np.array([_hsv['hmin'], _hsv['smin'], _hsv['vmin']], dtype='uint8')
                hi = np.array([_hsv['hmax'], _hsv['smax'], _hsv['vmax']], dtype='uint8')

            fh, fw = frame.shape[:2]
            roi_y  = 3 * fh // 4
            hsv_f  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask   = cv2.inRange(hsv_f[roi_y:, :], lo, hi)
            blue_px = int(cv2.countNonZero(mask))
            m      = cv2.moments(mask)

            if m['m00'] > 30:
                cx           = int(m['m10'] / m['m00'])
                err_px       = cx - fw // 2
                steer_angle  = max(45, min(135, 90 - int(err_px * 45.0 / (fw // 2))))
            else:
                steer_angle = 90

            # PD controller
            now   = time.time()
            dt    = max(now - last_time, 1e-6)
            error = steer_angle - 90
            pd    = Kp_steer * error + Kd_steer * (error - last_error) / dt
            steer_duty = _angle_to_duty(90 + pd)
            lgpio.tx_pwm(h_gpio, STEERING_PIN, PWM_FREQ, steer_duty)

            # Speed with turn slow-down
            turn_factor   = max(0.0, 1.0 - abs(error) / 60.0)
            throttle_duty = DUTY_NEUTRAL + (DUTY_FORWARD - DUTY_NEUTRAL) * turn_factor
            throttle_duty = max(DUTY_NEUTRAL, min(DUTY_MAX, throttle_duty))
            lgpio.tx_pwm(h_gpio, THROTTLE_PIN, PWM_FREQ, throttle_duty)

            last_error = error
            last_time  = now

            with _tele_lock:
                _tele.update({
                    'blue_px':      blue_px,
                    'steer_angle':  steer_angle,
                    'error':        error,
                    'throttle_pct': round(throttle_duty, 3),
                    'stop_count':   stop_count,
                    'is_stopped':   False,
                    'frame':        frame_num,
                })
            frame_num += 1

    finally:
        lgpio.tx_pwm(h_gpio, THROTTLE_PIN, PWM_FREQ, DUTY_NEUTRAL)
        lgpio.tx_pwm(h_gpio, STEERING_PIN,  PWM_FREQ, DUTY_NEUTRAL)
        time.sleep(0.2)
        lgpio.tx_pwm(h_gpio, THROTTLE_PIN, 0, 0)
        lgpio.tx_pwm(h_gpio, STEERING_PIN,  0, 0)
        lgpio.gpiochip_close(h_gpio)
        with _tele_lock:
            _tele['running'] = False
        print("Lane keeping stopped – motors neutral.")


# ── Frame processing for stream ───────────────────────────────────────────────

def _process(frame, mode):
    with _hsv_lock:
        lo = np.array([_hsv['hmin'], _hsv['smin'], _hsv['vmin']], dtype='uint8')
        hi = np.array([_hsv['hmax'], _hsv['smax'], _hsv['vmax']], dtype='uint8')

    fh, fw = frame.shape[:2]
    roi_y  = 3 * fh // 4
    hsv    = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask   = cv2.inRange(hsv[roi_y:, :], lo, hi)
    m      = cv2.moments(mask)

    if mode == 'raw':
        return frame

    colored = np.zeros_like(frame)
    colored[roi_y:, :][mask > 0] = [255, 120, 20]

    if mode == 'mask':
        return cv2.addWeighted(frame, 0.2, colored, 0.8, 0)

    # overlay
    out = cv2.addWeighted(frame.copy(), 0.6, colored, 0.4, 0)
    cv2.line(out, (0, roi_y), (fw, roi_y), (0, 220, 180), 1)
    if m['m00'] > 30:
        cx = int(m['m10'] / m['m00'])
        cy = roi_y + mask.shape[0] // 2
        cv2.circle(out, (cx, cy), 8, (0, 255, 100), -1)
        cv2.line(out, (fw // 2, roi_y), (cx, cy), (0, 255, 100), 2)

    with _tele_lock:
        t = dict(_tele)
    status = 'RUNNING' if t['running'] else 'IDLE'
    color  = (0, 255, 80) if t['running'] else (100, 100, 200)
    cv2.putText(out, f"{status}  Blue:{cv2.countNonZero(mask)}px  Steer:{t['steer_angle']}deg",
                (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
    return out


# ── HTML ──────────────────────────────────────────────────────────────────────

HTML = """\
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ELEC 424 Dashboard</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:#080812;color:#c8c8ff;font-family:'Segoe UI',system-ui,sans-serif;min-height:100vh;display:flex;flex-direction:column}
  header{background:#10102a;border-bottom:2px solid #3333cc;padding:14px 24px;display:flex;align-items:center;justify-content:space-between}
  .logo{display:flex;align-items:center;gap:12px}
  .dot{width:10px;height:10px;border-radius:50%;background:#4444ff;box-shadow:0 0 8px #4444ff;animation:pulse 1.5s infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
  header h1{font-size:1.05rem;letter-spacing:3px;color:#8888ff;text-transform:uppercase}
  .status-pill{padding:5px 14px;border-radius:20px;font-size:11px;letter-spacing:2px;font-weight:700;border:1px solid #333;transition:all .3s}
  .status-idle{background:#0a0a1e;color:#444488;border-color:#222244}
  .status-run{background:#0a2a0a;color:#33ff66;border-color:#226633;box-shadow:0 0 12px #226633}
  .status-stop{background:#2a0a0a;color:#ff4444;border-color:#662222}

  .main{display:grid;grid-template-columns:1fr 310px;gap:16px;padding:16px;flex:1}
  .cam-wrap{background:#0c0c20;border-radius:14px;overflow:hidden;position:relative}
  .cam-wrap img{width:100%;display:block}
  .cam-badge{position:absolute;top:12px;left:12px;background:rgba(0,0,0,.7);border:1px solid #3333cc;padding:3px 10px;border-radius:20px;font-size:11px;letter-spacing:2px;color:#6666ff}
  #pxbadge{position:absolute;top:12px;right:12px;background:rgba(0,0,0,.7);padding:3px 10px;border-radius:20px;font-size:11px;color:#44ff88;border:1px solid #226644}

  .panel{display:flex;flex-direction:column;gap:12px;overflow-y:auto}
  .card{background:#0e0e24;border:1px solid #222244;border-radius:12px;padding:16px}
  .card-title{font-size:10px;letter-spacing:2.5px;text-transform:uppercase;color:#4444aa;margin-bottom:12px}

  /* Run controls */
  .run-row{display:flex;gap:8px}
  .rbtn{flex:1;padding:14px 8px;border:none;border-radius:10px;font-size:12px;letter-spacing:2px;font-weight:700;cursor:pointer;transition:all .2s}
  .rbtn-run{background:#0d2a0d;color:#33ff66;border:1px solid #226633}
  .rbtn-run:hover{background:#1a4a1a;box-shadow:0 0 14px #22662244}
  .rbtn-stop{background:#2a0d0d;color:#ff4444;border:1px solid #662222}
  .rbtn-stop:hover{background:#4a1a1a}
  .rbtn:disabled{opacity:.35;cursor:not-allowed}

  /* Telemetry */
  .tele-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
  .tele-cell{background:#0a0a1e;border-radius:8px;padding:10px;text-align:center}
  .tele-val{font-size:1.4rem;font-weight:700;color:#8888ff}
  .tele-lbl{font-size:10px;letter-spacing:1.5px;color:#4444aa;margin-top:2px}
  .tele-cell.green .tele-val{color:#33ff88}
  .tele-cell.orange .tele-val{color:#ffaa33}

  /* Mode buttons */
  .mode-row{display:flex;gap:8px}
  .mbtn{flex:1;padding:8px 4px;border:1px solid #222255;border-radius:8px;background:#0a0a1e;color:#6666bb;font-size:11px;letter-spacing:1px;cursor:pointer;transition:all .2s}
  .mbtn.active,.mbtn:hover{background:#2222aa;border-color:#4444ff;color:#fff}

  /* HSV sliders */
  .hsv-row{display:grid;grid-template-columns:44px 1fr 34px;align-items:center;gap:8px;margin-bottom:9px}
  .hsv-row label{font-size:11px;color:#6666aa}
  input[type=range]{width:100%;accent-color:#4444ff;cursor:pointer}
  .hsv-row span{font-size:11px;color:#aaaaee;text-align:right}
  .hsv-divider{height:1px;background:#1a1a3a;margin:8px 0}

  /* Meter */
  .meter-track{height:8px;background:#0a0a1e;border-radius:4px;overflow:hidden;margin:8px 0 4px}
  .meter-fill{height:100%;width:0%;background:linear-gradient(90deg,#2222ff,#22ccff);border-radius:4px;transition:width .3s}
  .meter-labels{display:flex;justify-content:space-between;font-size:10px;color:#3333aa}

  /* Snapshot */
  .snap-btn{width:100%;padding:11px;border:none;border-radius:9px;font-size:11px;letter-spacing:2px;font-weight:700;cursor:pointer;background:#0d2a1a;color:#33ff99;border:1px solid #226644;transition:all .2s}
  .snap-btn:hover{background:#1a4a2a}
  .snap-msg{font-size:11px;color:#33ff99;min-height:16px;text-align:center;margin-top:6px}

  /* Gauge */
  .gauge-wrap{display:flex;justify-content:center}
  svg.gauge text{font-family:'Segoe UI',sans-serif}
</style>
</head>
<body>
<header>
  <div class="logo"><div class="dot"></div><h1>ELEC 424 &nbsp;|&nbsp; Car Dashboard</h1></div>
  <div class="status-pill status-idle" id="status-pill">IDLE</div>
</header>
<div class="main">
  <!-- Camera -->
  <div class="cam-wrap">
    <div class="cam-badge">&#11044; LIVE</div>
    <div id="pxbadge">Blue: — px</div>
    <img id="feed" src="/stream?mode=overlay" alt="stream">
  </div>

  <!-- Panel -->
  <div class="panel">

    <!-- Run controls -->
    <div class="card">
      <div class="card-title">Car Control</div>
      <div class="run-row">
        <button class="rbtn rbtn-run"  id="btn-run"  onclick="runCar()">&#9654; RUN</button>
        <button class="rbtn rbtn-stop" id="btn-stop" onclick="stopCar()" disabled>&#9646;&#9646; STOP</button>
      </div>
    </div>

    <!-- Telemetry -->
    <div class="card">
      <div class="card-title">Telemetry</div>
      <div class="tele-grid">
        <div class="tele-cell green">
          <div class="tele-val" id="t-steer">90°</div>
          <div class="tele-lbl">STEER ANGLE</div>
        </div>
        <div class="tele-cell orange">
          <div class="tele-val" id="t-throttle">—</div>
          <div class="tele-lbl">THROTTLE %</div>
        </div>
        <div class="tele-cell">
          <div class="tele-val" id="t-frame">0</div>
          <div class="tele-lbl">FRAME</div>
        </div>
        <div class="tele-cell">
          <div class="tele-val" id="t-stops">0</div>
          <div class="tele-lbl">STOPS</div>
        </div>
      </div>
    </div>

    <!-- Steering gauge -->
    <div class="card">
      <div class="card-title">Steering</div>
      <div class="gauge-wrap">
        <svg class="gauge" width="200" height="110" viewBox="0 0 200 110">
          <path d="M15,105 A85,85 0 0,1 185,105" fill="none" stroke="#1a1a3a" stroke-width="12" stroke-linecap="round"/>
          <path id="gauge-arc" d="M15,105 A85,85 0 0,1 100,20" fill="none" stroke="#3333ff" stroke-width="12" stroke-linecap="round"/>
          <line id="needle" x1="100" y1="105" x2="100" y2="24" stroke="#fff" stroke-width="2.5" stroke-linecap="round"/>
          <circle cx="100" cy="105" r="5" fill="#4444ff"/>
          <text x="4"   y="112" fill="#445" font-size="11">L</text>
          <text x="182" y="112" fill="#445" font-size="11">R</text>
          <text id="gauge-txt" x="100" y="92" fill="#8888ff" font-size="12" text-anchor="middle">90°</text>
        </svg>
      </div>
    </div>

    <!-- HSV Tuner -->
    <div class="card">
      <div class="card-title">HSV Blue Tuner</div>
      <div class="hsv-row"><label>H min</label><input type="range" min="0" max="180" value="80"  id="hmin" oninput="syncHSV(this,'hmin_v')"><span id="hmin_v">80</span></div>
      <div class="hsv-row"><label>H max</label><input type="range" min="0" max="180" value="140" id="hmax" oninput="syncHSV(this,'hmax_v')"><span id="hmax_v">140</span></div>
      <div class="hsv-divider"></div>
      <div class="hsv-row"><label>S min</label><input type="range" min="0" max="255" value="10"  id="smin" oninput="syncHSV(this,'smin_v')"><span id="smin_v">10</span></div>
      <div class="hsv-row"><label>S max</label><input type="range" min="0" max="255" value="255" id="smax" oninput="syncHSV(this,'smax_v')"><span id="smax_v">255</span></div>
      <div class="hsv-divider"></div>
      <div class="hsv-row"><label>V min</label><input type="range" min="0" max="255" value="40"  id="vmin" oninput="syncHSV(this,'vmin_v')"><span id="vmin_v">40</span></div>
      <div class="hsv-row"><label>V max</label><input type="range" min="0" max="255" value="255" id="vmax" oninput="syncHSV(this,'vmax_v')"><span id="vmax_v">255</span></div>
      <div class="meter-track"><div class="meter-fill" id="meter"></div></div>
      <div class="meter-labels"><span>0</span><span id="pxcount">— px</span><span>5000+</span></div>
    </div>

    <!-- Snapshot -->
    <div class="card">
      <div class="card-title">Snapshot</div>
      <button class="snap-btn" onclick="snap()">&#9654; CAPTURE FRAME</button>
      <div class="snap-msg" id="snap-msg"></div>
    </div>

  </div>
</div>
<script>
function runCar() {
  fetch('/api/run', {method:'POST'}).then(r=>r.json()).then(d=>{
    if(d.ok){ setRunning(true); }
  });
}
function stopCar() {
  fetch('/api/stop', {method:'POST'}).then(r=>r.json()).then(d=>{
    setRunning(false);
  });
}
function setRunning(on) {
  document.getElementById('btn-run').disabled  = on;
  document.getElementById('btn-stop').disabled = !on;
  const pill = document.getElementById('status-pill');
  pill.className = 'status-pill ' + (on ? 'status-run' : 'status-idle');
  pill.textContent = on ? 'RUNNING' : 'IDLE';
}

let currentMode = 'overlay';
function setMode(m) {
  currentMode = m;
  document.getElementById('feed').src = '/stream?mode='+m+'&_='+Date.now();
  ['raw','mask','overlay'].forEach(x=>document.getElementById('m-'+x).classList.remove('active'));
  document.getElementById('m-'+m).classList.add('active');
}

let hsvTimer=null;
function syncHSV(el,lbl){
  document.getElementById(lbl).textContent=el.value;
  clearTimeout(hsvTimer);
  hsvTimer=setTimeout(pushHSV,120);
}
function pushHSV(){
  const v={};
  ['hmin','hmax','smin','smax','vmin','vmax'].forEach(k=>v[k]=+document.getElementById(k).value);
  fetch('/api/hsv',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(v)});
}

function snap(){
  fetch('/api/snap').then(r=>r.json()).then(d=>{
    const m=document.getElementById('snap-msg');
    m.textContent=d.ok?'Saved → /tmp/snap.jpg':'Failed';
    setTimeout(()=>m.textContent='',3000);
  });
}

function polarXY(cx,cy,r,deg){
  const rad=(deg-90)*Math.PI/180;
  return [cx+r*Math.cos(rad),cy+r*Math.sin(rad)];
}
function updateGauge(angle){
  const[nx,ny]=polarXY(100,105,78,(angle-90));
  document.getElementById('needle').setAttribute('x2',nx.toFixed(1));
  document.getElementById('needle').setAttribute('y2',ny.toFixed(1));
  document.getElementById('gauge-txt').textContent=angle+'°';
  const dev=Math.abs(angle-90);
  const rr=Math.round(Math.min(255,dev*5));
  const bb=Math.round(Math.max(50,255-dev*5));
  document.getElementById('needle').setAttribute('stroke',`rgb(${rr},120,${bb})`);
}

function pollStatus(){
  fetch('/api/status').then(r=>r.json()).then(d=>{
    document.getElementById('pxbadge').textContent='Blue: '+d.blue_px+' px';
    document.getElementById('pxcount').textContent=d.blue_px+' px';
    document.getElementById('meter').style.width=Math.min(100,d.blue_px/50)+'%';
    document.getElementById('t-steer').textContent=d.steer_angle+'°';
    document.getElementById('t-throttle').textContent=d.throttle_pct?d.throttle_pct.toFixed(2)+'%':'—';
    document.getElementById('t-frame').textContent=d.frame;
    document.getElementById('t-stops').textContent=d.stop_count;
    updateGauge(d.steer_angle||90);
    setRunning(d.running);
  }).catch(()=>{});
  setTimeout(pollStatus,400);
}
pollStatus();
</script>
</body>
</html>
"""


# ── HTTP handler ──────────────────────────────────────────────────────────────

class _Handler(BaseHTTPRequestHandler):

    def do_GET(self):
        p = urlparse(self.path)
        qs = parse_qs(p.query)
        if p.path == '/':
            self._send(200, 'text/html; charset=utf-8', HTML.encode())
        elif p.path == '/stream':
            self._stream(qs.get('mode', ['overlay'])[0])
        elif p.path == '/api/status':
            with _tele_lock:
                data = dict(_tele)
            self._json(data)
        elif p.path == '/api/snap':
            with _frame_lock:
                f = _latest_frame
            if f is not None:
                cv2.imwrite('/tmp/snap.jpg', f)
                self._json({'ok': True})
            else:
                self._json({'ok': False})
        else:
            self.send_error(404)

    def do_POST(self):
        global _lk_thread, _lk_stop_ev
        p = urlparse(self.path).path
        n = int(self.headers.get('Content-Length', 0))

        if p == '/api/hsv':
            data = json.loads(self.rfile.read(n)) if n else {}
            with _hsv_lock:
                for k in _hsv:
                    if k in data:
                        _hsv[k] = int(data[k])
            self._json({'ok': True})

        elif p == '/api/run':
            if _lk_thread is None or not _lk_thread.is_alive():
                _lk_thread = threading.Thread(target=_lane_keeping_loop, daemon=True)
                _lk_thread.start()
                self._json({'ok': True})
            else:
                self._json({'ok': False, 'msg': 'already running'})

        elif p == '/api/stop':
            _lk_stop_ev.set()
            self._json({'ok': True})

        else:
            self.send_error(404)

    def _stream(self, mode):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        while True:
            with _frame_lock:
                f = _latest_frame
            if f is None:
                time.sleep(0.05)
                continue
            out = _process(f, mode)
            _, jpg = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, 80])
            try:
                self.wfile.write(
                    b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'
                    + jpg.tobytes() + b'\r\n'
                )
            except Exception:
                break

    def _send(self, code, ctype, body):
        self.send_response(code)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', len(body))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, data):
        self._send(200, 'application/json', json.dumps(data).encode())

    def log_message(self, *_):
        pass


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    global _latest_frame

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    server = HTTPServer(('0.0.0.0', 8080), _Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print('Dashboard ready → http://168.5.172.32:8080')

    while True:
        ret, frame = cap.read()
        if ret:
            with _frame_lock:
                _latest_frame = frame


if __name__ == '__main__':
    main()
