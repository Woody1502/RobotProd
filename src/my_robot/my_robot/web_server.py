#!/usr/bin/env python3
"""
web_server.py — FastAPI web interface: camera feed, attachment control, gamepad toggle.

Endpoints:
  GET  /                       — mobile-friendly control UI
  GET  /video                  — MJPEG camera stream
  WS   /ws                     — live status (JSON)
  POST /cmd/{device}/{state}   — send Int8 command to attachment topic
  POST /cmd/estop              — stop everything immediately
  POST /gamepad/enabled/{0|1}  — enable/disable gamepad without disconnecting it
"""

import asyncio
import json
import threading

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, Float64MultiArray, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, StreamingResponse
import uvicorn

# ── Embedded UI ──────────────────────────────────────────────────────────────

_HTML = """\
<!doctype html>
<html lang="ru">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>FitoBot</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:#111;color:#ddd;font-family:system-ui,-apple-system,sans-serif;
     max-width:600px;margin:0 auto;min-height:100dvh}
header{display:flex;align-items:center;gap:8px;padding:8px 12px;
       background:#181818;border-bottom:1px solid #2a2a2a}
header h1{flex:1;font-size:15px;font-weight:600;letter-spacing:.3px}
.sbar{display:flex;gap:16px;padding:5px 12px;background:#141414;
      font-size:12px;color:#888;border-bottom:1px solid #222}
.cam-wrap{background:#000;position:relative;width:100%}
.cam-wrap img{width:100%;display:block}
.no-sig{position:absolute;inset:0;display:none;align-items:center;
        justify-content:center;color:#444;font-size:13px}
.grid{padding:8px;display:grid;grid-template-columns:1fr 1fr;gap:8px}
.card{background:#1c1c1c;border-radius:10px;padding:10px}
.card h3{font-size:10px;text-transform:uppercase;letter-spacing:.6px;
         color:#555;margin-bottom:8px}
.row{display:flex;gap:6px}
.btn{flex:1;height:54px;border:none;border-radius:8px;font-size:22px;
     background:#252525;color:#bbb;cursor:pointer;touch-action:none;
     user-select:none;transition:background .08s;-webkit-user-select:none}
.btn.on {background:#1a5c2e;color:#7fff9a}
.btn.red{background:#6b1c1c;color:#ffaaaa}
.btn.pressed{background:#363636}
.gbtn{font-size:12px;padding:0 12px;height:32px;flex:none;border-radius:16px;
      white-space:nowrap}
.full{grid-column:span 2}
.cross{display:grid;grid-template-columns:repeat(3,1fr);
       grid-template-rows:repeat(3,44px);gap:5px}
.gap{background:transparent}
</style>
</head>
<body>
<header>
  <h1>🤖 FitoBot</h1>
  <button id="btn-gp" class="btn gbtn on">Геймпад ВКЛ</button>
  <button id="btn-stop" class="btn gbtn red" style="margin-left:4px">⛔ СТОП</button>
</header>
<div class="sbar">
  <span>🎮 <span id="s-gp">—</span></span>
  <span>⚡ <span id="s-spd">0.0</span> m/s</span>
</div>
<div class="cam-wrap">
  <img id="cam" src="/video" alt="">
  <div class="no-sig" id="nosig">Нет сигнала камеры</div>
</div>
<div class="grid">
  <div class="card">
    <h3>Ковш</h3>
    <div class="row">
      <button class="btn" data-dev="bucket" data-on="1">▲</button>
      <button class="btn" data-dev="bucket" data-on="2">▼</button>
    </div>
  </div>
  <div class="card">
    <h3>Рама</h3>
    <div class="row">
      <button class="btn" data-dev="frame" data-on="1">▲</button>
      <button class="btn" data-dev="frame" data-on="2">▼</button>
    </div>
  </div>
  <div class="card">
    <h3>Бункер</h3>
    <div class="row">
      <button class="btn" data-dev="bunker" data-on="1">▲</button>
      <button class="btn" data-dev="bunker" data-on="2">▼</button>
    </div>
  </div>
  <div class="card">
    <h3>Сепаратор</h3>
    <button id="btn-sep" class="btn" style="width:100%">◉ Включить</button>
  </div>
  <div class="card full">
    <h3>Манипулятор</h3>
    <div class="cross">
      <div class="gap"></div>
      <button class="btn" data-dev="manipulator" data-on="1">▲</button>
      <div class="gap"></div>
      <button class="btn" data-dev="manipulator" data-on="3">◀</button>
      <div class="gap"></div>
      <button class="btn" data-dev="manipulator" data-on="4">▶</button>
      <div class="gap"></div>
      <button class="btn" data-dev="manipulator" data-on="2">▼</button>
      <div class="gap"></div>
    </div>
  </div>
</div>
<script>
const $=id=>document.getElementById(id);

// Live status via WebSocket
const ws=new WebSocket(`ws://${location.host}/ws`);
ws.onmessage=({data})=>{
  const d=JSON.parse(data);
  $('s-gp').textContent=d.gp_connected?'Подключён':'Нет связи';
  $('s-spd').textContent=(d.speed??0).toFixed(1);
};
ws.onclose=()=>setTimeout(()=>location.reload(),3000);

// Camera
$('cam').onerror=()=>{$('cam').style.display='none';$('nosig').style.display='flex'};
$('cam').onload =()=>{$('cam').style.display='';$('nosig').style.display='none'};

// Hold-to-run attachment buttons
document.querySelectorAll('[data-dev]').forEach(btn=>{
  const post=v=>fetch(`/cmd/${btn.dataset.dev}/${v}`,{method:'POST'});
  btn.addEventListener('pointerdown',e=>{e.preventDefault();btn.classList.add('pressed');post(btn.dataset.on)});
  btn.addEventListener('pointerup',  e=>{e.preventDefault();btn.classList.remove('pressed');post(0)});
  btn.addEventListener('pointercancel',()=>{btn.classList.remove('pressed');post(0)});
  btn.addEventListener('contextmenu',e=>e.preventDefault());
});

// Gamepad enable/disable toggle
let gpOn=true;
$('btn-gp').onclick=()=>{
  gpOn=!gpOn;
  fetch(`/gamepad/enabled/${gpOn?1:0}`,{method:'POST'});
  $('btn-gp').textContent=gpOn?'Геймпад ВКЛ':'Геймпад ВЫКЛ';
  $('btn-gp').classList.toggle('on',gpOn);
};

// Emergency stop
$('btn-stop').onclick=()=>fetch('/cmd/estop',{method:'POST'});

// Separator toggle
let sepOn=false;
$('btn-sep').onclick=()=>{
  sepOn=!sepOn;
  fetch(`/cmd/separator/${sepOn?2:0}`,{method:'POST'});
  $('btn-sep').textContent=sepOn?'◉ Выключить':'◉ Включить';
  $('btn-sep').classList.toggle('on',sepOn);
};
</script>
</body>
</html>"""

# ── ROS2 node ────────────────────────────────────────────────────────────────

class WebServerNode(Node):
    def __init__(self):
        super().__init__('web_server')

        _latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._pubs = {
            name: self.create_publisher(Int8, f'/vim/{name}', 10)
            for name in ('bucket', 'frame', 'bunker', 'flaps', 'manipulator', 'separator')
        }
        self._vel_pub   = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self._steer_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands',  10)
        self._gp_en_pub = self.create_publisher(Bool, '/joy_bridge/enabled', _latched)

        self.create_subscription(Bool, '/joy_bridge/connected', self._on_gp_conn, _latched)
        self.create_subscription(
            Float64MultiArray, '/velocity_controller/commands', self._on_vel, 10)
        self.create_subscription(Image, '/camera/depth/pure_image', self._on_image, 10)

        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_jpeg: bytes | None = None

        self._status = {'gp_connected': False, 'speed': 0.0}
        self._ws_clients: set[WebSocket] = set()
        self._loop: asyncio.AbstractEventLoop | None = None

    # ── ROS callbacks ────────────────────────────────────────────────────────

    def _on_gp_conn(self, msg: Bool):
        self._status['gp_connected'] = msg.data
        self._push_status()

    def _on_vel(self, msg: Float64MultiArray):
        if msg.data:
            self._status['speed'] = abs(float(msg.data[0]))
            self._push_status()

    def _on_image(self, msg: Image):
        try:
            enc = msg.encoding
            if enc in ('16UC1', '32FC1'):
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                frame = cv2.applyColorMap(frame, cv2.COLORMAP_JET)
            else:
                frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 72])
        with self._frame_lock:
            self._latest_jpeg = buf.tobytes()

    # ── Status push ──────────────────────────────────────────────────────────

    def _push_status(self):
        if self._loop is None:
            return
        payload = json.dumps(self._status)
        asyncio.run_coroutine_threadsafe(self._broadcast(payload), self._loop)

    async def _broadcast(self, payload: str):
        dead: set[WebSocket] = set()
        for ws in list(self._ws_clients):
            try:
                await ws.send_text(payload)
            except Exception:
                dead.add(ws)
        self._ws_clients -= dead

    # ── Commands ─────────────────────────────────────────────────────────────

    def cmd(self, device: str, state: int):
        if device in self._pubs:
            self._pubs[device].publish(Int8(data=state))

    def estop(self):
        for pub in self._pubs.values():
            pub.publish(Int8(data=0))
        z4 = Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0])
        z1 = Float64MultiArray(data=[0.0])
        self._vel_pub.publish(z4)
        self._steer_pub.publish(z1)

    def set_gamepad_enabled(self, enabled: bool):
        self._gp_en_pub.publish(Bool(data=enabled))


# ── FastAPI ──────────────────────────────────────────────────────────────────

app = FastAPI()
_node: WebServerNode | None = None


@app.on_event('startup')
async def _startup():
    _node._loop = asyncio.get_running_loop()


@app.get('/', response_class=HTMLResponse)
async def ui():
    return _HTML


@app.get('/video')
async def video():
    async def _gen():
        while True:
            with _node._frame_lock:
                jpeg = _node._latest_jpeg
            if jpeg:
                yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n'
            await asyncio.sleep(0.033)
    return StreamingResponse(_gen(), media_type='multipart/x-mixed-replace; boundary=frame')


@app.websocket('/ws')
async def ws_endpoint(ws: WebSocket):
    await ws.accept()
    _node._ws_clients.add(ws)
    # send current status immediately on connect
    await ws.send_text(json.dumps(_node._status))
    try:
        while True:
            await ws.receive_text()
    except (WebSocketDisconnect, Exception):
        pass
    finally:
        _node._ws_clients.discard(ws)


@app.post('/cmd/estop')
async def cmd_estop():
    _node.estop()
    return {'ok': True}


@app.post('/cmd/{device}/{state}')
async def cmd_device(device: str, state: int):
    _node.cmd(device, state)
    return {'ok': True}


@app.post('/gamepad/enabled/{value}')
async def gamepad_toggle(value: int):
    _node.set_gamepad_enabled(bool(value))
    return {'ok': True}


# ── Entry point ──────────────────────────────────────────────────────────────

def main(args=None):
    global _node
    rclpy.init(args=args)
    _node = WebServerNode()

    threading.Thread(target=lambda: rclpy.spin(_node), daemon=True).start()

    uvicorn.run(app, host='0.0.0.0', port=8080, log_level='warning')
