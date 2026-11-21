#!/usr/bin/env python3
"""Spark browser keyboard bridge (run ON THE ROBOT; no Windows Python needed).

Works with the existing body.py JSON/TCP protocol on 127.0.0.1:8765.
It does not publish ROS topics, start motors, or control the pump itself.

Robot terminal 1: python3 ~/semi/body.py
Robot terminal 2: python3 ~/semi/keyboard_web.py
Windows PowerShell: ssh -N -L 8766:127.0.0.1:8766 spark@10.25.141.8
Windows browser:   http://127.0.0.1:8766/

The browser must remain visible/focused while controlling the robot. The page
starts DISARMED; click Enable keyboard to control. Key-up sends an immediate
zero state. If the browser stops sending updates, body.py's 0.25s watchdog
stops the base and arm. Space and the red STOP button stop motion; neither
releases the vacuum pump. The physical emergency stop is still essential.

The HTTP service is bound to loopback only, uses a per-process token, and
requires an explicit controller session, protecting against accidental remote
requests. Do not expose or port-forward this service to the public internet.
"""

import json
import os
import secrets
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

WEB_HOST = "127.0.0.1"
WEB_PORT = int(os.environ.get("SPARK_WEB_PORT", "8766"))
BODY_HOST = "127.0.0.1"
BODY_PORT = int(os.environ.get("BODY_PORT", "8765"))

TOKEN = secrets.token_urlsafe(32)

# Browser-side: use real KEYUP/KEYDOWN, not the SSH terminal's repeat stream.
# This HTML is deliberately embedded so only ONE new robot-side file is needed.
PAGE = r'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Spark Robot — Keyboard</title>
<style>
:root{color-scheme:dark;font-family:system-ui,Segoe UI,Arial,sans-serif}
body{margin:0;background:#111827;color:#f3f4f6;padding:22px;max-width:760px}
h1{font-size:1.5rem;margin:0 0 12px}
p{color:#cbd5e1;line-height:1.5}
button{padding:13px 20px;margin:4px 8px 8px 0;border-radius:9px;border:0;font-size:1rem;font-weight:700;cursor:pointer}
#arm{background:#22c55e;color:#052e16}#disarm{background:#64748b;color:white}
#stop{background:#ef4444;color:white}#shutdown{background:#475569;color:white}
#status{font-weight:bold;margin:10px 0;padding:12px;border-radius:8px;background:#243147}
#status[data-state="ready"]{background:#14532d}#status[data-state="error"]{background:#7f1d1d}
.controls{display:grid;grid-template-columns:repeat(auto-fit,minmax(240px,1fr));gap:12px;margin-top:14px}
section{border:1px solid #334155;padding:12px;border-radius:10px}
h2{font-size:1rem;margin:0 0 9px}code{font-weight:bold;color:#bae6fd}
small{color:#cbd5e1}strong{color:#fbbf24}
</style>
</head>
<body>
<h1>Spark Robot — Browser Keyboard</h1>
<p>No Python or pygame is needed on Windows. Keep this browser tab <strong>focused</strong> while controlling the robot.</p>
<div id="status">DISARMED — movement is disabled</div>
<button id="arm" type="button">Enable keyboard</button>
<button id="disarm" type="button">Disable keyboard</button>
<button id="stop" type="button">STOP movement</button>
<div class="controls">
<section><h2>Base</h2><div><code>W/S</code> forward / backward</div><div><code>A/D</code> left / right</div><div>Arrow keys also work. Hold W+A or W+D for an arc.</div><div><code>Q</code> FAST / SLOW</div></section>
<section><h2>Arm and wrist</h2><div><code>8 / 2</code> Z up / down</div><div><code>7 / 9</code> X forward / back</div><div><code>4 / 6</code> Y left / right</div><div><code>1 / 3</code> wrist rotation</div></section>
<section><h2>Presets and vacuum</h2><div><code>H</code> home, <code>V</code> hover</div><div><code>P</code> pick, <code>C</code> carry</div><div><code>Enter</code> pump on/off</div><div><code>0</code> pump off + home</div></section>
<section><h2>Safety</h2><div><code>Space</code> software emergency stop (pump unchanged)</div><div><code>Esc</code> shut down body.py (pump off)</div><div>Switching tabs or losing focus <strong>disables control</strong>.</div></section>
</div>
<p><small>Software controls are not a physical emergency stop. Test at low speeds with the robot clear of people and obstacles. Do not run other base/arm teleop scripts simultaneously.</small></p>
<script>
'use strict';
const TOKEN = __TOKEN_JSON__;
let session = null;
let armed = false;
let pressed = new Set();
let seq = 0;
let failed = false;
const statusEl = document.getElementById('status');
const moveCodes = new Set(['KeyW','KeyS','KeyA','KeyD','ArrowUp','ArrowDown','ArrowLeft','ArrowRight',
 'Digit8','Digit2','Digit7','Digit9','Digit4','Digit6','Digit1','Digit3',
 'Numpad8','Numpad2','Numpad7','Numpad9','Numpad4','Numpad6','Numpad1','Numpad3']);
const actions = new Map([['KeyQ','q'],['Enter','enter'],['NumpadEnter','enter'],
 ['KeyH','h'],['KeyV','v'],['KeyP','p'],['KeyC','c'],['Digit0','0'],['Numpad0','0']]);
const tracked = new Set([...moveCodes,...actions.keys(),'Space','Escape']);
function show(message, state='off'){statusEl.textContent=message;statusEl.dataset.state=state;}
function freshId(){
 if (window.crypto && crypto.randomUUID) return crypto.randomUUID();
 return String(Date.now())+'-'+Math.random().toString(36).slice(2);
}
function stateData(){
 const has=(...codes)=>codes.some(c=>pressed.has(c));
 const linear=Number(has('KeyW','ArrowUp'))-Number(has('KeyS','ArrowDown'));
 const turn=Number(has('KeyA','ArrowLeft'))-Number(has('KeyD','ArrowRight'));
 const armKeys=[
   [['Digit8','Numpad8'],'z+'],[['Digit2','Numpad2'],'z-'],
   [['Digit7','Numpad7'],'x+'],[['Digit9','Numpad9'],'x-'],
   [['Digit4','Numpad4'],'y+'],[['Digit6','Numpad6'],'y-'],
   [['Digit1','Numpad1'],'angle+'],[['Digit3','Numpad3'],'angle-']];
 let axis=null;
 for(const [keys, value] of armKeys){if(has(...keys)){axis=value;break;}}
 return {kind:'state',linear,turn,arm:axis,stop:pressed.has('Space')};
}
async function post(data, which=session){
 if(!which)return;
 const payload={session:which,seq:++seq,...data};
 try {
   const controller = new AbortController();
   const timeout = setTimeout(()=>controller.abort(),1000);
   let response;
   try {
     response=await fetch('/control',{method:'POST',cache:'no-store',
       headers:{'Content-Type':'application/json','X-Spark-Token':TOKEN},
       body:JSON.stringify(payload),signal:controller.signal});
   } finally {clearTimeout(timeout);}
   if(!response.ok)throw Error('HTTP '+response.status);
   failed=false;
   return true;
 } catch(err) {
   if(which === session && armed){
     failed=true; disarm('Connection lost — keyboard disabled');
     show('Connection lost — keyboard disabled','error');
   }
   return false;
 }
}
function sendState(){if(armed && document.hasFocus() && !document.hidden)post(stateData());}
async function enable(){
 if(armed || session)return;
 if(!document.hasFocus() || document.hidden){show('Focus this page first');return;}
 pressed.clear();seq=0;session=freshId();failed=false;
 const openingSession=session;
 show('Connecting to body.py — keyboard disabled');
 // The first *neutral* state clears body.py's safety interlock.
 // Awaiting its TCP write also prevents the initial W key from outrunning it.
 const connected=await post({kind:'state',linear:0,turn:0,arm:null,stop:false},openingSession);
 if(session!==openingSession)return;
 if(!connected){disarm('Could not connect to body.py — keyboard disabled');return;}
 if(!document.hasFocus() || document.hidden){disarm();return;}
 armed=true;
 show('KEYBOARD ENABLED — hold W/A/S/D to move','ready');
 sendState();
}
function disarm(reason='DISARMED — movement disabled'){
 const old=session;
 armed=false;pressed.clear();session=null;
 show(reason);
 if(old){
   // Revoke the old session server-side and close its TCP socket to body.py.
   fetch('/release',{method:'POST',cache:'no-store',keepalive:true,
     headers:{'Content-Type':'application/json','X-Spark-Token':TOKEN},
     body:JSON.stringify({session:old})}).catch(()=>{});
 }
}
function emergency(){
 if(!armed)return;
 pressed.clear();
 post({kind:'action',key:'space'});
 post({kind:'state',linear:0,turn:0,arm:null,stop:true});
 // Disable keyboard until explicitly re-enabled; release disconnects and stops.
 disarm('STOPPED — click Enable keyboard to resume');
}
function keyDown(e){
 if(tracked.has(e.code))e.preventDefault();
 if(!armed || !document.hasFocus() || document.hidden)return;
 if(e.repeat || pressed.has(e.code))return;
 pressed.add(e.code);
 if(e.code==='Space'){emergency();return;}
 if(e.code==='Escape'){
   // Wait for acknowledgement before disconnecting: otherwise the shutdown
   // action could be overtaken by /release and never reach body.py.
   const old=session;
   armed=false;pressed.clear();
   show('Shutdown requested — waiting for body.py');
   post({kind:'action',key:'exit'},old)
     .finally(()=>disarm('Shutdown requested — body.py should exit'));
   return;
 }
 const action=actions.get(e.code);
 if(action){post({kind:'action',key:action});return;}
 sendState();
}
function keyUp(e){
 if(tracked.has(e.code))e.preventDefault();
 pressed.delete(e.code);
 sendState(); // critical: send release immediately, not after terminal repeat delay
}
document.getElementById('arm').addEventListener('click',enable);
document.getElementById('disarm').addEventListener('click',()=>disarm());
document.getElementById('stop').addEventListener('click',emergency);
window.addEventListener('keydown',keyDown);
window.addEventListener('keyup',keyUp);
window.addEventListener('blur',()=>disarm('DISARMED — window lost focus'));
document.addEventListener('visibilitychange',()=>{if(document.hidden)disarm('DISARMED — tab hidden');});
window.addEventListener('pagehide',()=>disarm());
setInterval(sendState,50); // 20Hz heartbeat; body.py handles motor smoothing
</script>
</body>
</html>'''.replace('__TOKEN_JSON__', json.dumps(TOKEN))


class BodyBridge:
    """Exclusive, sequence-checked browser-session bridge to body.py TCP input."""

    def __init__(self):
        self.lock = threading.RLock()
        self.sock = None
        self.owner = None
        self.last_seen = 0.0
        self.last_seq = -1
        self.retired = {}  # sessions invalidated on blur/disable

    def _close_socket(self):
        sock, self.sock = self.sock, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def _retire(self, session):
        self.retired[session] = time.monotonic()
        cutoff = time.monotonic() - 60
        self.retired = {k: t for k, t in self.retired.items() if t > cutoff}

    def release(self, session):
        with self.lock:
            self._retire(session)
            if self.owner == session:
                self._close_socket()  # body.py disconnect = immediate stop
                self.owner = None
                self.last_seq = -1
                self.last_seen = 0.0

    def send(self, session, seq, packet):
        with self.lock:
            now = time.monotonic()
            if session in self.retired:
                return 409, 'This controller session is no longer active'
            if self.owner != session:
                if self.owner is not None and now - self.last_seen < 0.8:
                    return 409, 'Another browser currently owns the controller'
                self._close_socket()
                self.owner = session
                self.last_seq = -1
            if seq <= self.last_seq:
                return 200, 'Stale packet ignored'
            self.last_seq = seq
            self.last_seen = now
            if self.sock is None:
                try:
                    self.sock = socket.create_connection((BODY_HOST, BODY_PORT), timeout=0.5)
                    self.sock.settimeout(0.5)
                    self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                except OSError:
                    self._close_socket()
                    self._retire(session)
                    self.owner = None
                    return 503, 'body.py is not listening on port %d' % BODY_PORT
            try:
                encoded = json.dumps(packet, separators=(',', ':')).encode('ascii') + b'\n'
                self.sock.sendall(encoded)
                return 200, 'OK'
            except (OSError, ValueError):
                self._close_socket()
                self._retire(session)
                self.owner = None
                return 503, 'Connection to body.py was lost'

    def sweep(self):
        with self.lock:
            if self.owner is not None and time.monotonic() - self.last_seen > 0.8:
                self.release(self.owner)


BRIDGE = BodyBridge()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return  # Avoid 20 requests/s flooding the console

    def respond(self, code, data, content_type='text/plain; charset=utf-8'):
        raw = data.encode('utf-8') if isinstance(data, str) else data
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(raw)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Referrer-Policy', 'no-referrer')
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        if self.path != '/':
            return self.respond(404, 'Not found')
        self.respond(200, PAGE, 'text/html; charset=utf-8')

    def do_POST(self):
        if self.path not in ('/control', '/release'):
            return self.respond(404, 'Not found')
        if self.headers.get('X-Spark-Token') != TOKEN:
            return self.respond(403, 'Forbidden')
        if self.headers.get('Content-Type', '').split(';')[0].strip() != 'application/json':
            return self.respond(415, 'JSON required')
        try:
            length = int(self.headers.get('Content-Length', '-1'))
            if length < 2 or length > 2048:
                return self.respond(413, 'Invalid length')
            data = json.loads(self.rfile.read(length))
        except (ValueError, UnicodeDecodeError):
            return self.respond(400, 'Invalid JSON')
        if not isinstance(data, dict):
            return self.respond(400, 'Invalid payload')
        session = data.get('session')
        if not isinstance(session, str) or not (12 <= len(session) <= 100):
            return self.respond(400, 'Invalid session')
        if self.path == '/release':
            BRIDGE.release(session)
            return self.respond(200, 'Released')
        seq, kind = data.get('seq'), data.get('kind')
        if type(seq) is not int or seq < 1 or kind not in ('state', 'action'):
            return self.respond(400, 'Invalid sequence or kind')
        if kind == 'state':
            lin, turn, arm, stop = (data.get('linear'), data.get('turn'), data.get('arm'), data.get('stop'))
            if (type(lin) is not int or lin not in (-1, 0, 1) or
                    type(turn) is not int or turn not in (-1, 0, 1) or
                    (arm is not None and arm not in ('x+', 'x-', 'y+', 'y-', 'z+', 'z-', 'angle+', 'angle-')) or
                    type(stop) is not bool):
                return self.respond(400, 'Invalid movement state')
            packet = {'kind': 'state', 'linear': lin, 'turn': turn, 'arm': arm, 'stop': stop}
        else:
            key = data.get('key')
            if key not in ('space', 'q', 'enter', 'h', 'v', 'p', 'c', '0', 'exit'):
                return self.respond(400, 'Invalid action')
            packet = {'kind': 'action', 'key': key}
        code, result = BRIDGE.send(session, seq, packet)
        return self.respond(code, result)


def main():
    httpd = ThreadingHTTPServer((WEB_HOST, WEB_PORT), Handler)
    httpd.daemon_threads = True
    print('Spark browser keyboard:', 'http://127.0.0.1:%d/' % WEB_PORT, flush=True)
    print('Run body.py first. Forward local port %d over SSH. No Windows Python required.' % WEB_PORT, flush=True)
    print('Browser starts DISARMED; click Enable keyboard. Ctrl+C stops this bridge.', flush=True)

    def monitor():
        while not stop_event.wait(0.1):
            BRIDGE.sweep()

    stop_event = threading.Event()
    worker = threading.Thread(target=monitor, daemon=True)
    worker.start()
    try:
        httpd.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        BRIDGE.release(BRIDGE.owner or 'shutdown')
        httpd.server_close()
        print('Keyboard bridge stopped', flush=True)


if __name__ == '__main__':
    main()

