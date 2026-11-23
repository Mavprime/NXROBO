#!/usr/bin/env python3
"""Live browser view for the spatial simulation (Level 0B).

A tiny stdlib HTTP server: `/` serves a canvas page, `/state` returns the current world
snapshot as JSON, `/stop` asks the mission to stop. The page polls at ~8 Hz, so the
robot, lidar and cubes move in real time in a normal browser. No extra dependencies.

Run it via `run_live.py`; this module only provides the server and the page.
"""

from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable


HTML_TEMPLATE = r"""<!doctype html>
<html><head><meta charset="utf-8"><title>Spark sim</title>
<style>
  html,body{background:#0f1216;color:#e6e6e6;font-family:ui-monospace,Menlo,monospace;margin:0}
  #wrap{display:flex;align-items:flex-start}
  canvas{background:#0b0e12;display:block}
  #side{width:340px;padding:14px;font-size:12px;line-height:1.5}
  h3{margin:0 0 8px;font-size:12px;letter-spacing:.12em;color:#8fa3b8;font-weight:600}
  .row{display:flex;justify-content:space-between;border-bottom:1px solid #1d232b;padding:2px 0}
  .k{color:#8fa3b8}
  .big{font-size:20px;color:#7bd88f}
  table{width:100%;border-collapse:collapse;margin-top:6px}
  td{padding:2px 3px;border-bottom:1px solid #1a2027;vertical-align:top}
  .ok{color:#7bd88f}.bad{color:#ff6b6b}.warn{color:#ffd166}.muted{color:#5f6b78}
  button{margin-top:10px;width:100%;padding:8px;background:#3a1518;color:#ff9a9a;
         border:1px solid #7a2a30;border-radius:4px;font-family:inherit;font-size:13px;cursor:pointer}
  button:hover{background:#4a1a1e}
  #err{color:#ff6b6b;margin-top:8px}
</style></head><body>
<div id="wrap">
  <canvas id="c" width="760" height="760"></canvas>
  <div id="side">
    <h3>SPARK SIM — LEVEL 0B</h3>
    <div class="row"><span class="k">sim time</span><span id="t">0.0 s</span></div>
    <div class="row"><span class="k">clock left</span><span id="clock">-</span></div>
    <div class="row"><span class="k">state</span><span id="st">-</span></div>
    <div class="row"><span class="k">current</span><span id="cur">-</span></div>
    <div class="row"><span class="k">score</span><span id="score" class="big">0</span></div>
    <div class="row"><span class="k">collisions</span><span id="col">0</span></div>
    <div class="row"><span class="k">chassis cap</span><span id="chassis">-</span></div>
    <div class="row"><span class="k">velocity</span><span id="vel">-</span></div>
    <div class="row"><span class="k">playback</span><span id="spd">1x</span></div>
    <button onclick="fetch('/stop')">STOP MISSION</button>
    <div id="err"></div>
    <h3 style="margin-top:16px">STEPS</h3>
    <table id="steps"></table>
    <h3 style="margin-top:16px">LEGEND</h3>
    <div class="muted">brown=doll &nbsp; grey=clock &nbsp; yellow=banana &nbsp; pink=cake<br>
    red square=collection &nbsp; cyan=lidar &nbsp; green line=trail</div>
  </div>
</div>
<script>
const SC = __SCENARIO__;
const cv = document.getElementById('c'), cx = cv.getContext('2d');
const W = cv.width, H = cv.height;
const AR = SC.arena || {size:[3,3], origin:[0,0]};
const AX0 = AR.origin[0], AY0 = AR.origin[1], AW = AR.size[0], AH = AR.size[1];
const PAD = 0.18;
const S = Math.min(W,H) / (Math.max(AW,AH) + 2*PAD);
const X = x => (x - AX0 + PAD) * S;
const Y = y => H - (y - AY0 + PAD) * S;
const COL = {doll:'#8c5a33', clock:'#d9d9d9', banana:'#f2d91a', cake:'#e64c66'};

function rect(x0,y0,x1,y1,fill,stroke,lw){
  cx.beginPath(); cx.rect(X(x0), Y(y1), (x1-x0)*S, (y1-y0)*S);
  if(fill){cx.fillStyle=fill; cx.fill();}
  if(stroke){cx.strokeStyle=stroke; cx.lineWidth=lw||1; cx.stroke();}
}
function dot(x,y,r,fill,stroke){cx.beginPath();cx.arc(X(x),Y(y),r,0,7);cx.fillStyle=fill;cx.fill();
  if(stroke){cx.strokeStyle=stroke;cx.lineWidth=1;cx.stroke();}}

function drawStatic(){
  cx.clearRect(0,0,W,H);
  rect(AX0,AY0,AX0+AW,AY0+AH,'#11161c','#6f7c8a',2);
  for(const [z,info] of Object.entries(SC.zones)){
    const tc = info.transport_center, cc = info.collection_center;
    const fill = (z==='A'||z==='D') ? '#1d1b10' : '#101a13';
    rect(tc[0]-0.5,tc[1]-0.5,tc[0]+0.5,tc[1]+0.5,fill,'#232c36',1);
    rect(cc[0]-0.225,cc[1]-0.225,cc[0]+0.225,cc[1]+0.225,null,'#c9363f',1.6);
    cx.fillStyle='#586878'; cx.font='11px monospace'; cx.textAlign='center';
    cx.fillText('Zone '+z, X(tc[0]), Y(tc[1])+4);
  }
  const tb = SC.table;
  if(tb){
    const c = tb.center, s = tb.size;
    rect(c[0]-s[0]/2, c[1]-s[1]/2, c[0]+s[0]/2, c[1]+s[1]/2,
         '#2a3038','#4a5563',1);
    cx.fillStyle='#6b7887'; cx.font='10px monospace'; cx.textAlign='center';
    cx.fillText('table', X(c[0]), Y(c[1]+s[1]/2+0.10));
  }
  const bd = SC.board || {center:[0.165,1.5]};
  (SC.board_left_to_right||[]).forEach((ty,i)=>{
    const y = (bd.cards_y && bd.cards_y[i] !== undefined) ? bd.cards_y[i]
              : bd.center[1] + (i-1)*0.12;
    rect(bd.center[0]-0.05, y-0.05, bd.center[0]+0.05, y+0.05,
         COL[ty]||'#777', null, 0);
  });
  cx.fillStyle='#6b7887'; cx.fillText('board L→R', X(bd.center[0]), Y(bd.center[1]-0.32));
  for(const [name,p] of Object.entries(SC.points)){
    if(name.endsWith('_approach')){ cx.beginPath(); cx.arc(X(p[0]),Y(p[1]),3,0,7);
      cx.strokeStyle='#3e6f9e'; cx.lineWidth=1; cx.stroke(); continue; }
    dot(p[0],p[1],2.5,'#3e6f9e',null);
    cx.fillStyle='#3e6f9e'; cx.font='9px monospace'; cx.textAlign='center';
    cx.fillText(name.replaceAll('_',' '), X(p[0]), Y(p[1])-6);
  }
  const sp=SC.start_pose;
  if(sp){ dot(sp.x,sp.y,5,'#ff7f0e','#000'); }
}

function draw(state){
  drawStatic();
  // lidar
  cx.fillStyle='rgba(90,220,255,.45)';
  for(const p of state.lidar||[]){ cx.beginPath(); cx.arc(X(p[0]),Y(p[1]),1.1,0,7); cx.fill(); }
  // trail
  if(state.trail && state.trail.length>1){
    cx.beginPath(); cx.moveTo(X(state.trail[0][0]),Y(state.trail[0][1]));
    for(const p of state.trail) cx.lineTo(X(p[0]),Y(p[1]));
    cx.strokeStyle='rgba(90,200,120,.7)'; cx.lineWidth=1.4; cx.stroke();
  }
  // cubes
  for(const c of state.cubes||[]){
    const sz=0.05*S;
    cx.beginPath(); cx.rect(X(c.x)-sz,Y(c.y)-sz,2*sz,2*sz);
    cx.fillStyle=COL[c.cls]||'#999'; cx.fill();
    cx.strokeStyle = c.status==='carried' ? '#5adcff' : (c.delivered ? '#7bd88f' : '#0b0e12');
    cx.lineWidth = c.delivered ? 2 : 1; cx.stroke();
  }
  // robot
  const [x,y,yaw]=state.pose;
  dot(x,y,0.12*S,'#ff7f0e','#000');
  cx.beginPath(); cx.moveTo(X(x),Y(y));
  cx.lineTo(X(x+0.22*Math.cos(yaw)), Y(y+0.22*Math.sin(yaw)));
  cx.strokeStyle='#ffcf9b'; cx.lineWidth=2; cx.stroke();
  // hud
  document.getElementById('t').textContent = state.t.toFixed(1)+' s';
  document.getElementById('clock').textContent = state.clock!==undefined ?
      state.clock.toFixed(0)+' s' : '-';
  document.getElementById('st').textContent = state.state||'-';
  document.getElementById('cur').textContent = state.current||'-';
  document.getElementById('score').textContent = state.score;
  document.getElementById('col').textContent = state.collisions;
  document.getElementById('chassis').textContent =
      (state.max_linear||0).toFixed(2)+' m/s · '+(state.max_angular||0).toFixed(2)+' rad/s';
  const vel = state.vel || [0,0];
  document.getElementById('vel').textContent =
      vel[0].toFixed(2)+' m/s · '+vel[1].toFixed(2)+' rad/s';
  document.getElementById('spd').textContent = (state.speed||1)+'x';
  const rows=(state.steps||[]).map((s,i)=>{
    const cls = s.outcome==='ok'?'ok':(s.outcome==='failed'||s.outcome==='skipped'?'bad':'warn');
    return `<tr><td class="muted">${i+1}</td><td>${s.kind}</td>
            <td class="${cls}">${s.outcome}</td></tr>`;
  }).join('');
  document.getElementById('steps').innerHTML = rows ||
      '<tr><td class="muted">waiting…</td></tr>';
  if(state.mapping && Object.keys(state.mapping).length){
    document.getElementById('cur').title = JSON.stringify(state.mapping);
  }
}

async function tick(){
  try{
    const r = await fetch('/state',{cache:'no-store'});
    const s = await r.json();
    if(s.error){ document.getElementById('err').textContent = s.error; return; }
    document.getElementById('err').textContent = '';
    draw(s);
  }catch(e){ document.getElementById('err').textContent = String(e); }
}
setInterval(tick, 120);
tick();
</script></body></html>
"""


class ViewerServer:
    def __init__(self, port: int, scenario: dict, state_fn: Callable[[], dict],
                 stop_event: threading.Event, host: str = "127.0.0.1") -> None:
        html = HTML_TEMPLATE.replace("__SCENARIO__", json.dumps(scenario))
        html_bytes = html.encode("utf-8")

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    body, ctype = html_bytes, "text/html; charset=utf-8"
                elif self.path.startswith("/state"):
                    try:
                        body = json.dumps(state_fn()).encode("utf-8")
                    except Exception as exc:  # never kill the page on a snapshot bug
                        body = json.dumps({"error": str(exc)}).encode("utf-8")
                    ctype = "application/json"
                elif self.path.startswith("/stop"):
                    stop_event.set()
                    body, ctype = b"stopping", "text/plain"
                else:
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *args):
                pass

        self.httpd = ThreadingHTTPServer((host, port), Handler)
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.httpd.shutdown()
        self.httpd.server_close()

    def linger(self, seconds: float = 0.0) -> None:
        """Keep serving after the mission ends so the final state can be inspected."""
        try:
            if seconds and seconds > 0:
                time.sleep(seconds)
            else:
                while True:
                    time.sleep(0.5)
        except KeyboardInterrupt:
            pass
