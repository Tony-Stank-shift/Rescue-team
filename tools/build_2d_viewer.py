#!/usr/bin/env python3
"""Build self-contained 2D visualization HTML from the integrated simulation.

集成仿真可视化：把 rescue_robot 的决策/导航/转运/感知程序接进仿真，
导出场地俯视图动画（机器人轨迹 + 目标套取/投放 + 状态机流转）。
"""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from rescue_robot.simulation.integrated_sim import IntegratedSim
from rescue_robot.perception.field_elements import SafeZoneColor

# 目标形状值 → 前端绘制 key（前端只认识这些形状名）
SHAPE_MAP = {
    'cube': 'cube',
    'triangular_pyramid': 'pyramid',
    'cuboid': 'cuboid',
    'cylinder': 'cylinder',
    'cone_frustum': 'cone_frustum',
    'sphere': 'sphere',
    'unknown': 'cube',
}


def run_simulation(seed=42, duration_s=180.0, my_color='RED', start_zone=1):
    """Run integrated sim and export frame data + final state (mm → m)."""
    color = SafeZoneColor.RED if my_color == 'RED' else SafeZoneColor.BLUE
    sim = IntegratedSim(seed=seed, my_color=color, start_zone=start_zone)
    sim.setup_match()

    max_steps = int(min(duration_s, sim.MATCH_DURATION_S) / sim.DT)
    frames = []
    step_i = 0
    while step_i < max_steps and not sim.is_terminal:
        f = sim.step()
        if step_i % 5 == 0:  # every 100ms
            x, y, th = f['robot_pose']
            frames.append({
                'time': f['time_elapsed_s'],
                'score': f['score'],
                'delivered': f['targets_delivered'],
                'total': f['targets_total'],
                'trip': f['trip_count'],
                'rx': round(x / 1000.0, 4),
                'ry': round(y / 1000.0, 4),
                'ryaw': round(th, 4),
                'targets': [{
                    'id': t.world_id,
                    'x': round(t.x / 1000.0, 4),
                    'y': round(t.y / 1000.0, 4),
                    'shape': SHAPE_MAP.get(t.info.shape.value, 'cube'),
                    'color': t.info.color.value,
                    'points': t.points,
                    'delivered': t.delivered,
                    'dangerous': t.dangerous,
                    'carried': t.carried,
                    'valid': t.delivered_valid,
                } for t in sim.targets],
                'carried_id': f['carried_id'],
                'trajectory': [[round(p[0] / 1000.0, 4), round(p[1] / 1000.0, 4)]
                               for p in sim.trajectory[-60:]],
                'events': sim.events[-6:],
                'action': f['action'],
                'nav_state': f['nav_state'],
                'transport_phase': f['transport_phase'],
                'decision_state': f['decision_state'],
            })
        step_i += 1

    delivered = [t for t in sim.targets if t.delivered]
    valid = [t for t in delivered if t.delivered_valid]
    return {
        'seed': seed,
        'target_count': len(sim.targets),
        'duration_s': round(sim._time_elapsed, 1),
        'final_score': sim.score,
        'final_delivered': len(delivered),
        'final_valid': len(valid),
        'frames': frames,
    }


# ============================================================
HTML = r'''<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>救援机器人 - 集成仿真 2D 可视化</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0d1117;font-family:system-ui,'Segoe UI',sans-serif;color:#c9d1d9;display:flex;flex-direction:column;height:100vh;overflow:hidden}
#toolbar{display:flex;align-items:center;gap:2px;background:#161b22;border-bottom:1px solid #30363d;padding:0 8px;min-height:44px}
.tab{padding:10px 18px;cursor:pointer;border:none;background:none;color:#8b949e;font-size:13px;border-bottom:2px solid transparent;transition:all 0.15s;white-space:nowrap}
.tab:hover{color:#e6edf3;background:rgba(255,255,255,0.03)}
.tab.active{color:#58a6ff;border-bottom-color:#58a6ff;font-weight:600}
#viewContainer{flex:1;position:relative;overflow:hidden}
canvas{display:block;position:absolute;top:0;left:0}
#statusBar{display:flex;gap:20px;align-items:center;flex-wrap:wrap;padding:6px 16px;background:#161b22;border-top:1px solid #30363d;font-size:11px;color:#8b949e;min-height:32px}
#statusBar span{display:flex;align-items:center;gap:4px}
.stat-val{color:#e6edf3;font-weight:600}
.stat-val.warn{color:#f0883e}
.stat-val.ok{color:#3fb950}
#controlsBar{display:flex;gap:8px;align-items:center;padding:4px 16px;background:#161b22;border-top:1px solid #21262d;min-height:36px}
#controlsBar button{background:#21262d;border:1px solid #30363d;color:#c9d1d9;padding:4px 12px;border-radius:4px;cursor:pointer;font-size:12px}
#controlsBar button:hover{background:#30363d}
#controlsBar button.active{background:#1f6feb;border-color:#1f6feb;color:#fff}
#speedLabel{color:#8b949e;font-size:11px;min-width:30px;text-align:center}
#progress{flex:1;height:3px;background:#21262d;border-radius:2px;overflow:hidden;max-width:200px}
#progress div{height:100%;background:#58a6ff;transition:width 0.1s}
</style>
</head>
<body>

<div id="toolbar">
  <button class="tab active" data-view="scene">比赛场景</button>
</div>
<div id="viewContainer"><canvas id="mainCanvas"></canvas></div>
<div id="controlsBar">
  <button id="btnPrev">&#9664; 上一帧</button>
  <button id="btnPlay" class="active">&#9646;&#9646; 暂停</button>
  <button id="btnNext">下一帧 &#9654;</button>
  <div id="progress"><div style="width:0%"></div></div>
  <span id="speedLabel">1x</span>
  <button id="btnSpeed">变速</button>
  <span style="margin-left:12px;font-size:11px;color:#8b949e">Space 暂停 | &#8592; &#8594; 逐帧</span>
</div>
<div id="statusBar">
  <span>时间: <span class="stat-val" id="stTime">0.0s</span></span>
  <span>分数: <span class="stat-val" id="stScore">0</span></span>
  <span>送达: <span class="stat-val" id="stDel">0</span></span>
  <span>趟次: <span class="stat-val" id="stTrip">0</span></span>
  <span>动作: <span class="stat-val ok" id="stAction">WAIT</span></span>
  <span>决策: <span class="stat-val" id="stDecision">-</span></span>
  <span>转运: <span class="stat-val" id="stTransport">-</span></span>
</div>

<script>
// ---- EMBEDDED DATA ----
const SIM_DATA = __DATA_PLACEHOLDER__;

let currentFrameIdx = 0;
let playing = true;
let speed = 1;
let lastTick = 0;
let animId = null;

const canvas = document.getElementById('mainCanvas');
const ctx = canvas.getContext('2d');
const container = document.getElementById('viewContainer');

function resize() {
  canvas.width = container.clientWidth * (window.devicePixelRatio || 1);
  canvas.height = container.clientHeight * (window.devicePixelRatio || 1);
  canvas.style.width = container.clientWidth + 'px';
  canvas.style.height = container.clientHeight + 'px';
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.scale(window.devicePixelRatio || 1, window.devicePixelRatio || 1);
}
window.addEventListener('resize', resize);
resize();

const btnPlay = document.getElementById('btnPlay');
const btnPrev = document.getElementById('btnPrev');
const btnNext = document.getElementById('btnNext');
const btnSpeed = document.getElementById('btnSpeed');
const speedLabel = document.getElementById('speedLabel');
const progressBar = document.querySelector('#progress div');

function setFrame(idx) {
  currentFrameIdx = Math.max(0, Math.min(SIM_DATA.frames.length - 1, idx));
  progressBar.style.width = (currentFrameIdx / (SIM_DATA.frames.length - 1) * 100) + '%';
}

function updateStatusBar() {
  const f = SIM_DATA.frames[currentFrameIdx];
  if (!f) return;
  document.getElementById('stTime').textContent = f.time.toFixed(1) + 's';
  document.getElementById('stScore').textContent = f.score;
  document.getElementById('stDel').textContent = f.delivered + '/' + f.total;
  document.getElementById('stTrip').textContent = f.trip;
  const actEl = document.getElementById('stAction');
  actEl.textContent = f.action;
  actEl.className = 'stat-val ' + (f.action === 'WAIT' ? 'warn' : 'ok');
  document.getElementById('stDecision').textContent = f.decision_state;
  document.getElementById('stTransport').textContent = f.transport_phase;
}

btnPlay.onclick = () => {
  playing = !playing;
  btnPlay.innerHTML = playing ? '&#9646;&#9646; 暂停' : '&#9654; 播放';
  btnPlay.classList.toggle('active', playing);
  if (playing) lastTick = performance.now();
};
btnPrev.onclick = () => { playing = false; btnPlay.innerHTML = '&#9654; 播放'; btnPlay.classList.remove('active'); setFrame(currentFrameIdx - 1); updateStatusBar(); draw(); };
btnNext.onclick = () => { playing = false; btnPlay.innerHTML = '&#9654; 播放'; btnPlay.classList.remove('active'); setFrame(currentFrameIdx + 1); updateStatusBar(); draw(); };
btnSpeed.onclick = () => { const speeds = [0.5, 1, 2, 5, 10, 20]; speed = speeds[(speeds.indexOf(speed) + 1) % speeds.length]; speedLabel.textContent = speed + 'x'; };
document.addEventListener('keydown', e => {
  if (document.activeElement !== document.body) return;
  if (e.code === 'Space') { e.preventDefault(); btnPlay.click(); }
  if (e.code === 'ArrowRight') { e.preventDefault(); btnNext.click(); }
  if (e.code === 'ArrowLeft') { e.preventDefault(); btnPrev.click(); }
});

function tick(now) {
  if (playing && SIM_DATA.frames.length > 1) {
    if ((now - lastTick) / 1000 >= 0.1 / speed) {
      setFrame((currentFrameIdx + 1) % SIM_DATA.frames.length);
      updateStatusBar();
      lastTick = now;
    }
  }
  draw();
  animId = requestAnimationFrame(tick);
}
lastTick = performance.now();
animId = requestAnimationFrame(tick);

// ---- DRAWING HELPERS ----
const W = () => canvas.width / (window.devicePixelRatio || 1);
const H = () => canvas.height / (window.devicePixelRatio || 1);

function roundedRect(x, y, w, h, r, fill, stroke) {
  ctx.beginPath(); ctx.moveTo(x+r, y); ctx.lineTo(x+w-r, y);
  ctx.arcTo(x+w, y, x+w, y+r, r); ctx.lineTo(x+w, y+h-r);
  ctx.arcTo(x+w, y+h, x+w-r, y+h, r); ctx.lineTo(x+r, y+h);
  ctx.arcTo(x, y+h, x, y+h-r, r); ctx.lineTo(x, y+r);
  ctx.arcTo(x, y, x+r, y, r); ctx.closePath();
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (stroke) { ctx.strokeStyle = stroke; ctx.stroke(); }
}

function drawScene() {
  const w = W(), h = H();
  ctx.clearRect(0, 0, w, h);

  const f = SIM_DATA.frames[currentFrameIdx];
  if (!f) return;

  const margin = 30;
  const fieldPx = Math.min(w - 2*margin, h - 90 - margin);
  const ox = (w - fieldPx) / 2;
  const oy = margin + 10;
  const scale = fieldPx / 3.0; // 3m -> px

  ctx.fillStyle = '#0d1117'; ctx.fillRect(0, 0, w, h);
  ctx.fillStyle = '#1a3a1a'; roundedRect(ox, oy, 3*scale, 3*scale, 4, '#1a3a1a', null);

  // Grid
  ctx.strokeStyle = 'rgba(255,255,255,0.04)'; ctx.lineWidth = 0.5;
  for (let i = 0; i <= 10; i++) {
    const pos = i * 0.3 * scale;
    ctx.beginPath(); ctx.moveTo(ox+pos, oy); ctx.lineTo(ox+pos, oy+3*scale); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(ox, oy+pos); ctx.lineTo(ox+3*scale, oy+pos); ctx.stroke();
  }
  ctx.strokeStyle = '#555'; ctx.lineWidth = 3; ctx.strokeRect(ox, oy, 3*scale, 3*scale);

  // Safe zones (red left, blue right; field_elements: 100..700 / 2300..2900, y 2200..3000)
  ctx.fillStyle = 'rgba(239,83,80,0.28)'; ctx.fillRect(ox+0.1*scale, oy+2.2*scale, 0.6*scale, 0.8*scale);
  ctx.fillStyle = 'rgba(66,165,245,0.28)'; ctx.fillRect(ox+2.3*scale, oy+2.2*scale, 0.6*scale, 0.8*scale);
  ctx.strokeStyle = '#ef5350'; ctx.lineWidth = 1.5; ctx.strokeRect(ox+0.1*scale, oy+2.2*scale, 0.6*scale, 0.8*scale);
  ctx.strokeStyle = '#42a5f5'; ctx.strokeRect(ox+2.3*scale, oy+2.2*scale, 0.6*scale, 0.8*scale);
  // sub-areas (supply / injured)
  ctx.strokeStyle = 'rgba(255,255,255,0.25)'; ctx.lineWidth = 1;
  ctx.strokeRect(ox+0.1*scale, oy+2.6*scale, 0.3*scale, 0.4*scale);
  ctx.strokeRect(ox+0.4*scale, oy+2.6*scale, 0.3*scale, 0.4*scale);
  ctx.strokeRect(ox+2.3*scale, oy+2.6*scale, 0.3*scale, 0.4*scale);
  ctx.strokeRect(ox+2.6*scale, oy+2.6*scale, 0.3*scale, 0.4*scale);
  ctx.fillStyle = '#ef5350'; ctx.font = '11px system-ui'; ctx.fillText('红队安全区(物|伤)', ox+0.1*scale+4, oy+2.24*scale);
  ctx.fillStyle = '#42a5f5'; ctx.fillText('蓝队安全区(物|伤)', ox+2.3*scale+4, oy+2.24*scale);

  // Start zones
  ctx.fillStyle = 'rgba(206,147,216,0.3)';
  const starts = [[0,0],[2.7,0],[2.7,2.7],[0,2.7]];
  starts.forEach(([sx,sy]) => ctx.fillRect(ox+sx*scale, oy+sy*scale, 0.3*scale, 0.3*scale));
  ctx.strokeStyle = '#ce93d8'; ctx.lineWidth = 1;
  starts.forEach(([sx,sy]) => ctx.strokeRect(ox+sx*scale, oy+sy*scale, 0.3*scale, 0.3*scale));

  // --- TARGETS ---
  const shapeColors = {green:'#4caf50', black:'#616161', orange:'#ff9800', light_blue:'#81d4fa'};
  f.targets.forEach(t => {
    if (t.delivered) {
      // 已投放：画在落点，标注正确/错误
      const tx = ox + t.x * scale, ty = oy + t.y * scale;
      ctx.strokeStyle = t.valid ? 'rgba(63,185,80,0.8)' : 'rgba(240,67,54,0.9)';
      ctx.lineWidth = 1.5;
      ctx.beginPath(); ctx.arc(tx, ty, 4, 0, Math.PI*2); ctx.stroke();
      ctx.fillStyle = t.valid ? '#3fb950' : '#f44336';
      ctx.font = '8px system-ui';
      ctx.fillText(t.valid ? '✓' : '✗', tx-2, ty-4);
      return;
    }
    if (t.carried || t.id === f.carried_id) return; // 被携带的跟随机器人
    const tx = ox + t.x * scale, ty = oy + t.y * scale;
    const color = shapeColors[t.color] || '#888';
    ctx.fillStyle = color;
    ctx.strokeStyle = 'rgba(0,0,0,0.3)'; ctx.lineWidth = 1;
    const s = 6;
    switch(t.shape) {
      case 'cube': ctx.fillRect(tx-s/2, ty-s/2, s, s); ctx.strokeRect(tx-s/2, ty-s/2, s, s); break;
      case 'pyramid': ctx.beginPath(); ctx.moveTo(tx, ty-s/2); ctx.lineTo(tx+s/2, ty+s/2); ctx.lineTo(tx-s/2, ty+s/2); ctx.closePath(); ctx.fill(); ctx.stroke(); break;
      case 'cuboid': ctx.fillRect(tx-s, ty-s/3, s*2, s*2/3); ctx.strokeRect(tx-s, ty-s/3, s*2, s*2/3); break;
      case 'sphere': ctx.beginPath(); ctx.arc(tx, ty, s/2, 0, Math.PI*2); ctx.fill(); ctx.stroke(); break;
      case 'cylinder': ctx.fillRect(tx-s/3, ty-s, s*2/3, s*2); ctx.strokeRect(tx-s/3, ty-s, s*2/3, s*2); break;
      case 'cone_frustum':
        ctx.beginPath(); ctx.moveTo(tx-s/3, ty-s); ctx.lineTo(tx+s/3, ty-s); ctx.lineTo(tx+s/5, ty+s); ctx.lineTo(tx-s/5, ty+s); ctx.closePath(); ctx.fill(); ctx.stroke(); break;
    }
    if (t.dangerous) {
      ctx.fillStyle = '#f44336'; ctx.font = '7px system-ui'; ctx.fillText('!', tx-1, ty-7);
    }
  });

  // --- TRAJECTORY ---
  if (f.trajectory && f.trajectory.length > 1) {
    ctx.strokeStyle = 'rgba(88,166,255,0.3)'; ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(ox+f.trajectory[0][0]*scale, oy+f.trajectory[0][1]*scale);
    for (let i = 1; i < f.trajectory.length; i++)
      ctx.lineTo(ox+f.trajectory[i][0]*scale, oy+f.trajectory[i][1]*scale);
    ctx.stroke();
  }

  // --- ROBOT ---
  const rx = ox + f.rx * scale, ry = oy + f.ry * scale, ryaw = f.ryaw;
  const rw = 0.3 * scale, rh = 0.3 * scale;
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(ryaw);
  ctx.fillStyle = '#1a1a1a'; ctx.strokeStyle = '#4fc3f7'; ctx.lineWidth = 1.5;
  roundedRect(-rw/2, -rh/2, rw, rh, 3, '#1a1a1a', '#4fc3f7');
  if (f.carried_id) {
    const ct = f.targets.find(t => t.id === f.carried_id);
    if (ct) {
      ctx.fillStyle = shapeColors[ct.color] || '#888';
      ctx.fillRect(rw/2-4, -4, 8, 8);
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1; ctx.strokeRect(rw/2-4, -4, 8, 8);
    }
  }
  ctx.fillStyle = '#4fc3f7';
  ctx.beginPath(); ctx.moveTo(rw/2, 0); ctx.lineTo(rw/2-6, -4); ctx.lineTo(rw/2-6, 4); ctx.closePath(); ctx.fill();
  ctx.restore();

  // --- HUD ---
  ctx.fillStyle = '#fff'; ctx.font = '12px system-ui';
  ctx.fillText(`T=${f.time.toFixed(1)}s`, ox+4, oy+3*scale+14);
  ctx.fillText(`Score: ${f.score}`, ox+4, oy+3*scale+28);
  ctx.fillText(`Delivered: ${f.delivered}/${f.total}`, ox+4, oy+3*scale+42);

  const lx = ox+3*scale+10, ly = oy;
  ctx.font = '10px system-ui';
  const items = [['#4fc3f7','机器人'],['#4caf50','普通物资 5分'],['#616161','核心物资 10分'],['#ff9800','伤员 15分'],['#81d4fa','危险品 禁止']];
  items.forEach(([c, label], i) => {
    ctx.fillStyle = c; ctx.fillRect(lx, ly+i*16, 8, 8);
    ctx.fillStyle = '#8b949e'; ctx.fillText(label, lx+12, ly+i*16+8);
  });

  if (f.events && f.events.length > 0) {
    ctx.fillStyle = '#8b949e'; ctx.font = '9px system-ui';
    ctx.fillText('事件:', lx, ly+items.length*16+16);
    f.events.slice(-5).forEach((ev, i) => ctx.fillText(ev, lx, ly+items.length*16+28+i*12));
  }
}

function draw() { drawScene(); }

setFrame(0);
updateStatusBar();
draw();
</script>
</body>
</html>'''


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, '..', 'rescue_sim_2d.html')
    out = os.path.abspath(out)

    print("Running integrated simulation (decision + navigation + transport)...")
    data = run_simulation(seed=42, duration_s=180.0)
    print(f"  {len(data['frames'])} frames, {data['target_count']} targets")
    print(f"  Final: score={data['final_score']}, "
          f"delivered={data['final_delivered']}/{data['target_count']}, "
          f"valid={data['final_valid']}")

    data_json = json.dumps(data, separators=(',', ':'), ensure_ascii=False)
    print(f"  JSON data: {len(data_json):,} chars")

    html = HTML.replace('__DATA_PLACEHOLDER__', data_json)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(html)

    print(f"\nDone! Wrote {len(html):,} chars to:")
    print(f"  {out}")
    print(f"\nOpen rescue_sim_2d.html in a browser to view the animation.")


if __name__ == '__main__':
    main()
