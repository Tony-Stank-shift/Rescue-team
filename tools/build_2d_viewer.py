#!/usr/bin/env python3
"""Build self-contained 2D visualization HTML with 4 views."""

import json, os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from rescue_robot.simulation.sim_2d import Sim2D, DECISION_TIMESTEP_S


def run_simulation(seed=42, target_count=15, duration_s=6.0):
    """Run sim and export frame data + final state."""
    sim = Sim2D(seed=seed)
    sim.setup_match(target_count=target_count)

    max_steps = int(duration_s / DECISION_TIMESTEP_S)
    frames = []
    for step_i in range(max_steps):
        state = sim.step()
        if step_i % 5 == 0:  # every 100ms
            frames.append({
                'time': state.time_elapsed_s,
                'score': state.score,
                'delivered': state.targets_delivered,
                'rx': state.robot_pose.x,
                'ry': state.robot_pose.y,
                'ryaw': state.robot_pose.yaw,
                'targets': [{
                    'id': t.id, 'x': round(t.x_m, 4), 'y': round(t.y_m, 4),
                    'shape': t.shape, 'color': t.color, 'points': t.points,
                    'delivered': t.is_delivered, 'dangerous': t.is_dangerous
                } for t in sim.targets],
                'hw': {
                    'motor_rpm': [round(r) for r in sim.hw.motor_rpm],
                    'motor_current_ma': [round(c) for c in sim.hw.motor_current_ma],
                    'battery_v': round(sim.hw.battery_voltage_v, 2),
                    'battery_ma': round(sim.hw.battery_current_ma),
                    'ultrasonic_mm': round(sim.hw.ultrasonic_distance_mm),
                    'imu_gyro_z': round(sim.hw.imu_gyro_rad_s[2], 4),
                    'pusher_mm': round(sim.hw.pusher_position_mm, 1),
                    'latency_ms': round(sim.hw.heartbeat_latency_ms, 1),
                },
                'carried_id': sim._carried_target.id if sim._carried_target else None,
                'trajectory': [list(p) for p in sim.trajectory[-50:]],
                'events': sim.events[-5:],
            })

    # Calculate some stats
    last_state = sim._build_state()
    return {
        'seed': seed,
        'target_count': target_count,
        'duration_s': round(duration_s, 1),
        'final_score': last_state.score,
        'final_delivered': last_state.targets_delivered,
        'frames': frames,
    }


# ============================================================
HTML = r'''<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>救援机器人 - 2D 可视化仿真</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0d1117;font-family:system-ui,'Segoe UI',sans-serif;color:#c9d1d9;display:flex;flex-direction:column;height:100vh;overflow:hidden}
#toolbar{display:flex;align-items:center;gap:2px;background:#161b22;border-bottom:1px solid #30363d;padding:0 8px;min-height:44px}
.tab{padding:10px 18px;cursor:pointer;border:none;background:none;color:#8b949e;font-size:13px;border-bottom:2px solid transparent;transition:all 0.15s;white-space:nowrap}
.tab:hover{color:#e6edf3;background:rgba(255,255,255,0.03)}
.tab.active{color:#58a6ff;border-bottom-color:#58a6ff;font-weight:600}
.tab-sep{width:1px;height:20px;background:#30363d;margin:0 6px}
#viewContainer{flex:1;position:relative;overflow:hidden}
canvas{display:block;position:absolute;top:0;left:0}
#statusBar{display:flex;gap:24px;align-items:center;padding:6px 16px;background:#161b22;border-top:1px solid #30363d;font-size:11px;color:#8b949e;min-height:32px}
#statusBar span{display:flex;align-items:center;gap:4px}
.stat-val{color:#e6edf3;font-weight:600}
#controlsBar{display:flex;gap:8px;align-items:center;padding:4px 16px;background:#161b22;border-top:1px solid #21262d;min-height:36px;display:none}
#controlsBar.visible{display:flex}
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
  <span class="tab-sep"></span>
  <button class="tab" data-view="software">软件架构</button>
  <button class="tab" data-view="hardware">硬件系统</button>
  <button class="tab" data-view="mechanical">机械结构</button>
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
  <span>电池: <span class="stat-val" id="stBatt">12.2V</span></span>
  <span>状态: <span class="stat-val" id="stState">运行中</span></span>
</div>

<script>
// ---- EMBEDDED DATA ----
const SIM_DATA = __DATA_PLACEHOLDER__;

// ---- State ----
let currentView = 'scene';
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

// ---- Tab switching ----
document.querySelectorAll('.tab').forEach(btn => {
  btn.addEventListener('click', () => {
    document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
    btn.classList.add('active');
    currentView = btn.dataset.view;
    document.getElementById('controlsBar').classList.toggle('visible', currentView === 'scene');
    draw();
  });
});

// ---- Playback controls ----
const btnPlay = document.getElementById('btnPlay');
const btnPrev = document.getElementById('btnPrev');
const btnNext = document.getElementById('btnNext');
const btnSpeed = document.getElementById('btnSpeed');
const speedLabel = document.getElementById('speedLabel');
const progressBar = document.querySelector('#progress div');
const controlsBar = document.getElementById('controlsBar');
controlsBar.classList.add('visible');

function setFrame(idx) {
  currentFrameIdx = Math.max(0, Math.min(SIM_DATA.frames.length - 1, idx));
  progressBar.style.width = (currentFrameIdx / (SIM_DATA.frames.length - 1) * 100) + '%';
}

function updateStatusBar() {
  const f = SIM_DATA.frames[currentFrameIdx];
  if (!f) return;
  document.getElementById('stTime').textContent = f.time.toFixed(1) + 's';
  document.getElementById('stScore').textContent = f.score;
  document.getElementById('stDel').textContent = f.delivered;
  document.getElementById('stBatt').textContent = f.hw.battery_v.toFixed(1) + 'V';
  document.getElementById('stState').textContent = f.carried_id ? '搬运中 #'+f.carried_id : ((currentFrameIdx >= SIM_DATA.frames.length - 1) ? '已结束' : '运行中');
}

btnPlay.onclick = () => {
  playing = !playing;
  btnPlay.innerHTML = playing ? '&#9646;&#9646; 暂停' : '&#9654; 播放';
  btnPlay.classList.toggle('active', playing);
  if (playing) lastTick = performance.now();
};
btnPrev.onclick = () => { playing = false; btnPlay.innerHTML = '&#9654; 播放'; btnPlay.classList.remove('active'); setFrame(currentFrameIdx - 1); updateStatusBar(); draw(); };
btnNext.onclick = () => { playing = false; btnPlay.innerHTML = '&#9654; 播放'; btnPlay.classList.remove('active'); setFrame(currentFrameIdx + 1); updateStatusBar(); draw(); };
btnSpeed.onclick = () => { const speeds = [0.5, 1, 2, 5, 10]; speed = speeds[(speeds.indexOf(speed) + 1) % speeds.length]; speedLabel.textContent = speed + 'x'; };
document.addEventListener('keydown', e => {
  if (document.activeElement !== document.body) return;
  if (e.code === 'Space') { e.preventDefault(); btnPlay.click(); }
  if (e.code === 'ArrowRight') { e.preventDefault(); btnNext.click(); }
  if (e.code === 'ArrowLeft') { e.preventDefault(); btnPrev.click(); }
});

// ---- Render loop ----
function tick(now) {
  if (playing && currentView === 'scene' && SIM_DATA.frames.length > 1) {
    if ((now - lastTick) / 1000 >= 0.1 / speed) {
      setFrame((currentFrameIdx + 1) % SIM_DATA.frames.length);
      updateStatusBar();
      lastTick = now;
    }
  } else if (playing && currentView !== 'scene') {
    // For static views, still update occasionally
    if ((now - lastTick) / 1000 >= 0.5) { lastTick = now; }
  }
  draw();
  animId = requestAnimationFrame(tick);
}
lastTick = performance.now();
animId = requestAnimationFrame(tick);

// ============================================================
//  DRAWING HELPERS
// ============================================================
const W = () => canvas.width / (window.devicePixelRatio || 1);
const H = () => canvas.height / (window.devicePixelRatio || 1);

function r(x) { return Math.round(x); }
function p2c(mm, scale, ox, oy) { return [ox + mm[0]*scale/1000, oy + mm[1]*scale/1000]; }

function arrowLine(cx, cy, tx, ty, color, width) {
  ctx.strokeStyle = color; ctx.lineWidth = width || 1.5;
  ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(tx, ty); ctx.stroke();
  const a = Math.atan2(ty-cy, tx-cx), s = 7;
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.moveTo(tx, ty);
  ctx.lineTo(tx - s*Math.cos(a-0.5), ty - s*Math.sin(a-0.5));
  ctx.lineTo(tx - s*Math.cos(a+0.5), ty - s*Math.sin(a+0.5));
  ctx.fill();
}

function roundedRect(x, y, w, h, r, fill, stroke) {
  ctx.beginPath(); ctx.moveTo(x+r, y); ctx.lineTo(x+w-r, y);
  ctx.arcTo(x+w, y, x+w, y+r, r); ctx.lineTo(x+w, y+h-r);
  ctx.arcTo(x+w, y+h, x+w-r, y+h, r); ctx.lineTo(x+r, y+h);
  ctx.arcTo(x, y+h, x, y+h-r, r); ctx.lineTo(x, y+r);
  ctx.arcTo(x, y, x+r, y, r); ctx.closePath();
  if (fill) { ctx.fillStyle = fill; ctx.fill(); }
  if (stroke) { ctx.strokeStyle = stroke; ctx.stroke(); }
}

// ============================================================
//  VIEW 1: 比赛场景
// ============================================================
function drawScene() {
  const w = W(), h = H();
  ctx.clearRect(0, 0, w, h);

  const f = SIM_DATA.frames[currentFrameIdx];
  if (!f) return;

  const margin = 30;
  const fieldPx = Math.min(w - 2*margin, h - 70 - margin);
  const ox = (w - fieldPx) / 2;
  const oy = margin + 10;
  const scale = fieldPx / 3.0; // 3m -> pixels

  // Background
  ctx.fillStyle = '#0d1117'; ctx.fillRect(0, 0, w, h);

  // Field grass
  ctx.fillStyle = '#1a3a1a'; roundedRect(ox, oy, 3*scale, 3*scale, 4, '#1a3a1a', null);

  // Grid
  ctx.strokeStyle = 'rgba(255,255,255,0.04)'; ctx.lineWidth = 0.5;
  for (let i = 0; i <= 10; i++) {
    const pos = i * 0.3 * scale;
    ctx.beginPath(); ctx.moveTo(ox+pos, oy); ctx.lineTo(ox+pos, oy+3*scale); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(ox, oy+pos); ctx.lineTo(ox+3*scale, oy+pos); ctx.stroke();
  }

  // Walls (boundary)
  ctx.strokeStyle = '#555'; ctx.lineWidth = 3;
  ctx.strokeRect(ox, oy, 3*scale, 3*scale);

  // Safe zones
  ctx.fillStyle = 'rgba(239,83,80,0.3)'; ctx.fillRect(ox+0.05*scale, oy+2.55*scale, 0.6*scale, 0.4*scale);
  ctx.fillStyle = 'rgba(66,165,245,0.3)'; ctx.fillRect(ox+2.35*scale, oy+2.55*scale, 0.6*scale, 0.4*scale);
  ctx.strokeStyle = '#ef5350'; ctx.lineWidth = 1.5; ctx.strokeRect(ox+0.05*scale, oy+2.55*scale, 0.6*scale, 0.4*scale);
  ctx.strokeStyle = '#42a5f5'; ctx.strokeRect(ox+2.35*scale, oy+2.55*scale, 0.6*scale, 0.4*scale);
  // Labels
  ctx.fillStyle = '#ef5350'; ctx.font = '11px system-ui'; ctx.fillText('红队安全区', ox+0.05*scale+4, oy+2.55*scale+16);
  ctx.fillStyle = '#42a5f5'; ctx.fillText('蓝队安全区', ox+2.35*scale+4, oy+2.55*scale+16);

  // Start zones
  ctx.fillStyle = 'rgba(206,147,216,0.3)';
  const starts = [[0,0],[2.7,0],[2.7,2.7],[0,2.7]];
  starts.forEach(([sx,sy]) => ctx.fillRect(ox+sx*scale, oy+sy*scale, 0.3*scale, 0.3*scale));
  ctx.strokeStyle = '#ce93d8'; ctx.lineWidth = 1;
  starts.forEach(([sx,sy]) => ctx.strokeRect(ox+sx*scale, oy+sy*scale, 0.3*scale, 0.3*scale));

  // Speed bumps
  ctx.fillStyle = '#fdd835';
  starts.forEach(([sx,sy]) => {
    for (let i = 0; i < 3; i++) {
      ctx.fillRect(ox+(sx-0.005)*scale, oy+(sy+0.25+i*0.08)*scale, 0.31*scale, 0.005*scale);
    }
  });

  // Purple fences
  ctx.strokeStyle = '#9c27b0'; ctx.lineWidth = 2;
  [[0.05,2.55],[2.35,2.55]].forEach(([fx,fy]) => {
    ctx.beginPath(); ctx.moveTo(ox+fx*scale, oy+fy*scale); ctx.lineTo(ox+(fx+0.6)*scale, oy+fy*scale); ctx.stroke();
  });

  // --- TARGETS ---
  const shapeColors = {green:'#4caf50', black:'#616161', orange:'#ff9800', light_blue:'#81d4fa'};
  f.targets.forEach(t => {
    if (t.delivered || t.id === f.carried_id) return;
    const tx = ox + t.x * scale, ty = oy + t.y * scale;
    const color = shapeColors[t.color] || '#888';
    ctx.fillStyle = color;
    ctx.strokeStyle = 'rgba(0,0,0,0.3)'; ctx.lineWidth = 1;

    const s = 6; // size in px
    switch(t.shape) {
      case 'cube': ctx.fillRect(tx-s/2, ty-s/2, s, s); ctx.strokeRect(tx-s/2, ty-s/2, s, s); break;
      case 'pyramid': ctx.beginPath(); ctx.moveTo(tx, ty-s/2); ctx.lineTo(tx+s/2, ty+s/2); ctx.lineTo(tx-s/2, ty+s/2); ctx.closePath(); ctx.fill(); ctx.stroke(); break;
      case 'cuboid': ctx.fillRect(tx-s, ty-s/3, s*2, s*2/3); ctx.strokeRect(tx-s, ty-s/3, s*2, s*2/3); break;
      case 'sphere': ctx.beginPath(); ctx.arc(tx, ty, s/2, 0, Math.PI*2); ctx.fill(); ctx.stroke(); break;
      case 'cylinder': ctx.fillRect(tx-s/3, ty-s, s*2/3, s*2); ctx.strokeRect(tx-s/3, ty-s, s*2/3, s*2); break;
      case 'cone_frustum':
        ctx.beginPath(); ctx.moveTo(tx-s/3, ty-s); ctx.lineTo(tx+s/3, ty-s); ctx.lineTo(tx+s/5, ty+s); ctx.lineTo(tx-s/5, ty+s); ctx.closePath(); ctx.fill(); ctx.stroke(); break;
    }

    // Dangerous label
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
  const rw = 0.3 * scale, rh = 0.3 * scale; // 300x300mm
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(ryaw);

  // Chassis
  ctx.fillStyle = '#1a1a1a'; ctx.strokeStyle = '#4fc3f7'; ctx.lineWidth = 1.5;
  roundedRect(-rw/2, -rh/2, rw, rh, 3, '#1a1a1a', '#4fc3f7');

  // Carried target
  if (f.carried_id) {
    const ct = f.targets.find(t => t.id === f.carried_id);
    if (ct) {
      const tc = shapeColors[ct.color] || '#888';
      ctx.fillStyle = tc;
      ctx.fillRect(rw/2-4, -4, 8, 8);
      ctx.strokeStyle = '#fff'; ctx.lineWidth = 1;
      ctx.strokeRect(rw/2-4, -4, 8, 8);
      ctx.fillStyle = '#fff'; ctx.font = '6px system-ui'; ctx.fillText('#'+ct.id, rw/2-4, -6);
    }
  }

  // Wheels (4 corners)
  const whw = 0.012*scale, whh = 0.025*scale;
  ctx.fillStyle = '#333';
  [[rw/2-whw, rh/2], [rw/2-whw, -rh/2-whh*2], [-rw/2+whw, rh/2], [-rw/2+whw, -rh/2-whh*2]].forEach(([wx,wy]) => {
    ctx.fillRect(wx-whw, wy-whh, whw*2, whh*2);
  });

  // Direction arrow
  ctx.fillStyle = '#4fc3f7';
  ctx.beginPath(); ctx.moveTo(rw/2, 0); ctx.lineTo(rw/2-6, -4); ctx.lineTo(rw/2-6, 4); ctx.closePath(); ctx.fill();

  ctx.restore();

  // --- HUD on field ---
  ctx.fillStyle = '#fff'; ctx.font = '12px system-ui';
  ctx.fillText(`T=${f.time.toFixed(1)}s`, ox+4, oy+3*scale+14);
  ctx.fillText(`Score: ${f.score}`, ox+4, oy+3*scale+28);
  ctx.fillText(`Del: ${f.delivered}/${SIM_DATA.target_count}`, ox+4, oy+3*scale+42);

  // Legend
  const lx = ox+3*scale+10, ly = oy;
  ctx.font = '10px system-ui';
  const items = [['#4fc3f7','机器人'],['#4caf50','普通物资 5分'],['#616161','核心物资 10分'],['#ff9800','伤员 15分'],['#81d4fa','危险品 -10分']];
  items.forEach(([c, label], i) => {
    ctx.fillStyle = c; ctx.fillRect(lx, ly+i*16, 8, 8);
    ctx.fillStyle = '#8b949e'; ctx.fillText(label, lx+12, ly+i*16+8);
  });

  // Events
  if (f.events && f.events.length > 0) {
    ctx.fillStyle = '#8b949e'; ctx.font = '9px system-ui';
    const recent = f.events.slice(-3);
    ctx.fillText('事件:', lx, ly+items.length*16+16);
    recent.forEach((ev, i) => ctx.fillText(ev, lx, ly+items.length*16+28+i*12));
  }
}

// ============================================================
//  VIEW 2: 软件架构
// ============================================================
function drawSoftware() {
  const w = W(), h = H();
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#0d1117'; ctx.fillRect(0, 0, w, h);

  const cx = w/2, cy = h/2;
  const modules = [
    {name:'main.py\n主控', x:cx, y:30, color:'#58a6ff', desc:'入口・状态机・启动'},
    {name:'perception/', x:cx-250, y:120, color:'#3fb950', desc:'感知: 摄像头・目标检测\n颜色分割・形状分类'},
    {name:'decision/', x:cx, y:120, color:'#d2a8ff', desc:'决策: 决策引擎・目标选择\n异常处理・策略权重'},
    {name:'navigation/', x:cx-420, y:260, color:'#f0883e', desc:'导航: A*路径・局部避障\n运动控制・PID'},
    {name:'transport/', x:cx-160, y:260, color:'#f778ba', desc:'转运: 装载管理・夹爪\n首趟规则・推板控制'},
    {name:'robustness/', x:cx+160, y:260, color:'#a5d6ff', desc:'健壮性: 故障检测・看门狗\n传感器健康・降级保活'},
    {name:'innovation/', x:cx+420, y:260, color:'#ffa657', desc:'创新: 硬件配置・热加载\n模型切换・标定'},
    {name:'comm_server.py', x:cx+250, y:120, color:'#79c0ff', desc:'通信: WebSocket\nJSON协议・调试面板'},
    {name:'config/', x:cx+460, y:120, color:'#8b949e', desc:'配置: YAML参数\n策略配置・标定数据'},
    {name:'state_machine.py', x:cx-460, y:120, color:'#e5534b', desc:'状态: BOOT→DEBUG→AUTO\n生命周期管理'},
  ];

  // Draw modules
  modules.forEach(m => {
    roundedRect(m.x-80, m.y-20, 160, 40, 6, m.color+'22', m.color);
    ctx.fillStyle = m.color; ctx.font = 'bold 11px system-ui';
    const lines = m.name.split('\n');
    lines.forEach((line, i) => ctx.fillText(line, m.x-ctx.measureText(line).width/2, m.y-4+i*14));
    // Description tooltip
    ctx.fillStyle = '#8b949e'; ctx.font = '8px system-ui';
    const dlines = m.desc.split('\n');
    dlines.forEach((line, i) => ctx.fillText(line, m.x-ctx.measureText(line).width/2, m.y+22+i*10));
  });

  // Dependencies (arrows)
  const edges = [
    [0,1],[0,2],[0,3],[0,4],[0,5],[0,6],[0,7],[0,8],[0,9],
    [1,2],[1,3],[2,3],[2,4],[5,2],[5,3],[7,2],[8,2],[9,0],
  ];
  edges.forEach(([a,b]) => {
    const ma = modules[a], mb = modules[b];
    ctx.strokeStyle = 'rgba(255,255,255,0.08)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(ma.x, ma.y+20); ctx.lineTo(mb.x, mb.y-20); ctx.stroke();
  });

  // State machine flow
  const smY = 340;
  const states = ['BOOT', 'DEBUG', 'AUTONOMOUS'];
  const smX = [cx-200, cx, cx+200];
  ctx.fillStyle = '#8b949e'; ctx.font = '10px system-ui'; ctx.fillText('状态机流转', cx-30, smY-10);
  states.forEach((s, i) => {
    roundedRect(smX[i]-50, smY, 100, 28, 14, s==='AUTONOMOUS'?'#3fb95033':'#8b949e22', s==='AUTONOMOUS'?'#3fb950':'#8b949e');
    ctx.fillStyle = s==='AUTONOMOUS'?'#3fb950':'#8b949e'; ctx.font = '11px system-ui'; ctx.fillText(s, smX[i]-ctx.measureText(s).width/2, smY+19);
  });
  arrowLine(smX[0]+50, smY+14, smX[1]-50, smY+14, '#8b949e', 1);
  arrowLine(smX[1]+50, smY+14, smX[2]-50, smY+14, '#8b949e', 1);

  // Strategy levels
  const stY = 390;
  ctx.fillStyle = '#8b949e'; ctx.font = '10px system-ui'; ctx.fillText('决策策略层级', cx-35, stY-10);
  ['FIRST_TRIP', 'FREE_RUN', 'TIME_PRESSURE'].forEach((s, i) => {
    const px = cx-150 + i*150;
    roundedRect(px-60, stY, 120, 24, 12, '#d2a8ff22', '#d2a8ff');
    ctx.fillStyle = '#d2a8ff'; ctx.font = '10px system-ui'; ctx.fillText(s, px-ctx.measureText(s).width/2, stY+16);
    if (i < 2) arrowLine(px+60, stY+12, px+90, stY+12, '#d2a8ff', 1);
  });

  // Legend
  ctx.font = '9px system-ui';
  const ly2 = stY+50;
  [['#58a6ff','入口'],['#3fb950','感知'],['#d2a8ff','决策'],['#f0883e','导航'],['#f778ba','转运'],['#e5534b','状态'],['#a5d6ff','健壮性']].forEach(([c,n],i) => {
    const px = 20+i*85;
    ctx.fillStyle = c; ctx.fillRect(px, ly2, 10, 10);
    ctx.fillStyle = '#8b949e'; ctx.fillText(n, px+13, ly2+9);
  });
}

// ============================================================
//  VIEW 3: 硬件系统
// ============================================================
function drawHardware() {
  const w = W(), h = H();
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#0d1117'; ctx.fillRect(0, 0, w, h);

  const cx = w/2, cy = h/2 - 30;

  // Raspberry Pi center
  roundedRect(cx-50, cy-35, 100, 70, 8, '#1a3a2a', '#3fb950');
  ctx.fillStyle = '#3fb950'; ctx.font = 'bold 12px system-ui'; ctx.fillText('Raspberry Pi', cx-45, cy-8);
  ctx.fillText('4B', cx-15, cy+12);
  ctx.fillStyle = '#8b949e'; ctx.font = '8px system-ui'; ctx.fillText('Linux · Python 3.10+', cx-48, cy+26);

  // Connected peripherals with interface labels
  const devices = [
    {label:'Camera\nUSB 1080P', x:cx-250, y:cy-180, iface:'USB 2.0', color:'#79c0ff', pins:'640×480@30fps\nfx=500 fy=500'},
    {label:'IMU\nMPU6050', x:cx+250, y:cy-180, iface:'I2C (0x68)', color:'#ffa657', pins:'3轴陀螺+3轴加速\n互补滤波 α=0.95'},
    {label:'Motor×4\nJGB37-520', x:cx-350, y:cy+30, iface:'GPIO PWM', color:'#f0883e', pins:'12V · 300RPM\n30:1 减速 · 11PPR'},
    {label:'Servo\nMG996R', x:cx-150, y:cy+190, iface:'GPIO PWM', color:'#f778ba', pins:'推板 0-50mm\n0-90° 10kg·cm'},
    {label:'Ultrasonic\nHC-SR04', x:cx+150, y:cy+190, iface:'GPIO', color:'#a5d6ff', pins:'TRIG+ECHO\n2cm-400cm'},
    {label:'LED+Btn', x:cx+350, y:cy+30, iface:'GPIO', color:'#d2a8ff', pins:'LED: BCM22,27\nBTN: BCM17'},
    {label:'Battery\n3S LiPo', x:cx, y:cy+100, iface:'Power', color:'#e5534b', pins:'11.1V 5200mAh\nXT60→LM2596→5V'},
    {label:'WiFi AP', x:cx+300, y:cy-180, iface:'TCP/WS', color:'#3fb950', pins:'WebSocket :8765\nHeartbeat 500ms'},
  ];

  devices.forEach(d => {
    // Connection line
    ctx.strokeStyle = d.color+'55'; ctx.lineWidth = 2;
    ctx.setLineDash([4, 4]);
    ctx.beginPath(); ctx.moveTo(cx, cy); ctx.lineTo(d.x, d.y); ctx.stroke();
    ctx.setLineDash([]);

    // Device box
    const bw = 120, bh = 45;
    roundedRect(d.x-bw/2, d.y-bh/2, bw, bh, 6, d.color+'18', d.color);
    ctx.fillStyle = d.color; ctx.font = 'bold 10px system-ui';
    const lines = d.label.split('\n');
    lines.forEach((line, i) => ctx.fillText(line, d.x-ctx.measureText(line).width/2, d.y-bh/2+14+i*14));
    // Interface badge
    ctx.fillStyle = d.color+'33'; roundedRect(d.x-ctx.measureText(d.iface).width/2-4, d.y+bh/2-14, ctx.measureText(d.iface).width+8, 16, 3, d.color+'33', null);
    ctx.fillStyle = d.color; ctx.font = '8px system-ui'; ctx.fillText(d.iface, d.x-ctx.measureText(d.iface).width/2, d.y+bh/2);
    // Pin info
    ctx.fillStyle = '#8b949e'; ctx.font = '7px system-ui';
    const plines = d.pins.split('\n');
    plines.forEach((l, i) => ctx.fillText(l, d.x+ctx.measureText(d.label.split('\n')[0]).width/2+4, d.y-bh/2+14+i*10));
  });

  // Legend
  const ly = cy+250;
  ctx.font = '9px system-ui';
  [['#79c0ff','USB'],['#ffa657','I2C'],['#f0883e','GPIO PWM'],['#a5d6ff','GPIO'],['#e5534b','Power'],['#3fb950','WiFi']].forEach(([c,n],i) => {
    const px = 20+i*100;
    ctx.fillStyle = c; ctx.fillRect(px, ly, 10, 10);
    ctx.fillStyle = '#8b949e'; ctx.fillText(n, px+13, ly+9);
  });
}

// ============================================================
//  VIEW 4: 机械结构 (三视图)
// ============================================================
function drawMechanical() {
  const w = W(), h = H();
  ctx.clearRect(0, 0, w, h);
  ctx.fillStyle = '#0d1117'; ctx.fillRect(0, 0, w, h);

  const thirdW = w / 3;
  const vh = h - 60;
  const midY = 40 + vh/2;

  const views = [
    {title:'俯视图 (Top)', x:0, draw:drawTopView},
    {title:'侧视图 (Side)', x:thirdW, draw:drawSideView},
    {title:'正视图 (Front)', x:thirdW*2, draw:drawFrontView},
  ];

  views.forEach(v => {
    ctx.fillStyle = '#8b949e'; ctx.font = '12px system-ui'; ctx.fillText(v.title, v.x+thirdW/2-ctx.measureText(v.title).width/2, 30);

    // Viewport border
    ctx.strokeStyle = '#30363d'; ctx.lineWidth = 1;
    ctx.strokeRect(v.x+10, 40, thirdW-20, vh);

    // Draw the view
    v.draw(v.x+10, 40, thirdW-20, vh);
  });

  // Legend below
  const ly = 40+vh+10;
  ctx.font = '9px system-ui';
  [['#4fc3f7','300×300×200mm · 1.5kg · 差分驱动'],
   ['JGB37-520','4×电机 · 12V · 30:1 · 300RPM · 11PPR'],
   ['MPU6050','IMU · I2C 0x68'],
   ['USB Cam','640×480 · 30fps · 俯角30°'],
  ].forEach(([n,desc],i) => {
    ctx.fillStyle = n.startsWith('#') ? n : '#8b949e';
    ctx.fillText(n+':', 20, ly+i*16);
    ctx.fillStyle = '#8b949e'; ctx.fillText(desc, 20+ctx.measureText(n+':').width, ly+i*16);
  });
}

function drawTopView(ox, oy, vw, vh) {
  const cx = ox+vw/2, cy = oy+vh/2;
  const s = Math.min(vw, vh) / 350 * 300 / 2; // scale: fit 300mm in view

  // Chassis outline
  ctx.fillStyle = '#1a1a1a'; ctx.strokeStyle = '#4fc3f7'; ctx.lineWidth = 1.5;
  roundedRect(cx-s, cy-s, s*2, s*2, 2, '#1a1a1a', '#4fc3f7');

  // Wheels
  const wr = s * 65/300, wh = s * 6/300;
  ctx.fillStyle = '#333'; ctx.strokeStyle = '#555'; ctx.lineWidth = 1;
  [[s*0.9, s*0.85], [s*0.9, -s*0.85], [-s*0.9, s*0.85], [-s*0.9, -s*0.85]].forEach(([wx,wy]) => {
    ctx.beginPath(); ctx.ellipse(cx+wx, cy+wy, wr, wh, 0, 0, Math.PI*2); ctx.fill(); ctx.stroke();
  });

  // Motor positions
  ctx.fillStyle = '#f0883e55'; ctx.strokeStyle = '#f0883e';
  [[s*0.6, s*0.7], [s*0.6, -s*0.7], [-s*0.6, s*0.7], [-s*0.6, -s*0.7]].forEach(([mx,my]) => {
    ctx.fillRect(cx+mx-8, cy+my-8, 16, 16);
    ctx.strokeRect(cx+mx-8, cy+my-8, 16, 16);
  });
  ctx.fillStyle = '#f0883e'; ctx.font = '7px system-ui'; ctx.fillText('M', cx+s*0.6-3, cy+s*0.7+4);
  ctx.fillText('M', cx+s*0.6-3, cy-s*0.7+4);
  ctx.fillText('M', cx-s*0.6-3, cy+s*0.7+4);
  ctx.fillText('M', cx-s*0.6-3, cy-s*0.7+4);

  // Pusher plate (front)
  ctx.fillStyle = '#f778ba44'; ctx.strokeStyle = '#f778ba'; ctx.lineWidth = 1;
  roundedRect(cx+s-4, cy-s*0.5, s*0.15, s*1.0, 1, '#f778ba44', '#f778ba');
  ctx.fillStyle = '#f778ba'; ctx.font = '7px system-ui'; ctx.fillText('推板', cx+s-2, cy+s*0.3);

  // IMU
  ctx.fillStyle = '#ffa657'; ctx.fillRect(cx-4, cy-4, 8, 8);
  ctx.fillStyle = '#ffa657'; ctx.font = '6px system-ui'; ctx.fillText('IMU', cx-6, cy+14);

  // Camera (front)
  ctx.fillStyle = '#79c0ff'; ctx.fillRect(cx+s+2, cy-4, 8, 5);
  ctx.fillStyle = '#79c0ff'; ctx.font = '6px system-ui'; ctx.fillText('CAM', cx+s-4, cy-10);

  // Ultrasonic (front)
  ctx.fillStyle = '#a5d6ff'; ctx.beginPath(); ctx.arc(cx+s, cy-s*0.2, 3, 0, Math.PI*2); ctx.fill();
  ctx.fillText('US', cx+s-2, cy-s*0.2-8);

  // Dimensions
  ctx.strokeStyle = '#8b949e'; ctx.lineWidth = 0.5;
  // Width arrow
  const dy = cy+s+15;
  ctx.beginPath(); ctx.moveTo(cx-s, dy); ctx.lineTo(cx+s, dy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx-s, dy-4); ctx.lineTo(cx-s, dy+4); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx+s, dy-4); ctx.lineTo(cx+s, dy+4); ctx.stroke();
  ctx.fillStyle = '#8b949e'; ctx.font = '8px system-ui'; ctx.fillText('300mm', cx-12, dy-6);
  // Height arrow (vertical)
  const dx = cx-s-15;
  ctx.beginPath(); ctx.moveTo(dx, cy-s); ctx.lineTo(dx, cy+s); ctx.stroke();
  ctx.fillText('300mm', dx-22, cy);
}

function drawSideView(ox, oy, vw, vh) {
  const cx = ox+vw/2, cy = oy+vh/2;
  const sx = vw/300*150, sy = vh/200*100; // scale for 300mm wide x 200mm tall

  // Chassis
  ctx.fillStyle = '#1a1a1a'; ctx.strokeStyle = '#4fc3f7'; ctx.lineWidth = 1.5;
  roundedRect(cx-sx, cy-sy, sx*2, sy*2, 2, '#1a1a1a', '#4fc3f7');

  // Wheel (side view = circle)
  const wr = sy * 65/200;
  ctx.fillStyle = '#333'; ctx.strokeStyle = '#555';
  ctx.beginPath(); ctx.arc(cx-sx*0.7, cy+sy-wr*0.3, wr, 0, Math.PI*2); ctx.fill(); ctx.stroke();
  ctx.beginPath(); ctx.arc(cx+sx*0.7, cy+sy-wr*0.3, wr, 0, Math.PI*2); ctx.fill(); ctx.stroke();

  // Camera mount (front, angled 30°)
  const camX = cx+sx, camY = cy-sy*0.3;
  ctx.fillStyle = '#79c0ff'; ctx.fillRect(camX, camY-3, 10, 6);
  ctx.fillStyle = '#79c0ff'; ctx.font = '6px system-ui'; ctx.fillText('CAM', camX-10, camY-8);
  // Angle indicator
  ctx.strokeStyle = '#79c0ff55'; ctx.lineWidth = 1;
  ctx.setLineDash([2, 3]);
  ctx.beginPath(); ctx.moveTo(camX, camY); ctx.lineTo(camX+20, camY+10); ctx.stroke();
  ctx.setLineDash([]);
  ctx.fillStyle = '#79c0ff'; ctx.font = '7px system-ui'; ctx.fillText('30°', camX+12, camY+1);

  // Pusher
  ctx.fillStyle = '#f778ba44'; ctx.strokeStyle = '#f778ba';
  ctx.fillRect(camX-10, cy-sy*0.3, 10, sy*0.6);
  ctx.strokeRect(camX-10, cy-sy*0.3, 10, sy*0.6);
  ctx.fillStyle = '#f778ba'; ctx.font = '6px system-ui'; ctx.fillText('推板', camX-10, cy-sy*0.4);

  // Dimensions
  ctx.strokeStyle = '#8b949e'; ctx.lineWidth = 0.5;
  // Length
  const dy2 = cy+sy+15;
  ctx.beginPath(); ctx.moveTo(cx-sx, dy2); ctx.lineTo(cx+sx, dy2); ctx.stroke();
  ctx.fillStyle = '#8b949e'; ctx.font = '8px system-ui'; ctx.fillText('300mm', cx-12, dy2-6);
  // Height
  const dx2 = cx-sx-15;
  ctx.beginPath(); ctx.moveTo(dx2, cy-sy); ctx.lineTo(dx2, cy+sy); ctx.stroke();
  ctx.fillText('200mm', dx2-22, cy);
  // Wheel diameter
  ctx.fillText('φ65mm', cx-sx*0.7-15, cy+sy-wr*1.3);
}

function drawFrontView(ox, oy, vw, vh) {
  const cx = ox+vw/2, cy = oy+vh/2;
  const sx = vw/300*150, sy = vh/200*100;

  // Chassis
  ctx.fillStyle = '#1a1a1a'; ctx.strokeStyle = '#4fc3f7'; ctx.lineWidth = 1.5;
  roundedRect(cx-sy, cy-sx, sy*2, sx*2, 2, '#1a1a1a', '#4fc3f7'); // swapped: width=300(height), height=300(width)

  // Wheels (front view = 2 circles at bottom corners)
  const wr = sy * 65/300;
  ctx.fillStyle = '#333'; ctx.strokeStyle = '#555';
  ctx.beginPath(); ctx.arc(cx-sy*0.6, cy+sx-wr*0.3, wr, 0, Math.PI*2); ctx.fill(); ctx.stroke();
  ctx.beginPath(); ctx.arc(cx+sy*0.6, cy+sx-wr*0.3, wr, 0, Math.PI*2); ctx.fill(); ctx.stroke();

  // Camera (top center)
  ctx.fillStyle = '#79c0ff'; ctx.fillRect(cx-4, cy-sx-6, 8, 6);
  ctx.fillText('CAM', cx-8, cy-sx-12);

  // Dimensions
  ctx.strokeStyle = '#8b949e'; ctx.lineWidth = 0.5;
  // Width
  const dy3 = cy+sx+15;
  ctx.beginPath(); ctx.moveTo(cx-sy, dy3); ctx.lineTo(cx+sy, dy3); ctx.stroke();
  ctx.fillStyle = '#8b949e'; ctx.font = '8px system-ui'; ctx.fillText('300mm', cx-12, dy3-6);
  // Height
  const dx3 = cx-sy-15;
  ctx.beginPath(); ctx.moveTo(dx3, cy-sx); ctx.lineTo(dx3, cy+sx); ctx.stroke();
  ctx.fillText('200mm', dx3-22, cy);
}

// ============================================================
//  MAIN DRAW DISPATCH
// ============================================================
function draw() {
  switch(currentView) {
    case 'scene': drawScene(); break;
    case 'software': drawSoftware(); break;
    case 'hardware': drawHardware(); break;
    case 'mechanical': drawMechanical(); break;
  }
}

// Initial draw
setFrame(0);
updateStatusBar();
draw();
</script>
</body>
</html>'''


def main():
    out = os.path.expanduser('~/Desktop/sim_2d_viewer.html')

    print("Running 2D simulation...")
    data = run_simulation(seed=42, target_count=15, duration_s=30.0)
    print(f"  {len(data['frames'])} frames, {data['target_count']} targets")
    print(f"  Final: score={data['final_score']}, delivered={data['final_delivered']}")

    data_json = json.dumps(data, separators=(',', ':'), ensure_ascii=False)
    print(f"  JSON data: {len(data_json):,} chars")

    html = HTML.replace('__DATA_PLACEHOLDER__', data_json)
    with open(out, 'w', encoding='utf-8') as f:
        f.write(html)

    print(f"\nDone! Wrote {len(html):,} chars to:")
    print(f"  {out}")
    print(f"\nDouble-click sim_2d_viewer.html to open.")


if __name__ == '__main__':
    main()
