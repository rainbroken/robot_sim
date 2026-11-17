'use strict';
const $ = id => document.getElementById(id);
const clientId = globalThis.crypto?.randomUUID?.() || `web-${Date.now()}-${Math.random()}`;
let sequence = 0, state = null, connected = false, lastStateAt = 0, lastJob = '', mapBusy = false;
let mapInfo = null, mapImage = null, mapRevision = -1, driving = null, driveSending = false;
let tool = 'pan', follow = false, target = null, drag = null, fitted = false;
const view = {x: 0, y: 0, scale: 45};
const canvas = $('map'), ctx = canvas.getContext('2d');
let width = 0, height = 0;

function notice(message, type = '') {
  $('operation').textContent = message;
  $('operation').className = type;
}

async function request(path, options = {}) {
  const response = await fetch(path, {cache: 'no-store', signal: AbortSignal.timeout(6000), ...options});
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

async function command(action, data = {}, quiet = false) {
  try {
    const result = await request('/api/command', {method: 'POST',
      headers: {'Content-Type': 'application/json'}, body: JSON.stringify({action, data})});
    if (!quiet) notice(result.state === 'running' ? '操作处理中，请等待结果…' : (result.message || '操作已提交'));
    return result;
  } catch (error) {
    if (!quiet) notice(error.message, 'failed');
    throw error;
  }
}

function act(action, data = {}) {
  releaseDrive();
  return command(action, data).catch(() => {});
}

function badge(id, text, good, warn = false) {
  $(id).textContent = text;
  $(id).className = `badge${good ? ' good' : warn ? ' warn' : ''}`;
}

const navLabels = {idle: '无活动目标', sending: '发送目标中', navigating: '正在导航',
  canceling: '正在取消', succeeded: '已到达目标', canceled: '导航已取消', aborted: '导航失败', error: '导航异常'};

function renderStatus() {
  const s = state || {}, backend = s.backend || {}, busy = s.job?.state === 'running';
  const mode = backend.mode || 'idle', sim = connected && s.sim_ready;
  badge('connection', connected ? 'ROS 已连接' : '连接中断', connected);
  badge('sim-status', sim ? '仿真运行中' : '仿真未就绪 / 已暂停', sim, !sim);
  badge('mode-status', {idle: '待机', mapping: '建图模式', navigation: '导航模式', error: '后端异常'}[mode], mode !== 'idle' && mode !== 'error', mode === 'error');
  $('start-mapping').disabled = !sim || busy;
  $('stop-backend').disabled = !connected || busy || mode === 'idle';
  $('save-map').disabled = !connected || !s.mapping_ready || mode !== 'mapping' || busy;
  $('start-navigation').disabled = !sim || busy || !$('map-select').value;
  $('send-goal').disabled = !connected || !s.navigation_ready || busy || ['sending', 'navigating', 'canceling'].includes(s.navigation?.state);
  $('set-initial').disabled = !sim || !s.frontend_ready || mode !== 'navigation' || busy || ['sending', 'navigating', 'canceling'].includes(s.navigation?.state);
  document.querySelectorAll('[data-drive]').forEach(el => { el.disabled = !connected || !s.manual_ready || busy; });
  $('ready-dot').className = sim && s.manual_ready ? 'good' : '';
  $('ready-text').textContent = !sim ? '等待 /clock 推进和 /odom' : busy ? '操作进行中' :
    mode === 'mapping' ? (s.mapping_ready ? '建图已就绪，可以遥控采集' : '静止等待重力初始化') :
    mode === 'navigation' ? (s.navigation_ready ? '定位与 Nav2 就绪' : '等待定位收敛 / Nav2 激活') : '请选择工作模式';
  $('map-root').textContent = s.map_root || '';
  $('view-title').textContent = backend.map_name ? `实时地图 · ${backend.map_name}` : '实时地图';
  $('linear').textContent = Number(s.odom?.linear || 0).toFixed(2);
  $('angular').textContent = Number(s.odom?.angular || 0).toFixed(2);
  $('localization').textContent = mode === 'mapping' ? '建图中 · 不使用地图定位' : (s.localization?.state || '等待定位');
  $('localization-detail').textContent = mode === 'navigation' ?
    `${s.localization_fresh ? '' : '状态已过期 · '}${s.localization?.message || '等待状态'}` : '导航模式下使用 Open3D ICP 定位';
  $('nav-state').textContent = navLabels[s.navigation?.state] || '无活动目标';
  const n = s.navigation || {};
  $('nav-detail').textContent = n.message || (n.distance_remaining != null ?
    `剩余 ${n.distance_remaining.toFixed(2)} m · 恢复 ${n.recoveries || 0} 次 · ${n.navigation_seconds || 0} s` :
    (s.nav2_active ? 'Nav2 已激活' : '等待 Nav2 激活'));
  $('robot-pose').textContent = s.pose ? `x ${s.pose[0].toFixed(2)}  y ${s.pose[1].toFixed(2)}  θ ${(s.pose[2]*180/Math.PI).toFixed(1)}°` : '等待 map → base_footprint';
  $('sim-time').textContent = `仿真时间 ${(s.sim_time || 0).toFixed(1)} s`;
  if (backend.mode === 'error' && !busy) notice(`后端已退出，exit=${backend.exit_code}，请查看日志后停止后端。`, 'failed');
}

async function pollState() {
  try {
    const next = await request('/api/state');
    state = next; connected = true; lastStateAt = performance.now();
    const job = next.job;
    if (job && job.state !== 'running' && `${job.id}:${job.state}` !== lastJob) {
      lastJob = `${job.id}:${job.state}`;
      notice(job.message, job.state);
      if (['save_map', 'start_navigation'].includes(job.action)) refreshMaps();
    }
    if (next.map_revision !== mapRevision && !mapBusy) loadMap();
    if (driving && (!next.manual_ready || next.job?.state === 'running')) releaseDrive();
  } catch {
    connected = false;
    releaseDrive();
  }
  renderStatus();
  setTimeout(pollState, 200);
}

async function refreshMaps() {
  try {
    const result = await request('/api/maps'), previous = $('map-select').value;
    $('map-select').replaceChildren();
    for (const map of result.maps) {
      const option = document.createElement('option');
      option.value = map.name; option.textContent = map.name + (map.ready ? '' : '（文件不完整）');
      option.disabled = !map.ready; option.title = map.error;
      $('map-select').append(option);
    }
    const valid = result.maps.filter(m => m.ready);
    const active = state?.backend?.map_name;
    if (valid.some(m => m.name === active)) $('map-select').value = active;
    else if (valid.some(m => m.name === previous)) $('map-select').value = previous;
    else $('map-select').value = valid[0]?.name || '';
    if (!result.maps.length) $('map-select').append(new Option('暂无地图，请先建图保存', ''));
    renderStatus();
  } catch (error) { notice(`读取地图列表失败：${error.message}`, 'failed'); }
}

async function loadMap() {
  mapBusy = true;
  try {
    const result = await request('/api/map');
    mapInfo = result.info; mapRevision = result.revision;
    if (!mapInfo || !mapInfo.width || !result.data.some(v => v >= 0)) {
      mapImage = null; fitted = false;
      $('empty-map').hidden = false;
      $('empty-map').style.display = 'flex';
      $('map-meta').textContent = '等待有效 /map';
    } else {
      mapImage = document.createElement('canvas');
      mapImage.width = mapInfo.width; mapImage.height = mapInfo.height;
      const c = mapImage.getContext('2d'), pixels = c.createImageData(mapInfo.width, mapInfo.height);
      result.data.forEach((value, index) => {
        const color = value < 0 ? [217, 227, 233] : value < 50 ? [250, 252, 253] : [57, 77, 91];
        pixels.data.set([...color, 255], index*4);
      });
      c.putImageData(pixels, 0, 0);
      $('empty-map').style.display = 'none';
      $('map-meta').textContent = `${mapInfo.width} × ${mapInfo.height} · ${mapInfo.resolution.toFixed(2)} m/格 · ${mapInfo.frame}`;
      if (!fitted) { fitMap(); fitted = true; }
    }
  } catch (error) { notice(`读取栅格地图失败：${error.message}`, 'failed'); }
  finally { mapBusy = false; }
}

function mapWorld(x, y) {
  const [ox, oy, a] = mapInfo.origin;
  return [ox + x*Math.cos(a) - y*Math.sin(a), oy + x*Math.sin(a) + y*Math.cos(a)];
}

function fitMap() {
  follow = false; $('follow').classList.remove('selected');
  if (!mapInfo || !mapImage) { view.x = 0; view.y = 0; view.scale = 45; return; }
  const w = mapInfo.width*mapInfo.resolution, h = mapInfo.height*mapInfo.resolution;
  const corners = [[0,0], [w,0], [0,h], [w,h]].map(([x,y]) => mapWorld(x,y));
  const xs = corners.map(p => p[0]), ys = corners.map(p => p[1]);
  view.x = (Math.max(...xs) + Math.min(...xs))/2;
  view.y = (Math.max(...ys) + Math.min(...ys))/2;
  view.scale = Math.max(5, Math.min(350, Math.min(width/(Math.max(...xs)-Math.min(...xs)+2), height/(Math.max(...ys)-Math.min(...ys)+2))));
}

function worldAt(px, py) { return [view.x + (px-width/2)/view.scale, view.y - (py-height/2)/view.scale]; }
function pointer(event) { const rect = canvas.getBoundingClientRect(); return [event.clientX-rect.left, event.clientY-rect.top]; }
function arrow(x, y, angle, color, size = 0.45) {
  ctx.save(); ctx.translate(x,y); ctx.rotate(angle);
  ctx.fillStyle = color; ctx.strokeStyle = 'white'; ctx.lineWidth = 1.5/view.scale;
  ctx.beginPath(); ctx.moveTo(size,0); ctx.lineTo(-size*.65,size*.55); ctx.lineTo(-size*.35,0); ctx.lineTo(-size*.65,-size*.55); ctx.closePath(); ctx.fill(); ctx.stroke(); ctx.restore();
}

function draw() {
  const dpr = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  if (width !== rect.width || height !== rect.height || canvas.width !== Math.round(rect.width*dpr)) {
    width = rect.width; height = rect.height; canvas.width = Math.round(width*dpr); canvas.height = Math.round(height*dpr);
  }
  ctx.setTransform(dpr,0,0,dpr,0,0); ctx.clearRect(0,0,width,height);
  if (follow && state?.pose) { view.x = state.pose[0]; view.y = state.pose[1]; }
  ctx.translate(width/2,height/2); ctx.scale(view.scale,-view.scale); ctx.translate(-view.x,-view.y);
  const span = Math.max(width,height)/view.scale;
  ctx.strokeStyle = '#dce5ea'; ctx.lineWidth = .7/view.scale; ctx.beginPath();
  const spacing = view.scale < 15 ? 5 : 1;
  for (let x = Math.floor((view.x-span)/spacing)*spacing; x < view.x+span; x += spacing) {ctx.moveTo(x,view.y-span);ctx.lineTo(x,view.y+span);}
  for (let y = Math.floor((view.y-span)/spacing)*spacing; y < view.y+span; y += spacing) {ctx.moveTo(view.x-span,y);ctx.lineTo(view.x+span,y);}
  ctx.stroke();
  if (mapImage) {
    ctx.save(); const [x,y,a] = mapInfo.origin; ctx.translate(x,y); ctx.rotate(a); ctx.scale(mapInfo.resolution,mapInfo.resolution);
    ctx.imageSmoothingEnabled = false; ctx.drawImage(mapImage,0,0); ctx.restore();
  }
  const live = connected && performance.now()-lastStateAt < 2000;
  if (live && state?.path?.length) {
    ctx.strokeStyle = '#4586d8'; ctx.lineWidth = 2.5/view.scale; ctx.beginPath();
    state.path.forEach(([x,y],i) => i ? ctx.lineTo(x,y) : ctx.moveTo(x,y)); ctx.stroke();
  }
  if (live && state?.scan) {ctx.fillStyle = '#e99a50'; for (const [x,y] of state.scan) ctx.fillRect(x-1/view.scale,y-1/view.scale,2/view.scale,2/view.scale);}
  if (live && state?.pose) arrow(...state.pose, '#087f78', .32);
  if (target) {
    ctx.strokeStyle = tool === 'initial' ? '#a16dd6' : '#4182d1';ctx.lineWidth = 1.5/view.scale;
    ctx.beginPath();ctx.arc(target[0],target[1],.25,0,Math.PI*2);ctx.stroke();
    arrow(...target, ctx.strokeStyle, .48);
  }
  $('scale-label').textContent = `1 m ≈ ${view.scale.toFixed(0)} px`;
  requestAnimationFrame(draw);
}

function selectTool(next) {
  tool = next;
  document.querySelectorAll('[data-tool]').forEach(b => b.classList.toggle('selected', b.dataset.tool === next));
  $('map-hint').textContent = {pan: '滚轮缩放 · 拖动平移', goal: '按下设定目标位置，拖动设定朝向，再点击“发送导航目标”', initial: '按下设定机器人初始位置，拖动设定朝向，再点击“设置初始位姿”'}[tool];
  canvas.style.cursor = next === 'pan' ? 'grab' : 'crosshair';
}

function syncTarget() {
  $('target-x').value = target[0].toFixed(2); $('target-y').value = target[1].toFixed(2);
  $('target-yaw').value = (target[2]*180/Math.PI).toFixed(1);
}

function targetData() {
  const values = ['target-x','target-y','target-yaw'].map(id => $(id).value.trim() === '' ? NaN : Number($(id).value));
  if (!values.every(Number.isFinite)) throw new Error('请填写有效的 X、Y 和朝向');
  if (Math.abs(values[2]) > 180) throw new Error('朝向必须在 -180°..180°');
  return {x: values[0], y: values[1], yaw: values[2]*Math.PI/180};
}

canvas.addEventListener('pointerdown', event => {
  if (event.button !== 0) return;
  follow = false; $('follow').classList.remove('selected');
  const [px,py] = pointer(event), [x,y] = worldAt(px,py);
  drag = {px,py,x,y,vx:view.x,vy:view.y};
  if (tool !== 'pan') {target = [x,y,0];syncTarget();}
  canvas.setPointerCapture(event.pointerId);
});
canvas.addEventListener('pointermove', event => {
  if (!drag) return;
  const [px,py] = pointer(event);
  if (tool === 'pan') {view.x = drag.vx-(px-drag.px)/view.scale;view.y = drag.vy+(py-drag.py)/view.scale;}
  else {const [x,y] = worldAt(px,py);target[2] = Math.atan2(y-drag.y,x-drag.x);syncTarget();}
});
for (const name of ['pointerup','pointercancel','lostpointercapture']) canvas.addEventListener(name, () => {drag = null;});
canvas.addEventListener('wheel', event => {
  event.preventDefault(); follow = false; $('follow').classList.remove('selected');
  const [px,py] = pointer(event), before = worldAt(px,py);
  view.scale = Math.max(5,Math.min(500,view.scale*Math.exp(-event.deltaY*.001)));
  const after = worldAt(px,py);view.x += before[0]-after[0];view.y += before[1]-after[1];
}, {passive:false});

function releaseDrive(beacon = false) {
  if (!driving) return;
  driving = null;
  document.querySelectorAll('[data-drive]').forEach(b => b.classList.remove('active'));
  const data = {client_id:clientId, sequence:++sequence};
  if (beacon) navigator.sendBeacon('/api/command', new Blob([JSON.stringify({action:'release',data})], {type:'application/json'}));
  else command('release',data,true).catch(() => {});
}

async function driveTick() {
  if (!driving || driveSending) return;
  if (!connected || !state?.manual_ready || performance.now()-lastStateAt > 1500) {releaseDrive();return;}
  const speed = Number($('speed').value), angular = 0.45;
  const [linear,turn] = {forward:[speed,0],backward:[-Math.min(.15,speed),0],left:[0,angular],right:[0,-angular]}[driving];
  driveSending = true;
  try { await command('velocity',{client_id:clientId,sequence:++sequence,linear,angular:turn},true); }
  catch (error) {releaseDrive();notice(error.message,'failed');}
  finally {driveSending = false;}
}

function startDrive(direction) {
  if (!connected || !state?.manual_ready || state?.job?.state === 'running') return;
  driving = direction;
  document.querySelectorAll('[data-drive]').forEach(b => b.classList.toggle('active',b.dataset.drive === direction));
  driveTick();
}

document.querySelectorAll('[data-drive]').forEach(button => {
  button.addEventListener('pointerdown', event => {event.preventDefault();button.setPointerCapture(event.pointerId);startDrive(button.dataset.drive);});
  for (const name of ['pointerup','pointercancel','lostpointercapture']) button.addEventListener(name,() => releaseDrive());
});
const keys = {w:'forward',ArrowUp:'forward',s:'backward',ArrowDown:'backward',a:'left',ArrowLeft:'left',d:'right',ArrowRight:'right'};
window.addEventListener('keydown', event => {
  if (['INPUT','SELECT','TEXTAREA'].includes(document.activeElement.tagName)) return;
  const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
  if (key === ' ') {event.preventDefault();if (!event.repeat) act('stop');return;}
  if (keys[key]) {event.preventDefault();if (!event.repeat) startDrive(keys[key]);}
});
window.addEventListener('keyup', event => {if (keys[event.key] || keys[event.key.toLowerCase()]) releaseDrive();});
window.addEventListener('blur',() => releaseDrive(true));
window.addEventListener('pagehide',() => releaseDrive(true));
document.addEventListener('visibilitychange',() => {if (document.hidden) releaseDrive(true);});
$('speed').addEventListener('input',() => {$('speed-value').textContent = `${Number($('speed').value).toFixed(2)} m/s`;});
$('emergency').onclick = () => act('stop');
$('manual-stop').onclick = () => act('stop');
$('start-mapping').onclick = () => {
  if (state?.backend?.mode === 'mapping' && !window.confirm('这会结束当前建图并清空未保存的数据。是否开始新建图？')) return;
  act('start_mapping');
};
$('stop-backend').onclick = () => act('stop_backend');
$('save-map').onclick = () => act('save_map',{name:$('map-name').value.trim()});
$('start-navigation').onclick = () => act('start_navigation',{name:$('map-select').value});
$('refresh-maps').onclick = refreshMaps;
$('map-select').onchange = renderStatus;
$('fit-map').onclick = fitMap;
$('follow').onclick = () => {follow = !follow;$('follow').classList.toggle('selected',follow);};
document.querySelectorAll('[data-tool]').forEach(b => {b.onclick = () => selectTool(b.dataset.tool);});
for (const [id,action] of [['send-goal','goal'],['set-initial','initial_pose']]) $(id).onclick = () => {
  try {act(action,targetData());} catch(error) {notice(error.message,'failed');}
};
for (const id of ['target-x','target-y','target-yaw']) $(id).oninput = () => {
  try {const p = targetData();target = [p.x,p.y,p.yaw];} catch {target = null;}
};
$('show-log').onclick = async () => {
  $('log').hidden = !$('log').hidden;
  if (!$('log').hidden) {
    try {const result = await request('/api/log');$('log').textContent = result.text || '尚无后端日志';}
    catch(error) {$('log').textContent = error.message;}
  }
};
const now = new Date();
$('map-name').value = `map_${now.getFullYear()}${String(now.getMonth()+1).padStart(2,'0')}${String(now.getDate()).padStart(2,'0')}_${String(now.getHours()).padStart(2,'0')}${String(now.getMinutes()).padStart(2,'0')}`;
setInterval(driveTick,100);
setInterval(() => {if (performance.now()-lastStateAt > 2000) {connected=false;releaseDrive();renderStatus();}},250);
draw();pollState();refreshMaps();
