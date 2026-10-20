'use strict';
const UI_VERSION='20261019-r15-ui1';
const $ = id => document.getElementById(id);
let ws, session = '', challenge = '', sequence = 0, pressed = '', frame = '', frameAt = -Infinity;
let state = {}, mapImage = null, displayMap = null, mapVersion = '', mapLoading = false, hint = null, choosing = false;
let scale = 30, cx = 0, cy = 0, fitted = false, lastStateAt = -Infinity;
let pendingOperation = 0, clientError = '', reconnectTimer = null, taskKey = '', bundlesLoaded = false;
let activePointer = null;
let lastControlAt = -Infinity, lastDriveSentAt = -Infinity, lastLoopAt = -Infinity, latestControl = {}, lastFault = null;
let lastHeartbeatSentAt = -Infinity,lastHeartbeatChallenge='',lastDriveChallenge='';
let pressChallenge='';
let controlGapReported=false;
let rtcTransport=null,rtcActive=false,transportAwaitingStop=false,lastControlRevision=0;
let lastControlEpoch='';const retiredControlEpochs=[];
const clientEvents = [], sentAt = new Map(); let roundTripMs = null;
const pendingClientEvents=[];
let statusPath='/api/status',scanPoints=[],scanVersion='',lastScanAt=-Infinity,scanSourceAgeMs=0,previewGeneration=0;
const scanFresh=()=>['CURRENT','WAITING','CANDIDATE'].includes(state.phase)&&
  performance.now()-lastScanAt+scanSourceAgeMs<500;
function clearScan(){scanPoints=[];scanVersion='';lastScanAt=-Infinity;previewGeneration++;}
const controlFresh = () => performance.now()-lastControlAt < (latestControl.challenge_lifetime_ms||latestControl.control_lease_ms||200);
function clientLog(event, detail = {}) {
  const entry={event,client_time:new Date().toISOString(),control_gap_ms:Math.round(performance.now()-lastControlAt),
    http_gap_ms:Math.round(performance.now()-lastStateAt),round_trip_ms:roundTripMs,...detail};
  clientEvents.push(entry);if(clientEvents.length>128)clientEvents.shift();
  pendingClientEvents.push({action:'client_event',event,metrics:entry});
  if(pendingClientEvents.length>8)pendingClientEvents.shift();
}
const canvas = $('map'), ctx = canvas.getContext('2d'), video = $('video'), vctx = video.getContext('2d');
const phases = {STANDBY:'网页待命 · 硬件未启动',STOPPING:'正在关闭本次任务',WAITING:'等待建图输入',CURRENT:'实时建图',REFERENCE:'历史参考图：请选择初始位置',LOADING:'正在恢复工程',TRIAL:'首帧试匹配',CANDIDATE:'匹配候选：核对后确认',FAILED:'建图锁停：查看诊断后重启',SAVING:'已停车，正在保存'};
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
async function fetchTimed(url, options = {}, timeout = 1500) {
  const abort = new AbortController(), timer = setTimeout(() => abort.abort(), timeout);
  try {return await fetch(url, {...options, signal:abort.signal, cache:'no-store'});} finally {clearTimeout(timer);}
}
function send(value) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const realtime=['drive','intent','heartbeat'].includes(value.action);
  if ((!rtcActive||!realtime)&&ws.bufferedAmount > 4096) {clientLog('backpressure');pressed = '';resetStick(); ws.close(); return;}
  if(value.sequence){sentAt.set(value.sequence,performance.now());if(sentAt.size>128)sentAt.delete(sentAt.keys().next().value);}
  if(value.challenge&&value.challenge===challenge)value.response_delay_ms=Math.max(0,performance.now()-lastControlAt);
  const text=JSON.stringify(value);
  if(rtcActive&&(realtime||value.action==='release'||value.action==='stop'))rtcTransport?.send(text);
  // 同一条运动同时走可靠通道。UDP丢单包时，车端仍按原签发时刻计算300ms，迟到副本不能续租。
  if(rtcActive&&realtime&&ws.bufferedAmount<=4096)ws.send(JSON.stringify({...value,backup:true}));
  if(!rtcActive||!realtime)ws.send(text);
}
function resetStick() {activePointer = null;pressChallenge=''; $('stick').style.transform = 'translate(0,0)';}
function clearControlConnection() {
  rtcTransport?.close();rtcTransport=null;rtcActive=false;transportAwaitingStop=false;lastControlRevision=0;
  lastControlEpoch='';retiredControlEpochs.length=0;
  session='';challenge='';pressed='';latestControl={};lastControlAt=-Infinity;
  lastDriveSentAt=lastHeartbeatSentAt=-Infinity;roundTripMs=null;sentAt.clear();resetStick();
  lastHeartbeatChallenge=lastDriveChallenge='';
  controlGapReported=false;
  pendingClientEvents.length=0;
  $('enable').disabled=true;joystick.classList.add('disabled');
}
function stop(cause='manual') {if(typeof cause!=='string')cause='manual';if(pressed||cause==='manual')clientLog(cause);pressed = '';lastDriveSentAt=-Infinity; resetStick(); send({action:'stop',cause});}
function release() {clientLog('pointer_release');pressed = '';lastDriveSentAt=-Infinity; resetStick(); send({action:'release',sequence:++sequence});}
function drive() {
  if (!pressed) return;
  // 按住就按固定频率重复当前方向。不靠新挑战续租，状态晚到也不在浏览器里撤使能。
  const nowMs=performance.now();
  if(nowMs-lastDriveSentAt<50)return;
  lastDriveSentAt=nowMs;lastDriveChallenge=challenge;
  const value = typeof pressed === 'string' ? {action:'intent',direction:pressed} : {action:'drive',...pressed};
  send({...value,sequence:++sequence,challenge,frame});
}
function heartbeat() {
  if(!session||!challenge||document.hidden||!controlFresh())return;
  if(latestControl.idle_heartbeat_required===false)return;
  // 车端最多20Hz签发；每个新挑战回应一次。定时器不能重复旧挑战，也不能节流掉新挑战。
  if(rtcActive&&!latestControl.armed)return;
  if(rtcActive?challenge===lastHeartbeatChallenge||challenge===lastDriveChallenge:
    performance.now()-lastHeartbeatSentAt<30&&challenge===lastHeartbeatChallenge)return;
  lastHeartbeatSentAt=performance.now();lastHeartbeatChallenge=challenge;send({action:'heartbeat',sequence:++sequence,challenge});
}
function cancelReconnect() {
  if (reconnectTimer !== null) clearTimeout(reconnectTimer);
  reconnectTimer = null;
}
function disconnect() {
  stop('disconnect'); cancelReconnect();
  const socket = ws; ws = null;clearControlConnection();
  socket?.close();
  $('connection').textContent = '控制已断开，已撤销授权';
}
function beginRtc(socket) {
  if(typeof RTCPeerConnection==='undefined'||typeof MappingRtcTransport==='undefined'||$('transport').value==='websocket')return;
  const signal=value=>{if(socket===ws&&socket.readyState===WebSocket.OPEN)socket.send(JSON.stringify(value));};
  let transport;
  try{
    transport=new MappingRtcTransport(signal,text=>{
      if(socket!==ws||rtcTransport!==transport||!rtcActive)return;
      try{receiveControl(JSON.parse(text));}catch(_){stop('rtc_invalid_state');}
    },(status,why)=>{
      if(socket!==ws||rtcTransport!==transport)return;
      // 通道接通或回退都不撤销已有使能。正在推杆时可靠通道继续续租，静止就绪也不改成停车。
      rtcActive=status==='open';transportAwaitingStop=false;
      if(!rtcActive){rtcTransport=null;}
      $('connection').textContent=rtcActive?'控制已连接 · WebRTC':'控制已连接 · WebSocket';
      clientLog(rtcActive?'rtc_connected':'rtc_fallback',{why:why||''});
    });
    rtcTransport=transport;transport.start();
  }catch(_){signal({action:'rtc_cancel'});clientLog('rtc_unavailable');}
}
function receiveControl(value) {
    if(typeof value.control_epoch==='string'&&value.control_epoch!==lastControlEpoch){
      if(retiredControlEpochs.includes(value.control_epoch))return;
      if(lastControlEpoch){
        retiredControlEpochs.push(lastControlEpoch);if(retiredControlEpochs.length>8)retiredControlEpochs.shift();
        stop('guard_restarted');latestControl={};lastControlAt=-Infinity;transportAwaitingStop=true;
        lastHeartbeatSentAt=lastDriveSentAt=-Infinity;lastHeartbeatChallenge=lastDriveChallenge='';
      }
      lastControlEpoch=value.control_epoch;lastControlRevision=0;
    }
    // UDP可以乱序；迟到/重复的状态不能刷新本地新鲜度或恢复旧使能显示。
    if(value.control_revision&&value.control_revision<=lastControlRevision)return;
    if(transportAwaitingStop){if(value.armed)return;transportAwaitingStop=false;}
    if(value.control_revision)lastControlRevision=value.control_revision;
    const previouslyArmed=latestControl.armed;
    lastControlAt=performance.now();latestControl=value;controlGapReported=false;
    challenge=value.challenge||'';
    if(!value.armed){pressed='';resetStick();}
    if(value.armed&&pressed&&!document.hidden)drive();else if(!pressed)heartbeat();
    if(previouslyArmed&&!value.armed&&!value.last_stop?.server_steady){lastFault={source:'browser_observed_guard',reason:value.reason,health:value.health,client_time:new Date().toISOString()};clientLog('guard_stop',{reason:value.reason});}
    const ack=value.accepted_sequence??value.ack_sequence;
    if(sentAt.has(ack)){roundTripMs=Math.round(performance.now()-sentAt.get(ack));sentAt.delete(ack);}
    if(value.last_stop?.server_steady&&value.last_stop.server_steady!==lastFault?.server_steady){lastFault=value.last_stop;clientLog('guard_stop',{reason:lastFault.reason});}
    $('motion').textContent=value.reason||'未使能';
    $('enable').disabled=!!value.armed||!!value.health||state.phase!=='CURRENT'||state.busy||state.save_failed||!session||!controlFresh();
    joystick.classList.toggle('disabled',!value.armed||state.busy||!session);
    if(value.gear&&document.activeElement!==$('gear'))$('gear').value=value.gear;
}
function connect() {
  cancelReconnect();
  if (document.hidden || (ws && ws.readyState !== WebSocket.CLOSED)) return;
  const socket = new WebSocket(`ws://${location.host}/ws/control`); ws = socket;
  socket.onmessage = event => {
    if (socket !== ws) return;
    const value = JSON.parse(event.data);
    if(value.rtc_description||value.rtc_candidate||value.rtc_status){rtcTransport?.handle(value);return;}
    if (value.session) {session = value.session;
      if(value.transport_protocol>=1)send({action:'transport',profile:'compact'});
      clientLog('ui_r15');
      $('connection').textContent = '控制已连接 · WebSocket';if(value.rtc_protocol>=1)beginRtc(socket);return;}
    if(!rtcActive)receiveControl(value);
  };
  const closed = event => {
    if (socket !== ws) return;
    const owned = !!session; ws = null;clearControlConnection();
    clientLog('socket_closed',{ws_close_code:event?.code??0,ws_close_was_clean:event?.wasClean?1:0});
    $('connection').textContent = owned?'本页控制断开，已撤销授权':'未取得控制连接，请检查网络或其他操作者';
    cancelReconnect();
    if (!document.hidden) reconnectTimer = setTimeout(() => {reconnectTimer = null; connect();},1000);
  };
  socket.onclose = closed;
  socket.onerror = () => {
    if(socket!==ws)return;
    // 失败握手也必须结束本地状态；不依赖浏览器随后一定补发close事件。
    closed();try{socket.close();}catch(_){}
  };
}
connect();
setInterval(() => {
  const now=performance.now();if(lastLoopAt>0&&now-lastLoopAt>120)clientLog('script_delay',{loop_gap_ms:Math.round(now-lastLoopAt)});lastLoopAt=now;
  // 未使能/未推杆时也记录断流，允许停车状态下诊断通信，单次断流只报一次。
  if(session&&Number.isFinite(lastControlAt)&&!controlFresh()&&!controlGapReported){
    controlGapReported=true;clientLog('control_gap');
  }
  if (pressed && !document.hidden) drive();
  else if (now-lastHeartbeatSentAt>=(rtcActive?50:100))heartbeat();
},50);
// 诊断事件有界、低频发送，不插在控制回应之前；详细历史仍保留于页面。
setInterval(()=>{
  if(ws?.readyState===WebSocket.OPEN&&ws.bufferedAmount<=4096&&pendingClientEvents.length)
    ws.send(JSON.stringify(pendingClientEvents.shift()));
},250);
$('enable').onclick = () => {if(state.inspection_running)return;clientLog('enable');send({action:'enable',frame});};
$('stop').onclick = () => stop('manual');
$('gear').onchange = () => {stop('gear');clientLog('gear',{gear:$('gear').value});send({action:'gear',gear:$('gear').value});};
const joystick = $('joystick');
function moveStick(event) {
  const r = joystick.getBoundingClientRect(), knobRadius = ($('stick').offsetWidth || 56) / 2;
  const radius = Math.max(1, r.width / 2 - knobRadius);
  let dx = (event.clientX-r.left-r.width/2)/radius, dy = (event.clientY-r.top-r.height/2)/radius;
  const norm = Math.max(1, Math.hypot(dx,dy)); dx /= norm; dy /= norm;
  $('stick').style.transform = `translate(${dx*radius}px,${dy*radius}px)`;
  const magnitude = Math.hypot(dx,dy), gain = magnitude < .12 ? 0 : (magnitude-.12)/(.88*magnitude);
  pressed = {x:-dy*gain,yaw:-dx*gain}; drive();
}
joystick.onpointerdown = event => {
  event.preventDefault();
  if (!latestControl.armed || state.busy || !session || pressed || activePointer !== null) return;
  clientLog('pointer_down');
  pressChallenge=latestControl.idle_heartbeat_required===false?challenge:'';
  activePointer = event.pointerId; joystick.setPointerCapture(event.pointerId); moveStick(event);
};
joystick.onpointermove = event => {if (activePointer === event.pointerId) moveStick(event);};
joystick.onpointerup = event => {if (activePointer === event.pointerId) release();};
joystick.onpointercancel = event => {if (activePointer === event.pointerId) stop('pointer_cancel');};
joystick.onlostpointercapture = event => {if (activePointer === event.pointerId) stop('pointer_capture_lost');};
joystick.oncontextmenu = event => event.preventDefault();
// 切换到聊天/其他窗口也会触发blur；只有尚在推杆时才需要立即清除运动意图。
// 静止使能仍由新鲜挑战续租，页面隐藏/关闭和控制超时分别撤销连接或权限。
window.addEventListener('blur', () => {if(pressed)stop('window_blur');}); window.addEventListener('pagehide', disconnect);
document.addEventListener('visibilitychange', () => {
  clientLog(document.hidden?'page_hidden':'page_visible');
  if (document.hidden) disconnect(); else connect();
});
async function operation(path, body = {}) {
  stop('operation');clientLog('operation',{operation:path}); clientError='';
  if(['/api/start','/api/restart','/api/end'].includes(path)){clearScan();lastControlRevision=0;}
  try {
    const response = await fetchTimed(path,{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},body:JSON.stringify({...body,session})},5000);
    const value = await response.json(); if (!response.ok) throw Error(value.error || '操作失败');
    if(value.accepted)pendingOperation=value.operation;
    $('result').textContent = value.accepted ? '请求已接收，等待完成结果；车辆保持停车。' : '操作完成，按界面提示继续。';
  } catch (error) {clientError=error.message;$('result').textContent=clientError;}
}
$('save').onclick = () => operation('/api/save');
$('finish').onclick = () => operation('/api/finish');
$('mode').onchange = () => {$('bundle').hidden = $('mode').value !== 'resume';};
async function refreshBundles() {
  try {
    const r = await fetchTimed('/api/bundles',{},10000), value = await r.json(); if (!r.ok) throw Error(value.error);
    $('bundle').replaceChildren();
    for (const b of value.bundles) {const option = document.createElement('option'); option.value = b.id; option.textContent = b.id + (b.valid ? '' : '（不可续建：校验失败）'); option.disabled = !b.valid; $('bundle').append(option);}
    if (!value.bundles.some(b=>b.valid)) {const option=document.createElement('option');option.textContent='暂无完整有效工程';option.value='';$('bundle').append(option);}
    bundlesLoaded = true;
  } catch (e) {$('result').textContent = e.message;}
}
$('refresh-bundles').onclick = refreshBundles;
$('start').onclick = () => {
  if ($('mode').value === 'resume' && !$('bundle').value) {$('result').textContent='请选择完整有效的旧工程。';return;}
  if (!window.confirm('启动前确认：供电正常、轮子与周围人员安全、支架扫掠范围无干涉。底盘启动会发送S1=94，支架可能转动；车辆保持停车，之后另行使能。')) return;
  operation('/api/start',{mode:$('mode').value,bundle:$('mode').value==='resume'?$('bundle').value:''});
};
$('end').onclick = () => {if (window.confirm('结束会关闭本次建图硬件与服务。尚未保存的进度将放弃，旧工程保留；确定结束？')) operation('/api/end',{discard:true});};
$('restart').onclick = () => {if (window.confirm('重启会丢失未保存进度，并重新启动硬件（支架可能转动）。有成功保存版本时从最近版本续建，需重新核对位置；确认实物已停止且支架安全后继续。')) operation('/api/restart',{acknowledge_loss:true});};
$('reconnect').onclick = () => {disconnect(); connect();};
$('transport').onchange = () => {disconnect();connect();};
$('video-toggle').onclick = () => operation('/api/video',{enabled:!state.video_requested});
$('diagnose').onclick = async () => {
  $('diagnostics').hidden=false;
  try {const r=await fetchTimed('/api/diagnostics');const d=await r.json();delete d.control?.challenge;
    $('diagnostics').textContent=JSON.stringify({ui_version:UI_VERSION,phase:d.phase,task_state:d.task_state,error:d.error,control:d.control||{},transport:d.transport||{},reply_timing:d.reply_timing||{},last_stop:lastFault||d.control?.last_stop||null,client_events:clientEvents,server_events:d.events||[],health:d.control?.health,task_log:d.task_log,camera_log:d.camera_log},null,2);
  }catch(e){$('diagnostics').textContent=JSON.stringify({ui_version:UI_VERSION,error:e.message,last_stop:lastFault,client_events:clientEvents},null,2);}
};
$('export-diagnostics').onclick = () => {const file=new Blob([$('diagnostics').textContent||JSON.stringify({last_stop:lastFault,client_events:clientEvents},null,2)],{type:'application/json'});const url=URL.createObjectURL(file);const a=document.createElement('a');a.href=url;a.download='mapping-diagnostics-'+Date.now()+'.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
$('try').onclick = () => {if (hint) operation('/api/resume',{pose:hint});};
$('confirm').onclick = () => operation('/api/confirm');
$('hint').onclick = () => {stop(); choosing = !choosing; $('hint-info').textContent = choosing ? '在旧图空闲处按下车体中心，拖动到车头方向后松开。此箭头仅是人工初值。' : '';};
function screenToWorld(x,y) {return [cx+(x-canvas.clientWidth/2)/scale,cy-(y-canvas.clientHeight/2)/scale];}
function fit() {
  const m = displayMap; if (!m) return;
  const [ox,oy,yaw] = m.origin, w=m.width*m.resolution, h=m.height*m.resolution;
  cx=ox+Math.cos(yaw)*w/2-Math.sin(yaw)*h/2; cy=oy+Math.sin(yaw)*w/2+Math.cos(yaw)*h/2;
  scale=Math.min(canvas.clientWidth/(Math.abs(Math.cos(yaw))*w+Math.abs(Math.sin(yaw))*h),canvas.clientHeight/(Math.abs(Math.sin(yaw))*w+Math.abs(Math.cos(yaw))*h))*.9;
  fitted=true;
}
$('fit').onclick=fit; $('center').onclick=()=>{if(state.pose){cx=state.pose[0];cy=state.pose[1];}};
const pointers=new Map(); let gesture=null;
function point(event){const r=canvas.getBoundingClientRect();return [event.clientX-r.left,event.clientY-r.top];}
canvas.onpointerdown=event=>{
  event.preventDefault(); canvas.setPointerCapture(event.pointerId); const p=point(event); pointers.set(event.pointerId,p);
  if(choosing&&pointers.size===1){const world=screenToWorld(...p);hint=[...world,0];gesture={kind:'hint'};}
  else if(pointers.size===1) gesture={kind:'pan',point:p,cx,cy};
  else {const a=[...pointers.values()];gesture={kind:'pinch',distance:Math.hypot(a[0][0]-a[1][0],a[0][1]-a[1][1]),scale};}
};
canvas.onpointermove=event=>{
  if(!pointers.has(event.pointerId)||!gesture)return; const p=point(event);pointers.set(event.pointerId,p);
  if(gesture.kind==='hint'&&hint){const world=screenToWorld(...p);hint[2]=Math.atan2(world[1]-hint[1],world[0]-hint[0]);$('hint-info').textContent=`人工初值 x=${hint[0].toFixed(2)}m，y=${hint[1].toFixed(2)}m，朝向=${(hint[2]*180/Math.PI).toFixed(1)}°；尚未对齐。`;}
  else if(pointers.size===2&&gesture.kind==='pinch'){const a=[...pointers.values()];scale=Math.max(2,Math.min(1500,gesture.scale*Math.hypot(a[0][0]-a[1][0],a[0][1]-a[1][1])/Math.max(gesture.distance,1)));}
  else if(gesture.kind==='pan'){cx=gesture.cx-(p[0]-gesture.point[0])/scale;cy=gesture.cy+(p[1]-gesture.point[1])/scale;}
};
canvas.onpointerup=canvas.onpointercancel=event=>{pointers.delete(event.pointerId);gesture=null;};
canvas.onwheel=event=>{event.preventDefault();const p=point(event),before=screenToWorld(...p);scale=Math.max(2,Math.min(1500,scale*Math.exp(-event.deltaY*.001)));const after=screenToWorld(...p);cx+=before[0]-after[0];cy+=before[1]-after[1];};
function arrow(p,color,dashed=false){
  if(!p)return;ctx.save();ctx.translate(p[0],p[1]);ctx.rotate(p[2]);ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=2/scale;if(dashed)ctx.setLineDash([5/scale,4/scale]);
  ctx.strokeRect(-12/scale,-10/scale,24/scale,20/scale);ctx.beginPath();ctx.moveTo(28/scale,0);ctx.lineTo(-8/scale,10/scale);ctx.lineTo(-8/scale,-10/scale);ctx.closePath();ctx.fill();ctx.restore();
}
let lastPaintAt=-Infinity;
function draw(at=performance.now()){
  if(at-lastPaintAt<33){requestAnimationFrame(draw);return;}lastPaintAt=at;
  const ratio=Math.min(window.devicePixelRatio||1,2),w=canvas.clientWidth,h=canvas.clientHeight;
  if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);}
  ctx.setTransform(1,0,0,1,0,0);ctx.fillStyle='#a0a0a0';ctx.fillRect(0,0,canvas.width,canvas.height);
  ctx.setTransform(scale*ratio,0,0,-scale*ratio,(w/2-cx*scale)*ratio,(h/2+cy*scale)*ratio);
  if(mapImage&&displayMap){const m=displayMap;ctx.save();ctx.translate(m.origin[0],m.origin[1]);ctx.rotate(m.origin[2]);ctx.translate(0,m.height*m.resolution);ctx.scale(m.resolution,-m.resolution);ctx.imageSmoothingEnabled=false;ctx.drawImage(mapImage,0,0);ctx.restore();}
  ctx.fillStyle='#ee3038';for(const p of scanFresh()?scanPoints:[])ctx.fillRect(p[0]-1.5/scale,p[1]-1.5/scale,3/scale,3/scale);
  arrow(state.pose,'#087edb');if(choosing||['REFERENCE','LOADING','TRIAL','CANDIDATE'].includes(state.phase))arrow(hint,'#edba28',true);
  ctx.setTransform(ratio,0,0,ratio,0,0);ctx.strokeStyle='#102230';ctx.lineWidth=3;const meters=scale>120?.25:scale>40?1:5;
  ctx.beginPath();ctx.moveTo(16,h-20);ctx.lineTo(16+meters*scale,h-20);ctx.stroke();ctx.fillStyle='#102230';ctx.font='14px sans-serif';ctx.fillText(`${meters} m`,16,h-28);
  requestAnimationFrame(draw);
}
draw();
const age=value=>Number.isFinite(value)&&value<10000?`${Math.max(0,value).toFixed(1)}秒`:'等待';
(async function poll(){
  while(true){
    if(!document.hidden)try{
      let response=await fetchTimed(statusPath);
      if(response.status===404){statusPath='/api/state';response=await fetchTimed(statusPath);}
      if(!response.ok)throw Error('状态不可用');state=await response.json();lastStateAt=performance.now();
      if(state.dataflow_protocol>=1)statusPath='/api/status';
      else{scanPoints=state.scan||[];lastScanAt=performance.now();scanSourceAgeMs=Math.max(0,(state.scan_age||0)*1000);}
      const newKey = `${state.task_state}:${state.bundle||''}`;
      if (taskKey !== newKey && ['STANDBY','STARTING'].includes(state.task_state)) {
        stop();clearScan(); mapImage?.close(); mapImage=null;displayMap=null;mapVersion='';fitted=false;hint=null;choosing=false;bundlesLoaded=false;
      }
      taskKey=newKey;
      if(choosing&&['CURRENT','SAVING','FAILED'].includes(state.phase)){choosing=false;gesture=null;$('hint-info').textContent='';}
      $('phase').textContent=phases[state.phase]||'等待建图服务';
      $('map-empty').hidden=!!mapImage;
      const control=controlFresh()?latestControl:state.control;
      $('motion').textContent=state.inspection_running?'巡检还在进行，请先在巡检页取消或等这轮结束':state.phase==='STANDBY'?'网页待命，尚未启动硬件':state.task_state==='STARTING'?'正在预检和启动，车辆保持停车':!controlFresh()?'控制状态过期，请检查连接':control?.reason||'等待运动门控';
      $('enable').disabled=!!state.inspection_running||!session||state.busy||state.save_failed||state.phase!=='CURRENT'||!controlFresh()||!!control?.armed||!!control?.health;
      joystick.classList.toggle('disabled',!control?.armed||state.busy||!session);
      $('fast-option').disabled=!control?.supported_gears?.includes('fast');
      $('save').disabled=state.phase!=='CURRENT'||state.busy||!session;
      $('finish').hidden=!state.managed;$('finish').disabled=$('save').disabled;
      const idle=state.task_state==='STANDBY';
      $('task-panel').hidden=!state.managed||!idle;
      $('start').disabled=!!state.inspection_running||!idle||state.busy||!session;$('mode').disabled=$('bundle').disabled=$('refresh-bundles').disabled=!idle||state.busy;
      $('bundle').hidden=$('mode').value!=='resume';
      $('task-info').textContent=idle?'网页待命，硬件未启动':`本次运行 ${Math.floor((state.task_age||0)/60)}分 · ${state.bundle||'新工程'}${state.dirty?' · 有未保存进度':''}`;
      $('task-summary').textContent=idle?'选择新建或续建后启动':`建图 ${Math.floor((state.task_age||0)/60)} 分${state.dirty?' · 待保存':''}`;
      const recovered=state.phase==='CURRENT'&&controlFresh()&&latestControl.armed&&!latestControl.health;
      $('fault-summary').textContent=lastFault?(recovered?'上次停车记录（当前已就绪）：':'停车记录：')+lastFault.reason:'';
      $('fault-summary').classList.toggle('history',!!recovered);
      $('restart').hidden=$('end').hidden=!state.managed||idle;
      $('restart').disabled=!session||state.busy;$('end').disabled=!session||(state.busy&&state.task_state!=='STARTING');
      $('video-toggle').hidden=!state.managed;$('video-toggle').disabled=!session||state.busy||state.task_state!=='RUNNING';
      $('video-toggle').textContent=state.video_requested?'关闭辅助视频':'开启辅助视频';$('video-panel').hidden=state.managed&&!state.video_requested;
      $('save-age').textContent=idle?'':`距上次成功保存（或本次启动） ${Math.floor((state.last_save_age||0)/60)}分${state.save_failed?' · 保存失败，锁停等待重试':''}${state.last_saved_bundle?' · 最近版本 '+state.last_saved_bundle:''}`;
      if(state.managed&&idle&&!bundlesLoaded) {bundlesLoaded=true;refreshBundles();}
      $('hint').hidden=$('try').hidden=!['REFERENCE','CANDIDATE'].includes(state.phase);$('try').disabled=!hint||state.busy;
      $('confirm').hidden=state.phase!=='CANDIDATE';
      $('health').textContent=`雷达 ${age(state.scan_age)} · 里程计 ${age(state.odom_age)} · 地图 ${age(state.map_age)} · 控制 ${controlFresh()?'正常':'过期'}${roundTripMs===null?'':' · 往返 '+roundTripMs+'ms'}`;
      const operationPending=pendingOperation&&(state.completed_operation||0)<pendingOperation;
      $('result').textContent=(clientError|| (operationPending?'请求已接收，等待本次操作完成；车辆保持停车。':state.error||state.result||''))+(state.save_reminder&&state.phase==='CURRENT'?'；已超过5分钟，请选择安全位置停车保存。':'');
      if(state.map&&state.map_version!==mapVersion&&!mapLoading){
        mapLoading=true;
        const version=state.map_version,key=taskKey;
        (async()=>{try{const r=await fetchTimed('/api/map.png',{},10000);if(!r.ok)throw Error();const meta=JSON.parse(r.headers.get('X-Map-Meta'));const image=await createImageBitmap(await r.blob());if(key!==taskKey){image.close();return;}mapImage?.close();mapImage=image;displayMap=meta;mapVersion=version;if(!fitted)fit();}catch(_){}finally{mapLoading=false;}})();
      }
    }catch(_){clientLog('http_poll_timeout');}
    await delay(250);
  }
})();
(async function scanLoop(){
  while(true){
    if(!document.hidden&&state.dataflow_protocol>=1&&['CURRENT','WAITING','CANDIDATE'].includes(state.phase)){
      const generation=previewGeneration,started=performance.now();
      try{
        const response=await fetchTimed('/api/scan'+(scanVersion?'?version='+encodeURIComponent(scanVersion):''));
        if(response.status!==204){
          if(!response.ok)throw Error('激光预览不可用');const value=await response.json();
          if(generation===previewGeneration){
            scanVersion=value.version;
            if(value.valid&&value.frame_id==='map'&&Array.isArray(value.points)){
              scanPoints=value.points;lastScanAt=performance.now();
              scanSourceAgeMs=Math.max(0,value.source_age_ms||0)+performance.now()-started;
            }else{scanPoints=[];lastScanAt=-Infinity;}
          }
        }
      }catch(_){if(!scanFresh())scanPoints=[];}
    }
    await delay(200);
  }
})();
(async function videoLoop(){
  while(true){
    if(!document.hidden && (state.video_requested || !state.managed))try{
      const response=await fetchTimed('/api/video.jpg');if(!response.ok)throw Error();
      const id=response.headers.get('X-Frame-Id'),image=await createImageBitmap(await response.blob());
      vctx.drawImage(image,0,0,640,480);image.close();if(id!==frame){frame=id;frameAt=performance.now();}
      $('video-status').textContent=performance.now()-frameAt<1000?'RGB辅助参照 · 640×480 · 目标5～6帧/秒':'视频过期，请目视车辆；不影响遥控';
      $('video-status').className=performance.now()-frameAt<1000?'good':'bad';video.classList.toggle('stale',performance.now()-frameAt>=1000);
    }catch(_){$('video-status').textContent='视频未就绪或断流，请目视车辆；不影响遥控';$('video-status').className='bad';video.classList.add('stale');}
    await delay(167);
  }
})();
