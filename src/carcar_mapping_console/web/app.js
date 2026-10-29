'use strict';
const UI_VERSION='20261029-nav018-workflow';
const $ = id => document.getElementById(id);
let ws, session = '', challenge = '', sequence = 0, pressed = '', frame = '', frameAt = -Infinity;
let state = {}, mapImage = null, displayMap = null, mapVersion = '', mapLoading = false, hint = null, choosing = false;
let scale = 30, cx = 0, cy = 0, fitted = false, lastStateAt = -Infinity;
let pendingOperation = 0, clientError = '', reconnectTimer = null, taskKey = '', bundlesLoaded = false;
let operationSubmitting=false,autonomyNotice='',autonomyAction=null,bundleCatalog=[],preferredReviewBundle='';
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
function savedAutonomy(){return !!state.autonomy_enabled&&state.task_state==='RUNNING'&&state.autonomy?.phase==='REVIEW_REQUIRED'&&!!state.last_saved_bundle&&!state.dirty;}
function manualControlAllowed(){return !state.autonomy_enabled||state.autonomy?.phase==='MANUAL';}
function taskOperationBusy(){return operationSubmitting||!!(pendingOperation&&(state.completed_operation||0)<pendingOperation);}
function renderWorkflow(){
  const a=state.autonomy||{},idle=state.task_state==='STANDBY';
  let step=1,title='准备本轮建图',next=$('control-mode').value==='automatic'?
    '核对场地后检查并启动；地图和导航就绪后会自动行驶。':'选择新建或续建，检查并启动后再启用遥控。';
  if(state.task_state==='STARTING'||operationSubmitting&&idle){title='正在检查并启动';next='等待传感器与建图服务就绪，请勿重复启动。';}
  else if(state.task_state==='STOPPING'){step=3;title='正在结束本轮任务';next='正在关闭硬件，等待车端确认后再开始新任务。';}
  else if(!idle){
    step=2;
    if(savedAutonomy()){step=3;title='候选地图已保存';next='结束本次任务后审核候选；审核发布和巡检启用分别进行。';}
    else if(a.phase==='FAULT'||state.phase==='FAILED'){title='建图已锁停';next=a.quality_fault||a.error||state.error||'确认实物停稳，查看原因后结束任务；不自动恢复运动。';}
    else if(['REFERENCE','LOADING','TRIAL','CANDIDATE'].includes(state.phase)){title='续建位置核对';next='在地图指定实车位置与车头方向，试匹配后确认墙体对齐；确认前不会自动行驶。';}
    else if(a.phase==='STOPPING'){title='正在取消导航并停稳';next='等待导航终态、零输出与轮式停稳；请同时目视确认车辆。';}
    else if(a.phase==='RETURNING'){title='正在返回本轮起点';next='可以暂停返程；返回并停稳后将自动保存候选地图。';}
    else if(a.phase==='SAVE_PENDING'||state.phase==='SAVING'){step=3;title='正在保存候选地图';next='等待地图和工程文件保存确认，请勿重复保存或重启。';}
    else if(a.phase==='PAUSED'){title='自动探索已暂停';next='确认实物停稳后，可恢复探索、手动接管，或结束探索并返程。';}
    else if(a.phase==='EXPLORING'){title='自动探索中';next=a.goal_selecting?'正在筛选可达观测点；可随时暂停。':'车辆自主选点建图；可随时暂停，完成后选择结束探索并返程。';}
    else if(state.phase==='CURRENT'){title='手动建图已就绪';next=state.autonomy_enabled?'启用遥控后按住摇杆行驶；也可核对场地后开始自动探索。':'启用遥控后按住摇杆行驶，松手停车；完成后保存并结束。';}
    else{title='正在准备地图与导航';next='等待车端数据就绪，车辆保持停车。';}
  }
  $('workflow-title').textContent=title;$('workflow-next').textContent=next;
  ['workflow-prepare','workflow-mapping','workflow-review'].forEach((id,i)=>{
    $(id).classList.toggle('current',i+1===step);$(id).classList.toggle('complete',i+1<step);
  });
  const manual=!idle&&state.task_state!=='STARTING'&&manualControlAllowed()&&!savedAutonomy();
  $('manual-controls').hidden=!manual;$('enable').hidden=!manual;
  $('mapping-workspace').classList.toggle('automatic-workspace',!idle&&!!state.autonomy_enabled&&!manual);
  $('save-actions').hidden=idle||state.task_state==='STARTING'||savedAutonomy()||['EXPLORING','RETURNING','STOPPING','SAVE_PENDING','FAULT'].includes(a.phase);
  $('status-caption').textContent=savedAutonomy()?'本轮已结束 · 候选待审核':!idle&&a.control_mode==='automatic'?'自动任务独立运行 · 可随时暂停':'保持目视 · 松手停车';
  $('safety-note').textContent=!idle&&a.control_mode==='automatic'?
    '自动建图需要现场看护；二维激光不能覆盖所有高度的障碍。紧急时切断轮动力。':
    '遥控无自动避障。速度为命令上限，实物是否停稳须目视确认；紧急时切断轮动力。';
}
function renderPublicationControls(){
  const b=bundleCatalog.find(b=>b.id===$('review-bundle').value),status=b?.publication?.status;
  const available=!!session&&state.task_state==='STANDBY'&&!state.busy&&!taskOperationBusy()&&!publicationPending;
  const reviewable=!!b?.valid&&status!=='diagnostic',hasNote=!!$('review-note').value.trim();
  const reason=!session?'等待控制连接':state.task_state!=='STANDBY'?'先正常结束本次建图任务':publicationPending||state.busy||taskOperationBusy()?'正在处理本次操作':!b?'请选择完整有效工程':!b.valid?'该工程校验未通过':status==='diagnostic'?'故障证据不能审核发布':!hasNote?'请先填写现场核对结论':'';
  $('review-approve').disabled=!available||!reviewable||!hasNote||status==='published';
  $('review-reject').disabled=!available||!reviewable||!hasNote;
  $('map-activate').disabled=!available||!b?.valid||status!=='published';
  $('review-approve').title=$('review-approve').disabled?(status==='published'?'该版本已经发布':reason):'';
  $('review-reject').title=$('review-reject').disabled?reason:'';
  $('map-activate').title=$('map-activate').disabled?(!available?reason:'请先选择已审核发布的地图'):'';
  $('publication-help').textContent=!b?'暂无完整有效工程；先完成并保存一次建图。':!b.valid?'该工程校验未通过，请保留证据并检查保存文件。':status==='diagnostic'?'这是故障证据，仅供诊断，不能作为正式地图发布。':status==='published'?'该版本已发布；启用只加载地图，之后仍需在巡检页按实车位置重新定位。':'先核对墙线、尺寸与现场环境，并填写结论，再审核发布。';
}
function renderSiteConfirmation(){
  const idle=state.task_state==='STANDBY',automatic=$('control-mode').value==='automatic';
  const field=$('site-check'),slot=$(idle?'startup-site-slot':'autonomy-site-slot');
  if(field.parentElement!==slot)slot.append(field);
  field.hidden=!state.autonomy_enabled||savedAutonomy()||(idle&&!automatic)||(!idle&&!['MANUAL','PAUSED'].includes(state.autonomy?.phase));
  $('site-confirmed').disabled=state.busy||taskOperationBusy();
  $('startup-site-slot').hidden=!idle||field.hidden;$('autonomy-site-slot').hidden=idle||field.hidden;
  if(idle&&automatic)$('task-info').textContent=$('site-confirmed').checked?
    '场地条件已确认，可检查并启动硬件；自动就绪后可能开始行驶。':
    '自动启动前，请先核对并勾选本轮场地条件；当前硬件尚未启动。';
}
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
function stop(cause='manual') {if(typeof cause!=='string')cause='manual';if(pressed||cause==='manual')clientLog(cause);pressed = '';lastDriveSentAt=-Infinity; resetStick();
  if(state.autonomy?.control_mode==='automatic'&&cause!=='manual')return;
  send({action:'stop',cause});}
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
    if(!savedAutonomy())$('motion').textContent=value.reason||'未使能';
    $('enable').disabled=!manualControlAllowed()||!!value.armed||!!value.health||state.phase!=='CURRENT'||state.busy||state.save_failed||!session||!controlFresh();
    joystick.classList.toggle('disabled',!manualControlAllowed()||!value.armed||state.busy||!session);
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
    $('connection').textContent = state.autonomy?.control_mode==='automatic'&&['EXPLORING','RETURNING','STOPPING'].includes(state.autonomy?.phase)?
      '本页控制断开 · 自动任务可能仍在运行':owned?'本页控制断开，已撤销授权':'未取得控制连接，请检查网络或其他操作者';
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
  if(operationSubmitting||taskOperationBusy()&&!(path==='/api/end'&&state.task_state==='STARTING'))return;
  operationSubmitting=true;autonomyNotice='';autonomyAction=null;
  for(const id of ['start','save','finish','restart','end','mode','control-mode','bundle','refresh-bundles'])$(id).disabled=true;
  stop('operation');clientLog('operation',{operation:path}); clientError='';
  if(['/api/start','/api/restart','/api/end'].includes(path)){clearScan();lastControlRevision=0;}
  try {
    const response = await fetchTimed(path,{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},body:JSON.stringify({...body,session})},5000);
    const value = await response.json(); if (!response.ok) throw Error(value.error || '操作失败');
    if(value.accepted)pendingOperation=value.operation;
    $('result').textContent = value.accepted ? '请求已接收，等待车端状态确认；请求受理不等于执行完成。' : '操作完成，按界面提示继续。';
  } catch (error) {clientError=error.name==='AbortError'?'网页未收到操作应答：请核对车端任务阶段，不要重复提交；请求可能已到达上位机。':error.message;$('result').textContent=clientError;}
  finally{operationSubmitting=false;}
}
$('save').onclick = () => operation('/api/save');
$('finish').onclick = () => savedAutonomy()?endTask():operation('/api/finish');
$('mode').onchange = () => {$('bundle').hidden = $('mode').value !== 'resume';};
$('control-mode').onchange = renderSiteConfirmation;
$('site-confirmed').onchange = renderSiteConfirmation;
async function refreshBundles() {
  try {
    const r = await fetchTimed('/api/bundles',{},10000), value = await r.json(); if (!r.ok) throw Error(value.error);
    if(!Array.isArray(value.bundles))throw Error('工程列表应答不完整，请重新刷新；未修改地图。');
    const previous=$('bundle').value,review=preferredReviewBundle||$('review-bundle').value||state.last_saved_bundle;
    bundleCatalog=value.bundles;
    $('bundle').replaceChildren();
    $('review-bundle')?.replaceChildren();
    for (const b of value.bundles) {const option = document.createElement('option');option.value=b.id;
      const blocked=state.autonomy_enabled&&b.autonomy_compatible===false;
      option.textContent=b.id+(!b.valid?'（校验失败）':blocked?'（与本轮基线不兼容）':'');option.disabled=!b.valid||blocked;option.title=blocked?b.autonomy_block_reason||'请使用对应历史入口核对':'';$('bundle').append(option);}
    for(const b of value.bundles){const option=document.createElement('option');option.value=b.id;
      const labels={candidate:'待审核',published:'已发布',rejected:'审核未通过',diagnostic:'故障证据',legacy:'历史工程'};
      option.textContent=`${b.id} · ${labels[b.publication?.status]||'待核对'}`;option.disabled=!b.valid;
      option.dataset.published=b.publication?.status==='published'?'true':'false';$('review-bundle')?.append(option);}
    if (!value.bundles.some(b=>b.valid)) {const option=document.createElement('option');option.textContent='暂无完整有效工程';option.value='';$('bundle').append(option);}
    if(value.bundles.some(b=>b.id===previous&&b.valid))$('bundle').value=previous;
    if(value.bundles.some(b=>b.id===review))$('review-bundle').value=review;
    preferredReviewBundle='';renderPublicationControls();
    bundlesLoaded = true;
  } catch (e) {$('result').textContent = e.message;}
}
$('refresh-bundles').onclick = refreshBundles;
$('review-bundle').onchange=renderPublicationControls;
$('review-note').oninput=renderPublicationControls;
$('start').onclick = () => {
  if ($('mode').value === 'resume' && !$('bundle').value) {$('result').textContent='请选择完整有效的旧工程。';return;}
  const automatic=$('control-mode')?.value==='automatic';
  if(automatic&&!$('site-confirmed').checked){
    renderSiteConfirmation();
    const message='网页未提交启动请求：请先勾选本轮场地条件；底盘和雷达尚未启动。';
    $('task-info').textContent=$('result').textContent=message;
    $('site-check').scrollIntoView({block:'nearest'});$('site-confirmed').focus({preventScroll:true});return;
  }
  if (!window.confirm('启动前确认：供电正常、周围人员安全、支架扫掠范围无干涉。底盘启动会发送S1=94，支架可能转动。'+(automatic?'自动模式将在地图就绪（续建需先确认位置）后开始行驶。':'车辆保持停车，之后另行使能。'))) return;
  operation('/api/start',{mode:$('mode').value,bundle:$('mode').value==='resume'?$('bundle').value:'',control_mode:automatic?'automatic':'manual',site_confirmed:automatic&&$('site-confirmed').checked});
};
let autonomyPending=null,publicationPending=false;
const autonomyPhases={MANUAL:'手动控制',EXPLORING:'自动探索中',STOPPING:'取消动作并等待停稳',PAUSED:'已暂停',RETURNING:'返回任务起点',SAVE_PENDING:'已结束，等待停稳保存',REVIEW_REQUIRED:'候选已保存，等待审核',FAULT:'质量／执行锁停，选择可信工程恢复'};
async function autonomyOperation(action){
  if(autonomyPending||taskOperationBusy())return;
  const request_id=crypto.randomUUID?.()||`mapping-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  autonomyPending={request_id,action,at:performance.now()};autonomyAction={request_id,action,task_id:state.autonomy?.task_id};
  autonomyNotice='正在提交操作，等待车端确认；尚未确认动作完成。';clientError='';
  // 不等下一次HTTP轮询才禁用：车端可能已切换阶段，而页面尚未消费本次确认。
  for(const id of ['auto-start','auto-pause','auto-manual','auto-finish']){
    $(id).disabled=true;$(id).title='正在等待本次操作的车端确认';
  }
  let httpRejected=false;
  try{
    const r=await fetchTimed('/api/autonomy',{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},body:JSON.stringify({session,action,request_id,site_confirmed:!!$('site-confirmed').checked})},5000);
    const result=await r.json();if(!r.ok){httpRejected=true;throw Error(result.error||'建图请求未被受理');}
    if(autonomyAction?.request_id===request_id&&!autonomyAction.confirmed){autonomyNotice='请求已提交，等待车端终态及停稳确认。';$('result').textContent=autonomyNotice;}
  }catch(e){
    if(autonomyAction?.request_id!==request_id||autonomyAction.confirmed)return;
    if(httpRejected){autonomyPending=null;autonomyAction=null;clientError=e.message;}
    else autonomyNotice='网页未收到操作应答；请求可能已到达上位机，请核对车端阶段，不自动重发。';
    $('result').textContent=clientError||autonomyNotice;
  }
}
$('auto-start').onclick=()=>autonomyOperation('start');
$('auto-pause').onclick=()=>autonomyOperation('pause');
$('auto-manual').onclick=()=>autonomyOperation('manual');
$('auto-finish').onclick=()=>savedAutonomy()||state.autonomy?.phase==='FAULT'||state.phase==='FAILED'?endTask():autonomyOperation('finish');
async function reviewMap(approve){
  if(publicationPending)return;
  const bundle=$('review-bundle').value,note=$('review-note').value.trim();
  if(!bundle||!note){$('publication-result').textContent='选择工程并填写现场核对结论；没有提交审核。';return;}
  publicationPending=true;
  renderPublicationControls();
  try{const r=await fetchTimed('/api/maps/review',{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},body:JSON.stringify({session,bundle,approve,note,request_id:crypto.randomUUID?.()||`${Date.now()}-${++sequence}`})});
    const result=await r.json();if(!r.ok)throw Error(result.error);$('publication-result').textContent=approve?'已发布该版本，尚未切换定位；可明确启用。':'已保留并标记为审核未通过。';await refreshBundles();
  }catch(e){$('publication-result').textContent=e.message;}finally{publicationPending=false;renderPublicationControls();}
}
$('review-approve').onclick=()=>reviewMap(true);
$('review-reject').onclick=()=>reviewMap(false);
$('map-activate').onclick=async()=>{
  if(publicationPending)return;
  publicationPending=true;let inspectionSocket;
  renderPublicationControls();
  try{const option=$('review-bundle').selectedOptions[0];if(option?.dataset.published!=='true')throw Error('请选择已审核发布的地图；没有切换定位。');
    // 巡检持有独立操作者会话，不能用建图session冒充。此连接只用于停车启用地图。
    const inspectionSession=await new Promise((resolve,reject)=>{
      inspectionSocket=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/inspect/ws/control`);
      const timer=setTimeout(()=>reject(Error('巡检控制连接超时，未提交地图启用；请先启动纯激光定位组合')),3000);
      inspectionSocket.onmessage=e=>{try{const m=JSON.parse(e.data);if(m.session){clearTimeout(timer);resolve(m.session);}}catch{}};
      inspectionSocket.onerror=()=>{clearTimeout(timer);reject(Error('巡检控制连接不可用，没有提交地图启用'));};
    });
    const request_id=crypto.randomUUID?.()||`${Date.now()}-${++sequence}`;
    const r=await fetchTimed('/inspect/api/maps/activate',{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},body:JSON.stringify({map_id:option.value,session:inspectionSession,request_id})},8000);
    const result=await r.json();if(!r.ok)throw Error(result.error);
    $('publication-result').textContent='启用已受理，正在确认停车与地图加载；尚未完成。';
    let completed=false;
    for(let i=0;i<50;i++){
      const response=await fetchTimed('/inspect/api/state'),current=await response.json(),activation=current.map_activation;
      if(activation?.request_id===request_id&&activation.phase==='FAILED')throw Error(current.error||'地图加载未完成，保持停车；检查map_server实际地图');
      if(activation?.request_id===request_id&&activation.phase==='WAITING_LOCALIZATION'){completed=true;break;}
      await delay(200);
    }
    if(!completed)throw Error('未收到本次地图启用终态；保持停车，在巡检页核对实际地图，不自动重发');
    $('publication-result').textContent='地图已加载，车辆保持停车；请前往巡检页按实车位置重新定位，旧点位不迁移。';
  }catch(e){$('publication-result').textContent=e.message;}finally{inspectionSocket?.close();publicationPending=false;renderPublicationControls();}
};
function endTask(){
  if(state.dirty&&!window.confirm('结束会关闭本次建图硬件与服务，并放弃尚未保存的进度；已保存工程保留。确定结束？'))return;
  preferredReviewBundle=state.last_saved_bundle||'';
  operation('/api/end',{discard:!!state.dirty});
}
$('end').onclick = endTask;
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
let lastPaintAt=-Infinity,navImages={};
function layerOn(id){const node=$(id);return !node||node.checked!==false;}
function paintCost(slot,key,toggle){
  const image=navImages[slot],layer=state.navigation?.[key];
  if(!layerOn(toggle)||!layer?.available||!image?.bitmap||image.sequence!==layer.sequence)return;
  const meta=image.meta;if(!meta?.origin)return;
  ctx.save();ctx.translate(meta.origin.x,meta.origin.y);ctx.rotate(meta.origin.yaw||0);
  ctx.translate(0,meta.height*meta.resolution);ctx.scale(meta.resolution,-meta.resolution);
  ctx.imageSmoothingEnabled=false;ctx.drawImage(image.bitmap,0,0);ctx.restore();
}
function paintPath(layer,color,width,toggle){
  if(!layerOn(toggle)||!layer?.available||!layer.points?.length)return;
  ctx.save();ctx.strokeStyle=color;ctx.lineWidth=width/scale;ctx.lineJoin='round';ctx.lineCap='round';ctx.beginPath();
  layer.points.forEach((point,index)=>index?ctx.lineTo(point[0],point[1]):ctx.moveTo(point[0],point[1]));
  ctx.stroke();ctx.restore();
}
function navigationSentence(auto){
  if(!auto)return '';
  if(auto.loop_waiting_map)return 'SLAM正在修正地图，已暂停运动输出；等待闭环结束后的新地图。';
  if(auto.goal_revalidating)return '地图变化后正在重新检查原目标；车辆保持停车，检查通过后继续。';
  if(auto.phase==='EXPLORING'&&auto.goal_selecting)return '正在筛选可达观测点，尚未提交路径预检或下发运动目标；车辆保持停车，可随时暂停。';
  if(auto.phase==='EXPLORING'&&auto.exploration_note)return auto.exploration_note;
  if(auto.goal===null&&auto.navigation_terminal&&auto.navigation_detail)return '导航：'+auto.navigation_detail;
  if(auto.motion_note)return auto.motion_note;
  if(auto.navigation_detail)return '导航：'+auto.navigation_detail;
  const near=auto.near_field;
  if(near?.valid)return `最近激光在${near.side||'车身附近'} ${Number(near.range).toFixed(2)} 米，${near.inside?'已进入车身停车框':'在车身停车框外'}`;
  return '';
}
function layerAge(layer,name){
  if(!layer)return name+'未接入';
  if(!layer.available)return `${name}：${layer.error||'没有数据'}`;
  const points=Array.isArray(layer.points)?` ${layer.points.length}点`:'';
  return `${name} ${Math.max(0,layer.age_seconds||0).toFixed(1)}秒${points}`;
}
function refreshNavigationImages(){
  const nav=state.navigation||{};
  for(const [key,slot,url] of [['global_costmap','global','/api/navigation/global-costmap.png'],['local_costmap','local','/api/navigation/local-costmap.png']]){
    const layer=nav[key];
    if(!layer?.available||!layer.sequence||navImages[slot]?.sequence===layer.sequence||navImages[slot]?.loading)continue;
    const sequence=layer.sequence,generation=taskKey;
    navImages[slot]={...navImages[slot],loading:true,sequence};
    fetchTimed(url,{},4000).then(async response=>{
      if(!response.ok)throw Error('代价图不可用');
      const meta=JSON.parse(response.headers.get('X-Nav-Meta')||'{}');
      const bitmap=await createImageBitmap(await response.blob());
      if(generation!==taskKey){bitmap.close();return;}
      navImages[slot]?.bitmap?.close?.();
      navImages[slot]={sequence,meta,bitmap,loading:false};
    }).catch(()=>{if(navImages[slot]?.sequence===sequence)navImages[slot].loading=false;});
  }
}
function draw(at=performance.now()){
  if(at-lastPaintAt<33){requestAnimationFrame(draw);return;}lastPaintAt=at;
  const ratio=Math.min(window.devicePixelRatio||1,2),w=canvas.clientWidth,h=canvas.clientHeight;
  if(canvas.width!==Math.round(w*ratio)||canvas.height!==Math.round(h*ratio)){canvas.width=Math.round(w*ratio);canvas.height=Math.round(h*ratio);if(fitted&&displayMap)fit();}
  ctx.setTransform(1,0,0,1,0,0);ctx.fillStyle='#a0a0a0';ctx.fillRect(0,0,canvas.width,canvas.height);
  ctx.setTransform(scale*ratio,0,0,-scale*ratio,(w/2-cx*scale)*ratio,(h/2+cy*scale)*ratio);
  if(mapImage&&displayMap){const m=displayMap;ctx.save();ctx.translate(m.origin[0],m.origin[1]);ctx.rotate(m.origin[2]);ctx.translate(0,m.height*m.resolution);ctx.scale(m.resolution,-m.resolution);ctx.imageSmoothingEnabled=false;ctx.drawImage(mapImage,0,0);ctx.restore();}
  paintCost('global','global_costmap','layer-global-cost');paintCost('local','local_costmap','layer-local-cost');
  const auto=state.autonomy||{},nav=state.navigation||{};
  if(auto.anchored&&auto.origin){ctx.save();ctx.strokeStyle='#63846f';ctx.lineWidth=2/scale;ctx.setLineDash([8/scale,6/scale]);ctx.beginPath();ctx.arc(auto.origin[0],auto.origin[1],auto.radius_m||20,0,Math.PI*2);ctx.stroke();ctx.setLineDash([]);ctx.strokeRect(auto.origin[0]-5/scale,auto.origin[1]-5/scale,10/scale,10/scale);ctx.restore();}
  if(auto.goal){ctx.save();ctx.strokeStyle='#7646ae';ctx.lineWidth=2/scale;ctx.beginPath();ctx.arc(auto.goal[0],auto.goal[1],8/scale,0,Math.PI*2);ctx.stroke();ctx.restore();}
  paintPath(nav.global_path,'#00a8bd',3,'layer-global-path');
  paintPath(nav.local_path,'#ffe000',4,'layer-local-path');
  paintPath(nav.prediction,'#ff7a18',2.5,'layer-prediction');
  if(layerOn('layer-body')&&state.pose){const pose=state.pose;ctx.save();ctx.translate(pose[0],pose[1]);ctx.rotate(pose[2]||0);ctx.strokeStyle='#d35400';ctx.lineWidth=1.5/scale;ctx.strokeRect(-0.14,-0.13,0.28,0.26);ctx.restore();}
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
        autonomyPending=null;autonomyNotice='';autonomyAction=null;
        stop('task_changed');clearScan(); mapImage?.close(); mapImage=null;displayMap=null;mapVersion='';fitted=false;hint=null;choosing=false;bundlesLoaded=false;
        for(const slot of Object.values(navImages))slot?.bitmap?.close?.();navImages={};
        if(state.task_state==='STANDBY')$('site-confirmed').checked=false;
      }
      taskKey=newKey;
      if(choosing&&['CURRENT','SAVING','FAILED'].includes(state.phase)){choosing=false;gesture=null;$('hint-info').textContent='';}
      const saved=savedAutonomy(),uiBusy=state.busy||taskOperationBusy();
      $('phase').textContent=saved?'探索已结束 · 候选已保存':phases[state.phase]||'等待建图服务';
      $('map-empty').hidden=!!mapImage;
      const control=controlFresh()?latestControl:state.control;
      const auto=state.task_state==='STANDBY'?{}:state.autonomy||{},autoBusy=['EXPLORING','RETURNING','STOPPING'].includes(auto.phase);
      if($('autonomy-panel')){
        $('autonomy-panel').hidden=!state.autonomy_enabled||state.task_state==='STANDBY';
        $('autonomy-panel').parentElement?.classList.toggle('autonomous-layout',!!state.autonomy_enabled);
        $('autonomy-stage').textContent=state.task_state==='STANDBY'?'网页待命，尚未启动本轮建图':auto.auto_start_pending?'等待地图和导航就绪，自动尚未启动':autonomyPhases[auto.phase]||'等待地图初始化和车端任务';
        const returning=auto.phase==='RETURNING'||auto.return_result==='running',faulted=auto.phase==='FAULT'||state.phase==='FAILED';
        $('autonomy-budget').textContent=`半径 ${auto.radius_m||20}米 · ${returning?'返程':'探索'}剩余 ${Math.ceil((returning?(auto.return_remaining_seconds??600):(auto.remaining_seconds??2400))/60)}分钟 · 上限0.10m/s${auto.goal?` · 目标(${auto.goal.map(v=>v.toFixed(2)).join(', ')})米`:''}`;
        if(saved)$('autonomy-budget').textContent=`本轮探索已结束 · ${{succeeded:'已返回起点',failed:'返程未成功',timeout:'返程超时',not_started:'未执行返程'}[auto.return_result]||'返程结果待核对'} · 候选待审核`;
        const autonomyFresh=state.autonomy_age_ms>=0&&state.autonomy_age_ms<=500;
        const autoStartBlocked=auto.automatic_start_ready===false;
        const autoStartReason=auto.automatic_start_reason||'自动尚未开始：请检查起点周围整车足迹是否位于已知自由区域';
        $('autonomy-quality').textContent=state.task_state==='STANDBY'?'请选择工程类型和控制方式；自动前先核对场地':!autonomyFresh?'车端任务状态过期；操作未下发，请检查任务进程和连接':auto.quality_fault||auto.error||(['MANUAL','PAUSED'].includes(auto.phase)&&autoStartBlocked?autoStartReason:['SAVE_PENDING','REVIEW_REQUIRED'].includes(auto.phase)?auto.reason||auto.end_reason:auto.health_reason||auto.reason)||(auto.healthy?'输入健康，地图与任务状态正在更新。':'等待车端健康检查，请核对传感器与任务状态。');
        if(saved)$('autonomy-quality').textContent=auto.return_result==='succeeded'?'返程与文件保存已完成，地图质量仍需现场审核。':`候选已保存，${{failed:'返程未成功',timeout:'返程超时',not_started:'未执行返程'}[auto.return_result]||'返程结果待核对'}；请审核未覆盖区域并查看本轮原因。`;
        const sentence=navigationSentence(auto);
        $('nav-status').hidden=!sentence;$('nav-status').textContent=sentence;
        if(autonomyPending&&auto.request_id===autonomyPending.request_id){
          autonomyNotice=auto.request_result==='rejected'?(auto.error||'车端拒绝了本次操作；没有因该请求下发新目标。'):'车端已受理，等待执行终态及停稳确认。';
          if(auto.request_result==='rejected')autonomyAction=null;
          else if(autonomyAction)autonomyAction.confirmed=true;
          autonomyPending=null;
        }
        if(autonomyPending&&performance.now()-autonomyPending.at>8000){autonomyNotice='未收到本次操作的车端确认；请核对当前阶段和诊断，不自动重发。';autonomyPending=null;}
        if(autonomyAction&&autonomyAction.task_id===auto.task_id&&auto.request_id===autonomyAction.request_id&&auto.request_result==='accepted'){
          const stopped=auto.still&&auto.navigation_terminal&&auto.output?.every(v=>v===0);
          if(['pause','halt'].includes(autonomyAction.action))autonomyNotice=auto.phase==='PAUSED'&&stopped?'暂停完成：导航已取消、输出为零、轮式反馈停稳；请目视核对。':'暂停已受理，正在取消导航并等待停稳。';
          else if(autonomyAction.action==='manual')autonomyNotice=auto.phase==='MANUAL'&&stopped?'手动接管完成；启用遥控后再操作摇杆。':'接管已受理，等待取消导航和停稳。';
          else if(autonomyAction.action==='finish')autonomyNotice=saved?'返程结果与候选已保存；请结束任务并查看候选。':auto.phase==='RETURNING'?'返程正在执行；返回并停稳后将自动保存。':auto.phase==='SAVE_PENDING'?'探索已结束，正在等待候选保存确认。':'结束探索已受理，等待停稳和返程预检。';
          else autonomyNotice=auto.phase==='EXPLORING'?'自动探索已开始；可以随时暂停或结束探索并返程。':'自动请求已受理，等待地图和导航就绪。';
        }
        if(saved)autonomyNotice='返程结果与候选已保存；请结束任务并查看候选。';
        const unavailable=!session||uiBusy||!auto.task_id||!!autonomyPending||!autonomyFresh;
        $('autonomy-actions').classList.toggle('completed',saved||faulted);
        $('auto-finish').classList.toggle('primary',saved||faulted);
        $('auto-pause').classList.toggle('primary',['EXPLORING','RETURNING'].includes(auto.phase));
        $('autonomy-hint').textContent=saved?'本轮探索已结束。结束任务后可审核候选或新建；已保存工程保留。':'自动任务在网页断连后继续；停止请点停车并核对车端状态。手动断连仍停车。';
        $('auto-start').hidden=saved||faulted||!['MANUAL','PAUSED'].includes(auto.phase);
        $('auto-pause').hidden=saved||faulted||!['EXPLORING','RETURNING','STOPPING'].includes(auto.phase);
        $('auto-manual').hidden=saved||faulted||!['EXPLORING','PAUSED'].includes(auto.phase);
        $('auto-finish').hidden=!saved&&!faulted&&!['EXPLORING','PAUSED'].includes(auto.phase);
        $('auto-start').textContent=auto.phase==='PAUSED'?'恢复自动探索':'开始自动探索';
        $('auto-start').disabled=unavailable||!['MANUAL','PAUSED'].includes(auto.phase)||!$('site-confirmed').checked||!auto.healthy||autoStartBlocked;
        $('auto-pause').disabled=unavailable||!['EXPLORING','RETURNING'].includes(auto.phase);
        $('auto-manual').disabled=unavailable||!['EXPLORING','PAUSED'].includes(auto.phase);
        $('auto-finish').disabled=saved?!session||uiBusy:faulted?!session||uiBusy||!auto.still||!auto.navigation_terminal:unavailable||!['EXPLORING','PAUSED'].includes(auto.phase);
        $('auto-finish').textContent=saved?'结束任务并查看候选':faulted?'结束故障任务':'结束探索并返程';
        for(const id of ['auto-start','auto-pause','auto-manual','auto-finish'])$(id).title=$(id).disabled?(!session?'等待控制连接':saved?'本轮探索已结束，候选已保存；请先结束任务再审核或新建':!autonomyFresh?'车端状态过期':state.busy?'正在保存或处理任务':autonomyPending?'等待同一次操作确认':id==='auto-start'&&!$('site-confirmed').checked?'先核对封闭场地':id==='auto-start'&&autoStartBlocked?autoStartReason:auto.health_reason||'当前阶段不允许该操作'):'';
      }
      $('motion').textContent=saved?'探索已结束；确认实物停稳后结束任务，查看候选或开始新一轮':state.inspection_running?'巡检还在进行，请先在巡检页取消或等这轮结束':state.phase==='STANDBY'?'网页待命，尚未启动硬件':state.task_state==='STARTING'?'正在预检和启动，车辆保持停车':!controlFresh()?'控制状态过期，请检查连接':control?.reason||'等待运动门控';
      $('enable').disabled=!!state.inspection_running||!session||state.busy||state.save_failed||state.phase!=='CURRENT'||!controlFresh()||!!control?.armed||!!control?.health;
      if(!manualControlAllowed())$('enable').disabled=true;
      joystick.classList.toggle('disabled',!manualControlAllowed()||!control?.armed||state.busy||!session);
      $('fast-option').disabled=!control?.supported_gears?.includes('fast');
      $('save').disabled=saved||state.phase!=='CURRENT'||state.busy||!session;
      $('save').hidden=saved;
      if(autoBusy)$('save').disabled=true;
      $('finish').hidden=!state.managed;$('finish').disabled=saved?!session||state.busy:$('save').disabled;
      $('finish-label').textContent=saved?'结束任务并查看候选':'保存并结束';
      const idle=state.task_state==='STANDBY';
      if($('automatic-mode-option'))$('automatic-mode-option').disabled=!state.autonomy_enabled;
      if($('publication-panel'))$('publication-panel').hidden=!idle||!state.autonomy_enabled;
      renderPublicationControls();
      $('task-panel').hidden=!state.managed||!['STANDBY','STARTING'].includes(state.task_state);
      $('start').disabled=!!state.inspection_running||!idle||uiBusy||!session;$('mode').disabled=$('control-mode').disabled=$('bundle').disabled=$('refresh-bundles').disabled=!idle||uiBusy;
      $('start').textContent=state.task_state==='STARTING'||operationSubmitting&&idle?'正在检查并启动…':'检查并启动硬件';
      $('bundle').hidden=$('mode').value!=='resume';
      $('task-info').textContent=idle?'网页待命，硬件未启动':`本次运行 ${Math.floor((state.task_age||0)/60)}分 · ${state.bundle||'新工程'}${state.dirty?' · 有未保存进度':''}`;
      renderSiteConfirmation();
      $('task-summary').textContent=saved?'本轮探索已结束 · 候选待审核':idle?'选择新建或续建后启动':`建图 ${Math.floor((state.task_age||0)/60)} 分${state.dirty?' · 待保存':''}`;
      const recovered=state.phase==='CURRENT'&&controlFresh()&&latestControl.armed&&!latestControl.health;
      $('fault-summary').textContent=lastFault?(recovered?'上次停车记录（当前已就绪）：':'停车记录：')+lastFault.reason:'';
      $('fault-summary').classList.toggle('history',!!recovered);
      $('restart').hidden=$('end').hidden=!state.managed||idle;
      $('restart').disabled=!session||state.busy;$('end').disabled=!session||(state.busy&&state.task_state!=='STARTING');
      $('end').textContent=state.task_state==='STARTING'?'取消启动并结束':state.dirty?'放弃未保存进度并结束':'结束本次任务';
      $('video-toggle').hidden=!state.managed;$('video-toggle').disabled=!session||state.busy||state.task_state!=='RUNNING';
      $('video-toggle').textContent=state.video_requested?'关闭辅助视频':'开启辅助视频';$('video-panel').hidden=state.managed&&!state.video_requested;
      $('save-age').textContent=idle?'':`距上次成功保存（或本次启动） ${Math.floor((state.last_save_age||0)/60)}分${state.save_failed?' · 保存失败，锁停等待重试':''}${state.last_saved_bundle?' · 最近版本 '+state.last_saved_bundle:''}`;
      if(saved)$('save-age').textContent='地图与工程文件已保存完整；结束任务后可审核该候选。';
      if(state.managed&&idle&&!bundlesLoaded) {bundlesLoaded=true;refreshBundles();}
      $('hint').hidden=$('try').hidden=!['REFERENCE','CANDIDATE'].includes(state.phase);$('try').disabled=!hint||state.busy;
      $('confirm').hidden=state.phase!=='CANDIDATE';
      $('health').textContent=`雷达 ${age(state.scan_age)} · 里程计 ${age(state.odom_age)} · 地图 ${age(state.map_age)} · 控制 ${controlFresh()?'正常':'过期'}${roundTripMs===null?'':' · 往返 '+roundTripMs+'ms'}`;
      $('nav-layer-note').textContent=!state.autonomy_enabled||state.task_state==='STANDBY'?'':state.navigation?[
        layerAge(state.navigation.global_costmap,'全局代价'),layerAge(state.navigation.local_costmap,'局部代价'),
        layerAge(state.navigation.global_path,'全局路径'),layerAge(state.navigation.local_path,'局部跟随'),layerAge(state.navigation.prediction,'预测弧')
      ].join(' · '):'导航图层尚未从车端接入';
      refreshNavigationImages();
      $('result').textContent=(clientError|| (taskOperationBusy()?'请求已接收，等待本次操作完成；车辆保持停车。':autonomyNotice||state.error||state.result||''))+(state.save_reminder&&state.phase==='CURRENT'&&!saved?'；已超过5分钟，请选择安全位置停车保存。':'');
      renderWorkflow();
      if(state.map&&state.map_version!==mapVersion&&!mapLoading){
        mapLoading=true;
        const version=state.map_version,key=taskKey;
        (async()=>{try{const r=await fetchTimed('/api/map.png',{},10000);if(!r.ok)throw Error();const meta=JSON.parse(r.headers.get('X-Map-Meta'));const image=await createImageBitmap(await r.blob());if(key!==taskKey){image.close();return;}mapImage?.close();mapImage=image;displayMap=meta;mapVersion=version;if(!fitted)fit();}catch(_){}finally{mapLoading=false;}})();
      }
    }catch(error){
      clientLog(error.name==='AbortError'?'http_poll_timeout':'status_update_failed',{error:error.message});
      if(performance.now()-lastStateAt>1500){
        $('workflow-title').textContent='状态更新中断';
        $('workflow-next').textContent=state.autonomy?.control_mode==='automatic'?'未取得车端最新状态，自动任务可能仍在运行；保留现场看护并检查连接，停车请核对实物。':'未取得车端最新状态；请检查连接，恢复后重新核对阶段，不重放旧操作。';
        for(const id of ['start','enable','save','finish','auto-start','auto-pause','auto-manual','auto-finish'])$(id).disabled=true;
      }
    }
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
