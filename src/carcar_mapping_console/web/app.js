'use strict';
const UI_VERSION='20261101-nav018-shared-operators';
const $ = id => document.getElementById(id);
let ws, session = '', challenge = '', sequence = 0, pressed = '', frame = '', frameAt = -Infinity;
let state = {}, mapImage = null, displayMap = null, mapVersion = '', mapLoading = false, hint = null, choosing = false;
let scale = 30, cx = 0, cy = 0, fitted = false, lastStateAt = -Infinity;
let pendingOperation = 0, clientError = '', reconnectTimer = null, taskKey = '', bundlesLoaded = false;
let operationSubmitting=false,autonomyNotice='',autonomyAction=null,bundleCatalog=[],preferredReviewBundle='';
let speedPending=null,speedNotice='',speedDraft=null,speedTask='';
let operatorClientId='',operatorStatusSession='',operatorStatusPending=false,operatorClaimPending=false;
let operatorClaimAfterManual='',operatorNotice='',manualGeneration=null;
let reclaimOnConnect=false;
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
function manualControlAllowed(){return !state.autonomy_enabled||state.autonomy?.phase==='MANUAL'&&!state.autonomy?.auto_start_pending;}
function sharedOperators(){return state.manual_control?.protocol>=1;}
function isManualOwner(){return !sharedOperators()||!!operatorClientId&&state.manual_control.owner_id===operatorClientId;}
function ownsManualControl(){const m=state.manual_control;return !sharedOperators()||
  isManualOwner()&&m.phase==='READY'&&m.ready===true&&!operatorClaimPending;}
function operatorDeviceLabel(){return typeof navigator!=='undefined'&&
  (navigator.userAgentData?.mobile||/Android|iPhone|Mobile/i.test(navigator.userAgent||''))?'手机':'电脑';}
function renderManualAuthority(){
  const shared=sharedOperators(),m=state.manual_control||{},owner=ownsManualControl();
  $('manual-authority').hidden=!shared;if(!shared)return;
  const waiting=m.phase==='STOPPING',label=m.owner_label||'另一端';
  const handoverFailure=/接管未完成|接管设备已断开/.test(m.reason||'')?m.reason+'；':'';
  $('manual-owner').textContent=operatorNotice||(waiting?(m.reason||'正在停车，等待切换遥控持有者'):
    handoverFailure+(!operatorClientId?'正在核对本页操作会话':!m.owner_id?'遥控尚未接管':isManualOwner()?
      (owner?'本页持有遥控':'本页持有遥控，等待车端就绪'):`另一端（${label}）持有遥控`))+
    ` · ${m.connected_clients||0}端在线；任务操作共享`;
  const claim=$('manual-claim');claim.textContent=operatorClaimPending||waiting?'接管中…':owner?'本页已接管':'接管遥控';
  claim.disabled=!session||!operatorClientId||operatorClaimPending||waiting||isManualOwner()||!manualControlAllowed()||state.busy||!!state.inspection_running;
  claim.title=owner?'本页已持有遥控，启用后再操作摇杆':waiting?'等待取消运动、轮式停稳和新会话确认':'先停车，再把遥控切换到本页';
  $('gear').disabled=!owner||!manualControlAllowed();
  if(!owner){$('enable').disabled=true;joystick.classList.add('disabled');}
  $('enable').title=!owner?'请先在遥控面板接管到本页':'';
}
function applyManualControl(manual){
  if(!manual||manual.protocol<1)return;
  if(state.manual_control?.generation> manual.generation)return;
  state.manual_control=manual;
  if(!ownsManualControl()||manualGeneration!==manual.generation){
    // 所有权变化只清空本页旧手势，不能向新持有者补发停车或旧运动。
    pressed='';lastDriveSentAt=-Infinity;resetStick();
  }
  manualGeneration=manual.generation;renderManualAuthority();
}
async function operatorRequest(action){
  if(!session||!sharedOperators()||action==='status'&&operatorStatusPending)return;
  const requestSession=session,stateAtRequest=lastStateAt;
  if(action==='status')operatorStatusPending=true;else operatorClaimPending=true;
  renderManualAuthority();
  try{
    const response=await fetchTimed('/api/operator',{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},
      body:JSON.stringify({session:requestSession,action,device_label:operatorDeviceLabel()})},5000);
    const value=await response.json();if(requestSession!==session)return;
    if(!response.ok)throw Error(value.error||'上位机未受理遥控会话请求');
    operatorClientId=value.client_id||operatorClientId;operatorStatusSession=requestSession;
    operatorNotice='';if(action==='claim')operatorClaimPending=false;
    // 迟到的会话核对不能覆盖之后HTTP轮询已经取得的新接管阶段。
    if(lastStateAt===stateAtRequest||value.manual_control?.generation>state.manual_control?.generation)applyManualControl(value.manual_control);
  }catch(error){if(requestSession===session)operatorNotice=`${action==='claim'?'遥控接管':'本页会话核对'}未确认：${error.message}；检查上位机连接和停车状态。`;}
  finally{if(requestSession===session){
    if(action==='status')operatorStatusPending=false;else operatorClaimPending=false;
    renderManualAuthority();
    if(action==='status'&&operatorStatusSession===session&&reclaimOnConnect){
      reclaimOnConnect=false;
      // 仅用户明确重连且原来持有时重新申请；后端仍须停车交接，不能继承使能。
      if(manualControlAllowed()&&!isManualOwner())void operatorRequest('claim');
    }
  }}
}
async function claimManualControl(){
  if(operatorClaimPending||operatorClaimAfterManual||!session)return;
  if(!manualControlAllowed()){
    if(autonomyPending||taskOperationBusy())return;
    operatorClaimAfterManual=session;
    await autonomyOperation('manual');return;
  }
  return operatorRequest('claim');
}
function taskOperationBusy(){return operationSubmitting||!!(pendingOperation&&(state.completed_operation||0)<pendingOperation);}
function recoveryActive(auto){return auto.phase==='RECOVERING'||!!auto.recovery_pending||(auto.phase==='STOPPING'&&auto.stop_target==='RECOVER');}
function checkpointActive(auto){return auto.phase==='CHECKPOINTING'||(auto.phase==='STOPPING'&&auto.stop_target==='CHECKPOINT');}
function returnFeedback(auto){
  // 返程结论属于任务，不依赖最后一次请求ID；保存确认会更换请求ID。
  if(checkpointActive(auto)||recoveryActive(auto))return null;
  const destination=auto.return_destination||{},terminal=['SAVE_PENDING','REVIEW_REQUIRED'].includes(auto.phase);
  const waitingToSave=auto.phase==='STOPPING'&&auto.stop_target==='SAVE'&&['failed','timeout'].includes(auto.return_result);
  if(destination.phase==='BLOCKED'&&['PAUSED','STOPPING'].includes(auto.phase))return {
    summary:'返程起步受阻',text:destination.reason||auto.reason||'当前车位检查未通过，车辆保持停车；检查车旁障碍与地图。'};
  if((auto.phase==='RETURNING'||terminal)&&destination.phase==='VERIFYING')return {
    summary:'正在核对返程停车位置',
    text:'正在等待轮式停稳和当前位姿核对；尚未确认返程完成。',verifying:true};
  if(terminal||waitingToSave){
    const distance=destination.distance_to_origin_m;
    // 目标偏移只用于选点，不能代替车端核对后的位置距离。
    const nearbyDistance=Number.isFinite(distance)&&distance>=0?
      `距原起点 ${distance.toFixed(2)} m，地图估计`:'距原起点的距离待核对';
    const summary=({succeeded:'已返回原起点',nearby:`已返回起点附近（${nearbyDistance}）`,
      failed:'返程未成功',timeout:'返程超时',not_started:'未执行返程'})[auto.return_result]||'返程结果待核对';
    const reason=destination.reason||auto.reason||auto.end_reason||'';
    return {summary,text:summary+(reason?'；'+reason:''),terminal,waitingToSave};
  }
  if(auto.phase!=='RETURNING')return null;
  const nearby=destination.kind==='nearby';
  const summary=({SELECTING:'正在筛选起点附近的可达停车位置',PLANNING:'正在预检返程路径',
    NAVIGATING:nearby?'正在返回起点附近':'正在返回原起点'})[destination.phase]||'返程正在执行';
  return {summary,text:summary+'；到达后仍需停稳和位置核对，尚未确认返程完成。'};
}
function returnNotice(auto){
  const feedback=returnFeedback(auto);if(!feedback)return '';
  const stale=state.autonomy_age_ms>=0&&state.autonomy_age_ms<=500?'':'车端状态过期；上次记录：';
  const own=ownTaskResult(auto),rejected=['rejected','cancelled'].includes(own?.request_result);
  const actionLabel={pause:'暂停',halt:'停车',manual:'手动接管',return:'返程',finish:'结束探索并返程',start:'开始自动探索',recheck:'重新检查'};
  const rejection=rejected?`本次${actionLabel[autonomyAction.action]||'操作'}请求${own.request_result==='cancelled'?'已取消':'被车端拒绝'}：${own.reason||auto.error||'请检查车端任务日志'}；`:'';
  return stale+rejection+feedback.text+(savedAutonomy()?'；候选已保存，请结束任务并审核地图。':
    auto.phase==='SAVE_PENDING'?'；正在等待候选保存确认。':
      feedback.waitingToSave?'；等待取消导航并确认轮式停稳后保存，尚未开始保存。':'');
}
function checkpointSentence(auto){
  const revalidation=auto.recovery?.checkpoint_revalidation;
  if(auto.phase==='CHECKPOINTING'&&revalidation?.reason)
    return revalidation.reason+'；校验通过后继续原任务，超时仍保持停车。';
  return auto.phase==='STOPPING'?'正在取消当前目标并等待停稳；随后保存稳定检查点，再继续原任务。':
    '车辆保持停车，正在保存稳定检查点；完成后继续原任务，沿用本轮起点和预算。';
}
function recoverySentence(auto){
  if(auto.phase==='STOPPING'&&recoveryActive(auto))return '检测到受扰，正在取消导航并等待停稳；地图恢复验证尚未开始。';
  if(!recoveryActive(auto))return '';
  const recovery=auto.recovery||{};
  const after=auto.operator_check?.phase==='CHECKING'||auto.automatic_recovery?.operator_hold?
    '通过后保持暂停，由操作者选择下一步。':'通过后刷新导航并继续原任务。';
  return ({DISTURBED:'检测到受扰，正在暂停行驶和地图更新。',WAITING_STABLE:'等待车身平稳，再对照稳定地图重新定位。',
    WAITING_ROLLBACK_STABLE:'错误回环已撤销，地图保持冻结；等待车辆连续停稳。',
    VERIFYING_ROLLBACK:'正在核对撤销后的地图与六帧新激光；'+after,
    VALIDATING_CHECKPOINT:'正在核验稳定检查点的完整性；通过后重新匹配当前位置。',
    INITIALIZING:'正在当前位置附近搜索激光与稳定地图的匹配；车辆保持停车。',
    LOCALIZING:'正在对照稳定地图重新定位；候选位置仍需多帧核对。',RESTORING:'位置候选已通过核对，正在恢复稳定工程。',
    PRESERVING_SUSPECT:'正在归档受扰后的可疑地图增量，随后恢复最近可信工程。',WAIT_RESTORE:'正在恢复稳定工程，等待SLAM确认后核对新扫描。',
    VERIFYING:'正在核对恢复后的新激光与地图；'+after,VERIFY_READY:'位置核对已通过，正在刷新导航；'+after,
    SUCCEEDED:'地图恢复已验证，等待任务继续。',FAILED:'地图恢复未通过，保持停车并保留工程与故障证据。'})[recovery.phase]||
    '正在进行受扰恢复，等待车端完成停稳、定位与地图核对。';
}
function verifiedRecoveryPosition(auto){
  if(!(state.autonomy_age_ms>=0&&state.autonomy_age_ms<=500))return '';
  const recovery=auto.recovery||{},p=recovery.verified_pose;
  return ['VERIFY_READY','SUCCEEDED'].includes(recovery.phase)&&Array.isArray(p)&&p.length===3&&p.every(Number.isFinite)?
    `地图已复验 · 恢复位置（地图坐标）(${p[0].toFixed(2)}, ${p[1].toFixed(2)}) 米，朝向 ${(p[2]*180/Math.PI).toFixed(0)}°`:'';
}
function renderRecovery(auto,fresh){
  renderSnapshot(auto,fresh);
  const recovery=auto.recovery,element=$('recovery-status');element.hidden=!recovery?.phase;
  if(!recovery?.phase)return;
  const checkpoint=recovery.checkpoint_id?`恢复检查点 ${recovery.checkpoint_id}${Number.isFinite(recovery.checkpoint_age_seconds)?` · ${Math.max(0,recovery.checkpoint_age_seconds).toFixed(0)}秒前`:''}`:'尚无可信恢复检查点';
  const position=verifiedRecoveryPosition(auto);
  const imu=Number.isFinite(recovery.imu_age_seconds)&&recovery.imu_age_seconds>.5?'姿态观测过期':
    recovery.imu_valid===false?'姿态观测无效':!recovery.imu_ready?'等待姿态参考':recovery.imu_stable?'姿态观测平稳':
      ['EXPLORING','RETURNING'].includes(auto.phase)&&!recovery.requires_stop?'行驶中，姿态观测更新':'等待连续停稳观测';
  element.textContent=!fresh?'恢复状态过期；请检查车端连接，不能据此判断定位已恢复。':
    [recovery.phase==='CHECKPOINTING'&&recovery.checkpoint_revalidation?.waiting?recovery.checkpoint_revalidation.reason:
      checkpointActive(auto)||recoveryActive(auto)?'':recovery.phase==='CHECKPOINTING'?
      (auto.phase==='MANUAL'?'正在生成稳定恢复检查点；自动探索等待完成。':auto.phase==='PAUSED'?'保持暂停，正在更新稳定恢复检查点。':'正在更新稳定恢复检查点，任务继续运行。'):
      recoverySentence(auto)||recovery.reason,position,checkpoint,imu].filter(Boolean).join(' · ');
}
function renderSnapshot(auto,fresh){
  const element=$('snapshot-status'),snapshot=auto.snapshot_maintenance||auto.recovery?.snapshot||{};
  element.hidden=!auto.online_snapshots;if(element.hidden)return;
  if(!fresh){element.textContent='快照维护状态已过期；检查车端连接，不能据此判断恢复点有效。';return;}
  const running=['EXPLORING','RETURNING'].includes(auto.phase)&&auto.healthy===true,available=auto.checkpoint_usable===true;
  if(snapshot.phase==='FAILED'){
    element.textContent=`快照维护：本次更新未成功，未替换旧恢复点。${snapshot.reason||'车端保存或校验未通过'}。`+
      (running&&available?'导航继续运行，10秒后重试；检查磁盘空间与车端快照日志。':available?'导航保持当前状态，旧恢复点仍有效；检查车端快照日志。':'尚无有效恢复点，保持停车；检查存储和车端快照日志，更新成功后再恢复。');
    return;
  }
  const stages={REQUESTED:'请求已提交',PREPARING:'准备内存副本',CAPTURING:'捕获一致建图状态',COPYING:'移交后台工作进程',WRITING:'后台写入档案',RASTERIZING:'生成对应栅格',VALIDATING:'校验候选恢复点',READY:'可信恢复点已更新',IDLE:'等待维护时机'};
  element.textContent=`快照维护：${stages[snapshot.phase]||'等待车端状态'}。`+(running?'导航继续运行。':'导航保持当前状态。');
}
function renderWorkflow(){
  const a=state.autonomy||{},idle=!!state.managed&&state.task_state==='STANDBY';
  let step=1,title='准备本轮建图',next=$('control-mode').value==='automatic'?
    '核对场地后检查并启动；地图和导航就绪后会自动行驶。':'选择新建或续建，检查并启动后再启用遥控。';
  if(state.task_state==='STARTING'||operationSubmitting&&idle){title='正在检查并启动';next='等待传感器与建图服务就绪，请勿重复启动。';}
  else if(state.task_state==='STOPPING'){step=3;title='正在结束本轮任务';next='正在关闭硬件，等待车端确认后再开始新任务。';}
  else if(!idle){
    step=2;
    if(savedAutonomy()){step=3;title='候选地图已保存';next=returnNotice(a)+'结束本次任务后审核候选；审核发布和巡检启用分别进行。';}
    else if(a.operator_check?.phase==='CHECKING'){title=recoveryActive(a)?'正在重新定位':'正在重新检查';next=recoveryActive(a)?recoverySentence(a):a.operator_check.reason||'车辆保持停车，等待连续健康检查。';}
    else if(a.phase==='FAULT'||state.phase==='FAILED'){title='建图已锁停';next=a.actions?'查看任务控制中的原因；条件恢复后可重新检查。':a.fault?.reason||a.quality_fault||a.error||state.error||'确认实物停稳，查看原因后结束任务；不自动恢复运动。';}
    else if(['REFERENCE','LOADING','TRIAL','CANDIDATE'].includes(state.phase)){title='续建位置核对';next='在地图指定实车位置与车头方向，试匹配后确认墙体对齐；确认前不会自动行驶。';}
    else if(a.auto_start_pending){
      const failed=a.recovery?.phase==='FAILED';title=failed?'恢复检查点未通过':a.recovery?.phase==='CHECKPOINTING'?'正在建立恢复检查点':'正在准备自动探索';
      next=failed?`${a.recovery.reason||'恢复检查未通过'}；自动探索尚未启动，请查看原因并结束本轮处理。`:'等待姿态参考、稳定检查点和导航就绪；车端检查通过后开始自动行驶。';
    }
    else if(checkpointActive(a)){title='停稳保存检查点';next=checkpointSentence(a);}
    else if(recoveryActive(a)){title=a.phase==='STOPPING'?'受扰后正在停稳':'正在恢复地图与定位';next=[recoverySentence(a),verifiedRecoveryPosition(a)].filter(Boolean).join(' ');}
    else if(a.phase==='STOPPING'){title='正在取消导航并停稳';next=returnNotice(a)||'等待导航终态、零输出与轮式停稳；请同时目视确认车辆。';}
    else if(a.phase==='RETURNING'){title=returnFeedback(a)?.summary||'正在返回本轮起点';next=returnNotice(a)+'可以暂停返程；停稳和位置核对通过后才确认到达。';}
    else if(a.phase==='SAVE_PENDING'||state.phase==='SAVING'){step=3;title='正在保存候选地图';next=returnNotice(a)+'等待地图和工程文件保存确认，请勿重复保存或重启。';}
    else if(a.phase==='PAUSED'){
      const returning=a.return_result==='running';title=returning?'返程已暂停':'自动探索已暂停';
      next=(a.reason?a.reason+'；':'')+(returning?'确认实物停稳后，可恢复返程或手动接管；继续沿用原返程预算。':'确认实物停稳后，可恢复探索、手动接管，或结束探索并返程。');
    }
    else if(a.phase==='EXPLORING'&&a.exploration_wait?.phase==='WAITING'){
      title='等待可执行目标';next=`车端正在重新筛选，尚未下发新的行驶目标。${a.navigation_detail||a.exploration_note||''}；持续无目标将自动暂停，请检查地图未知区和通道。`;
    }
    else if(a.phase==='EXPLORING'){title='自动探索中';next=a.goal_selecting?'正在筛选可达观测点；可随时暂停。':'车辆自主选点建图；可随时暂停，完成后选择结束探索并返程。';
      if(a.healthy&&a.recovery?.phase==='SUCCEEDED'&&verifiedRecoveryPosition(a))next=verifiedRecoveryPosition(a)+'。正在继续建图，可随时暂停。';}
    else if(state.phase==='CURRENT'){title='手动建图已就绪';next=state.autonomy_enabled?'启用遥控后按住摇杆行驶；也可核对场地后开始自动探索。':'启用遥控后按住摇杆行驶，松手停车；完成后保存并结束。';}
    else{title='正在准备地图与导航';next='等待车端数据就绪，车辆保持停车。';}
  }
  $('workflow-title').textContent=title;$('workflow-next').textContent=next;
  ['workflow-prepare','workflow-mapping','workflow-review'].forEach((id,i)=>{
    $(id).classList.toggle('current',i+1===step);$(id).classList.toggle('complete',i+1<step);
  });
  const manual=!idle&&state.task_state!=='STARTING'&&manualControlAllowed()&&!savedAutonomy();
  $('manual-controls').hidden=!manual;$('enable').hidden=!manual;
  renderManualAuthority();
  $('mapping-workspace').classList.toggle('automatic-workspace',!idle&&!!state.autonomy_enabled&&!manual);
  $('save-actions').hidden=!!a.actions||idle||state.task_state==='STARTING'||!!a.auto_start_pending||savedAutonomy()||['EXPLORING','RETURNING','STOPPING','RECOVERING','CHECKPOINTING','CHECKING','SAVE_PENDING','FAULT'].includes(a.phase);
  $('status-caption').textContent=savedAutonomy()?'本轮已结束 · 候选待审核':checkpointActive(a)||recoveryActive(a)?'保存／恢复中 · 顶部停车可中止':!idle&&a.control_mode==='automatic'?'自动任务独立运行 · 可随时暂停':'保持目视 · 松手停车';
  if(a.phase==='FAULT'||state.phase==='FAILED')$('status-caption').textContent='故障保持停车 · 先检查再继续';
  if(a.phase==='CHECKING')$('status-caption').textContent='检查中 · 不会自动起步';
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
function renderStartInformation(){
  if(state.task_state==='STANDBY'&&$('control-mode').value==='automatic')
    $('task-info').textContent='检查并启动后，地图和导航就绪即开始自动探索；可在运行中调整速度上限。';
}
function renderSpeed(auto,fresh,uiBusy,saved){
  const speed=auto.speed||{},cap=Number.isFinite(speed.cap_mps)?speed.cap_mps:null;
  if(speedTask!==(auto.task_id||'')){speedTask=auto.task_id||'';speedPending=null;speedDraft=null;speedNotice='';}
  if(speedPending&&speed.request_id===speedPending.request_id&&!speed.pending&&['READY','FAILED'].includes(speed.phase)){
    speedNotice=speed.phase==='READY'?'本次调速已确认。':speed.error||'车端调速未确认，请检查导航参数服务。';
    speedPending=null;speedDraft=null;
  }
  if(speedPending&&auto.request_id===speedPending.request_id&&auto.request_result==='rejected'){
    speedNotice=auto.error||'车端拒绝调速，本次速度参数未修改。';speedPending=null;
  }
  if(speedPending&&performance.now()-speedPending.at>8000){
    speedNotice='未收到本次调速的车端确认；页面未自动重发。请检查连接和执行上限后重试。';speedPending=null;
  }
  const allowed=['MANUAL','PAUSED','EXPLORING','RETURNING'].includes(auto.phase)&&!saved;
  const reason=!session?'等待控制连接':!fresh?'车端任务状态过期，调速未提交':uiBusy?'正在处理任务':autonomyPending?'正在确认任务操作，请稍后调速':
    !auto.speed?'车端尚未提供调速状态':!allowed?'当前阶段不允许调速':speedPending||speed.pending?'等待上一次调速的控制器读回确认':'';
  $('auto-speed-controls').hidden=saved;
  $('auto-speed').disabled=$('auto-speed-apply').disabled=!!reason;
  $('auto-speed-apply').title=reason;
  if(speedDraft===null&&!speedPending)$('auto-speed').value=(cap??.10).toFixed(2);
  $('auto-speed-value').textContent=Number($('auto-speed').value).toFixed(2)+' m/s';
  $('auto-speed-status').textContent=[cap===null?'等待车端速度上限':`执行上限 ${cap.toFixed(2)} m/s`,
    Number.isFinite(speed.confirmed_mps)?`${speed.phase==='READY'?'控制器已确认':'控制器上次确认'} ${speed.confirmed_mps.toFixed(2)} m/s`:'控制器尚未确认',
    speedPending||speed.pending?'正在写入并读回参数':speed.error||speedNotice].filter(Boolean).join(' · ');
}
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));
async function fetchTimed(url, options = {}, timeout = 1500) {
  const abort = new AbortController(), timer = setTimeout(() => abort.abort(), timeout);
  try {return await fetch(url, {...options, signal:abort.signal, cache:'no-store'});} finally {clearTimeout(timer);}
}
function send(value) {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  if(sharedOperators()&&!ownsManualControl()&&['enable','drive','intent','heartbeat','release','gear'].includes(value.action))return;
  const realtime=['drive','intent','heartbeat'].includes(value.action);
  if ((!rtcActive||!realtime)&&ws.bufferedAmount > 4096) {clientLog('backpressure');pressed = '';resetStick(); ws.close(); return;}
  if(value.sequence){sentAt.set(value.sequence,performance.now());if(sentAt.size>128)sentAt.delete(sentAt.keys().next().value);}
  if(value.challenge&&value.challenge===challenge)value.response_delay_ms=Math.max(0,performance.now()-lastControlAt);
  const text=JSON.stringify(value);
  if(rtcActive&&(realtime||value.action==='release'||value.action==='stop'))rtcTransport?.send(text);
  // 同一条运动同时走可靠通道。UDP丢单包时，车端仍按原签发时刻计算300ms，迟到副本不能续租。
  if(rtcActive&&realtime&&ws.bufferedAmount<=4096)ws.send(JSON.stringify({...value,backup:true}));
  if(!rtcActive||!realtime)ws.send(text);
  return true;
}
function resetStick() {activePointer = null;pressChallenge=''; $('stick').style.transform = 'translate(0,0)';}
function clearControlConnection() {
  rtcTransport?.close();rtcTransport=null;rtcActive=false;transportAwaitingStop=false;lastControlRevision=0;
  lastControlEpoch='';retiredControlEpochs.length=0;
  session='';challenge='';pressed='';latestControl={};lastControlAt=-Infinity;
  operatorClientId='';operatorStatusSession='';operatorStatusPending=false;operatorClaimPending=false;
  operatorClaimAfterManual='';operatorNotice='';manualGeneration=null;
  lastDriveSentAt=lastHeartbeatSentAt=-Infinity;roundTripMs=null;sentAt.clear();resetStick();
  lastHeartbeatChallenge=lastDriveChallenge='';
  controlGapReported=false;
  pendingClientEvents.length=0;
  $('enable').disabled=true;joystick.classList.add('disabled');
}
function stop(cause='manual') {if(typeof cause!=='string')cause='manual';if(pressed||cause==='manual')clientLog(cause);pressed = '';lastDriveSentAt=-Infinity; resetStick();
  if(cause==='manual'){
    autonomyPending=null;autonomyAction=null;operatorClaimAfterManual='';
  }else if(sharedOperators()&&!ownsManualControl())return;
  if(state.autonomy?.control_mode==='automatic'&&cause!=='manual')return;
  const submitted=send({action:'stop',cause});
  if(cause==='manual'){
    autonomyNotice=submitted?'停车请求已提交，等待车端取消动作及停稳确认。':'本页控制连接不可用，停车请求未提交；检查连接并核对实物停车。';
    $('result').textContent=autonomyNotice;
  }
}
function release() {clientLog('pointer_release');pressed = '';lastDriveSentAt=-Infinity; resetStick(); send({action:'release',sequence:++sequence});}
function drive() {
  if (!pressed||!ownsManualControl()) return;
  // 按住就按固定频率重复当前方向。不靠新挑战续租，状态晚到也不在浏览器里撤使能。
  const nowMs=performance.now();
  if(nowMs-lastDriveSentAt<50)return;
  lastDriveSentAt=nowMs;lastDriveChallenge=challenge;
  const value = typeof pressed === 'string' ? {action:'intent',direction:pressed} : {action:'drive',...pressed};
  send({...value,sequence:++sequence,challenge,frame});
}
function heartbeat() {
  if(!session||!challenge||document.hidden||!controlFresh()||!ownsManualControl())return;
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
  reclaimOnConnect=false;
  stop('disconnect'); cancelReconnect();
  const socket = ws; ws = null;clearControlConnection();
  socket?.close();
  $('connection').textContent = '控制已断开，已撤销授权';
}
function reconnectControl(){
  const reclaim=sharedOperators()&&isManualOwner()&&manualControlAllowed();
  disconnect();reclaimOnConnect=reclaim;connect();
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
    $('enable').disabled=!manualControlAllowed()||!ownsManualControl()||!!value.armed||!!value.health||state.phase!=='CURRENT'||state.busy||state.save_failed||!session||!controlFresh();
    joystick.classList.toggle('disabled',!manualControlAllowed()||!ownsManualControl()||!value.armed||state.busy||!session);
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
$('enable').onclick = () => {if(state.inspection_running||!ownsManualControl())return;clientLog('enable');send({action:'enable',frame});};
$('manual-claim').onclick=()=>claimManualControl();
$('stop').onclick = () => stop('manual');
$('gear').onchange = () => {if(!ownsManualControl())return;stop('gear');clientLog('gear',{gear:$('gear').value});send({action:'gear',gear:$('gear').value});};
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
  if (!latestControl.armed || !ownsManualControl() || !manualControlAllowed() || state.busy || !session || pressed || activePointer !== null) return;
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
$('control-mode').onchange = renderStartInformation;
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
  operation('/api/start',{mode:$('mode').value,bundle:$('mode').value==='resume'?$('bundle').value:'',control_mode:automatic?'automatic':'manual'});
};
let autonomyPending=null,publicationPending=false;
const autonomyPhases={MANUAL:'手动控制',EXPLORING:'自动探索中',STOPPING:'取消动作并等待停稳',RECOVERING:'受扰后恢复地图与定位',CHECKPOINTING:'停稳保存检查点',CHECKING:'正在重新检查，保持停车',PAUSED:'已暂停',RETURNING:'返回任务起点',SAVE_PENDING:'已结束，等待停稳保存',REVIEW_REQUIRED:'候选已保存，等待审核',FAULT:'故障锁停'};
function taskRequestBody(action,request_id,extra={}){return {session,action,request_id,task_id:state.autonomy?.task_id,
  operation_epoch:state.task_operations?.epoch,...extra};}
function pendingMotion(){return !!autonomyPending&&['start','resume','return','finish'].includes(autonomyPending.action);}
function ownTaskResult(auto){
  if(!autonomyAction||autonomyAction.task_id!==auto.task_id)return null;
  if(state.task_operations?.protocol>=1){
    if(state.task_operations.task_id!==auto.task_id)return null;
    return state.task_operations.requests?.find(r=>r.request_id===autonomyAction.request_id)||null;
  }
  return auto.request_id===autonomyAction.request_id?{request_result:auto.request_result,reason:auto.error}:null;
}
async function autonomyOperation(action){
  const priority=action==='pause'||action==='halt';
  if(!priority&&(autonomyPending||taskOperationBusy()))return;
  if(priority)operatorClaimAfterManual='';
  const request_id=crypto.randomUUID?.()||`mapping-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  autonomyPending={request_id,action,at:performance.now()};autonomyAction={request_id,action,task_id:state.autonomy?.task_id};
  autonomyNotice='正在提交操作，等待车端确认；尚未确认动作完成。';clientError='';
  // 不等下一次HTTP轮询才禁用：车端可能已切换阶段，而页面尚未消费本次确认。
  for(const id of ['auto-start','auto-pause','auto-manual','auto-finish','auto-recheck','auto-return']){
    $(id).disabled=true;$(id).title='正在等待本次操作的车端确认';
  }
  if(pendingMotion()){$('auto-pause').hidden=false;$('auto-pause').disabled=!session;$('auto-pause').title='可中止正在提交的继续或返程';}
  $('auto-speed').disabled=$('auto-speed-apply').disabled=true;
  $('auto-speed-apply').title='正在确认任务操作，请稍后调速';
  let httpRejected=false;
  try{
    const r=await fetchTimed('/api/autonomy',{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},body:JSON.stringify(taskRequestBody(action,request_id))},5000);
    const result=await r.json();if(!r.ok){httpRejected=true;throw Error(result.error||'建图请求未被受理');}
    if(autonomyAction?.request_id===request_id&&!autonomyAction.confirmed){
      autonomyAction.canonical_request_id=result.canonical_request_id||request_id;
      autonomyNotice=result.coalesced?'已合并到同一次任务操作，等待车端确认。':result.queued?'操作已排队，等待车端确认；尚未下发新的运动目标。':'请求已提交，等待车端终态及停稳确认。';
      $('result').textContent=autonomyNotice;
    }
  }catch(e){
    if(autonomyAction?.request_id!==request_id||autonomyAction.confirmed)return;
    if(httpRejected){autonomyPending=null;autonomyAction=null;clientError=e.message;if(action==='manual')operatorClaimAfterManual='';}
    else autonomyNotice='网页未收到操作应答；请求可能已到达上位机，请核对车端阶段，不自动重发。';
    $('result').textContent=clientError||autonomyNotice;
  }
}
$('auto-start').onclick=()=>autonomyOperation('start');
$('auto-pause').onclick=()=>autonomyOperation('pause');
$('auto-manual').onclick=()=>sharedOperators()?claimManualControl():autonomyOperation('manual');
$('auto-finish').onclick=()=>savedAutonomy()||state.autonomy?.phase==='FAULT'||state.phase==='FAILED'?endTask():autonomyOperation('finish');
$('auto-recheck').onclick=()=>autonomyOperation('recheck');
$('auto-return').onclick=()=>autonomyOperation('return');
// 新车端逐项报告操作资格；旧车端保留原流程，不伪造恢复能力。
function renderOperatorActions(auto,fresh,uiBusy){
  const handover=sharedOperators()&&state.manual_control.phase==='STOPPING';
  const current=!!auto.actions,blocked=!session||uiBusy||!!autonomyPending||!fresh||handover;
  $('auto-recheck').hidden=$('auto-return').hidden=!current;
  if(!current){$('auto-finish').onclick=()=>savedAutonomy()||state.autonomy?.phase==='FAULT'||state.phase==='FAILED'?endTask():autonomyOperation('finish');return;}
  if(auto.phase==='FAULT')$('phase').textContent='故障锁停 · 等待重新检查';
  const terminal=['SAVE_PENDING','REVIEW_REQUIRED'].includes(auto.phase);
  const checking=auto.phase==='CHECKING'||auto.operator_check?.phase==='CHECKING';
  const labels={'auto-start':['resume',auto.return_result==='running'?'继续返程':'继续建图'],
    'auto-pause':['pause','暂停'],'auto-manual':['manual','手动接管'],
    'auto-return':['return','返回起点'],'auto-recheck':['recheck',checking?'检查中…':'重新检查']};
  for(const [id,[action,label]] of Object.entries(labels)){
    const capability=auto.actions[action]||{enabled:false,reason:'车端暂不支持该操作'},button=$(id);
    button.onclick=()=>action==='manual'&&sharedOperators()?claimManualControl():autonomyOperation(action);
    button.textContent=label;button.hidden=terminal||(action==='pause'&&!pendingMotion()&&!['EXPLORING','RETURNING','STOPPING'].includes(auto.phase))||
      (action==='recheck'&&!['FAULT','PAUSED','CHECKING','RECOVERING'].includes(auto.phase)&&state.phase!=='FAILED'&&!checking);
    const pausePending=action==='pause'&&pendingMotion();
    button.disabled=action==='pause'?(!session||uiBusy||!fresh||!capability.enabled&&!pausePending):blocked||!capability.enabled;
    button.title=button.disabled?(!session?'等待控制连接':!fresh?'车端状态过期，请检查连接':handover&&action!=='pause'?'正在停车交接遥控，等待本次接管完成':uiBusy||autonomyPending?'等待本次操作确认':capability.reason):'';
    button.setAttribute?.('aria-label',label+(button.title?'：'+button.title:''));
  }
  const faulted=auto.phase==='FAULT';
  $('auto-finish').hidden=false;$('auto-finish').textContent=terminal?'结束任务并查看候选':faulted?'结束任务':'保存并结束';
  $('auto-finish').disabled=blocked||!auto.still||!auto.navigation_terminal||checking||auto.phase==='STOPPING';
  $('auto-finish').title=$('auto-finish').disabled?'请先暂停并等待停稳，再保存结束':'';
  $('auto-finish').onclick=()=>terminal||faulted||!state.dirty?endTask():operation('/api/finish');
  $('auto-finish').classList.toggle('primary',terminal);
  $('auto-start').classList.toggle('primary',['MANUAL','PAUSED'].includes(auto.phase));
  $('autonomy-actions').classList.toggle('completed',false);
  $('auto-recheck').classList.toggle('primary',auto.phase==='FAULT');
  const explanation=!fresh?'车端状态过期，所有运动操作保持禁用。':checking?
    auto.operator_check?.reason||'检查中，车辆保持停车。':auto.phase==='FAULT'?
    (auto.actions.recheck?.enabled?'数据已恢复，可重新检查；通过后仍保持暂停。':auto.actions.recheck?.reason||'等待车端恢复条件。'):
    auto.phase==='PAUSED'?(!auto.healthy&&auto.actions.recheck?.enabled?'车辆保持停车，可重新检查并校正位置；验证通过后选择继续建图、返程或接管。':auto.reason||'车辆已暂停，选择下一步操作。'):auto.phase==='RETURNING'?returnNotice(auto):
    auto.phase==='EXPLORING'?(auto.exploration_wait?.phase==='WAITING'?`等待可执行目标；${auto.navigation_detail||auto.exploration_note||'车端正在重新筛选。'}`:auto.near_field?.inside?'近场障碍阻挡，车辆等待通道恢复。':'正在自动建图。'):auto.reason||'';
  $('autonomy-hint').textContent=terminal?'本轮探索已结束。'+returnNotice(auto):explanation;
  $('autonomy-quality').hidden=false;
  $('autonomy-quality').textContent=auto.fault?.active?auto.fault.reason:returnNotice(auto)||(
    auto.phase==='FAULT'?auto.quality_fault||auto.error||state.error:!auto.healthy&&!checking?auto.health_reason||'': '');
  $('action-reasons').textContent=Object.entries(labels).filter(([,v])=>!auto.actions[v[0]]?.enabled)
    .map(([,v])=>`${v[1]}：${auto.actions[v[0]]?.reason||'车端不支持'}`).join('\n');
}
$('auto-speed').oninput=()=>{speedDraft=Number($('auto-speed').value);$('auto-speed-value').textContent=speedDraft.toFixed(2)+' m/s';};
$('auto-speed-apply').onclick=async()=>{
  if($('auto-speed-apply').disabled||speedPending||state.autonomy?.speed?.pending)return;
  const speed_mps=Number($('auto-speed').value);
  if(!Number.isFinite(speed_mps)||speed_mps<.05||speed_mps>.18){speedNotice='网页未提交调速：请选择0.05～0.18 m/s。';return;}
  const request_id=crypto.randomUUID?.()||`speed-${Date.now()}-${Math.random().toString(16).slice(2)}`;
  speedPending={request_id,at:performance.now(),task_id:state.autonomy?.task_id};speedNotice='调速请求正在提交，等待控制器读回确认。';
  $('auto-speed').disabled=$('auto-speed-apply').disabled=true;$('auto-speed-status').textContent=speedNotice;
  let rejected=false;
  try{
    const response=await fetchTimed('/api/autonomy',{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},
      body:JSON.stringify(taskRequestBody('set_speed',request_id,{speed_mps}))},5000);
    const result=await response.json();if(!response.ok){rejected=true;throw Error(result.error||'上位机未受理调速请求');}
    if(speedPending?.request_id===request_id){speedNotice='调速已提交，等待车端写入与读回；执行上限以车端状态为准。';$('auto-speed-status').textContent=speedNotice;}
  }catch(error){
    if(speedPending?.request_id!==request_id)return;
    speedNotice=rejected?error.message:'网页未收到调速应答；请求可能已经到达车端，请核对执行上限，不自动重发。';
    if(rejected)speedPending=null;
    $('auto-speed-status').textContent=speedNotice;
  }
};
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
$('reconnect').onclick = reconnectControl;
$('transport').onchange = reconnectControl;
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
  if(checkpointActive(auto)||recoveryActive(auto))return '';
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
      if(sharedOperators()){
        applyManualControl(state.manual_control);
        if(session&&operatorStatusSession!==session&&!operatorStatusPending)void operatorRequest('status');
      }
      if(state.dataflow_protocol>=1)statusPath='/api/status';
      else{scanPoints=state.scan||[];lastScanAt=performance.now();scanSourceAgeMs=Math.max(0,(state.scan_age||0)*1000);}
      const newKey = `${state.task_state}:${state.bundle||''}`;
      if (taskKey !== newKey && ['STANDBY','STARTING'].includes(state.task_state)) {
        autonomyPending=null;autonomyNotice='';autonomyAction=null;
        stop('task_changed');clearScan(); mapImage?.close(); mapImage=null;displayMap=null;mapVersion='';fitted=false;hint=null;choosing=false;bundlesLoaded=false;
        for(const slot of Object.values(navImages))slot?.bitmap?.close?.();navImages={};
      }
      taskKey=newKey;
      if(choosing&&['CURRENT','SAVING','FAILED'].includes(state.phase)){choosing=false;gesture=null;$('hint-info').textContent='';}
      const saved=savedAutonomy(),uiBusy=state.busy||taskOperationBusy();
      $('phase').textContent=saved?'探索已结束 · 候选已保存':phases[state.phase]||'等待建图服务';
      $('map-empty').hidden=!!mapImage;
      const storage=state.storage||{};
      $('storage-warning').hidden=!storage.warning&&storage.valid!==false;
      $('storage-warning').textContent=storage.reason||'';
      const control=controlFresh()?latestControl:state.control;
      const auto=state.task_state==='STANDBY'?{}:state.autonomy||{},autoBusy=['EXPLORING','RETURNING','STOPPING','RECOVERING','CHECKPOINTING'].includes(auto.phase);
      if(checkpointActive(auto))$('phase').textContent='停稳保存检查点';
      else if(recoveryActive(auto))$('phase').textContent='地图入图暂停 · 正在恢复';
      if($('autonomy-panel')){
        const autonomyFresh=state.autonomy_age_ms>=0&&state.autonomy_age_ms<=500;
        $('autonomy-panel').hidden=!state.autonomy_enabled||state.task_state==='STANDBY';
        $('autonomy-panel').parentElement?.classList.toggle('autonomous-layout',!!state.autonomy_enabled);
        $('autonomy-stage').textContent=state.task_state==='STANDBY'?'网页待命，尚未启动本轮建图':auto.auto_start_pending?'等待地图和导航就绪，自动尚未启动':autonomyPhases[auto.phase]||'等待地图初始化和车端任务';
        if(auto.phase==='RETURNING')$('autonomy-stage').textContent=returnFeedback(auto)?.summary||'返程正在执行';
        const returning=auto.phase==='RETURNING'||auto.return_result==='running',faulted=auto.phase==='FAULT'||state.phase==='FAILED';
        $('autonomy-budget').textContent=`半径 ${auto.radius_m||20}米 · ${returning?'返程':'探索'}剩余 ${Math.ceil((returning?(auto.return_remaining_seconds??600):(auto.remaining_seconds??2400))/60)}分钟${auto.goal?` · 目标(${auto.goal.map(v=>v.toFixed(2)).join(', ')})米`:''}`;
        if(saved)$('autonomy-budget').textContent=`本轮探索已结束 · ${returnFeedback(auto)?.summary||'返程结果待核对'} · 候选待审核`;
        if(!autonomyFresh&&state.task_state!=='STANDBY'){
          $('autonomy-stage').textContent='状态过期 · 上次记录：'+$('autonomy-stage').textContent;
          $('autonomy-budget').textContent='状态过期 · 上次记录：'+$('autonomy-budget').textContent;
        }
        renderSpeed(auto,autonomyFresh,uiBusy,saved);
        renderRecovery(auto,autonomyFresh);
        const autoStartBlocked=auto.automatic_start_ready===false;
        const autoStartReason=auto.automatic_start_reason||'自动尚未开始：请检查起点周围整车足迹是否位于已知自由区域';
        $('autonomy-quality').textContent=state.task_state==='STANDBY'?'请选择工程类型和控制方式；自动前先核对场地':!autonomyFresh?'车端任务状态过期；操作未下发，请检查任务进程和连接':auto.quality_fault||auto.error||(['MANUAL','PAUSED'].includes(auto.phase)&&autoStartBlocked?autoStartReason:['SAVE_PENDING','REVIEW_REQUIRED'].includes(auto.phase)?auto.reason||auto.end_reason:auto.health_reason||auto.reason)||(auto.healthy?'输入健康，地图与任务状态正在更新。':'等待车端健康检查，请核对传感器与任务状态。');
        $('autonomy-quality').hidden=checkpointActive(auto)||recoveryActive(auto);
        if(returnFeedback(auto))$('autonomy-quality').textContent=returnNotice(auto)+(saved?'地图质量仍需现场审核。':'');
        const sentence=navigationSentence(auto);
        $('nav-status').hidden=!sentence;$('nav-status').textContent=sentence;
        const ownResult=ownTaskResult(auto);
        if(autonomyPending&&['accepted','rejected','cancelled'].includes(ownResult?.request_result)){
          autonomyNotice=ownResult.request_result==='accepted'?'车端已受理，等待执行终态及停稳确认。':
            (ownResult.reason||auto.error||'本次操作已拒绝或取消；没有因该请求下发新目标。');
          if(ownResult.request_result!=='accepted')operatorClaimAfterManual='';
          if(autonomyAction)autonomyAction.canonical_request_id=ownResult.canonical_request_id||autonomyAction.canonical_request_id;
          if(autonomyAction)autonomyAction.confirmed=true;
          autonomyPending=null;
        }
        // 轮询可能直接跨到保存终态；同一任务已结束时，不继续等被save_done替换的返程请求ID。
        if(autonomyPending&&autonomyAction?.task_id===auto.task_id&&
          ['finish','return'].includes(autonomyAction.action)&&returnFeedback(auto)?.terminal){
          autonomyAction.confirmed=true;autonomyPending=null;
        }
        if(autonomyPending&&performance.now()-autonomyPending.at>8000){autonomyNotice='未收到本次操作的车端确认；请核对当前阶段和诊断，不自动重发。';autonomyPending=null;operatorClaimAfterManual='';}
        if(autonomyAction&&ownResult?.request_result==='accepted'){
          const stopped=auto.still&&auto.navigation_terminal&&auto.output?.every(v=>v===0);
          if(['pause','halt'].includes(autonomyAction.action))autonomyNotice=auto.phase==='PAUSED'&&stopped?'暂停完成：导航已取消、输出为零、轮式反馈停稳；请目视核对。':'暂停已受理，正在取消导航并等待停稳。';
          else if(autonomyAction.action==='manual')autonomyNotice=auto.phase==='MANUAL'&&stopped?'手动接管完成；启用遥控后再操作摇杆。':'接管已受理，等待取消导航和停稳。';
          else if(autonomyAction.action==='recheck')autonomyNotice=auto.operator_check?.phase==='READY'?'检查通过，保持暂停，请选择下一步。':auto.operator_check?.phase==='FAILED'?auto.operator_check.reason:'检查已受理，车辆保持停车。';
          else if(['finish','return'].includes(autonomyAction.action))autonomyNotice=returnNotice(auto)||'返程已受理，等待停稳和路径预检；尚未确认返程完成。';
          else autonomyNotice=auto.phase==='RETURNING'?'返程已恢复，继续沿用原起点和返程预算。':auto.phase==='EXPLORING'?'自动探索已开始；可以随时暂停或结束探索并返程。':'自动请求已受理，等待地图和导航就绪。';
        }
        if(operatorClaimAfterManual===session&&manualControlAllowed()&&auto.still&&auto.navigation_terminal&&!autonomyPending){
          operatorClaimAfterManual='';void operatorRequest('claim');
        }
        if(returnFeedback(auto))autonomyNotice=returnNotice(auto);
        else if(checkpointActive(auto)||recoveryActive(auto))autonomyNotice='';
        const unavailable=!session||uiBusy||!auto.task_id||!!autonomyPending||!autonomyFresh;
        $('autonomy-actions').classList.toggle('completed',saved||faulted);
        $('auto-finish').classList.toggle('primary',saved||faulted);
        $('auto-pause').classList.toggle('primary',['EXPLORING','RETURNING'].includes(auto.phase));
        $('autonomy-hint').textContent=auto.phase==='PAUSED'&&auto.reason?auto.reason:saved?'本轮探索已结束。结束任务后可审核候选或新建；已保存工程保留。':checkpointActive(auto)?'检查点保存期间等待车端完成；需要停止自动续跑请点顶部停车，并核对车端状态。':recoveryActive(auto)?'受扰恢复期间不接收启动和接管；可信验证通过后自动继续。需要停止恢复请点顶部停车并核对车端状态。':'自动任务在网页断连后继续；停止请点停车并核对车端状态。手动断连仍停车。';
        $('auto-start').hidden=saved||faulted||!['MANUAL','PAUSED'].includes(auto.phase);
        $('auto-pause').hidden=saved||faulted||!['EXPLORING','RETURNING','STOPPING'].includes(auto.phase);
        $('auto-manual').hidden=saved||faulted||!['EXPLORING','PAUSED'].includes(auto.phase);
        $('auto-finish').hidden=!saved&&!faulted&&!['EXPLORING','PAUSED'].includes(auto.phase);
        $('auto-start').textContent=auto.phase==='PAUSED'?(returning?'恢复返程':'恢复自动探索'):'开始自动探索';
        $('auto-start').disabled=unavailable||!['MANUAL','PAUSED'].includes(auto.phase)||!auto.healthy||autoStartBlocked;
        $('auto-pause').disabled=unavailable||!['EXPLORING','RETURNING'].includes(auto.phase);
        $('auto-manual').disabled=unavailable||!['EXPLORING','PAUSED'].includes(auto.phase);
        $('auto-finish').disabled=saved?!session||uiBusy:faulted?!session||uiBusy||!auto.still||!auto.navigation_terminal:unavailable||!['EXPLORING','PAUSED'].includes(auto.phase);
        $('auto-finish').textContent=saved?'结束任务并查看候选':faulted?'结束故障任务':'结束探索并返程';
        for(const id of ['auto-start','auto-pause','auto-manual','auto-finish'])$(id).title=$(id).disabled?(!session?'等待控制连接':saved?'本轮探索已结束，候选已保存；请先结束任务再审核或新建':!autonomyFresh?'车端状态过期':state.busy?'正在保存或处理任务':autonomyPending?'等待同一次操作确认':id==='auto-start'&&autoStartBlocked?autoStartReason:auto.health_reason||'当前阶段不允许该操作'):'';
        renderOperatorActions(auto,autonomyFresh,uiBusy);
      }
      $('motion').textContent=saved?'探索已结束；确认实物停稳后结束任务，查看候选或开始新一轮':state.inspection_running?'巡检还在进行，请先在巡检页取消或等这轮结束':state.phase==='STANDBY'?'网页待命，尚未启动硬件':state.task_state==='STARTING'?'正在预检和启动，车辆保持停车':!controlFresh()?'控制状态过期，请检查连接':control?.reason||'等待运动门控';
      $('enable').disabled=!!state.inspection_running||!ownsManualControl()||!session||state.busy||state.save_failed||state.phase!=='CURRENT'||!controlFresh()||!!control?.armed||!!control?.health;
      if(!manualControlAllowed())$('enable').disabled=true;
      joystick.classList.toggle('disabled',!manualControlAllowed()||!ownsManualControl()||!control?.armed||state.busy||!session);
      $('fast-option').disabled=!control?.supported_gears?.includes('fast');
      $('save').disabled=saved||state.phase!=='CURRENT'||state.busy||!session;
      $('save').hidden=saved;
      if(autoBusy)$('save').disabled=true;
      $('finish').hidden=!state.managed;$('finish').disabled=saved?!session||state.busy:$('save').disabled;
      $('finish-label').textContent=saved?'结束任务并查看候选':'保存并结束';
      const idle=!!state.managed&&state.task_state==='STANDBY';
      if($('automatic-mode-option'))$('automatic-mode-option').disabled=!state.autonomy_enabled;
      if($('publication-panel'))$('publication-panel').hidden=!idle||!state.autonomy_enabled;
      renderPublicationControls();
      $('task-panel').hidden=!state.managed||!['STANDBY','STARTING'].includes(state.task_state);
      $('start').disabled=!!state.inspection_running||!idle||uiBusy||!session;$('mode').disabled=$('control-mode').disabled=$('bundle').disabled=$('refresh-bundles').disabled=!idle||uiBusy;
      $('start').textContent=state.task_state==='STARTING'||operationSubmitting&&idle?'正在检查并启动…':'检查并启动硬件';
      $('bundle').hidden=$('mode').value!=='resume';
      $('task-info').textContent=idle?'网页待命，硬件未启动':`本次运行 ${Math.floor((state.task_age||0)/60)}分 · ${state.bundle||'新工程'}${state.dirty?' · 有未保存进度':''}`;
      renderStartInformation();
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
      const returnStatus=returnNotice(auto);
      $('result').textContent=(returnStatus?returnStatus+((state.error||clientError)?'；'+(state.error||clientError):''):clientError|| (taskOperationBusy()?'请求已接收，等待本次操作完成；车辆保持停车。':checkpointActive(auto)||recoveryActive(auto)?'':autonomyNotice||state.error||state.result||''))+(state.save_reminder&&state.phase==='CURRENT'&&!saved?'；已超过5分钟，请选择安全位置停车保存。':'');
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
        const previousReturn=returnFeedback(state.autonomy||{});
        $('result').textContent='状态更新中断；无法确认车辆当前执行结果，请检查连接并核对实物。'+
          (previousReturn?'上次记录：'+previousReturn.text:'');
        for(const id of ['autonomy-stage','autonomy-budget']){
          const previous=$(id).textContent||'';
          if(!previous.startsWith('状态过期'))$(id).textContent='状态过期 · 上次记录：'+previous;
        }
        for(const id of ['start','enable','save','finish','auto-start','auto-pause','auto-manual','auto-finish','auto-recheck','auto-return','auto-speed','auto-speed-apply'])$(id).disabled=true;
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
