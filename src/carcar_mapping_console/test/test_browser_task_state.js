'use strict';
// 回放现场自动保存后的真实状态组合，只替换DOM和网络，不连接或控制实车。
const assert = require('assert'), fs = require('fs'), vm = require('vm');
const source = fs.readFileSync(process.argv[2], 'utf8');
async function render(snapshot, {selection='manual',clientId='browser-client',operatorReply=null,autonomyReply=null}={}) {
  const elements = {}, requests = [], confirmations=[],context2d = new Proxy({}, {get: () => () => {}});
  const sockets=[],socketMessages=[];
  let now=100,current=snapshot,statusError=null,timerId=0;
  const timers=new Map();
  const element = id => elements[id] ||= {id, style:{}, hidden:false, disabled:false,
    checked:true, value:'manual', textContent:'', classList:{toggle(){},add(){},remove(){}},
    getContext:()=>context2d, clientWidth:900,clientHeight:600, setPointerCapture(){},focus(){},scrollIntoView(){},append(child){child.parentElement=this;},
    getBoundingClientRect:()=>({left:0,top:0,width:240,height:240})};
  class Socket {
    static OPEN=1;static CLOSED=3;
    constructor(){this.readyState=1;this.bufferedAmount=0;sockets.push(this);}
    send(body){socketMessages.push(JSON.parse(body));} close(){this.readyState=3;}
  }
  const context = {document:{hidden:false,getElementById:element,querySelectorAll:()=>[],addEventListener(){}},
    window:{devicePixelRatio:1,addEventListener(){},confirm:message=>{confirmations.push(message);return true;}}, location:{host:'offline.test'},
    WebSocket:Socket,performance:{now:()=>now},setInterval(){},
    setTimeout:(callback,ms)=>{const id=++timerId;timers.set(id,{callback,ms});return id;},
    clearTimeout:id=>timers.delete(id),
    AbortController,crypto:require('crypto').webcrypto,requestAnimationFrame(){},console,
    fetch:async (url,options={})=>{
      requests.push({url,options});
      if(url==='/api/status'&&statusError)throw statusError;
      if(url==='/api/operator')return {ok:true,status:200,json:async()=>operatorReply?.(JSON.parse(options.body),current)||
        {client_id:clientId,is_owner:current.manual_control?.owner_id===clientId,manual_control:current.manual_control}};
      if(url==='/api/autonomy'&&autonomyReply)return {ok:true,status:202,json:async()=>autonomyReply(JSON.parse(options.body))};
      return {ok:true,status:200,json:async()=>url==='/api/status'?current:{accepted:true,operation:3}};
    }};
  vm.createContext(context);vm.runInContext(source,context);
  element('control-mode').value=selection;
  vm.runInContext("session='offline-observer';lastControlAt=100;latestControl={armed:false,health:''};",context);
  await new Promise(resolve=>setImmediate(resolve));
  // 每次只推进真实轮询的250ms等待，不触发控制或传感器模拟。
  async function next(updated=current,{elapsed=250,error=null}={}){
    current=updated;statusError=error;now+=elapsed;
    const pending=[...timers].find(([,timer])=>timer.ms===250);
    assert(pending,'网页状态轮询未继续，不能回放后续返程状态');
    timers.delete(pending[0]);pending[1].callback();
    await new Promise(resolve=>setImmediate(resolve));
  }
  return {elements,requests,context,confirmations,next,sockets,socketMessages};
}
const completed = {phase:'CURRENT',task_state:'RUNNING',managed:true,busy:false,
  autonomy_enabled:true,autonomy_age_ms:10,dirty:false,last_saved_bundle:'saved-candidate',
  result:'地图与工程保存完整:saved-candidate',control:{armed:false},dataflow_protocol:1,
  autonomy:{task_id:'completed-task',phase:'REVIEW_REQUIRED',control_mode:'automatic',healthy:false,
    still:true,navigation_terminal:true,outcome:'partial_unreachable',return_result:'failed',
    reason:'部分区域不可达，保留缺口；尝试返回；起点不可达，原地停稳保存候选',
    health_reason:'Nav2局部／全局代价图使用的车位陈旧；检查内部TF与代价图线程'}};
(async()=>{
  for(const phase of ['WAITING_ROLLBACK_STABLE','VERIFYING_ROLLBACK','VERIFY_READY','SUCCEEDED']){
    const restored=await render({...completed,dirty:true,last_saved_bundle:'',result:'',
      autonomy:{...completed.autonomy,phase:phase==='SUCCEEDED'?'EXPLORING':'RECOVERING',healthy:phase==='SUCCEEDED',
        recovery_pending:phase!=='SUCCEEDED',recovery:{phase,verified_pose:[.41,-3.72,-1.52],checkpoint_id:'rollback-reference',imu_ready:true,imu_stable:true}}});
    assert.equal(restored.elements.stop.disabled,false);
    assert.equal(restored.requests.filter(r=>r.options.method==='POST').length,0,'显示恢复结果不能由网页自行发送继续指令');
    if(['VERIFY_READY','SUCCEEDED'].includes(phase))assert.match(restored.elements['recovery-status'].textContent,/地图已复验.*0\.41.*-3\.72/);
    else assert(!restored.elements['recovery-status'].textContent.includes('地图已复验'),'未完成验证不能显示成功');
    if(phase==='VERIFYING_ROLLBACK')assert.match(restored.elements['workflow-next'].textContent,/六帧新激光/);
  }
  const staleRecovery=await render({...completed,dirty:true,last_saved_bundle:'',result:'',autonomy_age_ms:2000,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,recovery:{phase:'SUCCEEDED',verified_pose:[.41,-3.72,-1.52]}}});
  assert(!staleRecovery.elements['workflow-next'].textContent.includes('地图已复验'),'过期恢复位置不得当作当前状态');
  const retryReason='检查点尚未提交：轻微IMU波动，保持停车和冻结；等待连续稳定2秒后重新保存校验（最多等待12秒）';
  const checkpointRetry=await render({...completed,dirty:true,last_saved_bundle:'',result:'',
    autonomy:{...completed.autonomy,phase:'CHECKPOINTING',recovery:{phase:'CHECKPOINTING',imu_ready:true,imu_stable:false,
      checkpoint_revalidation:{waiting:true,attempts:1,reason:retryReason}}}});
  assert(checkpointRetry.elements['workflow-next'].textContent.includes(retryReason));
  assert(checkpointRetry.elements['recovery-status'].textContent.includes(retryReason));
  assert.equal(checkpointRetry.elements.stop.disabled,false);
  assert.equal(checkpointRetry.requests.filter(r=>r.options.method==='POST').length,0);
  const startBlocked=await render({...completed,dirty:true,last_saved_bundle:'',result:'',
    autonomy:{...completed.autonomy,phase:'PAUSED',healthy:true,return_result:'running',
      return_destination:{phase:'BLOCKED',reason:'实际车位存在动态障碍；未下发运动目标'},
      reason:'实际车位存在动态障碍；未下发运动目标；检查车旁障碍与地图'}});
  assert.match(startBlocked.elements['workflow-next'].textContent,/实际车位存在动态障碍/);
  assert.match(startBlocked.elements['autonomy-hint'].textContent,/未下发运动目标/);
  const stalled=await render({...completed,dirty:true,last_saved_bundle:'',result:'',
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,goal:null,
      exploration_wait:{phase:'WAITING',seconds:4,limit_seconds:15},
      navigation_detail:'车端规划拒绝：没有安全入口',exploration_note:'等待可执行目标'}});
  assert.equal(stalled.elements['workflow-title'].textContent,'等待可执行目标');
  assert.match(stalled.elements['workflow-next'].textContent,/没有安全入口/);
  const stalledPaused=await render({...completed,dirty:true,last_saved_bundle:'',result:'',
    autonomy:{...completed.autonomy,phase:'PAUSED',healthy:true,outcome:'',
      exploration_wait:{phase:'PAUSED',seconds:15,limit_seconds:15},
      reason:'车端连续15秒没有可执行目标，已暂停；检查地图未知区与通道'}});
  assert.match(stalledPaused.elements['workflow-next'].textContent,/没有可执行目标/);
  // 暂停后导航失效仍应展示车端允许的静止重定位入口。
  const frozenPaused=await render({...completed,dirty:true,last_saved_bundle:'',result:'',
    autonomy:{...completed.autonomy,phase:'PAUSED',healthy:false,outcome:'',
      health_reason:'地图写入已冻结，等待定位验证',
      actions:Object.fromEntries(['resume','pause','manual','return','recheck','halt'].map(action=>
        [action,{enabled:['recheck','halt'].includes(action),reason:'等待定位恢复'}]))}});
  assert.equal(frozenPaused.elements['auto-recheck'].hidden,false,'暂停状态隐藏了恢复入口');
  assert.equal(frozenPaused.elements['auto-recheck'].disabled,false);
  assert.equal(frozenPaused.elements['auto-start'].disabled,true);
  await frozenPaused.elements['auto-recheck'].onclick();
  assert(frozenPaused.requests.some(r=>r.url==='/api/autonomy'&&JSON.parse(r.options.body).action==='recheck'));
  const returnReason='操作者结束探索，尝试返回起点；起点不可达，原地停稳保存候选';
  const returnActions=Object.fromEntries(['resume','pause','manual','return','recheck'].map(action=>[action,{enabled:true,reason:''}]));
  const exploring={...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,return_result:'not_started',
      reason:'正在自动建图',health_reason:'',actions:returnActions}};
  const incident=await render(exploring);
  await incident.elements['auto-return'].onclick();
  const returnRequests=incident.requests.filter(r=>r.url==='/api/autonomy');
  const returnRequest=JSON.parse(returnRequests[0].options.body);
  assert.equal(returnRequest.action,'return');
  assert(incident.elements.result.textContent.includes('等待'),'HTTP受理未等待车端执行终态');
  assert(!incident.elements.result.textContent.includes('已返回'),'HTTP受理被当成返程完成');
  const accepted={...exploring,autonomy:{...exploring.autonomy,phase:'STOPPING',
    request_id:returnRequest.request_id,request_result:'accepted',reason:'等待取消探索目标'}};
  await incident.next(accepted);
  assert(!incident.elements.result.textContent.includes('已返回'),'车端受理被当成返程完成');
  const failed={...accepted,autonomy:{...accepted.autonomy,phase:'SAVE_PENDING',return_result:'failed',
    reason:returnReason,still:true,navigation_terminal:true,output:[0,0,0]}};
  for(const result of ['failed','timeout']){
    const stoppingFailure={...failed,autonomy:{...failed.autonomy,phase:'STOPPING',stop_target:'SAVE',
      return_result:result,still:false,navigation_terminal:false,output:[.04,.03,0],
      return_destination:{kind:'none',phase:'FAILED',reason:result==='failed'?'附近可达候选耗尽，返程未执行':'返程预算耗尽，正在取消导航'}}};
    await incident.next(stoppingFailure);
    const main=incident.elements.result.textContent;
    assert(main.includes(result==='failed'?'附近可达候选耗尽':'返程预算耗尽'),'保存前停车屏障丢失已确认的返程失败原因');
    assert(main.includes('轮式停稳后保存'),'返程失败停车屏障没有解释保存尚未开始');
    assert(!main.includes('等待停稳和路径预检'),'返程已经失败仍提示等待路径预检');
    assert(!main.includes('候选已保存'),'尚未停稳时提前宣称已保存候选');
  }
  await incident.next(failed);
  for(const id of ['result','autonomy-quality','workflow-next'])
    assert(incident.elements[id].textContent.includes('起点不可达'),`${id}没有保留导致车辆未返程的具体原因`);
  const savedFailure={...failed,last_saved_bundle:'saved-candidate',dirty:false,result:'地图与工程保存完整:saved-candidate',
    autonomy:{...failed.autonomy,phase:'REVIEW_REQUIRED',request_id:'save-done-request',request_result:'accepted',
      healthy:false,health_reason:'会话已保存，导航已退出'}};
  await incident.next(savedFailure);
  for(const id of ['result','autonomy-quality','workflow-next'])
    assert(incident.elements[id].textContent.includes('起点不可达'),`${id}在save_done更换request_id后丢失返程失败原因`);
  assert(!incident.elements['autonomy-quality'].textContent.includes('导航已退出'),'保存后健康状态覆盖返程结论');
  assert.equal(incident.requests.filter(r=>r.url==='/api/autonomy').length,1,'回放失败或保存后自动重复下发返程');
  const missedAck=await render(exploring);
  await missedAck.elements['auto-return'].onclick();
  await missedAck.next(savedFailure);
  assert.equal(missedAck.elements['auto-finish'].disabled,false,'轮询跨过返程受理状态后，save_done使结束入口继续等待旧request_id');
  assert(missedAck.elements.result.textContent.includes('起点不可达'),'漏过中间受理状态后丢失任务返程结论');
  const saveFailed=await render({...failed,phase:'FAILED',save_failed:true,error:'候选保存失败：磁盘写入失败'});
  assert(saveFailed.elements.result.textContent.includes('起点不可达'),'保存失败覆盖了返程未执行的原因');
  assert(saveFailed.elements.result.textContent.includes('磁盘写入失败'),'返程结论掩盖保存失败原因');
  const destination={kind:'origin',phase:'COMPLETED',target:[.01,0,0],search_radius_m:.8,
    arrival_tolerance_m:.15,distance_to_origin_m:.04,target_offset_m:0,reason:'原起点到达并停稳核对通过'};
  const exact=await render({...savedFailure,autonomy:{...savedFailure.autonomy,
    return_result:'succeeded',reason:destination.reason,return_destination:destination}});
  assert(exact.elements.result.textContent.includes('已返回原起点'),'原起点成功没有独立结论');
  assert(!exact.elements.result.textContent.includes('起点不可达'),'成功返程残留旧失败原因');
  const nearbyDestination={...destination,kind:'nearby',target:[.6,0,0],distance_to_origin_m:.72,
    target_offset_m:.60,reason:'原起点足迹被地图占用，已返回可达的起点附近并停稳核对通过'};
  const nearby=await render({...savedFailure,autonomy:{...savedFailure.autonomy,
    return_result:'nearby',reason:nearbyDestination.reason,return_destination:nearbyDestination}});
  for(const id of ['result','autonomy-budget','autonomy-quality','workflow-next']){
    const text=nearby.elements[id].textContent;
    assert(text.includes('起点附近'),`${id}未区分附近返程与准确原起点返程`);
    assert(text.includes('0.72 m')&&text.includes('地图估计'),`${id}未显示当前停车位置到原起点的地图估计距离`);
    assert(!text.includes('已返回原起点'),'附近成功被误标成原起点成功');
    assert(!text.includes('0.60 m'),'把候选目标偏移误报为车辆实际停车距离');
  }
  const unknownDistance=await render({...savedFailure,autonomy:{...savedFailure.autonomy,
    return_result:'nearby',return_destination:{...nearbyDestination,distance_to_origin_m:null}}});
  assert(!unknownDistance.elements.result.textContent.includes('0.60 m'),'没有当前位置时把目标偏移当成停车距离');
  assert(unknownDistance.elements.result.textContent.includes('距离待核对'),'缺少实测位置时没有说明距离待核对');
  const verifying={...exploring,autonomy:{...exploring.autonomy,phase:'RETURNING',return_result:'running',
    reason:'Nav2已报告到达，等待轮式停稳和当前位姿核对',return_destination:{...nearbyDestination,phase:'VERIFYING'}}};
  const verify=await render(verifying);
  assert(verify.elements['workflow-title'].textContent.includes('核对'),'Nav2完成后未显示独立停车位置核对阶段');
  for(const id of ['result','autonomy-quality','workflow-next']){
    assert(verify.elements[id].textContent.includes('尚未确认返程完成'),`${id}在VERIFYING提前确认完成`);
    assert(!verify.elements[id].textContent.includes('已返回起点附近'),`${id}在停稳和位置核对前宣布到达`);
  }
  assert(verify.elements['autonomy-stage'].textContent.includes('核对'),'任务阶段仍把停车核对描述为行驶返程');
  for(const [phase,word] of [['SELECTING','筛选'],['PLANNING','预检'],['NAVIGATING','起点附近']]){
    const progressing=await render({...verifying,autonomy:{...verifying.autonomy,
      return_destination:{...nearbyDestination,phase}}});
    assert(progressing.elements['workflow-title'].textContent.includes(word),`${phase}没有对应真实返程阶段`);
    assert(!progressing.elements.result.textContent.includes('已返回'),'筛选、预检或导航阶段提前宣布完成');
  }
  const pauseRejected=await render({...verifying,autonomy:{...verifying.autonomy,
    return_destination:{...nearbyDestination,phase:'NAVIGATING'}}});
  await pauseRejected.elements['auto-pause'].onclick();
  const pauseRequest=JSON.parse(pauseRejected.requests.find(r=>r.url==='/api/autonomy').options.body);
  const rejectionState={...verifying,autonomy:{...verifying.autonomy,request_id:pauseRequest.request_id,
    request_result:'rejected',error:'暂停请求未通过车端检查，请检查任务日志',
    return_destination:{...nearbyDestination,phase:'NAVIGATING'}}};
  await pauseRejected.next(rejectionState);
  for(const id of ['result','autonomy-quality','workflow-next']){
    assert(pauseRejected.elements[id].textContent.includes('暂停请求未通过车端检查'),`${id}用原返程进度掩盖本次暂停拒绝原因`);
    assert(pauseRejected.elements[id].textContent.includes('正在返回起点附近'),`${id}拒绝提示没有同时说明原返程状态`);
  }
  await pauseRejected.next(rejectionState);
  assert(pauseRejected.elements.result.textContent.includes('暂停请求未通过车端检查'),'下一次轮询丢失同一次操作的拒绝结果');
  assert.equal(pauseRejected.requests.filter(r=>r.url==='/api/autonomy').length,1,'车端拒绝后网页自动重发请求');
  const timedOut=await render({...savedFailure,autonomy:{...savedFailure.autonomy,return_result:'timeout',
    reason:'返程预算耗尽，已取消导航并停稳',return_destination:{...nearbyDestination,phase:'FAILED',reason:'返程预算耗尽，已取消导航并停稳'}}});
  assert(timedOut.elements.result.textContent.includes('返程超时'));
  assert(timedOut.elements.result.textContent.includes('返程预算耗尽'),'超时结果被候选保存成功掩盖');
  assert(!timedOut.requests.some(r=>r.options.method==='POST'),'超时展示触发自动重发');
  const staleNearby=await render({...savedFailure,autonomy_age_ms:2000,autonomy:{...savedFailure.autonomy,
    return_result:'nearby',return_destination:nearbyDestination}});
  assert(staleNearby.elements.result.textContent.includes('车端状态过期'),'过期状态没有标明返程结论来自历史记录');
  assert(staleNearby.elements.result.textContent.includes('上次记录'),'过期结果仍被当成本次实时结论');
  for(const id of ['autonomy-stage','autonomy-budget'])
    assert(staleNearby.elements[id].textContent.includes('状态过期'),`${id}未标明保存摘要来自陈旧状态`);
  assert.equal(staleNearby.elements['auto-return'].disabled,true,'过期状态下仍可下发返程');
  const staleVerifying=await render({...verifying,autonomy_age_ms:2000});
  for(const id of ['autonomy-stage','autonomy-budget'])
    assert(staleVerifying.elements[id].textContent.includes('状态过期'),`${id}把陈旧返程阶段显示为实时状态`);
  await verify.next(verifying,{elapsed:2000,error:Error('离线回放：网络断开')});
  assert(verify.elements['workflow-title'].textContent.includes('中断'),'断线后仍把返程状态作为实时状态');
  assert(verify.elements.result.textContent.includes('状态更新中断'),'断线主反馈没有说明当前结论不可确认');
  assert(!verify.elements.result.textContent.includes('已返回起点附近'),'断线时误报返程成功');
  for(const id of ['autonomy-stage','autonomy-budget'])
    assert(verify.elements[id].textContent.includes('状态过期'),`${id}在HTTP断连后仍展示无标记的旧状态`);
  assert.equal(verify.elements['auto-return'].disabled,true,'断线后仍允许提交返程');
  await verify.next({...savedFailure,autonomy:{...savedFailure.autonomy,return_result:'nearby',
    return_destination:nearbyDestination}});
  assert(verify.elements.result.textContent.includes('已返回起点附近'),'重连后未从车端重新读取本轮结论');
  assert(!verify.elements.result.textContent.includes('状态更新中断'),'恢复连接后仍保留断线占位结论');
  assert(!verify.elements['autonomy-stage'].textContent.includes('状态过期'),'重连取得新状态后未清除历史标记');
  assert(!verify.requests.some(r=>r.options.method==='POST'),'重连后自动重放返程操作');
  const pendingTimeout=await render(exploring);
  await pendingTimeout.elements['auto-return'].onclick();
  await pendingTimeout.next(exploring,{elapsed:8100});
  assert(pendingTimeout.elements.result.textContent.includes('未收到本次操作的车端确认'));
  assert.equal(pendingTimeout.requests.filter(r=>r.url==='/api/autonomy').length,1,'操作确认超时后自动重发返程');
  const done=await render(completed),e=done.elements;
  assert(e.result.textContent.includes('起点不可达'),'旧后端缺少return_destination时丢失具体返程失败原因');
  assert(e.phase.textContent.includes('候选已保存'),'已保存结束的探索仍显示实时建图');
  assert.equal(e['auto-finish'].disabled,false,'自动保存后没有可用的结束入口');
  assert(e['auto-finish'].textContent.includes('结束任务'),'完成后按钮仍要求再次探索返程');
  assert(e['auto-finish'].textContent.includes('候选'),'结束入口没有说明下一步审核候选');
  assert.equal(e.end.textContent,'结束本次任务','已保存后结束仍被描述为放弃未保存进度');
  assert.equal(e.save.hidden,true,'已自动保存后仍突出重复保存入口');
  assert.equal(e['auto-start'].hidden,true,'已结束后仍突出不可用的继续自动操作');
  assert(!source.includes('site-confirmed'),'前端仍保留场地勾选门控');
  assert(e['autonomy-hint'].textContent.includes('本轮探索已结束'),'已结束后仍提示自动任务继续');
  vm.runInContext("receiveControl({armed:false,health:'',reason:'操作者已断开'})",done.context);
  assert.equal(e.enable.disabled,true,'新控制心跳覆盖了终态，错误恢复遥控使能按钮');
  assert(e.motion.textContent.includes('探索已结束'),'新控制心跳覆盖了终态下一步引导');
  await e['auto-finish'].onclick();
  assert(done.requests.some(r=>r.url==='/api/end'),'已保存候选的结束入口没有关闭当前任务');
  assert(!done.requests.some(r=>r.url==='/api/save'||r.url==='/api/finish'||r.url==='/api/autonomy'),
    '已保存后结束仍重复保存或重新请求自动动作');
  const running=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true}});
  assert.equal(running.elements['auto-finish'].textContent,'结束探索并返程');
  await running.elements['auto-finish'].onclick();
  assert(running.requests.some(r=>r.url==='/api/autonomy'&&JSON.parse(r.options.body).action==='finish'),
    '探索中的结束按钮不再请求返程');
  const paused=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'PAUSED',healthy:true}});
  assert.equal(paused.elements['auto-start'].disabled,false);
  const pausedReturn=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'PAUSED',return_result:'running',healthy:true}});
  assert.equal(pausedReturn.elements['workflow-title'].textContent,'返程已暂停','暂停返程被误称为探索暂停');
  assert.equal(pausedReturn.elements['auto-start'].textContent,'恢复返程','恢复返程被标成重新探索');
  assert(pausedReturn.elements['workflow-next'].textContent.includes('原返程预算'));
  await paused.elements['auto-manual'].onclick();
  for(const id of ['auto-start','auto-pause','auto-manual','auto-finish'])
    assert.equal(paused.elements[id].disabled,true,'请求已提交但按钮仍短暂可点，后续点击会被pending静默忽略');
  const blockedReason='自动尚未开始：当前起点整车足迹未全部位于已知自由区域';
  const manual={...completed,last_saved_bundle:'',dirty:true,autonomy:{...completed.autonomy,
    phase:'MANUAL',control_mode:'manual',healthy:true,automatic_start_ready:false,automatic_start_reason:blockedReason}};
  const blocked=await render(manual);
  for(const reason of [
    '自动尚未启动：正在原地累计激光观测，已接纳7帧；没有下发目标',
    '自动尚未启动：当前车位整车范围内还有1个未知格（未确认空闲），并非检测到障碍；没有下发目标',
    '自动尚未启动：地图净空检查发现2个占用格；请对照实物核对地图障碍，没有下发目标'
  ]){
    const view=await render({...manual,autonomy:{...manual.autonomy,automatic_start_reason:reason}});
    assert.equal(view.elements['autonomy-quality'].textContent,reason);
    assert(view.elements['auto-start'].disabled,'预检拒绝时不能仅显示原因而开放启动');
    assert(!view.requests.some(r=>r.url==='/api/autonomy'),'显示预检状态不能自动下发动作');
  }
  assert.equal(blocked.elements['auto-start'].disabled,true,'自动起点预检拒绝时按钮仍可用');
  assert(blocked.elements['autonomy-quality'].textContent.includes(blockedReason),'自动未开始的原因没有显示');
  assert(blocked.elements['auto-start'].title.includes(blockedReason),'禁用按钮没有解释起点预检原因');
  assert.equal(blocked.elements.enable.disabled,false,'自动起点预检错误禁止手动控制');
  assert(!blocked.requests.some(r=>r.options.method==='POST'),'预检拒绝时网页自动提交了控制操作');
  const legacyManual={...manual,autonomy:{...manual.autonomy}};
  delete legacyManual.autonomy.automatic_start_ready;delete legacyManual.autonomy.automatic_start_reason;
  assert.equal((await render(legacyManual)).elements['auto-start'].disabled,false,'新增起点字段破坏旧后端兼容');
  const standby={...completed,phase:'STANDBY',task_state:'STANDBY',autonomy_age_ms:60000};
  const idle=await render(standby,{selection:'automatic'});
  assert(idle.elements['task-info'],'待命渲染中断：'+JSON.stringify(vm.runInContext('clientEvents',idle.context)));
  assert.equal(idle.elements['autonomy-panel'].hidden,true,'硬件尚未启动时仍突出一排不可用的自动运行操作');
  assert(idle.elements.phase.textContent.includes('待命'),'返回待命后仍把上一轮候选当成本轮终态');
  assert(!idle.elements['autonomy-stage'].textContent.includes('候选已保存'),'待命仍显示上一轮自动任务');
  idle.elements.mode.value='new';idle.elements.start.onclick();
  await new Promise(resolve=>setImmediate(resolve));
  const starts=idle.requests.filter(r=>r.url==='/api/start');
  assert.equal(starts.length,1,'点击检查启动没有提交恰好一次请求');
  assert.deepEqual(JSON.parse(starts[0].options.body),{mode:'new',bundle:'',control_mode:'automatic',session:'offline-observer'});
  assert.equal(idle.confirmations.length,0,'普通启动仍弹出重复确认');
  const manualStandby=await render(standby,{selection:'manual'});
  const repeatedStart=await render(standby,{selection:'manual'});
  repeatedStart.elements.start.onclick();repeatedStart.elements.start.onclick();
  await new Promise(resolve=>setImmediate(resolve));
  assert.equal(repeatedStart.requests.filter(r=>r.url==='/api/start').length,1,
    '状态轮询前连续点击启动提交了重复请求');
  assert.equal(repeatedStart.elements.start.disabled,true,'启动提交后未立即锁定按钮');
  const starting=await render({...standby,task_state:'STARTING',phase:'WAITING',busy:true});
  assert.equal(starting.elements['task-panel'].hidden,false,'启动过程中配置区消失，操作者看不到进度');
  assert(starting.elements['workflow-title'].textContent.includes('启动'),'启动过程缺少当前阶段说明');
  assert.equal(running.elements['manual-controls'].hidden,true,'自动探索时仍突出手动遥控面板');
  assert.equal(running.elements['auto-start'].hidden,true,'已经自动探索时仍突出开始自动按钮');
  assert.equal(running.elements.enable.hidden,true,'自动探索时仍突出不可用的遥控使能');
  assert.equal(manualStandby.elements['manual-controls'].hidden,true,'硬件待命仍展示遥控摇杆');
  assert.equal(starting.elements['control-mode'].disabled,true,'启动过程中仍可修改控制方式');
  const autoPreparing=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'MANUAL',control_mode:'automatic',auto_start_pending:true,healthy:false}});
  assert.equal(autoPreparing.elements['manual-controls'].hidden,true,'等待自动启动时仍突出无效手动摇杆');
  assert.equal(autoPreparing.elements.enable.hidden,true,'待自动启动时错误提供手动使能');
  assert.equal(autoPreparing.elements['save-actions'].hidden,true,'待自动启动时突出手动保存流程');
  const checkpointRejected=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'MANUAL',control_mode:'automatic',auto_start_pending:true,healthy:false,
      recovery:{phase:'FAILED',reason:'当前激光与参考图重叠不足'}}});
  assert.equal(checkpointRejected.elements['workflow-title'].textContent,'恢复检查点未通过');
  assert(checkpointRejected.elements['workflow-next'].textContent.includes('重叠不足'),'失败被泛化为等待就绪');
  const fault=await render({...completed,last_saved_bundle:'',dirty:true,autonomy:{...completed.autonomy,
    phase:'FAULT',still:true,navigation_terminal:true,quality_fault:'扫描过期，已锁停'}});
  assert.equal(fault.elements['auto-finish'].textContent,'结束故障任务');
  assert.equal(fault.elements['auto-finish'].disabled,false,'确认停稳的故障任务没有清晰退出入口');
  await fault.elements['auto-finish'].onclick();
  assert(fault.requests.some(r=>r.url==='/api/end'),'结束故障任务未使用正常关闭接口');
  assert(!fault.requests.some(r=>r.url==='/api/autonomy'),'结束故障任务却提交了返程或恢复');
  manualStandby.elements['control-mode'].value='automatic';manualStandby.elements['control-mode'].onchange();
  assert(manualStandby.elements['task-info'].textContent.includes('开始自动探索'),'切换自动模式没有立即说明启动后的行为');
  const explained=await render({...completed,autonomy:{...completed.autonomy,
    motion_note:'导航已经给出速度，但激光进入车身停车框：左侧 0.15 米处有回波。近场保护把发给轮子的速度清成零，所以车停在原地。'}});
  assert(explained.elements['nav-status'].textContent.includes('左侧 0.15'),'激光进入停车框的原因没有显示');
  assert.equal(explained.elements['nav-status'].hidden,false);
  const revalidating=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,goal_revalidating:true,
      navigation_detail:'上一目标的历史失败',motion_note:'旧近场提示'}});
  assert(revalidating.elements['nav-status'].textContent.includes('重新检查原目标'),'内部重验被旧失败提示覆盖');
  assert(!revalidating.elements['nav-status'].textContent.includes('历史失败'),'重验期间误报旧导航失败');
  assert(!revalidating.requests.some(r=>r.options.method==='POST'),'展示内部重验状态却自动提交控制操作');
  const selecting=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,goal_selecting:true,
      goal:null,navigation_detail:'上一候选的历史失败',motion_note:'旧近场提示'}});
  assert(selecting.elements['nav-status'].textContent.includes('筛选可达观测点'),'可达性筛选期间仍显示历史失败');
  assert(!selecting.elements['nav-status'].textContent.includes('历史失败'),'筛选期间误报旧导航失败');
  assert.equal(selecting.elements['auto-pause'].disabled,false,'筛选期间不能暂停');
  assert(!selecting.requests.some(r=>r.options.method==='POST'),'展示筛选状态却自动提交控制操作');
  const rejected=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,goal:null,navigation_terminal:true,
      navigation_detail:'前沿附近没有同时满足净空、视线和车位连通性的观测点；未提交路径预检，未下发运动目标',motion_note:'上一目标的近场提示'}});
  assert(rejected.elements['nav-status'].textContent.includes('未提交路径预检'),'可达性拒绝被上一目标近场提示覆盖');
  const waiting=await render({...completed,last_saved_bundle:'',dirty:true,autonomy:{...completed.autonomy,
    phase:'EXPLORING',healthy:true,exploration_note:'暂时没有适合继续前进的目标；探索尚未完成',navigation_detail:'上一个候选失败'}});
  assert(waiting.elements['nav-status'].textContent.includes('尚未完成'),'全部候选暂避被显示成旧失败或完成');
  const speedState={requested_mps:.10,confirmed_mps:.10,cap_mps:.10,pending:false,phase:'READY',error:'',request_id:''};
  const speedRun=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,speed:speedState}});
  const se=speedRun.elements;
  assert.equal(se['auto-speed-apply'].disabled,false,'探索中不能调整速度');
  se['auto-speed'].value='.18';se['auto-speed'].oninput();await se['auto-speed-apply'].onclick();
  const speedRequest=JSON.parse(speedRun.requests.find(r=>r.url==='/api/autonomy').options.body);
  assert.equal(speedRequest.action,'set_speed');assert.equal(speedRequest.speed_mps,.18);
  assert.equal(se['auto-speed-apply'].disabled,true,'调速提交后没有锁定重复请求');
  assert.equal(se['auto-pause'].disabled,false,'调速确认阻止了暂停入口');
  assert(!se['auto-speed-status'].textContent.includes('调速已确认'),'HTTP成功被误报为控制器调速成功');
  vm.runInContext('renderSpeed(state.autonomy,true,false,false)',speedRun.context);
  assert.equal(se['auto-speed-apply'].disabled,true,'旧速度状态错误解除了本次pending');
  vm.runInContext(`state.autonomy.speed={requested_mps:.18,confirmed_mps:.18,cap_mps:.18,pending:false,phase:'READY',error:'',request_id:${JSON.stringify(speedRequest.request_id)}};renderSpeed(state.autonomy,true,false,false)`,speedRun.context);
  assert(se['auto-speed-status'].textContent.includes('执行上限 0.18'),'车端读回确认后没有显示实际执行上限');
  assert.equal(se['auto-speed-apply'].disabled,false);
  se['auto-speed'].value='.05';se['auto-speed'].oninput();await se['auto-speed-apply'].onclick();
  const lowerRequest=JSON.parse(speedRun.requests.filter(r=>r.url==='/api/autonomy').at(-1).options.body);
  vm.runInContext(`state.autonomy.speed={requested_mps:.05,confirmed_mps:.18,cap_mps:.05,pending:false,phase:'FAILED',error:'控制器写入失败，保留较低上限',request_id:${JSON.stringify(lowerRequest.request_id)}};renderSpeed(state.autonomy,true,false,false)`,speedRun.context);
  assert(se['auto-speed-status'].textContent.includes('执行上限 0.05'),'调速失败后没有显示较低执行上限');
  assert(se['auto-speed-status'].textContent.includes('写入失败'),'调速失败没有说明控制器层原因');
  vm.runInContext("state.autonomy.phase='RETURNING';renderSpeed(state.autonomy,true,false,false)",speedRun.context);
  assert.equal(se['auto-speed-apply'].disabled,false,'返程中不能调速');
  vm.runInContext('renderSpeed(state.autonomy,false,false,false)',speedRun.context);
  assert.equal(se['auto-speed-apply'].disabled,true,'状态过期时仍能下发调速');
  assert(se['auto-speed-apply'].title.includes('状态过期'),'过期拒绝没有解释');
  vm.runInContext("state.autonomy.phase='EXPLORING';renderSpeed(state.autonomy,true,false,false)",speedRun.context);
  await se['auto-pause'].onclick();
  assert.equal(se['auto-speed-apply'].disabled,true,'暂停确认前仍允许插入调速覆盖同次请求确认');
  vm.runInContext('renderSpeed(state.autonomy,true,false,false)',speedRun.context);
  assert.equal(se['auto-speed-apply'].disabled,true,'状态轮询恢复了暂停确认期间的调速入口');
  for(const phase of ['WAITING_STABLE','LOCALIZING','RESTORING','VERIFYING','VERIFY_READY']){
    const recovery=await render({...completed,last_saved_bundle:'',dirty:true,
      autonomy:{...completed.autonomy,phase:'RECOVERING',healthy:false,recovery_pending:true,
        recovery:{phase,reason:'受扰恢复流程进行中',checkpoint_id:'stable-checkpoint',checkpoint_age_seconds:12,
          imu_ready:true,imu_stable:true,imu_age_seconds:.02}}});
    assert.equal(recovery.elements['workflow-title'].textContent,'正在恢复地图与定位','自动恢复误报成故障退出');
    assert.equal(recovery.elements['auto-start'].hidden,true);assert.equal(recovery.elements['auto-manual'].hidden,true);
    assert.equal(recovery.elements['auto-speed-apply'].disabled,true,'恢复期间仍允许调速');
    assert.equal(recovery.elements['save-actions'].hidden,true,'恢复期间仍突出保存动作');
    assert.equal(recovery.elements.stop.disabled,false,'恢复期间停车入口不可用');
    assert(recovery.elements['recovery-status'].textContent.includes('stable-checkpoint'));
    assert(recovery.elements['recovery-status'].textContent.includes('姿态观测平稳'));
    assert(!recovery.elements['nav-status'].textContent.includes('原地停稳保存候选'),'旧返程失败提示覆盖恢复阶段');
  }
  const recoveryStop=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'STOPPING',stop_target:'RECOVER',recovery_pending:true,
      recovery:{phase:'DISTURBED',reason:'扫描与轮式位置不一致'}}});
  assert.equal(recoveryStop.elements['workflow-title'].textContent,'受扰后正在停稳');
  assert(recoveryStop.elements['workflow-next'].textContent.includes('验证尚未开始'),'停稳前错误声称已恢复');
  const checkpointing=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,
      recovery:{phase:'CHECKPOINTING',reason:'正在写入检查点',imu_ready:true,imu_stable:true,imu_age_seconds:.01}}});
  assert.equal(checkpointing.elements['workflow-title'].textContent,'自动探索中','后台检查点保存错误改变任务阶段');
  assert(checkpointing.elements['recovery-status'].textContent.includes('任务继续运行'));
  const snapshotFailure=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,online_snapshots:true,checkpoint_usable:true,
      snapshot_maintenance:{phase:'FAILED',reason:'工作进程写入失败'},
      recovery:{phase:'CHECKPOINT_READY',checkpoint_id:'old-good',imu_ready:true,imu_stable:false,requires_stop:false,imu_age_seconds:.01}}});
  assert.equal(snapshotFailure.elements['workflow-title'].textContent,'自动探索中');
  assert(snapshotFailure.elements['snapshot-status'].textContent.includes('导航继续运行'));
  assert(snapshotFailure.elements['snapshot-status'].textContent.includes('未替换旧恢复点'));
  const snapshotUnhealthy=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:false,online_snapshots:true,checkpoint_usable:true,
      snapshot_maintenance:{phase:'FAILED',reason:'车端健康状态改变'},recovery:{phase:'CHECKPOINT_READY'}}});
  assert(!snapshotUnhealthy.elements['snapshot-status'].textContent.includes('导航继续运行'));
  const snapshotExpired=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'PAUSED',healthy:true,online_snapshots:true,checkpoint_usable:false,
      snapshot_maintenance:{phase:'FAILED',reason:'保存持续失败，旧恢复点已过期'},recovery:{phase:'CHECKPOINT_READY'}}});
  assert(snapshotExpired.elements['snapshot-status'].textContent.includes('保持停车'));
  assert(!snapshotExpired.elements['snapshot-status'].textContent.includes('导航继续运行'));
  const drivingObservation=await render({...completed,last_saved_bundle:'',dirty:true,
    autonomy:{...completed.autonomy,phase:'EXPLORING',healthy:true,
      recovery:{phase:'CHECKPOINT_READY',imu_ready:true,imu_valid:true,imu_stable:false,requires_stop:false,imu_age_seconds:.01}}});
  assert(drivingObservation.elements['recovery-status'].textContent.includes('行驶中，姿态观测更新'));
  assert(!drivingObservation.elements['recovery-status'].textContent.includes('受扰'),'正常行驶不满足停稳被误报为受扰');
  for(const phase of ['STOPPING','CHECKPOINTING']){
    const checkpointHold=await render({...completed,last_saved_bundle:'',dirty:true,
      autonomy:{...completed.autonomy,phase,stop_target:'CHECKPOINT',healthy:true,
        recovery:{phase:'CHECKPOINTING',reason:'保存稳定工程',imu_ready:true,imu_stable:true,imu_age_seconds:.02}}});
    assert.equal(checkpointHold.elements['workflow-title'].textContent,'停稳保存检查点');
    assert(checkpointHold.elements['workflow-next'].textContent.includes('原任务'),'检查点显示没有说明完成后继续');
    assert.equal(checkpointHold.elements['auto-start'].hidden,true);assert.equal(checkpointHold.elements['auto-manual'].hidden,true);
    assert.equal(checkpointHold.elements['auto-speed-apply'].disabled,true);assert.equal(checkpointHold.elements.stop.disabled,false);
    assert.equal(checkpointHold.elements['save-actions'].hidden,true);
    assert(!checkpointHold.elements['recovery-status'].textContent.includes('故障'),'定期检查点被显示成故障恢复');
  }
  // 两端共用任务操作资格，摇杆持有者独立；替换DOM/网络，不连接实车。
  const shared={...exploring,autonomy:{...exploring.autonomy,phase:'PAUSED',still:true,navigation_terminal:true},
    manual_control:{protocol:1,shared_tasks:true,owner_id:'computer',owner_label:'电脑',connected_clients:2,generation:1,phase:'READY',ready:true,reason:''},
    task_operations:{protocol:1,epoch:7,task_id:exploring.autonomy.task_id,requests:[]}};
  const phone=await render(shared,{clientId:'phone'});
  assert.equal(phone.elements['auto-start'].disabled,false,'手机没有摇杆所有权却被禁止继续任务');
  vm.runInContext("state.autonomy.phase='MANUAL';state.autonomy.control_mode='manual';receiveControl({armed:true,health:'',challenge:'other-owner',idle_heartbeat_required:false});",phone.context);
  const touch={pointerId:1,clientX:120,clientY:35,preventDefault(){}};
  phone.elements.joystick.onpointerdown(touch);
  assert(!phone.socketMessages.some(m=>['drive','intent','enable','heartbeat'].includes(m.action)),'非持有端使用了另一端armed状态发送摇杆命令');
  vm.runInContext("pressed='forward';stop('window_blur');stop('guard_restarted');",phone.context);
  assert(!phone.socketMessages.some(m=>m.action==='stop'),'非持有端失焦或门控代次变化停了另一端');
  phone.elements.stop.onclick();assert(phone.socketMessages.some(m=>m.action==='stop'&&m.cause==='manual'),'非持有端不能主动停车');
  phone.sockets[0].close();phone.elements.stop.onclick();assert.match(phone.elements.result.textContent,/停车请求未提交/,'断开时把未发送停车报成已经提交');
  const sharedPaused=()=>({...shared,autonomy:{...shared.autonomy,phase:'PAUSED',control_mode:'automatic'},
    manual_control:{...shared.manual_control},task_operations:{...shared.task_operations,requests:[]}});
  const merged=await render(sharedPaused(),{clientId:'phone',autonomyReply:request=>({accepted:true,request_id:request.request_id,
    canonical_request_id:'canonical-continue',queued:true,coalesced:true})});
  await merged.elements['auto-start'].onclick();
  const mergedRequest=JSON.parse(merged.requests.find(r=>r.url==='/api/autonomy').options.body);
  assert.equal(mergedRequest.operation_epoch,7);assert.equal(mergedRequest.task_id,shared.autonomy.task_id);
  assert.match(merged.elements.result.textContent,/合并.*等待车端/);
  assert.equal(merged.elements['auto-pause'].disabled,false,'继续排队期间暂停被本页pending禁用');
  assert.equal(merged.elements['auto-pause'].hidden,false,'继续排队期间没有暂停入口');
  await merged.next({...sharedPaused(),autonomy:{...sharedPaused().autonomy,request_id:'another-browser',request_result:'rejected',error:'另一端的拒绝'},
    task_operations:{...shared.task_operations,requests:[{request_id:mergedRequest.request_id,canonical_request_id:'canonical-continue',
      action:'resume',request_result:'accepted',reason:'',epoch:7}]}});
  assert.equal(vm.runInContext('autonomyPending',merged.context),null,'本页账本结果没有解除等待');
  assert(!merged.elements.result.textContent.includes('另一端的拒绝'),'另一端最近的结果覆盖本页已确认结果');
  assert(!merged.elements.result.textContent.includes('自动探索已开始'),'账本受理被当成已经行驶');
  const interrupted=await render(sharedPaused(),{clientId:'phone'});
  await interrupted.elements['auto-start'].onclick();await interrupted.elements['auto-pause'].onclick();
  const interruptedRequests=interrupted.requests.filter(r=>r.url==='/api/autonomy').map(r=>JSON.parse(r.options.body));
  assert.deepEqual(interruptedRequests.map(r=>r.action),['resume','pause'],'暂停没有抢占本页在途继续');
  const pauseId=interruptedRequests[1].request_id;
  await interrupted.next({...sharedPaused(),task_operations:{...shared.task_operations,epoch:8,requests:[
    {request_id:interruptedRequests[0].request_id,request_result:'cancelled',reason:'暂停已取消排队继续'},
    {request_id:pauseId,request_result:'accepted',reason:'',epoch:8}]}});
  assert.equal(vm.runInContext('autonomyPending',interrupted.context),null);
  assert.match(interrupted.elements.result.textContent,/暂停/);
  const manualState={...sharedPaused(),autonomy:{...sharedPaused().autonomy,phase:'MANUAL',control_mode:'manual'}};
  const takeover=await render(manualState,{clientId:'phone',operatorReply:(request,current)=>({client_id:'phone',is_owner:request.action==='claim',
    manual_control:request.action==='claim'?{...current.manual_control,phase:'STOPPING',ready:false,generation:2,reason:'等待旧端停车'}:current.manual_control})});
  assert.equal(takeover.elements['manual-claim'].disabled,false);
  assert.equal(takeover.requests.filter(r=>r.url==='/api/operator'&&JSON.parse(r.options.body).action==='claim').length,0,'仅打开页面就抢占摇杆');
  await takeover.elements['manual-claim'].onclick();
  assert.match(takeover.elements['manual-owner'].textContent,/等待旧端停车/);assert(takeover.elements.enable.disabled);
  await takeover.next({...manualState,manual_control:{...manualState.manual_control,phase:'STOPPING',ready:false,generation:2,reason:'等待旧端停车'}});
  assert(takeover.elements['auto-start'].disabled);assert.match(takeover.elements['auto-start'].title,/交接/);
  assert(!takeover.socketMessages.some(m=>['enable','drive','intent'].includes(m.action)),'接管受理后自行使能或运动');
  const readyManual={...manualState,manual_control:{...manualState.manual_control,generation:2,owner_id:'phone',owner_label:'手机',phase:'READY',ready:true}};
  await takeover.next(readyManual);
  vm.runInContext("receiveControl({armed:false,health:'',challenge:'phone-ready',idle_heartbeat_required:false});",takeover.context);
  assert.equal(takeover.elements.enable.disabled,false);assert.match(takeover.elements['manual-owner'].textContent,/本页持有/);
  takeover.elements.enable.onclick();assert(takeover.socketMessages.some(m=>m.action==='enable'));
  vm.runInContext("receiveControl({armed:true,health:'',challenge:'phone-armed',idle_heartbeat_required:false});",takeover.context);
  takeover.elements.joystick.onpointerdown(touch);assert(takeover.socketMessages.some(m=>m.action==='drive'));
  const beforeTransfer=takeover.socketMessages.length;
  vm.runInContext("applyManualControl({...state.manual_control,generation:3,owner_id:'computer',owner_label:'电脑'});",takeover.context);
  assert.equal(vm.runInContext('pressed',takeover.context),'');
  assert(!takeover.socketMessages.slice(beforeTransfer).some(m=>m.action==='stop'),'失去所有权后补发停车影响新端');
  takeover.elements.joystick.onpointerdown(touch);assert.equal(takeover.socketMessages.length,beforeTransfer,'失去所有权后仍能用旧armed推杆');
  const autoHandoff=await render({...sharedPaused(),autonomy:{...sharedPaused().autonomy,phase:'EXPLORING'}},{clientId:'phone'});
  await autoHandoff.elements['auto-manual'].onclick();
  assert.equal(autoHandoff.requests.filter(r=>r.url==='/api/operator'&&JSON.parse(r.options.body).action==='claim').length,0,'自动任务尚未停稳就接管摇杆');
  const manualRequest=JSON.parse(autoHandoff.requests.find(r=>r.url==='/api/autonomy').options.body);
  await autoHandoff.next({...manualState,task_operations:{...shared.task_operations,requests:[{request_id:manualRequest.request_id,
    action:'manual',request_result:'accepted',epoch:7}]}});
  assert.equal(autoHandoff.requests.filter(r=>r.url==='/api/operator'&&JSON.parse(r.options.body).action==='claim').length,1,'手动接管停稳确认后没有请求本页摇杆所有权');
  const reconnectOwner=await render(manualState,{clientId:'computer'});
  vm.runInContext("pressed='forward';latestControl={armed:true};",reconnectOwner.context);
  reconnectOwner.elements.reconnect.onclick();
  assert.equal(vm.runInContext('pressed',reconnectOwner.context),'');assert(!vm.runInContext('latestControl.armed',reconnectOwner.context));
  reconnectOwner.sockets.at(-1).onmessage({data:JSON.stringify({session:'new-manual-session'})});
  await reconnectOwner.next({...manualState,manual_control:{...manualState.manual_control,owner_id:'',owner_label:'',generation:2,ready:false,connected_clients:1}});
  const reclaimRequests=reconnectOwner.requests.filter(r=>r.url==='/api/operator'&&JSON.parse(r.options.body).action==='claim');
  assert.equal(reclaimRequests.length,1,'原持有者明确重连后没有通过停车流程重新claim');
  assert.equal(JSON.parse(reclaimRequests[0].options.body).session,'new-manual-session');
  assert(!reconnectOwner.socketMessages.some(m=>['enable','drive','intent'].includes(m.action)),'重新claim恢复了旧使能或旧运动');
  const reconnectObserver=await render(manualState,{clientId:'phone'});reconnectObserver.elements.reconnect.onclick();
  reconnectObserver.sockets.at(-1).onmessage({data:JSON.stringify({session:'new-observer-session'})});
  await reconnectObserver.next(manualState);
  assert.equal(reconnectObserver.requests.filter(r=>r.url==='/api/operator'&&JSON.parse(r.options.body).action==='claim').length,0,'旁观端仅重连就抢占了遥控');
  console.log('多端任务共享／非持有端运动隔离／接管先停稳／本页账本确认／重复继续合并／暂停抢占通过');
  console.log('返程主反馈／save_done关联切换／原起点与附近结果／停稳位置核对／超时断线及旧后端兼容通过');
  console.log('自动保存终态的清晰引导／结束任务不重复保存／探索中保持返程语义通过');
})().catch(error=>{console.error(error.stack);process.exitCode=1;});
