'use strict';
// 只回放网页协议和地图手势，不连接任何车辆、正式网关或ROS节点。
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const source=fs.readFileSync(process.argv[2]||path.join(__dirname,'../web/app.js'),'utf8');
const confirmed={global_inflation_radius:.5,global_cost_scaling_factor:3,local_inflation_radius:.3,local_cost_scaling_factor:5};
function fixture(){return {phase:'CURRENT',task_state:'RUNNING',managed:true,busy:false,dirty:true,dataflow_protocol:1,
  autonomy_enabled:true,autonomy_age_ms:10,control:{armed:false},task_operations:{protocol:1,task_id:'test-task',epoch:3,requests:[]},
  autonomy:{task_id:'test-task',phase:'EXPLORING',control_mode:'automatic',healthy:true,still:false,navigation_terminal:false,output:[.1,0],
    speed:{cap_mps:.1,confirmed_mps:.1,phase:'READY'},tuning:{task_id:'test-task',ready:true,age_seconds:.1,phase:'READY',pending:false,request_id:'',confirmed:{...confirmed},requested:{...confirmed}},
    handoff:{distance_m:.25,request_id:'',phase:'READY'},walls:{items:[],wall_revision:4,navigation_generation:9,phase:'READY',needs_confirmation:false,bypass_enabled:false,request_id:''},
    obstacle_escape:{field_validated:false,allowed:false,enabled:false},actions:{resume:{enabled:true},pause:{enabled:true},manual:{enabled:true},return:{enabled:true},recheck:{enabled:true}}}};}
async function browser(snapshot=fixture(),response=()=>({ok:true,status:202,value:{accepted:true}})){
  let state=snapshot,now=100,timer=0;const timers=new Map(),elements={},requests=[],listeners={};
  const context2d=new Proxy({}, {get:(target,key)=>target[key]||(()=>{})});
  function element(id){return elements[id]||=( {id,style:{},dataset:{},hidden:id==='wall-controls',disabled:false,checked:true,value:'manual',textContent:'',
    children:[],classList:{toggle(){},add(){},remove(){}},setAttribute(){},getContext:()=>context2d,clientWidth:900,clientHeight:600,
    setPointerCapture(){},focus(){},scrollIntoView(){},append(child){this.children.push(child);if(this.value==='manual'||!this.value)this.value=child.value;},
    getBoundingClientRect:()=>({left:0,top:0,width:900,height:600})});}
  class Socket{static OPEN=1;static CLOSED=3;constructor(){this.readyState=1;this.bufferedAmount=0;}send(){}close(){this.readyState=3;}}
  const context={document:{hidden:false,getElementById:element,querySelectorAll:()=>[],createElement:()=>({value:'',textContent:''}),addEventListener:(type,fn)=>{listeners[type]=fn;}},
    window:{devicePixelRatio:1,addEventListener(){},confirm(){throw Error('本轮常规操作不允许弹确认窗');}},location:{host:'fixture.invalid'},
    WebSocket:Socket,performance:{now:()=>now},setInterval(){},setTimeout:(callback,ms)=>{const id=++timer;timers.set(id,{callback,ms});return id;},clearTimeout:id=>timers.delete(id),
    AbortController,crypto:require('node:crypto').webcrypto,requestAnimationFrame(){},console,
    fetch:async(url,options={})=>{requests.push({url,options});if(options.method==='POST'){const reply=await response(JSON.parse(options.body));return {ok:reply.ok,status:reply.status,json:async()=>reply.value};}return {ok:true,status:200,json:async()=>url==='/api/status'?state:{}};}};
  vm.createContext(context);vm.runInContext(source,context);vm.runInContext("session='fixture-only';lastControlAt=100;latestControl={armed:false,health:''};",context);
  await new Promise(resolve=>setImmediate(resolve));
  async function next(updated=state,elapsed=250){state=updated;now+=elapsed;const pending=[...timers].find(([,t])=>t.ms===250);assert(pending,'轮询未继续');timers.delete(pending[0]);pending[1].callback();await new Promise(resolve=>setImmediate(resolve));}
  const posts=()=>requests.filter(r=>r.url==='/api/autonomy').map(r=>JSON.parse(r.options.body));
  return {context,e:elements,next,posts,listeners,run:code=>vm.runInContext(code,context)};
}
(async()=>{
  // 新策略明确允许本地手动，定位/人工墙仅限制自动任务，不能沿用旧中文原因锁住遥控。
  const local=fixture();local.phase='FAILED';local.autonomy.phase='MANUAL';local.autonomy.control_mode='manual';local.autonomy.healthy=false;
  local.autonomy.still=true;local.autonomy.navigation_terminal=true;local.autonomy.output=[0,0];
  local.autonomy.restricted_manual={active:true,ready:false,map_frozen:true,reason:'人工墙保护需要可信定位；先重新检查，或明确启用受限手动旁路'};
  local.autonomy.walls={...local.autonomy.walls,items:[{id:'墙',points:[[0,0],[1,0]]}],needs_confirmation:true,phase:'REVIEW_REQUIRED'};
  local.autonomy.manual_policy={version:1,mode:'local_safety',ignores_virtual_walls:true,requires_global_localization:false,restricted:true,
    motion_allowed:true,blocked_reason:'',direction_blocked:false,direction_reason:'',map_frozen:true,linear_limit_mps:.05,angular_limit_rps:.10};
  const localUI=await browser(local);localUI.run("receiveControl({armed:false,health:'',reason:'请使能',control_revision:10})");
  assert.match(localUI.e['manual-motion-title'].textContent,/手动已接管，可启用遥控/,'显式新能力应解除旧人工墙/定位UI锁定');
  assert.equal(localUI.e.enable.disabled,false);assert.match(localUI.e['manual-motion-detail'].textContent,/地图保持冻结.*自动.*不可用.*0\.05 m\/s.*0\.10 rad\/s.*激光.*扫掠.*松手停车/);
  assert.equal(localUI.posts().length,0,'读取新能力不得自动下发使能或运动');
  localUI.run("receiveControl({armed:true,health:'',reason:'已使能',control_revision:11})");
  localUI.e.joystick.onpointerdown({pointerId:1,clientX:450,clientY:100,preventDefault(){}});
  assert.notEqual(localUI.run('pressed'),'','新能力、真实许可和新手势齐备后可提交方向命令');
  const htmlCurrent=fs.readFileSync(process.argv[3]||path.join(__dirname,'../web/index.html'),'utf8');
  assert(!/id="(?:manual-wall-bypass|wall-bypass)"/.test(htmlCurrent),'手动和墙面板均不再提供旁路开关');
  const direction=structuredClone(local);direction.autonomy.manual_policy.direction_blocked=true;direction.autonomy.manual_policy.direction_reason='当前方向真实障碍进入车身扫掠范围';
  const directionUI=await browser(direction);directionUI.run("receiveControl({armed:true,health:'',reason:'按住行驶',control_revision:20})");
  assert.match(directionUI.e['manual-motion-title'].textContent,/当前方向.*拦截/);assert.match(directionUI.e['manual-motion-detail'].textContent,/真实障碍.*安全方向/);
  directionUI.e.joystick.onpointerdown({pointerId:2,clientX:450,clientY:500,preventDefault(){}});assert.notEqual(directionUI.run('pressed'),'','方向性障碍仍可松手换安全方向');
  // 计算超时、缺束与未知拒绝不能冒充真实障碍命中。
  for(const [reason,title,detail] of [
    ['方向扫掠检查超过8毫秒预算，保持停车','安全检查超时，保持停车',/计算未及时完成.*未下发速度.*松开摇杆/],
    ['所选方向的扫掠区仍有激光缺束或盲区，保持停车','当前方向观测不足',/未下发速度.*有效激光观测/],
    ['等待近场保护返回本次速度检查，底盘输出保持零','安全检查尚未完成，保持停车',/未下发速度/],
    ['方向观测数量超出审查预算','当前方向安全检查未通过',/未下发速度/],
    ['未知的车端保护原因','当前方向安全检查未通过',/检查车端原因/],
  ]){
    const sample=structuredClone(direction);sample.autonomy.manual_policy.direction_reason=reason;
    const h=await browser(sample);h.run("receiveControl({armed:true,health:'',reason:'按住行驶',control_revision:21})");
    assert.equal(h.e['manual-motion-title'].textContent,title,reason);
    assert.match(h.e['manual-motion-detail'].textContent,detail);assert(!h.e['manual-motion-title'].textContent.includes('真实防撞'));
    assert.equal(h.posts().length,0,'显示拒绝分类不能自动重发控制或恢复请求');
  }
  const wallOnly=fixture();wallOnly.autonomy.phase='PAUSED';wallOnly.autonomy.still=true;wallOnly.autonomy.navigation_terminal=true;
  wallOnly.autonomy.walls={...wallOnly.autonomy.walls,items:[{id:'待复核墙',points:[[0,0],[1,0]]}],needs_confirmation:true,review_allowed:true,requires_recheck:false};
  wallOnly.autonomy.actions.recheck={enabled:false,reason:'当前仅需复核人工墙位置；请打开临时墙核对并复核，无需重新定位'};
  wallOnly.autonomy.actions.confirm_walls={enabled:true};
  const wallOnlyUI=await browser(wallOnly);assert.equal(wallOnlyUI.e['auto-recheck'].textContent,'复核临时墙');
  assert.equal(wallOnlyUI.e['auto-recheck'].disabled,false);assert.match(wallOnlyUI.e['autonomy-hint'].textContent,/无需重新定位/);
  await wallOnlyUI.e['auto-recheck'].onclick();assert.equal(wallOnlyUI.e['wall-controls'].hidden,false);
  assert.equal(wallOnlyUI.posts().length,0,'打开墙复核面板不得提交重定位或自动确认墙');
  // 墙可复核不能证明定位健康；真实定位故障仍须走原重新检查事务。
  for(const change of [{healthy:false},{recovery_pending:true},{localization_requires_verification:true}]){
    const faultWithWall=structuredClone(wallOnly);Object.assign(faultWithWall.autonomy,change);
    faultWithWall.autonomy.actions.recheck={enabled:true};
    const h=await browser(faultWithWall);assert.equal(h.run('wallReviewOnly()'),false);
    assert.equal(h.e['auto-recheck'].textContent,'重新检查');await h.e['auto-recheck'].onclick();
    assert.equal(h.e['wall-controls'].hidden,true,'真实定位故障不得被截走到墙工具');
    assert.equal(h.posts().length,1);assert.equal(h.posts()[0].action,'recheck');
  }
  for(const reason of ['原始扫描超过0.5秒未更新','轮式里程计超过0.5秒未更新','本地控制通信超时','参数安全核验未通过']){
    const blocked=structuredClone(local);blocked.autonomy.manual_policy.motion_allowed=false;blocked.autonomy.manual_policy.blocked_reason=reason;
    const h=await browser(blocked);h.run("receiveControl({armed:true,health:'',reason:'按住行驶',control_revision:30})");
    assert.equal(h.e.enable.disabled,true);assert.match(h.e.motion.textContent,new RegExp(reason.replace('.', '\\.')));
    h.e.joystick.onpointerdown({pointerId:3,clientX:450,clientY:100,preventDefault(){}});assert.equal(h.run('pressed'),'');
  }
  const staleLocal=await browser({...local,autonomy_age_ms:501});assert.equal(staleLocal.e.enable.disabled,true);
  const unfrozen=structuredClone(local);unfrozen.autonomy.manual_policy.map_frozen=false;const unfrozenUI=await browser(unfrozen);
  assert.equal(unfrozenUI.e.enable.disabled,true,'受限策略没有冻结入图时必须拒绝');assert.equal(unfrozenUI.run('!!localManualPolicy()'),true,'等待冻结是受支持的策略暂态');
  assert.match(unfrozenUI.e['manual-motion-detail'].textContent,/等待地图冻结回执/);assert(!unfrozenUI.e['manual-motion-detail'].textContent.includes('版本不受支持'));
  assert.equal(unfrozenUI.run('!!restrictedControlReady()'),false);
  unfrozenUI.run("receiveControl({armed:true,health:'',reason:'按住行驶',control_revision:33});window.__sent=[];ws.send=text=>window.__sent.push(JSON.parse(text));pressed={x:.8,yaw:0};drive();");
  assert.equal(unfrozenUI.run("window.__sent.filter(v=>v.action==='drive').length"),0,'等待冻结回执不能复用旧手势');
  for(const policy of [null,false,undefined,{version:2}]){
    const missing=structuredClone(local);missing.autonomy.manual_policy=policy;missing.autonomy.restricted_manual={active:true,ready:true,reason:'',map_frozen:true};
    const h=await browser(missing);assert.equal(h.run('!!restrictedControlReady()'),false,'新策略字段存在但无效不能回退旧ready');assert.equal(h.e.enable.disabled,true);
    assert.match(h.e['manual-motion-detail'].textContent,/策略不完整或版本不受支持/);
  }
  const autoPending=structuredClone(local);autoPending.autonomy.auto_start_pending=true;const waitingAuto=await browser(autoPending);assert.equal(waitingAuto.e.enable.disabled,true,'自动启动尚未撤销时不得放行手动');
  const legacy=structuredClone(local);delete legacy.autonomy.manual_policy;const legacyUI=await browser(legacy);
  assert(!legacyUI.e['manual-motion-title'].textContent.includes('可启用遥控'));assert.equal(legacyUI.e.enable.disabled,true);
  assert.match(legacyUI.e['manual-motion-detail'].textContent,/尚未提供.*独立手动|原有.*保护/,'旧后端必须说明真实兼容边界');
  // 回放已批准的新范围，覆盖真实输入事件，防止HTML上限与提交校验混用新旧版本。
  for(const distance of [1.2,1.5])for(const id of ['handoff-distance','handoff-range']){
    const h=await browser();h.e[id].value=String(distance);h.e[id].oninput();h.e[id].onchange();await new Promise(setImmediate);
    assert.equal(h.posts().length,1,`${id}应提交已批准的${distance}米：${h.e['handoff-status'].textContent}`);
    assert.equal(h.posts()[0].action,'set_handoff');assert.equal(h.posts()[0].distance_m,distance);
    assert.match(h.e['handoff-status'].textContent,/尚未确认/,'HTTP受理不能当作车端确认');
  }
  for(const value of ['1.501','.149',''])for(const id of ['handoff-distance','handoff-range']){
    const h=await browser();h.e[id].value=value;h.e[id].onchange();await new Promise(setImmediate);
    assert.equal(h.posts().length,0,`${id}不应提交越界或空值${value}`);
    assert.match(h.e['handoff-status'].textContent,/网页未提交.*0\.15～1\.50/);
  }
  const html=fs.readFileSync(process.argv[3]||path.join(__dirname,'../web/index.html'),'utf8');
  for(const id of ['handoff-distance','handoff-range']){
    const input=html.match(new RegExp(`<input[^>]+id="${id}"[^>]*>`));assert(input,`缺少${id}`);
    assert.equal(Number(input[0].match(/\bmax="([^"]+)"/)[1]),1.5,`${id}显示上限应为1.5米`);
  }
  // 现场越界回放：有效地图以外不能形成可提交草稿，首个端点必须立即可见。
  const outside=await browser();outside.run("displayMap={width:240,height:225,resolution:.05,origin:[-7.55,-6.20,0]};renderLiveControls();");
  outside.e['wall-tool'].onclick();outside.e['wall-draw'].onclick();
  outside.run('addWallPoint([-5,0]);addWallPoint([6,0]);');
  assert.equal(outside.run('wallDraft.length'),1,'越界端点不能进入草稿');
  assert.match(outside.e['wall-status'].textContent,/网页未提交.*原图范围/);
  assert.equal(outside.e['wall-apply'].disabled,true);
  outside.run('addWallPoint([-4,0]);');assert.equal(outside.run('wallDraft.length'),2,'边界内未知栅格不能按占用颜色拒绝');
  outside.run('wallDraft.pop();renderLiveControls();');
  let painted=0;outside.context.drawPoint=()=>painted++;outside.run('ctx.arc=drawPoint;paintWalls()');
  assert.equal(painted,2,'首个端点及高对比标记必须立即绘制');
  assert.equal(outside.e['wall-map-feedback'].hidden,false);
  assert.match(outside.e['wall-map-feedback'].textContent,/已添加第 1 个点.*再点/);
  outside.run('addWallPoint([-3,0]);');assert.match(outside.e['wall-map-feedback'].textContent,/尚未应用/);
  outside.run('cancelWallDraft()');assert.equal(outside.e['wall-map-feedback'].hidden,true);
  // 现场失败回放：受限手动局部条件满足，但参数保护仍拒绝运动。
  const failedManual=fixture();failedManual.autonomy.phase='MANUAL';failedManual.autonomy.control_mode='manual';
  failedManual.autonomy.live_controls_enabled=true;failedManual.autonomy.healthy=false;
  failedManual.autonomy.restricted_manual={active:true,ready:true,map_frozen:true,reason:''};
  failedManual.autonomy.tuning={...failedManual.autonomy.tuning,phase:'FAILED',ready:false,requires_stop:true,error:'回退读取参数超时'};
  const blockedManual=await browser(failedManual);
  assert.equal(blockedManual.e.enable.disabled,true,'参数保护拒绝运动时不得开放使能');
  assert.match(blockedManual.e.motion.textContent,/参数安全核验/);
  blockedManual.run("receiveControl({armed:false,reason:'输入已就绪，请确认现场后使能',control_revision:99})");
  assert.match(blockedManual.e.motion.textContent,/参数安全核验/,'门控心跳不得覆盖参数阻断原因');
  assert.match(blockedManual.e['workflow-title'].textContent,/参数/,'页面主状态不得宣称已就绪');
  failedManual.autonomy.still=true;failedManual.autonomy.navigation_terminal=true;
  failedManual.autonomy.tuning.restore_available=true;
  await blockedManual.next(failedManual);
  assert.equal(blockedManual.e['tuning-restore'].disabled,false,'停稳后应提供独立参数恢复入口');
  await blockedManual.e['tuning-restore'].onclick();await new Promise(setImmediate);
  assert.equal(blockedManual.posts()[0].action,'restore_tuning');
  const failedRecover=structuredClone(failedManual);failedRecover.autonomy.phase='RECOVERING';failedRecover.autonomy.recovery_pending=true;
  failedRecover.autonomy.recovery={phase:'VERIFY_READY',reason:'等待监督器提交',progress:{remaining_seconds:54}};
  const recoveryBlocked=await browser(failedRecover);
  assert.match(recoveryBlocked.run('recoverySentence(state.autonomy)'),/参数安全核验阻止/);
  assert.equal(recoveryBlocked.e['tuning-restore'].disabled,true,'恢复未中止时不得并发更改参数');
  const b=await browser();assert.equal(b.e['tuning-global_inflation_radius'].disabled,false);assert.equal(b.e['escape-enabled'].disabled,true);assert.match(b.e['escape-status'].textContent,/现场验证/);
  b.e['tuning-global_inflation_radius'].value='.6';b.e['tuning-global_inflation_radius'].oninput();await b.e['tuning-global_inflation_radius'].onchange();await new Promise(resolve=>setImmediate(resolve));
  const command=b.posts()[0];assert.equal(command.action,'set_tuning');assert.equal(command.values.global_inflation_radius,.6);assert.equal(command.task_id,'test-task');assert.equal(command.operation_epoch,3);
  assert.match(b.e['tuning-status'].textContent,/尚未确认/);assert.match(b.e['readback-global_inflation_radius'].textContent,/0.50/);assert.equal(b.e['tuning-save'].disabled,true);
  const applying=fixture();applying.autonomy.tuning={...applying.autonomy.tuning,phase:'APPLYING',pending:true,request_id:command.request_id,requested:{...confirmed,global_inflation_radius:.6}};
  await b.next(applying);assert.match(b.e['readback-global_inflation_radius'].textContent,/请求 0.60.*读回 0.50/);
  const done=fixture();done.autonomy.tuning={...done.autonomy.tuning,request_id:command.request_id,confirmed:{...confirmed,global_inflation_radius:.6}};
  await b.next(done);assert.match(b.e['tuning-status'].textContent,/已应用并确认/);assert.equal(b.posts().length,1);
  const incomplete=fixture();incomplete.autonomy.tuning={};incomplete.autonomy.walls={};const waiting=await browser(incomplete);assert(waiting.e['tuning-global_inflation_radius'].disabled);assert(waiting.e['wall-tool'].disabled);
  const stale=await browser({...fixture(),autonomy_age_ms:501});assert(stale.e['tuning-global_inflation_radius'].disabled);await stale.run("liveRequest('set_tuning',{values:{global_inflation_radius:.4}})");assert.equal(stale.posts().length,0);
  const invalid=await browser();invalid.e['tuning-global_inflation_radius'].value='0.01';invalid.e['tuning-global_inflation_radius'].onchange();assert.equal(invalid.posts().length,0);assert.match(invalid.e['tuning-status'].textContent,/网页未提交/);
  const reject=await browser(fixture(),()=>({ok:false,status:409,value:{error:'代价图事务正在处理'}}));await reject.run("liveRequest('restore_tuning')");assert.match(reject.e['tuning-status'].textContent,/上位机未受理.*代价图事务/);
  const ledger=await browser();await ledger.run("liveRequest('restore_tuning')");const request=ledger.posts()[0];const refused=fixture();refused.task_operations.requests=[{request_id:'another-client',request_result:'rejected',reason:'别的请求'}];await ledger.next(refused);assert.match(ledger.e['tuning-status'].textContent,/等待车端/);refused.task_operations.requests.push({request_id:request.request_id,request_result:'rejected',reason:'车端当前定位不健康'});await ledger.next(refused);assert.match(ledger.e['tuning-status'].textContent,/车端未执行.*定位不健康/);
  const mismatch=await browser();await mismatch.run("liveRequest('set_tuning',{values:{local_cost_scaling_factor:7}})");const bad=fixture();bad.autonomy.tuning.request_id=mismatch.posts()[0].request_id;await mismatch.next(bad);assert.match(mismatch.e['tuning-status'].textContent,/读回与请求不一致/);
  const rollback=await browser();await rollback.run("liveRequest('restore_tuning')");const rolled=fixture();rolled.autonomy.tuning={...rolled.autonomy.tuning,request_id:rollback.posts()[0].request_id,rolled_back:true,error:'局部代价图读回不一致'};await rollback.next(rolled);assert.match(rollback.e['tuning-status'].textContent,/局部代价图读回不一致/);assert(!rollback.e['tuning-status'].textContent.includes('已应用并确认'));
  const moduleReject=await browser();await moduleReject.run("liveRequest('set_tuning',{values:{global_inflation_radius:.6}})");const untouched=fixture();untouched.autonomy.tuning.request_id='startup';untouched.autonomy.tuning.rejected_requests=[{request_id:'unrelated',reason:'其他端请求'},{request_id:moduleReject.posts()[0].request_id,request_result:'rejected',reason:'代价参数服务拒绝本次事务'}];await moduleReject.next(untouched);assert.match(moduleReject.e['tuning-status'].textContent,/车端未执行.*代价参数服务/);assert.equal(moduleReject.run('!!livePending'),false,'保留旧READY的拒绝也须按同id立即完成错误反馈');
  const rejectedWall=await browser();await rejectedWall.run("liveRequest('set_walls',{walls:[],wall_revision:4,navigation_generation:9})");const wallFailed=fixture();wallFailed.autonomy.walls={...wallFailed.autonomy.walls,request_id:rejectedWall.posts()[0].request_id,request_result:'rejected',error:'墙点超出当前原图'};await rejectedWall.next(wallFailed);assert.match(rejectedWall.e['wall-status'].textContent,/墙点超出/);assert(!rejectedWall.e['wall-status'].textContent.includes('已由车端应用'));
  const save=await browser();await save.e['tuning-save'].onclick();assert.match(save.e['tuning-status'].textContent,/尚未确认/);const saved=fixture();saved.autonomy.tuning.defaults_saved_request_id=save.posts()[0].request_id;await save.next(saved);assert.match(save.e['tuning-status'].textContent,/已保存为新任务默认/);
  const oldReadback=fixture();oldReadback.autonomy.tuning.task_id='previous-task';const noSave=await browser(oldReadback);assert(noSave.e['tuning-save'].disabled);await noSave.run("liveRequest('save_tuning')");assert.equal(noSave.posts().length,0);oldReadback.autonomy.tuning.task_id='test-task';delete oldReadback.autonomy.tuning.age_seconds;await noSave.next(oldReadback);assert(noSave.e['tuning-save'].disabled);
  const handoff=await browser();handoff.e['handoff-distance'].value='.45';handoff.e['handoff-distance'].oninput();handoff.e['handoff-distance'].onchange();await new Promise(resolve=>setImmediate(resolve));assert.equal(handoff.posts()[0].distance_m,.45);const hd=fixture();hd.autonomy.handoff={distance_m:.45,phase:'READY',request_id:handoff.posts()[0].request_id};await handoff.next(hd);assert.match(handoff.e['handoff-status'].textContent,/0.45 m/);
  const escape=await browser();escape.e['escape-enabled'].checked=true;escape.e['escape-enabled'].onchange();assert.equal(escape.posts().length,0);assert.equal(escape.e['escape-enabled'].checked,false);
  const trialState=fixture();trialState.autonomy.phase='PAUSED';trialState.autonomy.still=true;trialState.autonomy.navigation_terminal=true;
  trialState.autonomy.obstacle_escape={enabled:false,field_validated:false,trial_supported:true,trial_ready:true};
  const trial=await browser(trialState);assert.equal(trial.e['escape-enabled'].disabled,false,'已停稳且后端允许时开放本轮受看护验证');
  trial.e['escape-enabled'].checked=true;trial.e['escape-enabled'].onchange();await new Promise(setImmediate);
  assert.equal(trial.posts()[0].supervised_trial,true);assert.match(trial.e['escape-status'].textContent,/受看护/);
  const wall=await browser();wall.run("displayMap={width:100,height:100,resolution:.05,origin:[-2,-2,0]};renderLiveControls();");wall.e['wall-tool'].onclick();wall.e['wall-draw'].onclick();
  const pointer=(id,x,y)=>({pointerId:id,clientX:x,clientY:y,preventDefault(){}});
  wall.e.map.onpointerdown(pointer(1,420,300));wall.e.map.onpointerup(pointer(1,420,300));wall.e.map.onpointerdown(pointer(1,480,300));wall.e.map.onpointerup(pointer(1,480,300));assert.match(wall.e['wall-draft'].textContent,/2 点.*2.00 m/);assert.equal(wall.e['wall-apply'].disabled,false);
  const before=wall.run('wallDraft.length');wall.e.map.onpointerdown(pointer(1,300,250));wall.e.map.onpointerdown(pointer(2,600,350));wall.e.map.onpointermove(pointer(2,650,400));wall.e.map.onpointerup(pointer(2,650,400));wall.e.map.onpointerup(pointer(1,300,250));assert.equal(wall.run('wallDraft.length'),before,'双指缩放不得产生幽灵端点');
  wall.e['wall-apply'].onclick();await new Promise(resolve=>setImmediate(resolve));const wc=wall.posts()[0];assert.equal(wc.action,'set_walls');assert.equal(wc.wall_revision,4);assert.equal(wc.navigation_generation,9);assert.deepEqual(wc.walls[0].points,[[-1,0],[1,0]]);assert.equal(wall.run('wallDraft.length'),2,'HTTP受理前不能丢草稿');
  const applied=fixture();applied.autonomy.walls={...applied.autonomy.walls,items:wc.walls,wall_revision:5,request_id:wc.request_id};await wall.next(applied);assert.equal(wall.run('wallDraft.length'),0);assert.match(wall.e['wall-status'].textContent,/车端应用/);
  const refreshed=await browser(applied);assert.equal(refreshed.run('wallItems().length'),1,'刷新应以车端墙恢复');assert.equal(refreshed.run('wallDraft.length'),0);
  refreshed.run('displayMap={};renderLiveControls()');refreshed.e['wall-tool'].onclick();refreshed.e['wall-draw'].onclick();refreshed.run('addWallPoint([0,0]);addWallPoint([1,1]);');refreshed.listeners.keydown({key:'Escape',preventDefault(){}});assert.equal(refreshed.run('wallDraft.length'),0);
  refreshed.e['wall-draw'].onclick();refreshed.run('addWallPoint([0,0]);addWallPoint([1,1]);');const rebased=structuredClone(applied);rebased.autonomy.walls.navigation_generation=10;rebased.autonomy.walls.needs_confirmation=true;await refreshed.next(rebased);assert.equal(refreshed.run('wallDraft.length'),0);assert.equal(refreshed.e['wall-protection'].hidden,false);assert.equal(refreshed.e['wall-review'].disabled,false);
  const nextTask=fixture();nextTask.autonomy.task_id='new-task';nextTask.task_operations.task_id='new-task';await refreshed.next(nextTask);assert.equal(refreshed.run('wallItems().length'),0);assert.equal(refreshed.e['wall-controls'].hidden,true);
  const recoveringWalls=fixture();recoveringWalls.autonomy.phase='RECOVERING';recoveringWalls.autonomy.recovery_pending=true;recoveringWalls.autonomy.walls={...recoveringWalls.autonomy.walls,items:[{id:'existing',points:[[0,0],[1,0]]}],needs_confirmation:true,review_allowed:true,review_wait_seconds:20,review_wait_limit_seconds:300,review_reason:'定位已完成原图多帧复验，等待墙复核'};const reviewing=await browser(recoveringWalls);assert.equal(reviewing.e['wall-review'].disabled,false,'恢复墙等待阶段应开放复核入口');assert.equal(reviewing.e['wall-tool'].disabled,false);assert.match(reviewing.e['workflow-title'].textContent,/等待临时墙位置复核/);assert.match(reviewing.e['wall-status'].textContent,/280 秒/);recoveringWalls.autonomy.walls.review_allowed=false;await reviewing.next(recoveringWalls);assert.equal(reviewing.e['wall-review'].disabled,true);
  const noBypass=await browser(local);await noBypass.run("liveRequest('set_wall_bypass',{enabled:true})");
  assert.equal(noBypass.posts().length,0,'已删除的旁路动作不能通过旧函数入口发送');
  const autoWalls=fixture();autoWalls.autonomy.walls.needs_confirmation=true;autoWalls.autonomy.manual_policy={...local.autonomy.manual_policy,motion_allowed:false};
  const automaticUI=await browser(autoWalls);assert.match(automaticUI.e['wall-protection'].textContent,/自动导航人工墙待复核/);
  assert.equal(automaticUI.e['manual-motion-feedback'].hidden,true,'自动阶段不展示旧手动许可');
  const normal=structuredClone(local);normal.phase='CURRENT';normal.autonomy.healthy=true;normal.autonomy.restricted_manual={active:false,ready:false,reason:''};
  normal.autonomy.manual_policy={...normal.autonomy.manual_policy,restricted:false,map_frozen:false,linear_limit_mps:null,angular_limit_rps:null};
  const normalUI=await browser(normal);normalUI.run("receiveControl({armed:false,health:'',reason:'请使能',control_revision:40})");
  assert.equal(normalUI.e.enable.disabled,false);assert.match(normalUI.e['manual-motion-detail'].textContent,/当前遥控档位/);assert(!normalUI.e['manual-motion-detail'].textContent.includes('0.05'));
  const invalidPolicy=structuredClone(local);invalidPolicy.autonomy.manual_policy.version=2;const invalidUI=await browser(invalidPolicy);assert.equal(invalidUI.e.enable.disabled,true);
  assert.match(invalidUI.e['manual-motion-detail'].textContent,/版本不受支持/);
  const noOwner=structuredClone(local);noOwner.manual_control={protocol:1,phase:'READY',ready:true,owner_id:'other-owner',generation:1};const observer=await browser(noOwner);
  assert.equal(observer.e.enable.disabled,true);assert.match(observer.e['manual-motion-title'].textContent,/取得遥控权/);
  const pausedOwner=structuredClone(local);pausedOwner.autonomy.phase='PAUSED';pausedOwner.manual_control={protocol:1,phase:'READY',ready:true,owner_id:'owner-client',generation:2};
  const pausedUI=await browser(pausedOwner);pausedUI.run("operatorClientId='owner-client';renderWorkflow();receiveControl({armed:true,health:'',reason:'按住行驶',control_revision:45});window.__commands=[];ws.send=text=>window.__commands.push(JSON.parse(text));pressed={x:.8,yaw:0};drive();");
  assert.equal(pausedUI.e.enable.disabled,true,'仅取得遥控权但仍PAUSED时不得启用');assert.equal(pausedUI.e['manual-motion-feedback'].hidden,true);
  assert.equal(pausedUI.run("window.__commands.filter(v=>v.action==='drive').length"),0,'遥控权不等于手动模式许可');
  const disconnected=await browser(local);disconnected.run('clearControlConnection()');assert.equal(disconnected.e.enable.disabled,true);assert.match(disconnected.e['manual-motion-title'].textContent,/未确认/);
  const beforeTakeover=structuredClone(local);beforeTakeover.autonomy.phase='FAULT';beforeTakeover.autonomy.control_mode='automatic';
  const takeover=await browser(beforeTakeover);takeover.run("pressed={x:.6,yaw:0};activePointer=9;latestControl.armed=true;");await takeover.run("autonomyOperation('manual')");
  assert.equal(takeover.posts().length,1);assert.equal(takeover.posts()[0].action,'manual');assert.equal(takeover.run('pressed'),'');assert.equal(takeover.run('activePointer'),null);
  assert.equal(takeover.run('latestControl.armed'),false);assert.equal(takeover.run('transportAwaitingStop'),true,'接管必须等待新的停车回应再使能');
  console.log('运行参数真实读回／请求账本／失败与过期／衔接距离／短退边界／临时墙触屏与代次／刷新恢复／明确本地手动策略与旧后端保守兼容通过');
})().catch(error=>{console.error(error);process.exitCode=1;});
