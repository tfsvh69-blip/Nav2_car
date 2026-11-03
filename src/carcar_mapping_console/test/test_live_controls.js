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
  const context2d=new Proxy({}, {get:()=>()=>{}});
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
  const wall=await browser();wall.run("displayMap={width:100,height:100,resolution:.05,origin:[0,0,0]};renderLiveControls();");wall.e['wall-tool'].onclick();wall.e['wall-draw'].onclick();
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
  const bypass=fixture();bypass.autonomy.phase='MANUAL';bypass.autonomy.control_mode='manual';bypass.autonomy.walls.bypass_enabled=true;const manual=await browser(bypass);assert.equal(manual.e['wall-protection'].hidden,false);assert.match(manual.e['wall-protection'].textContent,/人工墙保护不可用/);assert.equal(manual.e['wall-bypass'].textContent,'关闭人工墙旁路');
  const ownerScope=fixture();ownerScope.autonomy.phase='MANUAL';ownerScope.autonomy.still=true;ownerScope.autonomy.navigation_terminal=true;ownerScope.manual_control={protocol:1,generation:1,phase:'READY',owner_id:'owner-client',owner_label:'持有端',connected_clients:2};const observer=await browser(ownerScope);observer.run("operatorClientId='observer-client';renderLiveControls()");assert.equal(observer.e['wall-bypass'].disabled,true);assert.match(observer.e['wall-bypass'].title,/先取得遥控权/);ownerScope.autonomy.walls.bypass_enabled=true;await observer.next(ownerScope);assert.equal(observer.e['wall-bypass'].disabled,false,'旁观端仍应能关闭人工旁路');
  const bypassAck=await browser();await bypassAck.run("liveRequest('set_wall_bypass',{enabled:true})");const bypassId=bypassAck.posts()[0].request_id;const bound=fixture();bound.autonomy.phase='MANUAL';bound.autonomy.still=true;bound.autonomy.navigation_terminal=true;bound.autonomy.restricted_manual={active:true,ready:false,map_frozen:true};bound.autonomy.walls={...bound.autonomy.walls,bypass_enabled:true,bypass_request_id:bypassId,requires_recheck:true,request_id:'earlier-wall',request_result:'rejected',error:'上次墙请求超出原图'};bound.task_operations.requests=[{request_id:bypassId,request_result:'accepted'}];await bypassAck.next(bound);assert.equal(bypassAck.run('!!livePending'),true,'受限人工尚未就绪不能确认旁路完成');bound.autonomy.restricted_manual.ready=true;await bypassAck.next(bound);assert.equal(bypassAck.run('!!livePending'),false);bound.autonomy.walls.bypass_enabled=false;bound.autonomy.phase='PAUSED';await bypassAck.next(bound);assert.match(bypassAck.e['wall-protection'].textContent,/等待定位重新检查/);
  console.log('运行参数真实读回／请求账本／失败与过期／衔接距离／短退边界／临时墙触屏与代次／刷新恢复与人工旁路通过');
})().catch(error=>{console.error(error);process.exitCode=1;});
