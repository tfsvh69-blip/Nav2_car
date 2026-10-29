'use strict';
// 回放现场自动保存后的真实状态组合，只替换DOM和网络，不连接或控制实车。
const assert = require('assert'), fs = require('fs'), vm = require('vm');
const source = fs.readFileSync(process.argv[2], 'utf8');
async function render(snapshot, {selection='manual',siteConfirmed=true}={}) {
  const elements = {}, requests = [], context2d = new Proxy({}, {get: () => () => {}});
  const element = id => elements[id] ||= {id, style:{}, hidden:false, disabled:false,
    checked:true, value:'manual', textContent:'', classList:{toggle(){},add(){},remove(){}},
    getContext:()=>context2d, clientWidth:900,clientHeight:600, setPointerCapture(){},focus(){},scrollIntoView(){},append(child){child.parentElement=this;},
    getBoundingClientRect:()=>({left:0,top:0,width:240,height:240})};
  class Socket {
    static OPEN=1;static CLOSED=3;
    constructor(){this.readyState=1;this.bufferedAmount=0;}
    send(){} close(){this.readyState=3;}
  }
  const context = {document:{hidden:false,getElementById:element,querySelectorAll:()=>[],addEventListener(){}},
    window:{devicePixelRatio:1,addEventListener(){},confirm:()=>true}, location:{host:'offline.test'},
    WebSocket:Socket,performance:{now:()=>100},setInterval(){},setTimeout:()=>1,clearTimeout(){},
    AbortController,crypto:require('crypto').webcrypto,requestAnimationFrame(){},console,
    fetch:async (url,options={})=>{
      requests.push({url,options});
      return {ok:true,status:200,json:async()=>url==='/api/status'?snapshot:{accepted:true,operation:3}};
    }};
  vm.createContext(context);vm.runInContext(source,context);
  element('control-mode').value=selection;element('site-confirmed').checked=siteConfirmed;
  vm.runInContext("session='offline-observer';lastControlAt=100;latestControl={armed:false,health:''};",context);
  await new Promise(resolve=>setImmediate(resolve));
  return {elements,requests,context};
}
const completed = {phase:'CURRENT',task_state:'RUNNING',managed:true,busy:false,
  autonomy_enabled:true,autonomy_age_ms:10,dirty:false,last_saved_bundle:'saved-candidate',
  result:'地图与工程保存完整:saved-candidate',control:{armed:false},dataflow_protocol:1,
  autonomy:{task_id:'completed-task',phase:'REVIEW_REQUIRED',control_mode:'automatic',healthy:false,
    still:true,navigation_terminal:true,outcome:'partial_unreachable',return_result:'failed',
    reason:'部分区域不可达，保留缺口；尝试返回；起点不可达，原地停稳保存候选',
    health_reason:'Nav2局部／全局代价图使用的车位陈旧；检查内部TF与代价图线程'}};
(async()=>{
  const done=await render(completed),e=done.elements;
  assert(e.phase.textContent.includes('候选已保存'),'已保存结束的探索仍显示实时建图');
  assert.equal(e['auto-finish'].disabled,false,'自动保存后没有可用的结束入口');
  assert(e['auto-finish'].textContent.includes('结束任务'),'完成后按钮仍要求再次探索返程');
  assert(e['auto-finish'].textContent.includes('候选'),'结束入口没有说明下一步审核候选');
  assert.equal(e.end.textContent,'结束本次任务','已保存后结束仍被描述为放弃未保存进度');
  assert.equal(e.save.hidden,true,'已自动保存后仍突出重复保存入口');
  assert.equal(e['auto-start'].hidden,true,'已结束后仍突出不可用的继续自动操作');
  assert.equal(e['site-check'].hidden,true,'已结束后仍要求勾选启动场地条件');
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
  await paused.elements['auto-manual'].onclick();
  for(const id of ['auto-start','auto-pause','auto-manual','auto-finish'])
    assert.equal(paused.elements[id].disabled,true,'请求已提交但按钮仍短暂可点，后续点击会被pending静默忽略');
  const blockedReason='自动尚未开始：当前起点整车足迹未全部位于已知自由区域';
  const manual={...completed,last_saved_bundle:'',dirty:true,autonomy:{...completed.autonomy,
    phase:'MANUAL',control_mode:'manual',healthy:true,automatic_start_ready:false,automatic_start_reason:blockedReason}};
  const blocked=await render(manual);
  assert.equal(blocked.elements['auto-start'].disabled,true,'自动起点预检拒绝时按钮仍可用');
  assert(blocked.elements['autonomy-quality'].textContent.includes(blockedReason),'自动未开始的原因没有显示');
  assert(blocked.elements['auto-start'].title.includes(blockedReason),'禁用按钮没有解释起点预检原因');
  assert.equal(blocked.elements.enable.disabled,false,'自动起点预检错误禁止手动控制');
  assert(!blocked.requests.some(r=>r.options.method==='POST'),'预检拒绝时网页自动提交了控制操作');
  const legacyManual={...manual,autonomy:{...manual.autonomy}};
  delete legacyManual.autonomy.automatic_start_ready;delete legacyManual.autonomy.automatic_start_reason;
  assert.equal((await render(legacyManual)).elements['auto-start'].disabled,false,'新增起点字段破坏旧后端兼容');
  const standby={...completed,phase:'STANDBY',task_state:'STANDBY',autonomy_age_ms:60000};
  const idle=await render(standby,{selection:'automatic',siteConfirmed:false});
  assert(idle.elements['site-check'],'待命渲染中断：'+JSON.stringify(vm.runInContext('clientEvents',idle.context)));
  assert.equal(idle.elements['site-check'].hidden,false,'结束后旧REVIEW_REQUIRED隐藏了新一轮自动启动所必需的场地勾选');
  assert.equal(idle.elements['autonomy-panel'].hidden,true,'硬件尚未启动时仍突出一排不可用的自动运行操作');
  assert(idle.elements.phase.textContent.includes('待命'),'返回待命后仍把上一轮候选当成本轮终态');
  assert(!idle.elements['autonomy-stage'].textContent.includes('候选已保存'),'待命仍显示上一轮自动任务');
  idle.elements.mode.value='new';idle.elements.start.onclick();
  assert(!idle.requests.some(r=>r.url==='/api/start'),'没有勾选场地条件时错误提交了硬件启动');
  assert(idle.elements['task-info'].textContent.includes('勾选'),'启动被前端拒绝却没有在启动按钮附近解释');
  idle.elements['site-confirmed'].checked=true;idle.elements.start.onclick();
  await new Promise(resolve=>setImmediate(resolve));
  const starts=idle.requests.filter(r=>r.url==='/api/start');
  assert.equal(starts.length,1,'勾选后点击检查启动没有提交恰好一次请求');
  assert.deepEqual(JSON.parse(starts[0].options.body),{mode:'new',bundle:'',control_mode:'automatic',site_confirmed:true,session:'offline-observer'});
  const manualStandby=await render(standby,{selection:'manual',siteConfirmed:false});
  assert.equal(manualStandby.elements['site-check'].hidden,true,'手动待命被要求填写自动场地条件');
  const repeatedStart=await render(standby,{selection:'manual',siteConfirmed:false});
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
  const fault=await render({...completed,last_saved_bundle:'',dirty:true,autonomy:{...completed.autonomy,
    phase:'FAULT',still:true,navigation_terminal:true,quality_fault:'扫描过期，已锁停'}});
  assert.equal(fault.elements['auto-finish'].textContent,'结束故障任务');
  assert.equal(fault.elements['auto-finish'].disabled,false,'确认停稳的故障任务没有清晰退出入口');
  await fault.elements['auto-finish'].onclick();
  assert(fault.requests.some(r=>r.url==='/api/end'),'结束故障任务未使用正常关闭接口');
  assert(!fault.requests.some(r=>r.url==='/api/autonomy'),'结束故障任务却提交了返程或恢复');
  manualStandby.elements['control-mode'].value='automatic';manualStandby.elements['control-mode'].onchange();
  assert.equal(manualStandby.elements['site-check'].hidden,false,'切换自动模式没有立即显示启动条件');
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
  console.log('自动保存终态的清晰引导／结束任务不重复保存／探索中保持返程语义通过');
})().catch(error=>{console.error(error.stack);process.exitCode=1;});
