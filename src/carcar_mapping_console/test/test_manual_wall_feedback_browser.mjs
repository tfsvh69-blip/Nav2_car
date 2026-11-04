// 真实Chromium仅连接随机本机端口的策略夹具；不连接正式网关、ROS或车辆。
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import http from 'node:http';
import {spawn} from 'node:child_process';
import {fileURLToPath,pathToFileURL} from 'node:url';
const root=fileURLToPath(new URL('../../../',import.meta.url));
const webRoot=path.resolve(process.argv[3]||path.join(root,'src/carcar_mapping_console/web'));
const evidence=path.resolve(process.argv[2]||path.join(root,'evidence/ui/browser'));
await fs.mkdir(evidence,{recursive:true});
const runtime='/home/jetson/luhao/my_nav_carcar_inspection_20261103';
const {chromium}=await import(pathToFileURL(process.env.CARCAR_PLAYWRIGHT||path.join(runtime,'src/carcar_inspection/web/node_modules/playwright-core/index.mjs')).href);
const executable=process.env.CARCAR_CHROMIUM||path.join(runtime,'build/carcar_mapping_console/mapping_test_chromium');
const profile=await fs.mkdtemp(path.join(os.tmpdir(),'carcar-manual-local-'));
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const commands=[],operatorCommands=[],errors=[],external=[],checks=[];
const values={global_inflation_radius:.6,global_cost_scaling_factor:4,local_inflation_radius:.3,local_cost_scaling_factor:5};
function fixture(){return {phase:'FAILED',task_state:'RUNNING',managed:true,busy:false,dirty:true,autonomy_enabled:true,autonomy_age_ms:10,
 dataflow_protocol:1,control:{armed:false,health:''},navigation:{},scan_age:.01,odom_age:.01,map_age:.1,
 manual_control:{protocol:1,phase:'READY',ready:true,owner_id:'fixture-client',generation:1,connected_clients:1},
 task_operations:{protocol:1,task_id:'isolated-local-manual',epoch:3,requests:[]},
 autonomy:{task_id:'isolated-local-manual',phase:'FAULT',control_mode:'automatic',healthy:false,still:true,navigation_terminal:true,output:[0,0],
  live_controls_enabled:true,restricted_manual:{active:false,ready:false,map_frozen:true,reason:''},
  manual_policy:{version:1,mode:'local_safety',ignores_virtual_walls:true,requires_global_localization:false,restricted:true,
   motion_allowed:false,blocked_reason:'请先手动接管',direction_blocked:false,direction_reason:'',map_frozen:true,linear_limit_mps:.05,angular_limit_rps:.10},
  tuning:{task_id:'isolated-local-manual',ready:true,age_seconds:.1,phase:'READY',pending:false,confirmed:values,requested:values},
  speed:{phase:'READY',cap_mps:.18,confirmed_mps:.18},handoff:{distance_m:.8,phase:'READY'},obstacle_escape:{enabled:false,allowed:false,field_validated:false},
  walls:{items:[{id:'虚拟墙',points:[[0,0],[1,0]]}],wall_revision:1,navigation_generation:15,applied_navigation_generation:2,phase:'REVIEW_REQUIRED',needs_confirmation:true,review_allowed:false},
  actions:{resume:{enabled:false,reason:'定位与人工墙尚未复核'},pause:{enabled:false},manual:{enabled:true},manual_restricted:{enabled:true},return:{enabled:false},recheck:{enabled:true}}}};}
function ready(s){const a=s.autonomy;a.phase='MANUAL';a.control_mode='manual';a.restricted_manual={active:true,ready:true,map_frozen:true,reason:''};a.manual_policy.motion_allowed=true;a.manual_policy.blocked_reason='';return s;}
let snapshot=fixture(),browser,child,rejectManual=false;
const server=http.createServer(async(req,res)=>{
 try{
  const url=new URL(req.url,'http://localhost'),json=(v,status=200)=>{res.writeHead(status,{'Content-Type':'application/json'});res.end(JSON.stringify(v));};
  if(req.method==='POST'){
   let text='';for await(const chunk of req)text+=chunk;const body=JSON.parse(text);
   if(url.pathname==='/api/operator'){
    assert(['status','claim'].includes(body.action));operatorCommands.push(body.action);
    if(body.action==='claim')snapshot.manual_control={...snapshot.manual_control,owner_id:'fixture-client',ready:true,phase:'READY',generation:snapshot.manual_control.generation+1};
    json({client_id:'fixture-client',manual_control:snapshot.manual_control});return;
   }
   assert.equal(url.pathname,'/api/autonomy');assert.equal(body.task_id,snapshot.autonomy.task_id);assert.equal(body.operation_epoch,3);
   assert(['manual','recheck'].includes(body.action),'页面不得发送人工墙旁路或其他未授权动作');commands.push(body);
   if(rejectManual){json({error:'原始扫描超过0.5秒未更新，尚未授予手动资格'},409);return;}
   if(body.action==='manual'){
    const requested=snapshot;
    setTimeout(()=>{if(snapshot!==requested)return;ready(snapshot);snapshot.task_operations.requests.push({request_id:body.request_id,request_result:'accepted'});},500);
   }
   json({accepted:true},202);return;
  }
  if(url.pathname==='/api/status'){json(snapshot);return;}
  if(url.pathname==='/api/scan'){json({valid:false,version:'fixture',points:[]});return;}
  if(url.pathname.startsWith('/api/')){json({bundles:[]});return;}
  const name={'/':'index.html','/app.js':'app.js','/style.css':'style.css','/rtc_control.js':'rtc_control.js'}[url.pathname];
  if(!name){res.writeHead(404);res.end();return;}
  const data=await fs.readFile(path.join(webRoot,name));res.writeHead(200,{'Content-Type':name.endsWith('.js')?'application/javascript':name.endsWith('.css')?'text/css':'text/html'});res.end(data);
 }catch(error){errors.push(String(error));if(!res.headersSent)res.writeHead(500);res.end('隔离夹具错误');}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));const base=`http://127.0.0.1:${server.address().port}`;
async function open(touch=false){
 const context=await browser.newContext({viewport:touch?{width:390,height:844}:{width:1440,height:1000},hasTouch:touch,isMobile:touch});
 await context.addInitScript(()=>{
  window.__commands=[];window.__allowSocket=true;
  window.WebSocket=class FixtureSocket{
   static OPEN=1;static CLOSED=3;
   constructor(){this.readyState=1;this.bufferedAmount=0;this.armed=false;window.__fixtureSocket=this;
    setTimeout(()=>{if(window.__allowSocket)this.onmessage?.({data:JSON.stringify({session:'isolated-fixture-session'})});},20);
    this.timer=setInterval(()=>{if(window.__allowSocket)this.onmessage?.({data:JSON.stringify({armed:this.armed,health:'',reason:this.armed?'已使能，按住行驶':'输入已就绪，请使能',idle_heartbeat_required:false,control_lease_ms:300})});},80);}
   send(text){const v=JSON.parse(text);window.__commands.push(v);if(v.action==='stop')this.armed=false;if(v.action==='enable')this.armed=true;}
   close(){clearInterval(this.timer);this.readyState=3;this.onclose?.({code:1000,wasClean:true});}
  };
 });
 const page=await context.newPage();page.on('pageerror',e=>errors.push(e.stack||e.message));page.on('dialog',d=>{errors.push('非预期弹窗');void d.dismiss();});
 await page.route('**/*',async route=>{if(new URL(route.request().url()).origin!==base){external.push(route.request().url());await route.abort();}else await route.continue();});
 await page.goto(base);await page.waitForFunction(()=>session&&operatorClientId==='fixture-client',null,{timeout:5000});return page;
}
async function manualReady(page){await page.waitForFunction(()=>state.autonomy.phase==='MANUAL'&&!document.getElementById('enable').disabled,null,{timeout:5000});}
async function screenshot(page,label){await page.locator('#manual-controls').screenshot({path:path.join(evidence,label+'.png')});}
async function flow(touch=false){
 snapshot=fixture();const p=await open(touch),before=commands.length;
 await p.waitForFunction(()=>!document.getElementById('auto-manual').disabled);
 await p.evaluate(()=>{pressed={x:.7,yaw:0};activePointer=25;latestControl.armed=true;document.getElementById('auto-manual').click();});
 await manualReady(p);
 assert.equal(commands.length,before+1);assert.equal(commands.at(-1).action,'manual');
 assert.equal(await p.evaluate(()=>pressed),'');assert.equal(await p.evaluate(()=>activePointer),null);assert.equal(await p.evaluate(()=>latestControl.armed),false);
 assert.match(await p.locator('#manual-motion-title').innerText(),/手动已接管，可启用遥控/);
 assert.match(await p.locator('#manual-motion-detail').innerText(),/地图保持冻结.*自动.*不可用.*0\.05 m\/s.*0\.10 rad\/s.*激光.*扫掠.*松手停车/);
 assert.equal(await p.locator('#manual-wall-bypass,#wall-bypass').count(),0);
 assert.equal(await p.evaluate(()=>window.__commands.filter(v=>['enable','drive','intent'].includes(v.action)).length),0,'接管与状态读取不会继承旧手势或自动使能');
 await screenshot(p,touch?'mobile-ready-390':'desktop-ready');
 await p.locator('#enable').click();await p.waitForFunction(()=>latestControl.armed===true);
 assert.equal(await p.evaluate(()=>window.__commands.filter(v=>['drive','intent'].includes(v.action)).length),0,'必须有新的按住动作');
 await p.locator('#joystick').scrollIntoViewIfNeeded();
 const rect=await p.locator('#joystick').boundingBox();
 if(touch)await p.touchscreen.tap(rect.x+rect.width/2,rect.y+rect.height/2-60);
 else {await p.mouse.move(rect.x+rect.width/2,rect.y+rect.height/2-60);await p.mouse.down();await pause(150);await p.mouse.up();}
 assert(await p.evaluate(()=>window.__commands.some(v=>v.action==='drive')));assert(await p.evaluate(()=>window.__commands.some(v=>v.action==='release')));
 if(touch)assert.equal(await p.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
 checks.push((touch?'390触屏':'桌面')+'：故障接管→明确许可→新使能→新按住；零静默旁路，无旧手势运动');
 await screenshot(p,touch?'mobile-armed-390':'desktop-armed');await p.close();
}
try{
 child=spawn(executable,['--headless=new','--no-sandbox','--disable-gpu','--no-proxy-server','--disable-dev-shm-usage','--disable-background-networking',
  '--disable-extensions','--disable-component-update','--remote-debugging-port=0',`--user-data-dir=${profile}`,'about:blank'],
  {detached:true,env:{...process.env,ROS_DOMAIN_ID:'219',ROS_LOCALHOST_ONLY:'1'},stdio:['ignore','ignore','pipe']});
 let stderr='',port;child.stderr.on('data',v=>stderr=(stderr+v).slice(-2000));
 for(let i=0;i<200&&!port;i++){try{port=(await fs.readFile(path.join(profile,'DevToolsActivePort'),'utf8')).split('\n')[0];}catch{await pause(50);}}
 assert(port,'隔离Chromium未就绪：'+stderr);browser=await chromium.connectOverCDP(`http://127.0.0.1:${port}`);
 await flow(false);await flow(true);
 for(const reason of ['原始扫描超过0.5秒未更新','轮式里程计超过0.5秒未更新','本地控制通信超时','参数安全核验未通过']){
  snapshot=ready(fixture());const p=await open();await manualReady(p);snapshot.autonomy.manual_policy.motion_allowed=false;snapshot.autonomy.manual_policy.blocked_reason=reason;
  await p.waitForFunction(r=>state.autonomy.manual_policy.blocked_reason===r&&document.getElementById('enable').disabled,reason);
  assert.match(await p.locator('#motion').innerText(),new RegExp(reason.replace('.', '\\.')));
  const before=await p.evaluate(()=>window.__commands.filter(v=>v.action==='enable').length);await p.locator('#enable').dispatchEvent('click');
  assert.equal(await p.evaluate(()=>window.__commands.filter(v=>v.action==='enable').length),before);
  checks.push(reason+'：明确停车原因，不能从旧使能心跳或事件绕过');await p.close();
 }
 snapshot=ready(fixture());snapshot.autonomy.manual_policy.map_frozen=false;const freezePending=await open();
 await freezePending.waitForFunction(()=>document.getElementById('manual-motion-detail').textContent.includes('等待地图冻结回执'));
 assert.equal(await freezePending.locator('#enable').isDisabled(),true);assert.equal(await freezePending.evaluate(()=>!!localManualPolicy()),true);
 assert.equal(await freezePending.evaluate(()=>!!restrictedControlReady()),false);
 await freezePending.evaluate(()=>{window.__fixtureSocket.armed=true;pressed={x:.8,yaw:0};drive();});await pause(150);
 assert.equal(await freezePending.evaluate(()=>window.__commands.filter(v=>['enable','drive','intent'].includes(v.action)).length),0);
 checks.push('等待冻结回执是受支持暂态；整体保持停车，旧ready与旧手势均不放行');await freezePending.close();
 snapshot=ready(fixture());snapshot.autonomy.manual_policy=null;const nullPolicy=await open();
 await nullPolicy.waitForFunction(()=>document.getElementById('manual-motion-detail').textContent.includes('策略不完整或版本不受支持'));
 assert.equal(await nullPolicy.locator('#enable').isDisabled(),true);assert.equal(await nullPolicy.evaluate(()=>!!restrictedControlReady()),false);
 assert(!(await nullPolicy.locator('#manual-motion-detail').innerText()).includes('按原有保护条件执行'));
 checks.push('manual_policy字段存在但为null时不回退旧ready，明确保持停车');await nullPolicy.close();
 snapshot=ready(fixture());const direction=await open();await manualReady(direction);await direction.locator('#enable').click();await direction.waitForFunction(()=>latestControl.armed);
 snapshot.autonomy.manual_policy.direction_blocked=true;snapshot.autonomy.manual_policy.direction_reason='当前方向真实障碍进入车身扫掠范围';
 await direction.waitForFunction(()=>document.getElementById('manual-motion-title').textContent.includes('当前方向'));
 assert.match(await direction.locator('#manual-motion-detail').innerText(),/真实障碍.*安全方向/);
 assert.equal(await direction.locator('#joystick').evaluate(e=>e.classList.contains('disabled')),false);
 checks.push('真实障碍仅拒绝当前方向时，仍可松手选择安全方向');await direction.close();
 // 真实浏览器分别核对桌面与390触屏的拒绝分类；纯反馈不得下发恢复或确认。
 for(const touch of [false,true]){
  for(const [reason,title,detail] of [
   ['方向扫掠检查超过8毫秒预算，保持停车','安全检查超时，保持停车',/计算未及时完成.*未下发速度.*松开摇杆/],
   ['所选方向的扫掠区仍有激光缺束或盲区，保持停车','当前方向观测不足',/未下发速度.*有效激光观测/],
   ['等待近场保护返回本次速度检查，底盘输出保持零','安全检查尚未完成，保持停车',/未下发速度/],
   ['未知的车端保护原因','当前方向安全检查未通过',/未下发速度.*检查车端原因/],
  ]){
   snapshot=ready(fixture());Object.assign(snapshot.autonomy.manual_policy,{direction_blocked:true,direction_reason:reason});
   const before=commands.length,p=await open(touch);await manualReady(p);
   assert.equal(await p.locator('#manual-motion-title').innerText(),title);
   assert.match(await p.locator('#manual-motion-detail').innerText(),detail);
   assert.equal(commands.length,before);assert.equal(await p.evaluate(()=>window.__commands.some(v=>['enable','drive','intent'].includes(v.action))),false);
   assert.equal(await p.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false);
   if(reason.includes('8毫秒'))await screenshot(p,touch?'mobile-timeout-390':'desktop-timeout');
   checks.push((touch?'390触屏':'桌面')+'：'+title+'，零自动控制请求');await p.close();
  }
  snapshot=fixture();snapshot.phase='CURRENT';Object.assign(snapshot.autonomy,{phase:'PAUSED',healthy:true});
  Object.assign(snapshot.autonomy.walls,{review_allowed:true,requires_recheck:false});
  snapshot.autonomy.actions.recheck={enabled:false,reason:'当前仅需复核人工墙位置；请打开临时墙核对并复核，无需重新定位'};
  snapshot.autonomy.actions.confirm_walls={enabled:true};
  const before=commands.length,p=await open(touch);await p.waitForFunction(()=>!document.getElementById('auto-recheck').disabled);
  assert.equal(await p.locator('#auto-recheck').innerText(),'复核临时墙');
  assert.match(await p.locator('#autonomy-hint').innerText(),/无需重新定位/);
  await p.locator('#auto-recheck').click();assert.equal(await p.locator('#wall-controls').isVisible(),true);
  assert.equal(commands.length,before,'只打开现有临时墙，不提交重定位或confirm_walls');
  await p.locator('#wall-controls').screenshot({path:path.join(evidence,touch?'mobile-wall-review-390.png':'desktop-wall-review.png')});
  checks.push((touch?'390触屏':'桌面')+'：仅需墙复核时直达现有面板，不自动确认墙位');await p.close();
  // 即使墙报告允许复核，定位不健康、恢复未结束或位置待验证仍保留重新检查。
  for(const change of [{healthy:false},{recovery_pending:true},{localization_requires_verification:true}]){
   Object.assign(snapshot.autonomy,{healthy:true,recovery_pending:false,localization_requires_verification:false},change);
   snapshot.autonomy.actions.recheck={enabled:true};
   const before=commands.length,q=await open(touch);await q.waitForFunction(()=>!document.getElementById('auto-recheck').disabled);
   assert.equal(await q.evaluate(()=>wallReviewOnly()),false);
   assert.equal(await q.locator('#auto-recheck').innerText(),'重新检查');await q.locator('#auto-recheck').click();
   await q.waitForFunction(()=>autonomyPending?.action==='recheck');
   for(let i=0;i<20&&commands.length===before;i++)await pause(25);
   assert.equal(commands.length,before+1);assert.equal(commands.at(-1).action,'recheck');
   assert.equal(await q.locator('#wall-controls').isHidden(),true,'定位故障不打开墙工具替代恢复');
   checks.push((touch?'390触屏':'桌面')+'：'+Object.keys(change)[0]+'异常且墙可复核，仍明确提交recheck');await q.close();
  }
 }
 for(const [label,change] of [['stale',s=>{s.autonomy_age_ms=501;}],['no-owner',s=>{s.manual_control.owner_id='another-client';}],['unsupported-policy',s=>{s.autonomy.manual_policy.version=2;}]]){
  snapshot=ready(fixture());const p=await open();await manualReady(p);change(snapshot);
  await p.waitForFunction(expected=>state.autonomy_age_ms===expected.age&&state.manual_control.owner_id===expected.owner&&state.autonomy.manual_policy.version===expected.version,
   {age:snapshot.autonomy_age_ms,owner:snapshot.manual_control.owner_id,version:snapshot.autonomy.manual_policy.version});
  await p.waitForFunction(()=>document.getElementById('enable').disabled);checks.push(label+'：撤销启用资格，不恢复旧授权');await p.close();
 }
 snapshot=ready(fixture());const disconnected=await open();await manualReady(disconnected);await disconnected.evaluate(()=>{window.__allowSocket=false;window.__fixtureSocket.close();});
 await disconnected.waitForFunction(()=>document.getElementById('enable').disabled);assert.match(await disconnected.locator('#manual-motion-title').innerText(),/未确认/);checks.push('断线立即清手势、撤销使能，显示运动许可未知');await disconnected.close();
 snapshot=ready(fixture());delete snapshot.autonomy.manual_policy;snapshot.autonomy.restricted_manual.reason='人工墙保护需要可信定位';const legacy=await open();
 await legacy.waitForFunction(()=>document.getElementById('manual-motion-detail').textContent.includes('尚未提供独立手动能力'));
 assert.equal(await legacy.locator('#enable').isDisabled(),true);assert(!(await legacy.locator('#manual-motion-title').innerText()).includes('可启用遥控'));
 checks.push('旧后端能力缺失：显示真实保护边界，不伪装已解锁，也不显示旁路开关');await legacy.close();
 snapshot=ready(fixture());snapshot.phase='CURRENT';snapshot.autonomy.healthy=true;snapshot.autonomy.restricted_manual.active=false;
 Object.assign(snapshot.autonomy.manual_policy,{restricted:false,map_frozen:false,linear_limit_mps:null,angular_limit_rps:null});
 const normal=await open();await manualReady(normal);assert.match(await normal.locator('#manual-motion-detail').innerText(),/当前遥控档位/);assert(!(await normal.locator('#manual-motion-detail').innerText()).includes('0.05'));
 checks.push('正常手动保持原遥控档位，不强制套用故障受限速度');await normal.close();
 snapshot=fixture();snapshot.autonomy.phase='EXPLORING';const auto=await open();await auto.waitForFunction(()=>document.getElementById('wall-manual-policy').textContent.includes('自动探索与返程'));
 assert.equal(await auto.locator('#manual-motion-feedback').isHidden(),true);assert.match(await auto.locator('#wall-protection').innerText(),/自动导航人工墙待复核/);checks.push('自动任务继续显示人工墙与定位复核要求，手动许可不替代自动条件');await auto.close();
 snapshot=fixture();rejectManual=true;const rejected=await open();await rejected.waitForFunction(()=>!document.getElementById('auto-manual').disabled);await rejected.locator('#auto-manual').click();
 await rejected.waitForFunction(()=>document.getElementById('result').textContent.includes('原始扫描超过0.5秒'));
 assert.equal(snapshot.autonomy.phase,'FAULT');checks.push('接管请求被后端拒绝时显示本次原因，不显示手动已就绪');await rejected.close();
 assert.equal(errors.length,0,errors.join('\n'));assert.equal(external.length,0);
 assert.deepEqual(commands.map(v=>v.action),['manual','manual',...Array(6).fill('recheck'),'manual'],'仅显式接管与六次定位异常重新检查；不发送旁路或自动确认墙');
 await fs.writeFile(path.join(evidence,'results.json'),JSON.stringify({passed:true,real_chromium:true,web_root:webRoot,source:'隔离策略HTTP/WS夹具，无ROS和车辆',checks,
  commands:commands.map(v=>({...v,session:'<REDACTED>'})),operator_commands:operatorCommands,errors,external},null,2));
 console.log('独立本地手动真实Chromium通过；'+checks.length+'组检查；证据 '+evidence);
}catch(error){await fs.writeFile(path.join(evidence,'failure.json'),JSON.stringify({passed:false,error:String(error),errors,external},null,2));throw error;}
finally{if(browser)await browser.close().catch(()=>{});if(child?.pid){try{process.kill(-child.pid,'SIGTERM');}catch{}}server.closeAllConnections();await new Promise(resolve=>server.close(resolve));await fs.rm(profile,{recursive:true,force:true});}
