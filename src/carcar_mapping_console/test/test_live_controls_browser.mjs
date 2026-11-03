// 隔离HTTP状态夹具与真实Chromium验收，不读取或调用正式8080、ROS及车辆控制。
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import http from 'node:http';
import {spawn} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {chromium} from '/home/jetson/luhao/my_nav_carcar/src/carcar_inspection/web/node_modules/playwright-core/index.mjs';
const root=fileURLToPath(new URL('../../../',import.meta.url));
const evidence=path.resolve(process.argv[2]||path.join(root,'log/live-controls-browser-'+Date.now()));
await fs.mkdir(evidence,{recursive:true});
const profile=await fs.mkdtemp(path.join(os.tmpdir(),'carcar-live-controls-browser-'));
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const errors=[],dialogs=[],commands=[],external=[];
const values={global_inflation_radius:.5,global_cost_scaling_factor:3,local_inflation_radius:.3,local_cost_scaling_factor:5};
const state={phase:'CURRENT',task_state:'RUNNING',managed:true,busy:false,dirty:true,autonomy_enabled:true,autonomy_age_ms:10,
 dataflow_protocol:1,control:{armed:false,health:''},navigation:{},scan_age:.01,odom_age:.01,map_age:.1,pose:[2,2,0],
 task_operations:{protocol:1,task_id:'isolated-browser-task',epoch:3,requests:[]},
 autonomy:{task_id:'isolated-browser-task',phase:'EXPLORING',control_mode:'automatic',healthy:true,still:false,navigation_terminal:false,output:[.1,0],
  speed:{phase:'READY',pending:false,cap_mps:.1,confirmed_mps:.1},tuning:{task_id:'isolated-browser-task',ready:true,age_seconds:.1,phase:'READY',pending:false,confirmed:{...values},requested:{...values},request_id:''},
  handoff:{distance_m:.25,phase:'READY',request_id:''},obstacle_escape:{enabled:false,allowed:false,field_validated:false},
  walls:{items:[],wall_revision:0,navigation_generation:2,phase:'READY',request_id:'',needs_confirmation:false,bypass_enabled:false},
  actions:Object.fromEntries(['resume','pause','manual','return','recheck'].map(action=>[action,{enabled:true,reason:''}]))}};
let rejectNext=false,offline=false,browser,child;
const server=http.createServer(async(req,res)=>{
 try{
  const url=new URL(req.url,'http://localhost'),json=(value,status=200)=>{res.writeHead(status,{'Content-Type':'application/json'});res.end(JSON.stringify(value));};
  if(req.method==='POST'){
   let text='';for await(const chunk of req)text+=chunk;const body=JSON.parse(text);commands.push(body);
   assert.equal(url.pathname,'/api/autonomy','浏览器只能向隔离任务接口提交');
   if(rejectNext){rejectNext=false;json({error:'隔离夹具：参数服务暂不可用'},409);return;}
   assert.equal(body.task_id,state.autonomy.task_id);assert.equal(body.operation_epoch,3);
   const a=state.autonomy,module=['set_tuning','restore_tuning','save_tuning'].includes(body.action)?a.tuning:body.action==='set_handoff'?a.handoff:a.walls;
   module.phase='APPLYING';module.pending=true;module.request_id=body.request_id;
   state.task_operations.requests.push({request_id:body.request_id,request_result:'accepted'});
   if(body.action==='set_tuning')module.requested={...module.confirmed,...body.values};
   setTimeout(()=>{
    if(body.action==='set_tuning')module.confirmed={...module.confirmed,...body.values};
    if(body.action==='restore_tuning')module.confirmed={...values};
    if(body.action==='save_tuning')module.defaults_saved_request_id=body.request_id;
    if(body.action==='set_handoff')module.distance_m=body.distance_m;
    if(body.action==='set_walls'){module.items=body.walls;module.wall_revision++;}
    if(body.action==='confirm_walls')module.needs_confirmation=false;
    if(body.action==='set_wall_bypass'){module.bypass_enabled=body.enabled;module.bypass_request_id=body.request_id;module.requires_recheck=true;state.autonomy.phase=body.enabled?'MANUAL':'PAUSED';state.autonomy.restricted_manual={active:body.enabled,ready:body.enabled,map_frozen:true};}
    module.phase='READY';module.pending=false;
   },650);
   json({accepted:true},202);return;
  }
  if(url.pathname==='/api/status'){json(state,offline?503:200);return;}
  if(url.pathname==='/api/scan'){json({valid:false,version:'offline-fixture',points:[]});return;}
  if(url.pathname.startsWith('/api/')){json({bundles:[]});return;}
  const file={'/':'index.html','/app.js':'app.js','/style.css':'style.css','/rtc_control.js':'rtc_control.js'}[url.pathname];
  if(!file){res.writeHead(404);res.end();return;}
  const data=await fs.readFile(path.join(root,'src/carcar_mapping_console/web',file));res.writeHead(200,{'Content-Type':file.endsWith('.js')?'application/javascript':file.endsWith('.css')?'text/css':'text/html'});res.end(data);
 }catch(error){errors.push(error.message);if(!res.headersSent)res.writeHead(500);res.end('隔离测试错误');}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));const base=`http://127.0.0.1:${server.address().port}`;
async function open(viewport,touch=false){
 const context=await browser.newContext({viewport,hasTouch:touch,isMobile:touch});
 await context.addInitScript(()=>{window.WebSocket=class FixtureSocket{
  static OPEN=1;static CLOSED=3;constructor(){this.readyState=1;this.bufferedAmount=0;setTimeout(()=>this.onmessage?.({data:JSON.stringify({session:'isolated-fixture-session'})}),20);this.timer=setInterval(()=>this.onmessage?.({data:JSON.stringify({armed:false,health:'',idle_heartbeat_required:false,reason:'隔离夹具，无真实车辆',control_lease_ms:300})}),100);}send(){}close(){clearInterval(this.timer);this.readyState=3;}};});
 const page=await context.newPage();page.on('pageerror',error=>errors.push(error.message));page.on('dialog',dialog=>{dialogs.push(dialog.message());void dialog.dismiss();});
 page.on('request',request=>{if(new URL(request.url()).origin!==base)external.push(request.url());});
 await page.goto(base);await page.waitForFunction(()=>!document.getElementById('wall-tool').disabled);
 await page.evaluate(async()=>{const source=document.createElement('canvas');source.width=source.height=100;const p=source.getContext('2d');p.fillStyle='#f2f2f2';p.fillRect(0,0,100,100);p.fillStyle='#424242';p.fillRect(0,0,100,3);p.fillRect(0,0,3,100);mapImage=await createImageBitmap(source);displayMap={width:100,height:100,resolution:.05,origin:[0,0,0]};fit();document.getElementById('map-empty').hidden=true;renderLiveControls();});
 return {context,page};
}
try{
 child=spawn('/home/jetson/luhao/my_nav_carcar/build/carcar_mapping_console/mapping_test_chromium',['--headless=new','--no-sandbox','--disable-gpu','--no-proxy-server','--disable-dev-shm-usage','--disable-background-networking','--disable-extensions','--disable-component-update','--remote-debugging-port=0',`--user-data-dir=${profile}`,'about:blank'],{detached:true,env:{...process.env,ROS_DOMAIN_ID:'219',ROS_LOCALHOST_ONLY:'1'},stdio:['ignore','ignore','pipe']});
 let chromiumError='';child.stderr.on('data',v=>chromiumError=(chromiumError+v).slice(-2000));let port;
 for(let i=0;i<200&&!port;i++){try{port=(await fs.readFile(path.join(profile,'DevToolsActivePort'),'utf8')).split('\n')[0];}catch{await pause(50);}}
 assert(port,'Chromium未就绪：'+chromiumError);browser=await chromium.connectOverCDP(`http://127.0.0.1:${port}`);
 const desktop=await open({width:1440,height:1000}),page=desktop.page;
 await page.locator('#live-settings summary').click();
 assert(await page.locator('#escape-enabled').isDisabled());
 const count=commands.length;await page.locator('#range-global_inflation_radius').evaluate(input=>{input.value='.62';input.dispatchEvent(new Event('input'));});assert.equal(commands.length,count,'拖动预览不应逐帧提交');
 await page.locator('#range-global_inflation_radius').dispatchEvent('change');
 await page.waitForFunction(()=>['尚未确认','等待真实读回','等待车端应用'].some(word=>document.getElementById('tuning-status').textContent.includes(word)));
 assert.match(await page.locator('#readback-global_inflation_radius').innerText(),/0.50/);
 await page.waitForFunction(()=>document.getElementById('tuning-status').textContent.includes('已应用并确认'));
 assert.equal(commands.length,count+1);assert.equal(commands.at(-1).values.global_inflation_radius,.62);
 await page.locator('#tuning-save').click();await page.waitForFunction(()=>document.getElementById('tuning-status').textContent.includes('已保存为新任务默认'));
 rejectNext=true;await page.locator('#tuning-restore').click();await page.waitForFunction(()=>document.getElementById('tuning-status').textContent.includes('参数服务暂不可用'));
 await page.locator('#wall-tool').click();await page.locator('#wall-draw').click();
 const bounds=await page.locator('#map').boundingBox();assert(bounds);
 await page.locator('#map').click({position:{x:bounds.width*.3,y:bounds.height*.5}});await page.locator('#map').click({position:{x:bounds.width*.7,y:bounds.height*.5}});
 assert.match(await page.locator('#wall-draft').innerText(),/2 点/);await page.locator('#wall-apply').click();await page.waitForFunction(()=>document.getElementById('wall-status').textContent.includes('已由车端应用'));
 assert.equal(state.autonomy.walls.items.length,1);await page.evaluate(()=>scrollTo(0,0));await pause(100);await page.screenshot({path:path.join(evidence,'desktop-applied.png'),fullPage:true});
 await page.reload();await page.waitForFunction(()=>document.getElementById('wall-selection').options.length===1&&document.getElementById('wall-selection').options[0].value.startsWith('wall-'));
 const mobile=await open({width:390,height:844},true),mp=mobile.page;
 await mp.locator('#live-settings summary').click();await mp.locator('#wall-tool').tap();await mp.locator('#wall-draw').tap();
 const mb=await mp.locator('#map').boundingBox();await mp.locator('#map').tap({position:{x:mb.width*.2,y:mb.height*.5}});await mp.locator('#map').tap({position:{x:mb.width*.8,y:mb.height*.5}});
 assert.match(await mp.locator('#wall-draft').innerText(),/2 点/);
 await mp.keyboard.press('Escape');assert.match(await mp.locator('#wall-draft').innerText(),/尚无草稿/);
 state.autonomy.phase='RECOVERING';state.autonomy.recovery_pending=true;state.autonomy.walls.needs_confirmation=true;state.autonomy.walls.navigation_generation++;state.autonomy.walls.review_allowed=false;state.autonomy.walls.review_wait_seconds=15;state.autonomy.walls.review_wait_limit_seconds=300;state.autonomy.walls.review_reason='原图定位已复验，等待墙位置核对';
 await mp.waitForFunction(()=>document.getElementById('wall-review').disabled);state.autonomy.walls.review_allowed=true;await mp.waitForFunction(()=>!document.getElementById('wall-review').disabled&&document.getElementById('workflow-title').textContent.includes('等待临时墙位置复核'));
 await mp.waitForFunction(()=>!document.getElementById('wall-protection').hidden);assert.match(await mp.locator('#wall-protection').innerText(),/人工墙保护不可用/);
 await mp.locator('#wall-review').tap();await mp.waitForFunction(()=>document.getElementById('wall-protection').hidden);
 state.autonomy.phase='MANUAL';state.autonomy.still=true;state.autonomy.navigation_terminal=true;state.autonomy.recovery_pending=false;state.autonomy.control_mode='manual';await mp.waitForFunction(()=>!document.getElementById('wall-bypass').disabled);
 await mp.locator('#wall-bypass').tap();await mp.waitForFunction(()=>document.getElementById('wall-protection').textContent.includes('旁路已开启')&&!document.getElementById('wall-protection').hidden);
 await mp.locator('#wall-bypass').tap();await mp.waitForFunction(()=>document.getElementById('wall-protection').textContent.includes('等待定位重新检查'));assert.equal(state.autonomy.phase,'PAUSED');state.autonomy.walls.requires_recheck=false;await mp.waitForFunction(()=>document.getElementById('wall-protection').hidden);
 assert.equal(await mp.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false,'移动布局横向溢出');
 await mp.locator('#live-settings').scrollIntoViewIfNeeded();await mp.screenshot({path:path.join(evidence,'mobile-tuning.png')});
 await mp.locator('#wall-controls').scrollIntoViewIfNeeded();await mp.screenshot({path:path.join(evidence,'mobile-walls.png')});
 await mp.locator('#stop').scrollIntoViewIfNeeded();const stopBounds=await mp.locator('#stop').boundingBox();assert(stopBounds&&stopBounds.width>=100&&stopBounds.height>=40);
 offline=true;await mp.waitForFunction(()=>document.getElementById('tuning-global_inflation_radius').disabled&&document.getElementById('wall-apply').disabled,null,{timeout:6000});
 assert.equal(dialogs.length,0,'普通调参和画墙不能弹常规确认窗');assert.equal(errors.length,0,errors.join('\n'));assert.equal(external.length,0,'不能访问隔离服务之外的地址');
 await fs.writeFile(path.join(evidence,'results.json'),JSON.stringify({passed:true,real_chromium:true,source:'明确隔离状态与地图夹具，未连接ROS或真实车辆',viewports:['1440x1000','390x844 touch'],commands:commands.map(v=>({...v,session:'<REDACTED>'})),errors,dialogs,external},null,2));
 console.log('真实Chromium桌面/触屏：调参真实确认、失败分层、墙绘制/刷新/复核/旁路、断线禁用与停车入口通过；证据 '+evidence);
}catch(error){
 await fs.writeFile(path.join(evidence,'failure-'+Date.now()+'.json'),JSON.stringify({passed:false,error:String(error),errors,dialogs,external},null,2));throw error;
}finally{
 if(browser)await browser.close().catch(()=>{});if(child?.pid){try{process.kill(-child.pid,'SIGTERM');}catch{}}
 await new Promise(resolve=>server.close(resolve));await fs.rm(profile,{recursive:true,force:true});
}
