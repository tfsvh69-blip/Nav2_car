// 使用显式读回闩锁，避免编译负载下错过短暂等待状态；真实Chromium仅连接本脚本的隔离HTTP夹具，不连接ROS、正式网关或车辆。
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import http from 'node:http';
import {spawn} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {chromium} from '../../carcar_inspection/web/node_modules/playwright-core/index.mjs';
const root=fileURLToPath(new URL('../../../',import.meta.url));
const evidence=path.resolve(process.argv[2]||path.join(root,'log/parameter-range-browser-'+Date.now()));
await fs.mkdir(evidence,{recursive:true});
const profile=await fs.mkdtemp(path.join(os.tmpdir(),'carcar-range-browser-'));
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const errors=[],commands=[],external=[],checks=[],pendingAcks=[];
const values={global_inflation_radius:.5,global_cost_scaling_factor:3,local_inflation_radius:.3,local_cost_scaling_factor:5};
const state={phase:'CURRENT',task_state:'RUNNING',managed:true,busy:false,dirty:true,autonomy_enabled:true,autonomy_age_ms:10,
 dataflow_protocol:1,control:{armed:false,health:''},navigation:{},scan_age:.01,odom_age:.01,map_age:.1,
 task_operations:{protocol:1,task_id:'isolated-range-task',epoch:3,requests:[]},
 autonomy:{task_id:'isolated-range-task',phase:'EXPLORING',control_mode:'automatic',healthy:true,still:false,navigation_terminal:false,output:[.1,0],
  speed:{phase:'READY',pending:false,cap_mps:.1,confirmed_mps:.1},
  tuning:{task_id:'isolated-range-task',ready:true,age_seconds:.1,phase:'READY',pending:false,confirmed:values,requested:values},
  handoff:{distance_m:.25,phase:'READY',request_id:''},obstacle_escape:{enabled:false,allowed:false,field_validated:false},
  walls:{items:[],wall_revision:0,navigation_generation:2,phase:'READY',request_id:'',needs_confirmation:false},
  actions:Object.fromEntries(['resume','pause','manual','return','recheck'].map(action=>[action,{enabled:true,reason:''}]))}};
let browser,child,page;
const server=http.createServer(async(req,res)=>{
 try{
  const url=new URL(req.url,'http://localhost');
  const json=(value,status=200)=>{res.writeHead(status,{'Content-Type':'application/json'});res.end(JSON.stringify(value));};
  if(req.method==='POST'){
   let text='';for await(const chunk of req)text+=chunk;const body=JSON.parse(text);
   assert.equal(url.pathname,'/api/autonomy');assert.equal(body.task_id,state.autonomy.task_id);assert.equal(body.operation_epoch,3);
   assert(['set_speed','set_handoff'].includes(body.action),'隔离夹具只接受本次参数操作');commands.push(body);
   const module=body.action==='set_speed'?state.autonomy.speed:state.autonomy.handoff;
   module.phase=body.action==='set_speed'?'SETTING':'APPLYING';module.pending=true;module.request_id=body.request_id;
   state.task_operations.requests.push({request_id:body.request_id,request_result:'accepted'});
   pendingAcks.push(()=>{
    if(body.action==='set_speed')Object.assign(module,{cap_mps:body.speed_mps,confirmed_mps:body.speed_mps,requested_mps:body.speed_mps});
    else module.distance_m=body.distance_m;
    module.phase='READY';module.pending=false;
   });
   json({accepted:true},202);return;
  }
  if(url.pathname==='/api/status'){json(state);return;}
  if(url.pathname==='/api/scan'){json({valid:false,version:'isolated-fixture',points:[]});return;}
  if(url.pathname.startsWith('/api/')){json({bundles:[]});return;}
  const filename={'/':'index.html','/app.js':'app.js','/style.css':'style.css','/rtc_control.js':'rtc_control.js'}[url.pathname];
  if(!filename){res.writeHead(404);res.end();return;}
  const data=await fs.readFile(path.join(root,'src/carcar_mapping_console/web',filename));
  res.writeHead(200,{'Content-Type':filename.endsWith('.js')?'application/javascript':filename.endsWith('.css')?'text/css':'text/html'});res.end(data);
 }catch(error){errors.push(error.message);if(!res.headersSent)res.writeHead(500);res.end('隔离夹具错误');}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const base=`http://127.0.0.1:${server.address().port}`;
async function open(viewport,touch=false){
 const context=await browser.newContext({viewport,hasTouch:touch,isMobile:touch});
 await context.addInitScript(()=>{window.WebSocket=class FixtureSocket{
  static OPEN=1;static CLOSED=3;
  constructor(){this.readyState=1;this.bufferedAmount=0;setTimeout(()=>this.onmessage?.({data:JSON.stringify({session:'isolated-fixture-session'})}),20);
   this.timer=setInterval(()=>this.onmessage?.({data:JSON.stringify({armed:false,health:'',idle_heartbeat_required:false,reason:'隔离夹具，无真实车辆',control_lease_ms:300})}),100);}
  send(){}close(){clearInterval(this.timer);this.readyState=3;}};});
 const p=await context.newPage();p.on('pageerror',error=>errors.push(error.message));p.on('dialog',dialog=>{errors.push('非预期弹窗');void dialog.dismiss();});
 p.on('request',request=>{if(new URL(request.url()).origin!==base)external.push(request.url());});
 await p.goto(base);await p.waitForFunction(()=>!document.getElementById('auto-speed-apply').disabled);
 await p.locator('#live-settings summary').click();
 return p;
}
async function handoff(p,distance,label){
 const count=commands.length,oldDistance=state.autonomy.handoff.distance_m;
 await p.locator('#handoff-distance').fill(String(distance));await p.locator('#handoff-distance').dispatchEvent('change');
 await p.waitForFunction(()=>livePending?.action==='set_handoff'&&document.getElementById('handoff-status').textContent.includes('等待'));
 assert.equal(state.autonomy.handoff.distance_m,oldDistance,'HTTP受理前后保留上次读回值');
 assert.equal(pendingAcks.length,1,'等待本次参数请求再提供读回');pendingAcks.shift()();
 await p.waitForFunction(v=>document.getElementById('handoff-status').textContent.includes(`车端已确认 ${v.toFixed(2)} m`),distance);
 assert.equal(commands.length,count+1);assert.equal(commands.at(-1).action,'set_handoff');assert.equal(commands.at(-1).distance_m,distance);
 assert.equal(Number(await p.locator('#handoff-range').inputValue()),distance);
 checks.push({label,distance_m:distance,submitted:1,confirmed:true});
 await p.locator('.handoff-controls').screenshot({path:path.join(evidence,label+'.png')});
}
try{
 child=spawn(path.join(root,'build/carcar_mapping_console/mapping_test_chromium'),['--headless=new','--no-sandbox','--disable-gpu',
  '--no-proxy-server','--disable-dev-shm-usage','--disable-background-networking','--disable-extensions','--disable-component-update',
  '--remote-debugging-port=0',`--user-data-dir=${profile}`,'about:blank'],{detached:true,env:{...process.env,ROS_DOMAIN_ID:'219',ROS_LOCALHOST_ONLY:'1'},stdio:['ignore','ignore','pipe']});
 let chromiumError='',port;child.stderr.on('data',v=>chromiumError=(chromiumError+v).slice(-2000));
 for(let i=0;i<200&&!port;i++){try{port=(await fs.readFile(path.join(profile,'DevToolsActivePort'),'utf8')).split('\n')[0];}catch{await pause(50);}}
 assert(port,'隔离Chromium未就绪：'+chromiumError);browser=await chromium.connectOverCDP(`http://127.0.0.1:${port}`);
 page=await open({width:1440,height:1000});
 for(const id of ['handoff-distance','handoff-range'])assert.equal(Number(await page.locator('#'+id).getAttribute('max')),1.5);
 await page.locator('#auto-speed').evaluate(input=>{input.closest('details').open=true;input.value='.20';input.dispatchEvent(new Event('input'));});
 const beforeSpeed=commands.length;await page.locator('#auto-speed-apply').click();
 await page.waitForFunction(()=>document.getElementById('auto-speed-status').textContent.includes('等待车端'));
 assert.equal(state.autonomy.speed.cap_mps,.1,'HTTP受理不能提前提高执行上限');
 assert.equal(pendingAcks.length,1,'等待本次速度请求再提供读回');pendingAcks.shift()();
 await page.waitForFunction(()=>document.getElementById('auto-speed-status').textContent.includes('执行上限 0.20 m/s'));
 assert.equal(commands.length,beforeSpeed+1);assert.equal(commands.at(-1).speed_mps,.2);
 checks.push({label:'speed-0.20',speed_mps:.2,submitted:1,confirmed:true});
 await page.locator('#auto-speed-controls').screenshot({path:path.join(evidence,'speed-0.20.png')});
 await handoff(page,1.2,'desktop-handoff-1.20');await handoff(page,1.5,'desktop-handoff-1.50');
 for(const value of ['1.501','.149','']){
  const before=commands.length;await page.locator('#handoff-distance').fill(value);await page.locator('#handoff-distance').dispatchEvent('change');
  await page.waitForFunction(()=>document.getElementById('handoff-status').textContent.includes('网页未提交：提前衔接距离需在0.15～1.50米'));
  assert.equal(commands.length,before);assert.equal(state.autonomy.handoff.distance_m,1.5);
  checks.push({label:'rejected-input',value,submitted:0,confirmed_distance_m:1.5});
 }
 await page.locator('.handoff-controls').screenshot({path:path.join(evidence,'desktop-rejected-input.png')});
 const mobile=await open({width:390,height:844},true);await handoff(mobile,1.2,'mobile-handoff-1.20');
 assert.equal(await mobile.evaluate(()=>document.documentElement.scrollWidth>innerWidth),false,'移动布局不应横向溢出');
 assert.equal(errors.length,0,errors.join('\n'));assert.equal(external.length,0,'不得请求隔离服务之外的地址');
 await fs.writeFile(path.join(evidence,'results.json'),JSON.stringify({passed:true,real_chromium:true,source:'隔离HTTP状态夹具，未连接ROS或真实车辆',
  viewports:['1440x1000','390x844 touch'],checks,commands:commands.map(v=>({...v,session:'<REDACTED>'})),errors,external},null,2));
 console.log('真实Chromium：0.20m/s提交与读回、1.2/1.5米衔接提交与读回、越界/空值拒绝及移动触屏通过；证据 '+evidence);
}catch(error){
 await fs.writeFile(path.join(evidence,'failure.json'),JSON.stringify({passed:false,error:String(error),errors,external,commands:commands.map(v=>({...v,session:'<REDACTED>'})),state,handoff:page?await page.locator('#handoff-status').textContent().catch(()=>null):null},null,2));throw error;
}finally{
 if(browser)await browser.close().catch(()=>{});if(child?.pid){try{process.kill(-child.pid,'SIGTERM');}catch{}}
 server.closeAllConnections();await new Promise(resolve=>server.close(resolve));await fs.rm(profile,{recursive:true,force:true});
}
