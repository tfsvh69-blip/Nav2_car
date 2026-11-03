// 完整真实软件堆栈E2E；仅域227与18099，模拟传感器不接硬件，未显式--run时只说明入口。
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import net from 'node:net';
import {spawn,execFile} from 'node:child_process';
import {promisify} from 'node:util';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import {chromium} from '/home/jetson/luhao/my_nav_carcar/src/carcar_inspection/web/node_modules/playwright-core/index.mjs';
const root=path.resolve(fileURLToPath(new URL('../../../',import.meta.url)));
const base='http://127.0.0.1:18099',run=process.argv[2]==='--run';
if(!run){console.log('仅准备：显式设置ROS_DOMAIN_ID=227、ROS_LOCALHOST_ONLY=1并加载候选install后，传--run [新证据目录]执行；仅访问18099，不操作正式8080。');process.exit(0);}
assert.equal(process.env.ROS_DOMAIN_ID,'227','拒绝非隔离域');assert.equal(process.env.ROS_LOCALHOST_ONLY,'1','拒绝非本机ROS');
const execFileAsync=promisify(execFile),pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const evidence=path.resolve(process.argv[3]||path.join(root,'log/nav018_live_tuning_20261103/full-stack-'+new Date().toISOString().replace(/[:.]/g,'-')));
await fs.mkdir(path.dirname(evidence),{recursive:true});await fs.mkdir(evidence,{recursive:false});
const mapRoot=path.join(evidence,'maps'),logRoot=path.join(evidence,'tasks'),defaultsFile=path.join(evidence,'tuning-defaults.json');
await fs.mkdir(mapRoot);await fs.mkdir(logRoot);
const profile=await fs.mkdtemp(path.join(os.tmpdir(),'carcar-live-workflow-'));
const started=Date.now(),deadline=started+360000,states=[],requests=[],pageErrors=[],children=[],stages=[];
let browser,page,last={},task='',sequence=0,aborted=false,complete=false,stage='preflight';
const sourceHash=async file=>createHash('sha256').update(await fs.readFile(file)).digest('hex');
const result={script_sha256:await sourceHash(fileURLToPath(import.meta.url)),app_sha256:await sourceHash(path.join(root,'src/carcar_mapping_console/web/app.js')),hardware:false,source:'真实网页/参数服务/Nav2/SLAM；扫描、IMU、轮式与运动为模拟输入',domain:227,port:18099,limit_seconds:360,stages};
await fs.writeFile(path.join(evidence,'workflow-source.mjs'),await fs.readFile(fileURLToPath(import.meta.url)),{flag:'wx'});
const clean=value=>JSON.parse(JSON.stringify(value,(key,v)=>/(session|challenge|token|password|secret)/i.test(key)?'<REDACTED>':v));
const stopSignal=()=>{aborted=true;};process.on('SIGINT',stopSignal);process.on('SIGTERM',stopSignal);
function budget(){if(aborted)throw Error('已中断软件验收');if(Date.now()>deadline)throw Error('完整软件验收超过360秒预算');}
function mark(name,detail={}){stage=name;const value={time:new Date().toISOString(),stage:name,...detail};stages.push(clean(value));console.log('LIVE_WORKFLOW '+JSON.stringify(clean(value)));}
function launch(executable,args,name,environment={}){
 const output=path.join(evidence,name+'.log');const child=spawn(executable,args,{cwd:root,detached:true,env:{...process.env,...environment},stdio:['ignore','pipe','pipe']});
 const write=buffer=>{void fs.appendFile(output,buffer).catch(()=>{});};child.stdout.on('data',write);child.stderr.on('data',write);children.push({child,name});return child;
}
async function descendants(parent){
 const records=[];for(const name of await fs.readdir('/proc'))if(/^\d+$/.test(name)){try{const stat=await fs.readFile('/proc/'+name+'/stat','utf8');const after=stat.slice(stat.lastIndexOf(')')+2).split(' ');records.push({pid:Number(name),ppid:Number(after[1])});}catch{}}
 const ids=new Set([parent]);let added=true;while(added){added=false;for(const entry of records)if(ids.has(entry.ppid)&&!ids.has(entry.pid)){ids.add(entry.pid);added=true;}}
 return [...ids];
}
async function verifySoftwareOnly(){
 const gateway=children.find(v=>v.name==='gateway')?.child;if(!gateway?.pid)return;
 for(const pid of await descendants(gateway.pid)){try{const args=(await fs.readFile('/proc/'+pid+'/cmdline')).toString().split('\0');const names=args.slice(0,3).map(v=>path.basename(v));const name=names.find(v=>['rosmaster_node','sllidar_node','rplidar_node','realsense2_camera_node'].includes(v));if(name)throw Error('隔离栈意外出现硬件驱动，立即停止自有软件任务：'+name);}catch(error){if(error.code!=='ENOENT'&&error.code!=='ESRCH')throw error;}}
}
async function snapshot(ignoreBudget=false){
 if(!ignoreBudget)budget();
 const s=await page.evaluate(async()=>{const response=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(3000)});if(!response.ok)throw Error('状态HTTP '+response.status);return response.json();});
 last=s;states.push({time:new Date().toISOString(),state:clean(s)});
 await fs.appendFile(path.join(evidence,'states.ndjson'),JSON.stringify(states.at(-1))+'\n');
 return s;
}
function stopped(s){const a=s.autonomy||{};return s.autonomy_age_ms>=0&&s.autonomy_age_ms<=500&&a.still&&a.navigation_terminal&&Array.isArray(a.output)&&a.output.every(v=>v===0);}
function liveReady(s){return s.task_state==='RUNNING'&&s.autonomy_age_ms>=0&&s.autonomy_age_ms<=500&&s.autonomy?.healthy&&s.autonomy?.automatic_start_ready&&stopped(s)&&s.autonomy?.tuning?.phase==='READY'&&!s.autonomy.tuning.pending&&s.autonomy?.walls?.phase==='READY';}
function requestReady(module,id){const rejection=module?.rejected_requests?.find(r=>r.request_id===id);if(rejection)throw Error('车端明确拒绝 '+id+'：'+rejection.reason);if(module?.request_id!==id)return false;if(module.error||module.rolled_back||module.request_result==='rejected')throw Error('车端本次未应用 '+id+'：'+module.error);return module.phase==='READY'&&!module.pending;}
async function waitFor(predicate,seconds,description,{ignoreBudget=false,allowFault=false,interval=200}={}){
 const end=Date.now()+seconds*1000;
 while(Date.now()<end){const s=await snapshot(ignoreBudget);if(predicate(s))return s;if(!allowFault&&(s.phase==='FAILED'||s.autonomy?.phase==='FAULT'))throw Error(description+'：车端锁停 '+(s.autonomy?.quality_fault||s.autonomy?.reason||s.error));await pause(interval);}
 throw Error(description+'等待超时；末状态已保留');
}
async function post(endpoint,body={},ignoreBudget=false){
 if(!ignoreBudget)budget();assert(endpoint.startsWith('/api/'));
 const reply=await page.evaluate(async({endpoint,body})=>{
  let request={...body,session};
  if(endpoint==='/api/autonomy'){
   const state=await fetch('/api/status',{cache:'no-store',signal:AbortSignal.timeout(3000)}).then(r=>r.json());
   request=taskRequestBody(body.action,body.request_id,{...body,task_id:state.autonomy?.task_id,operation_epoch:state.task_operations?.epoch});
  }
  const response=await fetch(endpoint,{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},body:JSON.stringify(request),signal:AbortSignal.timeout(7000)});
  return {status:response.status,body:await response.json()};
 },{endpoint,body});
 requests.push({time:new Date().toISOString(),endpoint,request:clean(body),reply:clean(reply)});
 if(reply.status>=400)throw Error(endpoint+'未受理：'+(reply.body.error||reply.status));return reply.body;
}
async function command(action,extra={},ignoreBudget=false){
 const request_id='live-e2e-'+(++sequence)+'-'+Date.now();await post('/api/autonomy',{action,request_id,...extra},ignoreBudget);return request_id;
}
async function accepted(id,seconds=12){
 const s=await waitFor(s=>s.task_operations?.requests?.some(r=>r.request_id===id&&['accepted','rejected','cancelled'].includes(r.request_result)),seconds,'等待同一请求账本 '+id,{allowFault:true});
 const record=s.task_operations.requests.find(r=>r.request_id===id);assert.equal(record.request_result,'accepted',record.reason||'车端拒绝');return s;
}
async function capture(name){await fs.writeFile(path.join(evidence,name+'.json'),JSON.stringify(clean(last),null,2));await page.screenshot({path:path.join(evidence,name+'.png'),fullPage:true,timeout:5000});}
async function parameter(node,name,expected){
 const attempts=[];for(let attempt=0;attempt<2;attempt++){
  budget();try{
   const {stdout}=await execFileAsync('/opt/ros/humble/bin/ros2',['param','get',node,name,'--no-daemon','--spin-time','3'],{env:{...process.env,ROS_DOMAIN_ID:'227',ROS_LOCALHOST_ONLY:'1'},timeout:15000,maxBuffer:65536});
   const value=Number(stdout.trim().split(':').at(-1));assert(Number.isFinite(value)&&Math.abs(value-expected)<1e-6,node+' '+name+'真实读回不符：'+stdout.trim());
   attempts.push({attempt:attempt+1,result:'read',value});await fs.appendFile(path.join(evidence,'parameter-read-attempts.ndjson'),JSON.stringify({node,name,...attempts.at(-1)})+'\n');return {node,name,expected,value,attempts};
  }catch(error){
   attempts.push({attempt:attempt+1,result:'error',error:String(error)});await fs.appendFile(path.join(evidence,'parameter-read-attempts.ndjson'),JSON.stringify({node,name,...attempts.at(-1)})+'\n');
   if(attempt||!String(error.stderr||error).includes('Node not found'))throw error;
   const s=await snapshot();assert(s.autonomy?.healthy,'冷发现重试前车端健康已失效');
  }
 }
}
async function uiNumber(id,value,action){
 const before=requests.length;await page.locator('#'+id).fill(String(value));await page.locator('#'+id).press('Tab');
 const end=Date.now()+6000;while(Date.now()<end){budget();const sent=requests.slice(before).find(v=>v.endpoint==='/api/autonomy'&&v.request?.action===action);if(sent)return sent.request.request_id;await pause(50);}throw Error('网页输入未产生预期请求：'+id);
}
// 原图、全局和局部网格分别按自己的时间戳与TF对齐；不把滚动窗口索引当地图坐标。
const transform=(point,tf)=>[tf[0]+point[0]*Math.cos(tf[2])-point[1]*Math.sin(tf[2]),tf[1]+point[0]*Math.sin(tf[2])+point[1]*Math.cos(tf[2])];
const inverse=(point,tf)=>{const x=point[0]-tf[0],y=point[1]-tf[1];return[x*Math.cos(tf[2])+y*Math.sin(tf[2]),-x*Math.sin(tf[2])+y*Math.cos(tf[2])];};
function cellAt(grid,point){const local=inverse(inverse(point,grid.to_map),grid.origin),x=Math.floor(local[0]/grid.resolution),y=Math.floor(local[1]/grid.resolution);return x>=0&&y>=0&&x<grid.width&&y<grid.height?{x,y,value:grid.data[y*grid.width+x]}:null;}
const mapCell=(grid,x,y)=>transform(transform([(x+.5)*grid.resolution,(y+.5)*grid.resolution],grid.origin),grid.to_map);
const lineSamples=points=>Array.from({length:33},(_,i)=>[points[0][0]+(points[1][0]-points[0][0])*i/32,points[0][1]+(points[1][1]-points[0][1])*i/32]);
function distanceToLine(point,points){const [a,b]=points,dx=b[0]-a[0],dy=b[1]-a[1],t=Math.max(0,Math.min(1,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/(dx*dx+dy*dy)));return Math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy);}
async function mapProbe(name){
 const file=path.join(evidence,name+'.json');await execFileAsync(path.join(root,'build/carcar_mapping_console/mapping_wall_map_probe'),[file],{env:{...process.env,ROS_DOMAIN_ID:'227',ROS_LOCALHOST_ONLY:'1'},timeout:15000,maxBuffer:65536});
 const grids=JSON.parse(await fs.readFile(file,'utf8'));assert(grids.read_only&&grids.domain===227);return grids;
}
function chooseWall(grids,pose){
 assert(Array.isArray(pose)&&pose.length>=3,'缺少真实map车体位姿');
 for(const radius of [.85,.7,1.0])for(const offset of [Math.PI/2,-Math.PI/2,Math.PI*3/4,-Math.PI*3/4,Math.PI]){
  const angle=pose[2]+offset,center=[pose[0]+radius*Math.cos(angle),pose[1]+radius*Math.sin(angle)],direction=pose[2];
  const points=[-.15,.15].map(t=>[center[0]+t*Math.cos(direction),center[1]+t*Math.sin(direction)]);
  if(lineSamples(points).every(point=>{const raw=cellAt(grids.raw,point),local=cellAt(grids.local,point),global=cellAt(grids.global,point);return raw&&raw.value>=0&&raw.value<50&&local?.value===0&&global&&global.value>=0&&global.value<50;}))return points;
 }
 throw Error('真实已知空闲区内找不到远离车体的短墙候选，不能把未知/原障碍冒充人工墙证据');
}
function compareWall(before,after,points){
 const samples=lineSamples(points),proof={coordinates:'map',points,raw_free_before:0,raw_free_after:0,global_new_lethal:0,local_new_lethal:0,local_soft_gradient:0,gradient_values:[]};
 for(const point of samples){const raw0=cellAt(before.raw,point),raw1=cellAt(after.raw,point);if(raw0&&raw0.value>=0&&raw0.value<50)proof.raw_free_before++;if(raw1&&raw1.value>=0&&raw1.value<50)proof.raw_free_after++;
  for(const grid of ['global','local']){const a=cellAt(before[grid],point),b=cellAt(after[grid],point);if(a&&a.value>=0&&a.value<50&&b?.value===100)proof[grid+'_new_lethal']++;}}
 const gradient=new Set();for(let y=0;y<after.local.height;y++)for(let x=0;x<after.local.width;x++){
  const value=after.local.data[y*after.local.width+x];if(value<=0||value>=99)continue;const point=mapCell(after.local,x,y),d=distanceToLine(point,points),old=cellAt(before.local,point),raw=cellAt(after.raw,point);
  if(d>.08&&d<.38&&old?.value===0&&raw&&raw.value>=0&&raw.value<50){proof.local_soft_gradient++;gradient.add(value);}}
 proof.gradient_values=[...gradient].sort((a,b)=>a-b);
 assert.equal(proof.raw_free_before,samples.length,'候选墙原图起始不是空闲');assert.equal(proof.raw_free_after,samples.length,'人工墙写入了raw地图或原图坐标失配');
 assert(proof.global_new_lethal>5,'真实全局地图中未找到新增人工墙致命单元');assert(proof.local_new_lethal>5,'真实局部地图中未找到新增人工墙致命单元');
 assert(proof.local_soft_gradient>=4&&gradient.size>=2,'真实局部人工墙周围没有至少两级inflation衰减证据');return proof;
}
try{
 const disk=await fs.statfs(root);assert(disk.bavail*disk.bsize>23*1024**3,'不足23GiB空闲，停止启动录包和完整场景');
 const socket=net.createServer();await new Promise((resolve,reject)=>{socket.once('error',reject);socket.listen(18099,'127.0.0.1',resolve);});await new Promise(resolve=>socket.close(resolve));
 for(const name of ['carcar_mapping_console','carcar_mapping_autonomy']){const {stdout}=await execFileAsync('/opt/ros/humble/bin/ros2',['pkg','prefix',name],{timeout:10000});assert(stdout.trim().startsWith(root+'/install/'),'未加载候选安装：'+name+' '+stdout.trim());}
 const discovery=await execFileAsync('/opt/ros/humble/bin/ros2',['node','list','--no-daemon','--spin-time','1'],{timeout:12000});assert.equal(discovery.stdout.trim(),'','域227已有其他节点，拒绝混入现有任务');
 mark('software-only-start');
 launch(path.join(root,'build/carcar_mapping_autonomy/mapping_virtual_room'),[],'virtual-room',{CARCAR_ROOM_CONTINUOUS:'1',CARCAR_ROOM_MANAGED:'1',CARCAR_ROOM_EXTERNAL:'1',CARCAR_ROOM_MAX_SECONDS:'420'});
 launch(path.join(root,'build/carcar_mapping_console/mapping_web_gateway'),['--ros-args','-p','mode:=standby','-p','backend_hardware:=false','-p','autonomy_enabled:=true','-p','bind_address:=127.0.0.1','-p','port:=18099','-p','map_root:='+mapRoot,'-p','log_root:='+logRoot,'-p','tuning_defaults_file:='+defaultsFile,'-p','web_root:='+path.join(root,'src/carcar_mapping_console/web')],'gateway',{CARCAR_MAPPING_RECOVERY_ROOT:path.join(evidence,'recovery')});
 const chrome=launch(path.join(root,'build/carcar_mapping_console/mapping_test_chromium'),['--headless=new','--no-sandbox','--disable-gpu','--no-proxy-server','--disable-dev-shm-usage','--disable-background-networking','--disable-extensions','--disable-component-update','--remote-debugging-port=0','--user-data-dir='+profile,'about:blank'],'chromium',{ROS_DOMAIN_ID:'219'});
 let port;for(let i=0;i<200&&!port;i++){budget();try{port=(await fs.readFile(path.join(profile,'DevToolsActivePort'),'utf8')).split('\n')[0];}catch{await pause(50);}}
 assert(port,'真实Chromium未就绪');browser=await chromium.connectOverCDP('http://127.0.0.1:'+port);
 const context=await browser.newContext({viewport:{width:1440,height:1000}});page=await context.newPage();
 page.on('pageerror',error=>pageErrors.push(error.message));page.on('dialog',dialog=>{pageErrors.push('非预期弹窗：'+dialog.message());void dialog.dismiss();});
 page.on('request',request=>{if(request.method()==='POST'&&new URL(request.url()).origin===base){try{requests.push({time:new Date().toISOString(),endpoint:new URL(request.url()).pathname,request:clean(JSON.parse(request.postData()))});}catch{}}});
 for(let i=0;i<50;i++){try{const response=await fetch(base+'/api/status',{signal:AbortSignal.timeout(1000)});if(response.ok)break;}catch{}budget();await pause(200);}
 await page.goto(base);await page.waitForFunction(()=>typeof session==='string'&&!!session,null,{timeout:15000});
 await waitFor(s=>s.task_state==='STANDBY',10,'网页软件待命');
 await post('/api/start',{mode:'new',control_mode:'manual'});
 await waitFor(liveReady,90,'手动新任务健康、参数与场景就绪');task=last.autonomy.task_id;result.first_task=task;
 await verifySoftwareOnly();await capture('manual-ready');
 if(process.env.CARCAR_TEST_FIELD_RECOVERY==='1'){
  mark('failed-tuning-restore-and-manual');
  // 仅在隔离域227且真实轮式停稳时，使局部生命周期暂时inactive，制造真实调参与回退失败。
  assert(stopped(last));
  const lifecycle=async id=>{
   const {stdout}=await execFileAsync('/opt/ros/humble/bin/ros2',['service','call','/local_costmap/local_costmap/change_state','lifecycle_msgs/srv/ChangeState',`{transition: {id: ${id}}}`],{timeout:12000,maxBuffer:65536});
   assert(/success=True/.test(stdout),'软件局部代价图生命周期切换失败：'+stdout);
  };
  await lifecycle(4);
  const failedId=await command('set_tuning',{values:{local_cost_scaling_factor:5.1}});await accepted(failedId);
  await waitFor(s=>s.autonomy.tuning?.phase==='FAILED'&&stopped(s),20,'故障注入后保持停车',{allowFault:true});
  await page.waitForFunction(()=>document.getElementById('workflow-title').textContent.includes('参数核验失败'),null,{timeout:5000});
  await pause(300);
  assert((await page.locator('#motion').textContent()).includes('参数安全核验'),'控制心跳覆盖了真实参数故障');
  assert.equal(await page.locator('#enable').isEnabled(),false);
  result.failed_tuning_request=failedId;await capture('parameter-failed');
  await lifecycle(3);
  await page.locator('#live-settings').evaluate(el=>{el.open=true;});
  const before=requests.length;await page.locator('#tuning-restore').click();
  await waitFor(s=>s.autonomy.tuning?.request_result==='completed'&&s.autonomy.tuning?.ready&&stopped(s),30,'独立恢复参数读回与膨胀代次核验',{allowFault:true});
  assert(requests.slice(before).some(v=>v.request?.action==='restore_tuning'));
  assert.equal(last.autonomy.phase,'MANUAL');assert.equal(last.autonomy.tuning.previous_failure?.request_id,failedId);
  await page.locator('#live-settings').evaluate(el=>{el.open=false;});
  if(await page.locator('#manual-claim').isEnabled())await page.locator('#manual-claim').click();
  await page.waitForFunction(()=>!document.getElementById('enable').disabled,null,{timeout:12000});
  await page.locator('#enable').click();await page.waitForFunction(()=>latestControl.armed,null,{timeout:5000});
  await page.locator('#joystick').scrollIntoViewIfNeeded();const box=await page.locator('#joystick').boundingBox();assert(box);
  await page.mouse.move(box.x+box.width/2,box.y+box.height*.2);await page.mouse.down();
  await waitFor(s=>s.autonomy.output?.[0]>.005,4,'恢复参数后的真实网页摇杆输出');
  await page.mouse.up();await waitFor(stopped,5,'松杆后轮式停稳');
  if(process.env.CARCAR_TEST_FIELD_RESTRICTED==='1'){
   const haltId=await command('halt');await accepted(haltId);
   await waitFor(s=>s.autonomy?.phase==='PAUSED'&&stopped(s),10,'受限接管前明确停车');
   const restrictedId=await command('manual',{restricted:true});await accepted(restrictedId);
   await waitFor(s=>s.autonomy?.phase==='MANUAL'&&s.autonomy.restricted_manual?.ready&&stopped(s),15,'受限接管冻结确认',{allowFault:true});
   assert.equal(last.autonomy.restricted_manual.map_frozen,true);
   assert.equal(last.autonomy.actions.resume.enabled,false,'冻结后不能直接切回自动');
   const checkId=await command('recheck');await accepted(checkId);
   await waitFor(s=>s.autonomy?.phase==='PAUSED'&&s.autonomy.healthy&&s.autonomy.actions.resume.enabled&&stopped(s),65,'受限手动重新检查后恢复自动资格',{allowFault:true});
   result.restricted_recheck_passed=true;
  }
  result.parameter_failure_recovered=true;result.manual_after_restore=true;await capture('parameter-restored-manual');
 }
 mark('automatic-motion');const startId=await command('start');await accepted(startId);
 const origin=last.autonomy.wheel_position||[0,0];
 await waitFor(s=>s.autonomy?.phase==='EXPLORING'&&s.autonomy.output?.[0]>.005&&Array.isArray(s.autonomy.wheel_position)&&Math.hypot(s.autonomy.wheel_position[0]-origin[0],s.autonomy.wheel_position[1]-origin[1])>.15,45,'真实导航驱动模拟车平移');
 if(process.env.CARCAR_TEST_FIELD_RESTRICTED==='1'){
  // 此场景验收受限冻结、定位复验和恢复自动；自动墙的运动阶段由独立完整场景覆盖。
  result.scope='参数故障恢复→普通手动→停车受限接管→冻结定位复验→自动平移→停车保存';
  result.automatic_after_restricted=true;await capture('restricted-restored-automatic');
  const haltId=await command('halt');await accepted(haltId);await waitFor(s=>s.autonomy?.phase==='PAUSED'&&stopped(s),10,'受限恢复场景结束停车');
  await post('/api/finish');await waitFor(s=>s.task_state==='STANDBY'&&!s.busy,35,'受限恢复场景保存并正常结束');task='';
 }else{
 mark('online-tuning');await page.locator('#live-settings summary').click();const tuningStart=states.length;
 const globalId=await uiNumber('tuning-global_inflation_radius',.55,'set_tuning');
 await waitFor(s=>requestReady(s.autonomy.tuning,globalId)&&Math.abs(s.autonomy.tuning.confirmed?.global_inflation_radius-.55)<1e-6,20,'全局膨胀参数真实事务确认');
 const localId=await uiNumber('tuning-local_cost_scaling_factor',5.2,'set_tuning');
 await waitFor(s=>requestReady(s.autonomy.tuning,localId)&&Math.abs(s.autonomy.tuning.confirmed?.local_cost_scaling_factor-5.2)<1e-6,20,'局部衰减与RPP联动事务确认');
 assert(!states.slice(tuningStart).some(v=>v.state.autonomy?.phase==='STOPPING'),'在线调参意外切入STOPPING');
 result.parameter_readbacks=[await parameter('/global_costmap/global_costmap','inflation_layer.inflation_radius',.55),await parameter('/local_costmap/local_costmap','inflation_layer.cost_scaling_factor',5.2),await parameter('/controller_server','FollowPath.inflation_cost_scaling_factor',5.2)];
 const handoffId=await uiNumber('handoff-distance',.35,'set_handoff');await waitFor(s=>s.autonomy.handoff?.request_id===handoffId&&Math.abs(s.autonomy.handoff.distance_m-.35)<1e-6,10,'提前衔接距离确认');
 const saveId=await command('save_tuning');await waitFor(s=>s.autonomy.tuning?.defaults_saved_request_id===saveId,10,'保存新任务默认');result.saved_defaults=JSON.parse(await fs.readFile(defaultsFile,'utf8'));assert(!('walls' in result.saved_defaults));
 mark('automatic-wall-transaction');const mapBefore=await mapProbe('wall-maps-before');await snapshot();const wallGeneration=last.autonomy.walls.navigation_generation,restoreGeneration=last.autonomy.slam.restore_generation;
 const points=chooseWall(mapBefore,last.pose);result.wall_selection={points,pose:last.pose,navigation_generation:wallGeneration,raw:'已知空闲',local_before:0};
 const wall={id:'e2e-temporary-wall',points};
 await waitFor(s=>s.autonomy.phase==='EXPLORING'&&!s.autonomy.still&&!s.autonomy.navigation_terminal&&s.autonomy.output?.[0]>.005&&distanceToLine(s.pose,points)>.55,15,'确认真实平移中提交墙事务',{interval:50});
 const wallStart=states.length;const wallId=await command('set_walls',{walls:[wall],wall_revision:last.autonomy.walls.wall_revision??last.autonomy.walls.revision,navigation_generation:last.autonomy.walls.navigation_generation});
 await waitFor(s=>s.autonomy.walls?.request_id===wallId&&s.autonomy.walls.phase==='READY'&&!s.autonomy.walls.error&&s.autonomy.walls.items?.length===1,35,'真实墙应用与导航保护更新',{interval:50});
 assert(states.slice(wallStart).some(v=>['STOPPING','PAUSED'].includes(v.state.autonomy?.phase)&&v.state.autonomy.output?.every(x=>x===0)),'墙应用没有观测到真实停车屏障');
 await waitFor(s=>s.autonomy.phase==='EXPLORING',15,'墙应用后原任务继续');await capture('wall-applied');
 const mapAfter=await mapProbe('wall-maps-after');await snapshot();assert.equal(last.autonomy.slam.restore_generation,restoreGeneration,'墙比较期间SLAM恢复了地图坐标，拒绝直接比较');
 result.wall_map_proof=compareWall(mapBefore,mapAfter,points);await fs.writeFile(path.join(evidence,'wall-map-proof.json'),JSON.stringify(result.wall_map_proof,null,2));
 mark('human-stop-priority');await waitFor(s=>s.autonomy.phase==='EXPLORING'&&!s.autonomy.still&&!s.autonomy.navigation_terminal&&s.autonomy.output?.[0]>.005,15,'确认平移中启动可被人工停车抢占的墙事务',{interval:50});
 const pendingWall=await command('set_walls',{walls:[],wall_revision:last.autonomy.walls.wall_revision??last.autonomy.walls.revision,navigation_generation:last.autonomy.walls.navigation_generation});
 await waitFor(s=>s.autonomy.walls?.request_id===pendingWall&&s.autonomy.walls.pending&&['STOPPING','APPLYING'].includes(s.autonomy.walls.phase),4,'确认真实墙事务在途后才测试人工抢占',{interval:50});
 const haltId=await command('halt');await accepted(haltId);await waitFor(s=>s.autonomy.phase==='PAUSED'&&stopped(s),15,'人工停车抢占墙事务');
 const quiet=Date.now()+2500;while(Date.now()<quiet){const s=await snapshot();assert.equal(s.autonomy.phase,'PAUSED','人工停车后不应被墙回执自动恢复');assert(stopped(s));await pause(200);}result.stop_priority={wall_request:pendingWall,halt_request:haltId,confirmed:true};
 const clearId=await command('set_walls',{walls:[],wall_revision:last.autonomy.walls.wall_revision??last.autonomy.walls.revision,navigation_generation:last.autonomy.walls.navigation_generation});
 await waitFor(s=>s.autonomy.walls?.request_id===clearId&&s.autonomy.walls.phase==='READY'&&!s.autonomy.walls.error&&s.autonomy.walls.items?.length===0,25,'暂停中清除临时墙');assert.equal(last.autonomy.phase,'PAUSED');
 mark('return-save-close');const returnId=await command('return');await accepted(returnId);
 await waitFor(s=>s.autonomy?.phase==='REVIEW_REQUIRED'&&!s.dirty&&s.last_saved_bundle&&['succeeded','nearby'].includes(s.autonomy.return_result)&&stopped(s),90,'同任务返程、保存与停稳');result.first_candidate=last.last_saved_bundle;result.return_result=last.autonomy.return_result;await capture('returned-saved');
 const manifest=JSON.parse(await fs.readFile(path.join(mapRoot,last.last_saved_bundle,'manifest.json'),'utf8'));assert(!manifest.autonomy?.walls,'临时墙不能进入工程元数据');
 await post('/api/end');await waitFor(s=>s.task_state==='STANDBY'&&!s.busy,30,'结束第一轮软件任务');task='';
 mark('new-task-defaults-and-no-walls');await post('/api/start',{mode:'new',control_mode:'manual'});await waitFor(liveReady,70,'第二轮手动任务就绪');task=last.autonomy.task_id;assert.notEqual(task,result.first_task);result.second_task=task;
 assert.equal(last.autonomy.walls.items.length,0,'新任务继承了旧临时墙');assert(Math.abs(last.autonomy.tuning.confirmed.global_inflation_radius-.55)<1e-6);assert(Math.abs(last.autonomy.tuning.confirmed.local_cost_scaling_factor-5.2)<1e-6);assert(Math.abs(last.autonomy.handoff.distance_m-.35)<1e-6);
 result.second_parameter_readbacks=[await parameter('/global_costmap/global_costmap','inflation_layer.inflation_radius',.55),await parameter('/controller_server','FollowPath.inflation_cost_scaling_factor',5.2)];
 await verifySoftwareOnly();await capture('new-task-defaults');await post('/api/finish');await waitFor(s=>s.task_state==='STANDBY'&&!s.busy,35,'正常保存并结束第二轮软件任务');task='';
 }
 assert.equal(pageErrors.length,0,pageErrors.join('\n'));complete=true;result.passed=true;mark('completed');
}catch(error){result.passed=false;result.error=String(error);result.failed_stage=stage;process.exitCode=1;console.error('LIVE_WORKFLOW_FAILED '+JSON.stringify({stage,error:String(error)}));}
finally{
 if(page&&!complete){try{const s=await snapshot(true);if(s.task_state==='RUNNING'&&s.autonomy?.task_id){await command('halt',{},true);await waitFor(s=>s.task_state==='STANDBY'||stopped(s),20,'失败收尾软件停车',{ignoreBudget:true,allowFault:true});result.failure_stop_confirmed=true;}}catch(error){result.failure_stop_error=String(error);}await capture('failure').catch(()=>{});}
 result.requests=clean(requests);result.page_errors=pageErrors;result.finished_at=new Date().toISOString();result.elapsed_seconds=(Date.now()-started)/1000;result.final_state=clean(last);
 await fs.writeFile(path.join(evidence,'results.json'),JSON.stringify(clean(result),null,2));
 if(browser)await browser.close().catch(()=>{});
 // 仅终止本脚本创建的进程树；先让网关析构正常停止其软件后端。
 for(const {child,name} of [...children].reverse()){
  if(!child.pid)continue;const owned=await descendants(child.pid);try{process.kill(-child.pid,'SIGINT');}catch{}
  for(let i=0;i<100&&child.exitCode===null;i++)await pause(50);
  for(const pid of owned.reverse()){try{process.kill(pid,'SIGTERM');}catch{}}
  if(child.exitCode===null){try{process.kill(-child.pid,'SIGKILL');}catch{}}
 }
 await fs.rm(profile,{recursive:true,force:true});process.off('SIGINT',stopSignal);process.off('SIGTERM',stopSignal);
 console.log('LIVE_WORKFLOW_RESULT '+JSON.stringify({passed:result.passed,error:result.error,evidence}));
}
