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
const started=Date.now(),deadline=started+180000,states=[],requests=[],pageErrors=[],children=[],stages=[];
let browser,page,last={},task='',sequence=0,aborted=false,complete=false,stage='preflight';
const sourceHash=async file=>createHash('sha256').update(await fs.readFile(file)).digest('hex');
const result={script_sha256:await sourceHash(fileURLToPath(import.meta.url)),app_sha256:await sourceHash(path.join(root,'src/carcar_mapping_console/web/app.js')),hardware:false,source:'真实网页/参数服务/Nav2/SLAM；扫描、IMU、轮式与运动为模拟输入',domain:227,port:18099,limit_seconds:180,stages};
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
 launch(path.join(root,'build/carcar_mapping_autonomy/mapping_virtual_room'),[],'virtual-room',{CARCAR_ROOM_CONTINUOUS:'1',CARCAR_ROOM_MANAGED:'1',CARCAR_ROOM_EXTERNAL:'1',CARCAR_ROOM_MAX_SECONDS:'180'});
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

 await page.waitForFunction(()=>navImages.global?.bitmap&&navImages.keepout?.bitmap&&keepoutAligned(),null,{timeout:20000});
 const proof=await page.evaluate(()=>{
   const image=navImages.global,canvas=document.createElement('canvas');canvas.width=image.meta.width;canvas.height=image.meta.height;
   const ctx=canvas.getContext('2d');ctx.drawImage(image.bitmap,0,0);const data=ctx.getImageData(0,0,canvas.width,canvas.height).data,colors=new Set();
   for(let i=0;i<data.length;i+=4)if(data[i+3]&&data[i+1]===0&&data[i]!==255)colors.add([data[i],data[i+1],data[i+2]].join(','));
   return {soft_levels:colors.size,meta:image.meta,keepout_meta:navImages.keepout.meta};
 });
 assert(proof.soft_levels>=2,'合成前真实数据缺少膨胀梯度');result.layer_proof=proof;
 await page.locator('details.legend summary').click();
 assert.equal(await page.locator('#layer-keepout').isChecked(),false,'禁入默认应隐藏');
 await capture('keepout-hidden');const count=requests.length;
 await page.locator('#layer-keepout').check();await pause(400);await capture('keepout-shown');
 assert.equal(requests.length,count,'显示开关不能发送控制请求');
 await page.locator('#layer-keepout').uncheck();await pause(200);assert.equal(requests.length,count);
 // 手机尺寸下仍能独立操作，不触发地图选点或控制按钮。
 await page.setViewportSize({width:390,height:844});await page.locator('#layer-keepout').check();await capture('mobile-keepout');
 assert.equal(requests.length,count);await page.setViewportSize({width:1440,height:1000});
 await post('/api/finish');await waitFor(s=>s.task_state==='STANDBY'&&!s.busy,35,'软件任务保存结束');task='';
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
