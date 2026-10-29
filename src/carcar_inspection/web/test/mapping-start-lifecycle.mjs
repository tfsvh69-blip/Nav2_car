// 真实浏览器→待命网关→prepare→SLAM/Nav2/探索→保存/结束→再次启动。
// 仅隔离域203/localhost:18098，backend_hardware=false，地图写入独立临时目录。
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {spawn} from 'node:child_process';
import {once} from 'node:events';
import {chromium} from 'playwright-core';
const root=fileURLToPath(new URL('../../../../',import.meta.url));
const directory=path.resolve(process.argv[2]||path.join(root,'log/nav018-start-lifecycle'));
const url='http://127.0.0.1:18098';
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
await fs.mkdir(directory,{recursive:true});
const temporary=await fs.mkdtemp(path.join(os.tmpdir(),'carcar-mapping-lifecycle-'));
const profile=path.join(temporary,'browser');
const children=[],handles=[],events=[],rounds=[],errors=[];
let browser,page,lastState,interrupted=false,maxLinear=0,maxAngular=0;
const interrupt=()=>{interrupted=true;if(page)void page.close().catch(()=>{});};
process.on('SIGINT',interrupt);process.on('SIGTERM',interrupt);
function brief(s){return s&&{phase:s.phase,task_state:s.task_state,busy:s.busy,error:s.error,result:s.result,
  auto:s.autonomy&&{phase:s.autonomy.phase,error:s.autonomy.error,reason:s.autonomy.reason,health_reason:s.autonomy.health_reason,
    start_ready:s.autonomy.automatic_start_ready,start_reason:s.autonomy.automatic_start_reason,return_result:s.autonomy.return_result}};}
async function status(){lastState=await(await fetch(url+'/api/status',{signal:AbortSignal.timeout(2000)})).json();
  const output=lastState.autonomy?.output;if(Array.isArray(output)){maxLinear=Math.max(maxLinear,Math.abs(output[0]));maxAngular=Math.max(maxAngular,Math.abs(output[1]));}
  return lastState;
}
async function waitFor(predicate,description,seconds=30){
  const end=Date.now()+seconds*1000;
  while(Date.now()<end){if(interrupted)throw Error('操作者中断隔离测试');try{const s=await status();if(await predicate(s))return s;}catch{}await sleep(80);}
  throw Error(description+'：'+JSON.stringify(brief(lastState)));
}
async function launch(executable,args,logName,extraEnv={}){
  await fs.access(executable);
  const handle=await fs.open(path.join(directory,logName),'w');handles.push(handle);
  const child=spawn(executable,args,{cwd:root,detached:true,env:{...process.env,ROS_DOMAIN_ID:'203',ROS_LOCALHOST_ONLY:'1',...extraEnv},stdio:['ignore',handle.fd,handle.fd]});
  children.push(child);await once(child,'spawn');return child;
}
try{
  let occupied=false;try{await fetch(url+'/api/status',{signal:AbortSignal.timeout(500)});occupied=true;}catch{}
  assert(!occupied,'18098已有入口；不覆盖、不终止未知进程');
  await launch(path.join(root,'build/carcar_mapping_autonomy/mapping_virtual_room'),[],'room.log',
    {CARCAR_ROOM_EXTERNAL:'1',CARCAR_ROOM_MANAGED:'1'});
  await launch(path.join(root,'install/carcar_mapping_console/lib/carcar_mapping_console/mapping_web_gateway'),
    ['--ros-args','-p','mode:=standby','-p','backend_hardware:=false','-p','autonomy_enabled:=true',
      '-p','bind_address:=127.0.0.1','-p','port:=18098','-p',`map_root:=${temporary}/maps`,'-p',`log_root:=${directory}/tasks`],'gateway.log');
  await waitFor(s=>s.task_state==='STANDBY','真实待命网关未启动');
  const runtime=path.join(root,'build/carcar_mapping_console/mapping_test_chromium');
  const browserChild=await launch(runtime,['--headless=new','--no-sandbox','--disable-gpu','--no-proxy-server','--disable-dev-shm-usage',
    '--no-first-run','--disable-background-networking','--disable-extensions','--disable-component-update','--remote-debugging-port=0',
    `--user-data-dir=${profile}`,'about:blank'],'chromium.log',{ROS_DOMAIN_ID:'219'});
  let port;for(let i=0;i<200&&!port;i++){try{port=(await fs.readFile(path.join(profile,'DevToolsActivePort'),'utf8')).split('\n')[0];}catch{await sleep(50);}}
  assert(port,`Chromium未启动，pid=${browserChild.pid}`);browser=await chromium.connectOverCDP(`http://127.0.0.1:${port}`);
  const context=await browser.newContext({viewport:{width:1024,height:850}});page=await context.newPage();
  page.on('pageerror',e=>errors.push(e.message));page.on('dialog',d=>d.accept());
  const starts=[];page.on('request',r=>{if(r.method()==='POST'&&r.url()===url+'/api/start')starts.push(r.postDataJSON());});
  page.on('response',r=>{if(r.request().method()==='POST')events.push({path:new URL(r.url()).pathname,status:r.status()});});
  await page.goto(url);
  for(let round=1;round<=2;round++){
    await page.waitForFunction(()=>!document.getElementById('start').disabled);
    await page.selectOption('#mode','new');await page.selectOption('#control-mode','automatic');
    assert(await page.locator('#task-panel #site-confirmed').isVisible(),'启动条件未出现在启动区');
    assert.equal(await page.locator('#site-confirmed').isChecked(),false,'新一轮沿用旧场地确认');
    const prior=starts.length;await page.click('#start');await sleep(100);assert.equal(starts.length,prior,'未确认场地却提交启动');
    await page.check('#site-confirmed');await page.click('#start');
    await waitFor(()=>starts.length===prior+1,'网页未向真实后端提交启动');
    const duplicate=await page.evaluate(async body=>{const r=await fetch('/api/start',{method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},body:JSON.stringify(body)});return r.status;},starts.at(-1));
    assert.equal(duplicate,409,'重复启动没有被真实后端拒绝');
    let s=await waitFor(s=>s.task_state==='RUNNING'&&s.autonomy?.phase==='EXPLORING','真实后台未进入自动探索',40);
    const task=s.autonomy.task_id;assert(!rounds.some(r=>r.task_id===task),'第二轮复用旧任务');
    const firstWheel=s.autonomy.wheel_position;assert(Array.isArray(firstWheel),'缺少独立轮式位移反馈');
    s=await waitFor(s=>s.autonomy?.accepted_sequence>=3&&Array.isArray(s.autonomy.wheel_position)&&
      Math.hypot(s.autonomy.wheel_position[0]-firstWheel[0],s.autonomy.wheel_position[1]-firstWheel[1])>=.15,
      '探索没有实际平移；静止补帧序号不能代替运动闭环',30);
    const observedTravel=Math.hypot(s.autonomy.wheel_position[0]-firstWheel[0],s.autonomy.wheel_position[1]-firstWheel[1]);
    if(round===1){await page.reload();await sleep(500);assert.equal((await status()).autonomy.task_id,task);}
    await page.locator('#auto-pause').click();
    s=await waitFor(s=>s.autonomy?.phase==='PAUSED'&&s.autonomy.still&&s.autonomy.navigation_terminal,'暂停未完成取消和停稳');
    assert.deepEqual(s.autonomy.output,[0,0]);
    await page.locator('#auto-manual').click();await waitFor(s=>s.autonomy?.phase==='MANUAL','未进入手动接管');
    await page.check('#site-confirmed');await page.locator('#auto-start').click();
    await waitFor(s=>s.autonomy?.phase==='EXPLORING','接管后恢复自动失败');
    await page.locator('#auto-finish').click();
    s=await waitFor(s=>s.autonomy?.phase==='REVIEW_REQUIRED'&&!!s.last_saved_bundle,'返程保存未完成',70);
    assert.equal(s.autonomy.return_result,'succeeded','软件房间返程失败');assert.deepEqual(s.autonomy.output,[0,0]);
    const manifest=JSON.parse(await fs.readFile(path.join(temporary,'maps',s.last_saved_bundle,'manifest.json'),'utf8'));
    assert(manifest.complete,'保存工程不完整');
    const publication=JSON.parse(await fs.readFile(path.join(temporary,'maps',s.last_saved_bundle,'publication.json'),'utf8'));
    assert.equal(publication.status,'candidate');
    await page.waitForFunction(()=>{
      const finish=document.getElementById('auto-finish');
      return finish.textContent==='结束任务并查看候选'&&!finish.disabled&&
        document.getElementById('auto-start').hidden&&document.getElementById('save').hidden&&
        document.getElementById('task-summary').textContent.includes('本轮探索已结束');
    },null,{timeout:10000});
    await page.screenshot({path:path.join(directory,`round-${round}-saved.png`),fullPage:true});
    rounds.push({task_id:task,candidate:s.last_saved_bundle,return_result:s.autonomy.return_result,manifest_complete:manifest.complete,observed_exploration_displacement:observedTravel});
    await page.locator('#auto-finish').click();await waitFor(s=>s.task_state==='STANDBY'&&!s.busy,'结束后未回待命');
    await page.waitForFunction(()=>!document.getElementById('task-panel').hidden);
    // 页面刷新后下拉框安全地回到默认手动；新一轮显式选择自动再检查对应条件。
    await page.selectOption('#control-mode','automatic');
    assert(await page.locator('#task-panel #site-confirmed').isVisible(),'结束后旧终态隐藏下一轮确认');
    assert.equal(await page.locator('#site-confirmed').isChecked(),false);
    console.log(`ROUND ${round} passed`);
  }
  assert.deepEqual(errors,[]);assert(maxLinear<=.100001&&maxAngular<=.360001,'软件最终速度超出原运动包络');
  assert.equal(events.filter(e=>e.path==='/api/start'&&e.status===202).length,2);
  const result={success:true,hardware:false,real_browser:true,real_backend:true,mocked_http_responses:false,
    rounds,events,errors,max_observed_linear:maxLinear,max_observed_angular:maxAngular,map_root:path.join(temporary,'maps'),final:brief(await status())};
  await fs.writeFile(path.join(directory,'result.json'),JSON.stringify(result,null,2));console.log(JSON.stringify(result));
}catch(error){
  if(page)await page.screenshot({path:path.join(directory,'failure.png'),fullPage:true}).catch(()=>{});
  await fs.writeFile(path.join(directory,'failure.json'),JSON.stringify({error:error.message,state:brief(lastState),events,errors},null,2));
  throw error;
}finally{
  if(browser)await browser.close().catch(()=>{});
  for(const child of children.reverse()){
    if(child.exitCode!==null||!child.pid)continue;
    try{process.kill(-child.pid,'SIGINT');}catch{}
    for(let i=0;i<30&&child.exitCode===null;i++)await sleep(100);
    if(child.exitCode===null)try{process.kill(-child.pid,'SIGKILL');}catch{}
  }
  for(const handle of handles)await handle.close();
  await fs.rm(profile,{recursive:true,force:true});
  process.off('SIGINT',interrupt);process.off('SIGTERM',interrupt);
}
