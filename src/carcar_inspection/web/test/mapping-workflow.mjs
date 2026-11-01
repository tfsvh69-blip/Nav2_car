// 真实Chromium布局与交互验收；HTTP/控制消息为明确的测试夹具，不访问ROS或实车8080。
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import os from 'node:os';
import http from 'node:http';
import {spawn} from 'node:child_process';
import {fileURLToPath} from 'node:url';
import {chromium} from 'playwright-core';
const root=fileURLToPath(new URL('../../../../',import.meta.url));
const directory=path.resolve(process.argv[2]||path.join(root,'log/mapping-workflow'));
await fs.mkdir(directory,{recursive:true});
const profile=await fs.mkdtemp(path.join(os.tmpdir(),'carcar-workflow-browser-'));
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
let offline=false,browser,page,child,chromiumError='',errors=[],requests=[],dialogs=[],operation=0;
const bundles=[{id:'published-map',valid:true,publication:{status:'published'}},
  {id:'latest-candidate',valid:true,publication:{status:'candidate'}},
  {id:'diagnostic-map',valid:true,publication:{status:'diagnostic'}}];
const initial={phase:'STANDBY',task_state:'STANDBY',managed:true,busy:false,dirty:false,
  autonomy_enabled:true,autonomy_age_ms:10,dataflow_protocol:1,completed_operation:0,
  last_saved_bundle:'latest-candidate',control:{armed:false,health:''},navigation:{},
  scan_age:0,odom_age:0,map_age:0,autonomy:{phase:'REVIEW_REQUIRED',task_id:'previous-task',control_mode:'automatic'}};
let state=structuredClone(initial);
const automatic=phase=>({phase:'CURRENT',task_state:'RUNNING',busy:false,dirty:true,completed_operation:1,
  autonomy:{task_id:'workflow-fixture',phase,control_mode:'automatic',healthy:true,still:false,navigation_terminal:false,
    output:[.1,0],automatic_start_ready:true,origin:[0,0,0],remaining_seconds:2390,return_result:'not_started',
    speed:{requested_mps:.10,confirmed_mps:.10,cap_mps:.10,pending:false,phase:'READY',error:'',request_id:''}}});
const server=http.createServer(async(req,res)=>{
  try{
    const url=new URL(req.url,'http://localhost');
    const json=(value,status=200)=>{res.writeHead(status,{'Content-Type':'application/json'});res.end(JSON.stringify(value));};
    if(req.method==='POST'){
      let text='';for await(const chunk of req)text+=chunk;
      const body=JSON.parse(text);requests.push({path:url.pathname,body});
      if(url.pathname==='/api/start'){const id=++operation;await pause(350);state={...state,task_state:'STARTING',phase:'WAITING',busy:true};json({accepted:true,operation:id},202);return;}
      if(url.pathname==='/api/autonomy'){
        if(body.action==='set_speed'){
          state.autonomy.speed={...state.autonomy.speed,requested_mps:body.speed_mps,
            cap_mps:Math.min(state.autonomy.speed.cap_mps,body.speed_mps),pending:true,phase:'SETTING',request_id:body.request_id};
          setTimeout(()=>{state.autonomy.speed={...state.autonomy.speed,confirmed_mps:body.speed_mps,cap_mps:body.speed_mps,pending:false,phase:'READY'};},700);
          json({accepted:true},202);return;
        }
        state.autonomy={...state.autonomy,phase:'STOPPING',output:[0,0],request_id:body.request_id,request_result:'accepted'};
        // 状态先确认、HTTP迟到：迟到超时不能覆盖已确认的停车终态。
        await pause(5800);
        json({accepted:true},202);return;
      }
      if(url.pathname==='/api/end'){const id=++operation;state={...structuredClone(initial),completed_operation:id};json({accepted:true,operation:id},202);return;}
      if(url.pathname==='/api/maps/review'){bundles.find(b=>b.id===body.bundle).publication.status=body.approve?'published':'rejected';json({success:true});return;}
      json({error:'测试夹具未实现该操作'},404);return;
    }
    if(url.pathname==='/api/status'||url.pathname==='/api/state'){json(state,offline?503:200);return;}
    if(url.pathname==='/api/bundles'){json({bundles});return;}
    if(url.pathname==='/api/scan'){json({valid:false,version:'fixture'});return;}
    if(url.pathname.startsWith('/api/')){res.writeHead(204);res.end();return;}
    const filename={'/':'index.html','/app.js':'app.js','/style.css':'style.css','/rtc_control.js':'rtc_control.js'}[url.pathname];
    if(!filename){res.writeHead(404);res.end();return;}
    const content=await fs.readFile(path.join(root,'src/carcar_mapping_console/web',filename));
    res.writeHead(200,{'Content-Type':filename.endsWith('.js')?'application/javascript':filename.endsWith('.css')?'text/css':'text/html'});res.end(content);
  }catch(error){res.writeHead(500);res.end(error.message);}
});
await new Promise(resolve=>server.listen(0,'127.0.0.1',resolve));
const url=`http://127.0.0.1:${server.address().port}`;
try{
  child=spawn(path.join(root,'build/carcar_mapping_console/mapping_test_chromium'),['--headless=new','--no-sandbox','--disable-gpu',
    '--no-proxy-server','--disable-dev-shm-usage','--disable-background-networking','--disable-extensions','--disable-component-update',
    '--remote-debugging-port=0',`--user-data-dir=${profile}`,'about:blank'],{detached:true,env:{...process.env,ROS_DOMAIN_ID:'219',ROS_LOCALHOST_ONLY:'1'},stdio:['ignore','ignore','pipe']});
  child.stderr.on('data',data=>{chromiumError=(chromiumError+data).slice(-4000);});
  let port;for(let i=0;i<200&&!port;i++){try{port=(await fs.readFile(path.join(profile,'DevToolsActivePort'),'utf8')).split('\n')[0];}catch{await pause(50);}}
  assert(port,'隔离Chromium未启动：'+chromiumError);browser=await chromium.connectOverCDP(`http://127.0.0.1:${port}`);
  const context=await browser.newContext({viewport:{width:1366,height:900}});
  await context.addInitScript(()=>{
    window.WebSocket=class FixtureSocket{
      static OPEN=1;static CLOSED=3;
      constructor(){this.readyState=1;this.bufferedAmount=0;setTimeout(()=>this.onmessage?.({data:JSON.stringify({session:'fixture-controller'})}),20);
        this.timer=setInterval(()=>this.onmessage?.({data:JSON.stringify({armed:false,health:'',reason:'测试夹具保持停车',idle_heartbeat_required:false,control_lease_ms:300})}),100);}
      send(){}close(){clearInterval(this.timer);this.readyState=3;this.onclose?.({code:1000,wasClean:true});}
    };
  });
  page=await context.newPage();page.on('pageerror',error=>errors.push(error.message));page.on('dialog',dialog=>{dialogs.push(dialog.message());void dialog.accept();});
  await page.goto(url);
  await page.waitForFunction(()=>!document.getElementById('start').disabled&&document.getElementById('review-bundle').value==='latest-candidate');
  assert.equal(await page.locator('#manual-controls').isVisible(),false);
  assert.equal(await page.locator('#enable').isVisible(),false);
  assert(await page.locator('#map-activate').isDisabled(),'未发布候选不能突出启用入口');
  assert(await page.locator('#review-approve').isDisabled(),'未填写核对结论不能审核');
  await page.screenshot({path:path.join(directory,'desktop-standby.png'),fullPage:true});
  await page.selectOption('#control-mode','automatic');assert.equal(await page.locator('#site-confirmed').count(),0);
  const submitted=page.waitForResponse(response=>new URL(response.url()).pathname==='/api/start'&&response.status()===202);
  await page.click('#start');await page.evaluate(()=>document.getElementById('start').click());
  await page.waitForFunction(()=>document.getElementById('workflow-title').textContent.includes('启动'));
  assert.equal(requests.filter(r=>r.path==='/api/start').length,1);
  assert(await page.locator('#task-panel').isVisible());assert(await page.locator('#control-mode').isDisabled());
  await submitted;
  Object.assign(state,automatic('EXPLORING'));
  await page.waitForFunction(()=>document.getElementById('workflow-title').textContent==='自动探索中');
  assert.equal(await page.locator('#manual-controls').isVisible(),false);
  assert.equal(await page.locator('#auto-start').isVisible(),false,'自动探索中仍突出开始按钮');
  await page.locator('.task-details > summary').click();
  await page.waitForFunction(()=>!document.getElementById('auto-speed-apply').disabled);
  await page.locator('#auto-speed').evaluate(input=>{input.value='0.18';input.dispatchEvent(new Event('input'));});
  await page.click('#auto-speed-apply');
  assert(await page.locator('#auto-speed-apply').isDisabled());assert(await page.locator('#auto-pause').isEnabled(),'调速阻止暂停');
  assert(!(await page.locator('#auto-speed-status').textContent()).includes('调速已确认'),'HTTP受理被误报调速完成');
  await page.waitForFunction(()=>document.getElementById('auto-speed-status').textContent.includes('执行上限 0.18'));
  await page.setViewportSize({width:1024,height:768});await page.screenshot({path:path.join(directory,'tablet-exploring.png'),fullPage:true});
  await page.setViewportSize({width:390,height:844});
  const mobileLayout=[];
  for(const width of [375,390]){
    await page.setViewportSize({width,height:844});
    mobileLayout.push(await page.evaluate(()=>{
      const rect=selector=>{const r=document.querySelector(selector).getBoundingClientRect();return {left:r.left,right:r.right,width:r.width,height:r.height};};
      return {viewport:innerWidth,client_width:document.documentElement.clientWidth,scroll_width:document.documentElement.scrollWidth,stop:rect('#stop'),navigation:rect('.modes'),header:rect('.page-header')};
    }));
  }
  await fs.writeFile(path.join(directory,'mobile-layout.json'),JSON.stringify(mobileLayout,null,2));
  for(const layout of mobileLayout){
    assert(layout.stop.right<=layout.client_width-11,`停车按钮右侧留白不足：${JSON.stringify(layout)}`);
    assert(layout.navigation.right<=layout.client_width-11,`顶部导航右侧留白不足：${JSON.stringify(layout)}`);
    assert(layout.stop.height>=44&&layout.stop.width>=100,'停车点击区不完整');
  }
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1),'手机横向溢出');
  const operation=await page.locator('.operate').boundingBox(),map=await page.locator('.map-panel').boundingBox();
  assert(operation.y<map.y,'手机自动模式应先提供任务操作，再展示地图');
  await page.screenshot({path:path.join(directory,'phone-exploring.png'),fullPage:true});
  const runningAuto=structuredClone(state.autonomy);
  // 后台快照与导航分别展示；这些状态是明确的浏览器夹具，不代表真实车端返回。
  state.autonomy={...runningAuto,online_snapshots:true,checkpoint_usable:true,
    snapshot_maintenance:{phase:'WRITING'},recovery:{phase:'CHECKPOINT_READY',checkpoint_id:'fixture-valid-snapshot',checkpoint_age_seconds:31}};
  await page.waitForFunction(()=>document.getElementById('snapshot-status').textContent.includes('后台写入档案'));
  assert.equal(await page.locator('#workflow-title').textContent(),'自动探索中');
  assert(await page.locator('#auto-pause').isEnabled());
  state.autonomy.snapshot_maintenance={phase:'FAILED',reason:'浏览器夹具：模拟磁盘写入失败'};
  await page.waitForFunction(()=>document.getElementById('snapshot-status').textContent.includes('本次更新未成功'));
  assert((await page.locator('#snapshot-status').textContent()).includes('导航继续运行'));
  assert.equal(await page.locator('#workflow-title').textContent(),'自动探索中');
  assert(!(await page.locator('#auto-finish').textContent()).includes('故障'));
  await page.screenshot({path:path.join(directory,'phone-snapshot-warning.png'),fullPage:true});
  state.autonomy={...state.autonomy,phase:'PAUSED',checkpoint_usable:false,still:true,navigation_terminal:true,output:[0,0]};
  await page.waitForFunction(()=>document.getElementById('snapshot-status').textContent.includes('保持停车'));
  assert(!(await page.locator('#snapshot-status').textContent()).includes('导航继续运行'));
  state.autonomy=structuredClone(runningAuto);
  state.autonomy={...state.autonomy,phase:'STOPPING',stop_target:'CHECKPOINT',output:[0,0],
    recovery:{phase:'CHECKPOINTING',checkpoint_id:'fixture-stable-checkpoint',checkpoint_age_seconds:31,imu_ready:true,imu_stable:true,imu_age_seconds:.02}};
  await page.waitForFunction(()=>document.getElementById('workflow-title').textContent==='停稳保存检查点');
  state.autonomy={...state.autonomy,phase:'CHECKPOINTING',still:true,navigation_terminal:true};
  await page.waitForFunction(()=>document.getElementById('autonomy-stage').textContent==='停稳保存检查点');
  assert(await page.locator('#auto-speed-apply').isDisabled());assert(await page.locator('#stop').isEnabled());
  assert.equal(await page.locator('#auto-start').isVisible(),false);assert.equal(await page.locator('#auto-manual').isVisible(),false);
  await page.screenshot({path:path.join(directory,'phone-checkpoint.png'),fullPage:true});
  state.autonomy={...state.autonomy,phase:'RECOVERING',recovery_pending:true,
    recovery:{...state.autonomy.recovery,phase:'LOCALIZING',reason:'测试夹具：对照稳定地图重新定位'}};
  await page.waitForFunction(()=>document.getElementById('workflow-title').textContent==='正在恢复地图与定位');
  assert((await page.locator('#workflow-next').textContent()).includes('多帧核对'));
  assert(await page.locator('#stop').isEnabled());assert(await page.locator('#auto-speed-apply').isDisabled());
  await page.screenshot({path:path.join(directory,'phone-recovery.png'),fullPage:true});
  state.autonomy=runningAuto;
  await page.waitForFunction(()=>document.getElementById('workflow-title').textContent==='自动探索中'&&!document.getElementById('auto-pause').disabled);
  await page.click('#auto-pause');
  await page.waitForFunction(()=>document.getElementById('result').textContent.includes('等待停稳'));
  await pause(800);
  assert((await page.locator('#result').textContent()).includes('等待停稳'),'执行中反馈被后续轮询覆盖');
  state.autonomy={...state.autonomy,phase:'PAUSED',still:true,navigation_terminal:true,output:[0,0]};
  await page.waitForFunction(()=>document.getElementById('result').textContent.includes('暂停完成'));
  assert.equal(await page.locator('#auto-start').textContent(),'恢复自动探索');
  await pause(5500);assert((await page.locator('#result').textContent()).includes('暂停完成'),'迟到HTTP应答或超时覆盖了车端已确认终态');
  const posts=requests.length;offline=true;
  await page.waitForFunction(()=>document.getElementById('workflow-title').textContent==='状态更新中断');
  assert(await page.locator('#auto-start').isDisabled());assert(await page.locator('#stop').isEnabled());
  offline=false;await page.waitForFunction(()=>document.getElementById('workflow-title').textContent==='自动探索已暂停');
  assert.equal(requests.length,posts,'恢复连接不应重发操作');
  state={...state,dirty:false,last_saved_bundle:'latest-candidate',autonomy:{...state.autonomy,phase:'REVIEW_REQUIRED',healthy:false,return_result:'succeeded'}};
  await page.waitForFunction(()=>document.getElementById('auto-finish').textContent==='结束任务并查看候选');
  assert(!(await page.locator('#result').textContent()).includes('等待候选保存确认'),'保存终态仍显示正在等待保存');
  assert(!(await page.locator('#status-caption').textContent()).includes('独立运行'),'任务结束后仍提示自动任务继续');
  assert.equal(await page.locator('#save-actions').isVisible(),false,'候选已保存仍展示重复保存');
  await page.screenshot({path:path.join(directory,'phone-saved.png'),fullPage:true});
  await page.click('#auto-finish');
  await page.waitForFunction(()=>!document.getElementById('task-panel').hidden&&document.getElementById('review-bundle').value==='latest-candidate');
  await page.locator('#review-note').fill('测试夹具现场核对结论，不代表实物地图验收');
  assert(await page.locator('#review-approve').isEnabled());await page.click('#review-approve');
  await page.waitForFunction(()=>!document.getElementById('map-activate').disabled);
  assert.equal(await page.locator('#review-bundle').inputValue(),'latest-candidate','刷新地图列表丢失当前选择');
  await page.selectOption('#review-bundle','diagnostic-map');assert(await page.locator('#review-approve').isDisabled());assert(await page.locator('#map-activate').isDisabled());
  state={...state,...automatic('FAULT'),completed_operation:state.completed_operation,dirty:false,autonomy:{...automatic('FAULT').autonomy,
    healthy:false,still:true,navigation_terminal:true,output:[0,0],quality_fault:'测试夹具：扫描过期，锁停'}};
  await page.waitForFunction(()=>document.getElementById('auto-finish').textContent==='结束故障任务');
  await page.click('#auto-finish');await page.waitForFunction(()=>document.getElementById('workflow-title').textContent==='准备本轮建图');
  assert.deepEqual(errors,[]);assert.deepEqual(dialogs,[],'普通启动与已保存结束出现多余确认');
  const result={success:true,hardware:false,mocked_http_responses:true,mocked_control_messages:true,real_browser:true,
    checked:['startup-without-redundant-confirmation','startup-deduplication','startup-progress','runtime-speed-confirmed-state','speed-keeps-pause-available','mode-specific-controls','desktop-tablet-phone','mobile-no-overflow',
      'online-snapshot-navigation-status-separated','snapshot-failure-continues-valid-navigation','expired-snapshot-protection-pause',
      'checkpoint-hold-and-resume','recovery-stage-with-stop-available','request-acceptance-versus-stop-completion','late-http-cannot-override-confirmed-state','persistent-feedback','offline-recovery-no-replay','saved-next-step','candidate-selection-and-review-prerequisites','fault-exit-without-resume'],errors};
  await fs.writeFile(path.join(directory,'result.json'),JSON.stringify(result,null,2));console.log(JSON.stringify(result));
}catch(error){if(page)await page.screenshot({path:path.join(directory,'failure.png'),fullPage:true}).catch(()=>{});await fs.writeFile(path.join(directory,'failure.json'),JSON.stringify({error:error.message,state,requests,errors},null,2));throw error;}
finally{
  if(browser)await browser.close();if(child?.pid)try{process.kill(-child.pid,'SIGTERM');}catch{}
  await new Promise(resolve=>server.close(resolve));await fs.rm(profile,{recursive:true,force:true});
}
