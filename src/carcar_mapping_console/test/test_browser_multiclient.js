'use strict';
// 仅连接D219/18089隔离夹具：两个真实Chromium、真实页面/HTTP/WS和真实guard，无实车硬件。
const assert=require('assert'),fs=require('fs'),os=require('os'),path=require('path'),{spawn}=require('child_process');
assert.equal(process.env.ROS_DOMAIN_ID,'219');assert.equal(Number(process.argv[2]),18089);
const chromeExe=process.argv[3],directory=process.argv[4],base='http://127.0.0.1:18089';
fs.mkdirSync(directory,{recursive:true});
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const clients=[],checks=[];
async function until(test,label,ms=12000){const end=Date.now()+ms;while(Date.now()<end){if(await test())return;await sleep(20);}throw Error(label);}
class Browser {
  async start(mobile=false){
    this.profile=fs.mkdtempSync(path.join(os.tmpdir(),'carcar-dual-browser-'));this.pending=new Map();this.command=0;this.exceptions=[];this.errors='';
    this.child=spawn(chromeExe,['--headless=new','--no-sandbox','--disable-gpu','--no-proxy-server','--disable-dev-shm-usage',
      '--no-first-run','--disable-background-networking','--disable-extensions','--disable-component-update','--remote-debugging-port=0',
      '--user-data-dir='+this.profile,'--window-size=1280,900','about:blank'],
      {detached:true,env:{...process.env,XDG_CONFIG_HOME:path.join(this.profile,'config'),XDG_CACHE_HOME:path.join(this.profile,'cache'),
        LD_LIBRARY_PATH:process.env.MAPPING_TEST_CHROME_LIBS||process.env.LD_LIBRARY_PATH||''},stdio:['ignore','ignore','pipe']});
    this.child.stderr.on('data',data=>{this.errors=(this.errors+data).slice(-4000);});
    const file=path.join(this.profile,'DevToolsActivePort');await until(()=>fs.existsSync(file),'Chromium启动失败');
    const port=fs.readFileSync(file,'utf8').split('\n')[0],pages=await(await fetch(`http://127.0.0.1:${port}/json/list`)).json();
    this.socket=new WebSocket(pages.find(page=>page.type==='page').webSocketDebuggerUrl.replace('ws://localhost:','ws://127.0.0.1:'));
    await new Promise((resolve,reject)=>{this.socket.onopen=resolve;this.socket.onerror=reject;});
    this.socket.onmessage=event=>{const value=JSON.parse(event.data);if(value.method==='Runtime.exceptionThrown')this.exceptions.push(value.params.exceptionDetails);
      const pending=this.pending.get(value.id);if(!pending)return;clearTimeout(pending.timer);this.pending.delete(value.id);
      value.error?pending.reject(Error(value.error.message)):pending.resolve(value.result);};
    await this.call('Runtime.enable');await this.call('Page.enable');await this.call('Network.enable');
    if(mobile)await this.call('Network.setUserAgentOverride',{userAgent:'Mozilla/5.0 (Linux; Android 13) AppleWebKit/537.36 Chrome/120.0 Mobile Safari/537.36'});
    await this.call('Emulation.setDeviceMetricsOverride',{width:mobile?390:1280,height:mobile?844:900,deviceScaleFactor:1,mobile});
    // 默认用WebSocket，RTC与混合端的真实UDP另由transport套件验证。
    await this.call('Page.addScriptToEvaluateOnNewDocument',{source:'window.RTCPeerConnection=undefined;'});
    await this.call('Page.navigate',{url:base});
    await until(()=>this.read("typeof session==='string'&&!!session&&typeof operatorClientId==='string'&&!!operatorClientId&&state.phase==='CURRENT'"),'页面会话/任务未就绪');
    return this;
  }
  call(method,params={}){return new Promise((resolve,reject)=>{const id=++this.command,timer=setTimeout(()=>{this.pending.delete(id);reject(Error('CDP超时 '+method));},10000);
    this.pending.set(id,{resolve,reject,timer});this.socket.send(JSON.stringify({id,method,params}));});}
  async read(expression){const result=await this.call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
    if(result.exceptionDetails)throw Error(result.exceptionDetails.text+' '+(result.exceptionDetails.exception?.description||''));return result.result.value;}
  async click(id){await this.read(`document.getElementById(${JSON.stringify(id)}).click()`);}
  async pressStick(){await this.read("document.getElementById('joystick').scrollIntoView({block:'center'})");
    const p=await this.read("(()=>{const r=document.getElementById('joystick').getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height*.2};})()");
    await this.call('Input.dispatchMouseEvent',{type:'mouseMoved',...p});await this.call('Input.dispatchMouseEvent',{type:'mousePressed',button:'left',buttons:1,clickCount:1,...p});this.stick=p;}
  async releaseStick(){if(this.stick)await this.call('Input.dispatchMouseEvent',{type:'mouseReleased',button:'left',buttons:0,clickCount:1,...this.stick});this.stick=null;}
  async screenshot(name){
    // 从文档顶部截整页，避免滚动中的sticky栏被CDP拼到图片中部。
    await this.read("new Promise(resolve=>{document.activeElement?.blur();window.scrollTo(0,0);requestAnimationFrame(()=>requestAnimationFrame(resolve));})");
    const metrics=await this.call('Page.getLayoutMetrics'),size=metrics.cssContentSize||metrics.contentSize;
    const result=await this.call('Page.captureScreenshot',{format:'png',captureBeyondViewport:true,
      clip:{x:0,y:0,width:size.width,height:size.height,scale:1}});
    fs.writeFileSync(path.join(directory,name+'.png'),Buffer.from(result.data,'base64'));
  }
  async close(){if(this.socket?.readyState===WebSocket.OPEN){try{await this.call('Browser.close');}catch(_){}this.socket.close();}
    if(this.child?.pid){try{process.kill(-this.child.pid,'SIGTERM');}catch(_){}await sleep(80);try{process.kill(-this.child.pid,'SIGKILL');}catch(_){}}
    for(const pending of this.pending?.values()||[])clearTimeout(pending.timer);
    if(this.profile)fs.rmSync(this.profile,{recursive:true,force:true,maxRetries:5,retryDelay:100});}
}
async function open(mobile){const browser=new Browser();clients.push(browser);return browser.start(mobile);}
async function both(first,second,expression,label){await until(async()=>await first.read(expression)&&await second.read(expression),label);}
async function snapshot(browser){return browser.read("({phase:state.phase,autonomy:state.autonomy?.phase,manual:state.manual_control,operations:state.task_operations,pending:autonomyPending,notice:autonomyNotice,client_id:operatorClientId,errors:clientError})");}
(async()=>{let desktop,phone;try{
  desktop=await open(false);phone=await open(true);
  await both(desktop,phone,"state.manual_control.connected_clients===2&&!document.getElementById('auto-start').disabled",'两端未同时开放继续');
  await Promise.all([desktop.click('auto-start'),phone.click('auto-start')]);
  await both(desktop,phone,"state.autonomy.phase==='EXPLORING'&&!autonomyPending",'双端继续没有按本页请求确认');
  const ids=await Promise.all([desktop.read('autonomyAction.canonical_request_id'),phone.read('autonomyAction.canonical_request_id')]);
  assert(ids[0]);assert.equal(ids[0],ids[1],'双端继续没有合并同一车端请求');checks.push('两端继续可用、同一次启动、两端确认');
  await desktop.click('auto-pause');await both(desktop,phone,"state.autonomy.phase==='PAUSED'",'暂停没有同步到两端');
  await desktop.click('auto-start');const queuedId=await desktop.read('autonomyAction.request_id');await phone.click('stop');
  await until(()=>desktop.read(`state.task_operations.requests.some(r=>r.request_id===${JSON.stringify(queuedId)}&&r.request_result==='cancelled')`),'停车没有作废排队继续');
  await both(desktop,phone,"state.autonomy.phase==='PAUSED'&&!autonomyPending",'停车后仍等待旧继续');checks.push('另一端停车优先，旧继续取消且不重放');
  await desktop.click('auto-manual');
  await both(desktop,phone,"state.autonomy.phase==='MANUAL'",'自动任务未先转入手动');
  await until(()=>desktop.read("ownsManualControl()&&!document.getElementById('enable').disabled"),'电脑未取得手动所有权');
  assert(await phone.read("!ownsManualControl()&&document.getElementById('enable').disabled"));
  await desktop.click('auto-start');const manualQueuedId=await desktop.read('autonomyAction.request_id');await phone.click('stop');
  await until(()=>desktop.read(`state.task_operations.requests.some(r=>r.request_id===${JSON.stringify(manualQueuedId)}&&r.request_result==='cancelled')&&!autonomyPending`),'手动模式停车没有作废排队继续');
  await both(desktop,phone,"state.autonomy.phase==='MANUAL'&&state.autonomy.still",'手动模式停车后错误启动了自动任务');
  checks.push('MANUAL排队继续被另一端停车作废，未交给车端');
  await desktop.click('enable');await until(()=>desktop.read('latestControl.armed'),'电脑使能失败');
  await desktop.pressStick();await until(()=>desktop.read('latestControl.x>.02'),'电脑摇杆未产生合成输出');
  await phone.pressStick();assert.equal(await phone.read('!!pressed'),false,'手机借电脑armed状态推杆');
  // 绕过前端stop清理函数，验证后端也拒绝旧页面发出的非持有者自动停车。
  await phone.read("send({action:'stop',cause:'window_blur'})");await sleep(180);
  assert(await desktop.read('latestControl.armed&&latestControl.x>.02'),'后端把非持有端window_blur当成明确停车');
  await phone.read("window.dispatchEvent(new Event('blur'))");await phone.close();
  await sleep(350);assert(await desktop.read('latestControl.armed&&latestControl.x>.02'),'非持有端断开停止了电脑推杆');
  await desktop.releaseStick();await until(()=>desktop.read('latestControl.armed&&latestControl.x===0'),'电脑松手未归零');checks.push('单端摇杆、非持有端失焦/断开不干扰');
  phone=await open(true);await until(()=>phone.read("!document.getElementById('manual-claim').disabled"),'手机接管入口不可用');
  await phone.read("document.getElementById('manual-claim').onclick()");
  await until(()=>phone.read("state.manual_control.phase==='STOPPING'"),'接管未提供停车确认阶段');
  const denied=await desktop.read(`(async()=>{let s;const until=performance.now()+2000;
    do{s=await(await fetch('/api/status',{cache:'no-store'})).json();if(s.manual_control.phase==='STOPPING')break;
      await new Promise(resolve=>setTimeout(resolve,10));}while(performance.now()<until);
    const r=await fetch('/api/autonomy',{
    method:'POST',headers:{'Content-Type':'application/json','X-Mapping-Console':'1'},body:JSON.stringify({session,
      action:'resume',request_id:'handover-block-'+Date.now(),task_id:s.autonomy.task_id,operation_epoch:s.task_operations.epoch})});
    return {status:r.status,body:await r.json(),phase:s.manual_control.phase};})()`);
  assert.equal(denied.phase,'STOPPING');assert.equal(denied.status,409,'持有最新epoch仍穿透接管STOPPING屏障');
  assert.match(denied.body.error,/接管|交接/);checks.push('接管STOPPING期间即便最新epoch也拒绝继续');
  await until(()=>phone.read('ownsManualControl()&&state.manual_control.ready'),'手机接管未完成停稳/绑定确认');
  assert(await desktop.read('!ownsManualControl()'),'电脑仍持有旧所有权');
  await until(()=>phone.read("!document.getElementById('enable').disabled"),'手机接管后使能入口未就绪');
  assert.equal(await phone.read('latestControl.armed'),false,'接管直接使能，未等待显式启用');
  await phone.click('enable');await until(()=>phone.read('latestControl.armed'),'手机使能失败');
  await phone.pressStick();await until(()=>phone.read('latestControl.x>.02'),'手机摇杆未产生合成输出');
  await desktop.click('stop');await both(desktop,phone,'!latestControl.armed&&!pressed&&latestControl.x===0','非持有端明确停车没有停止手机');
  checks.push('手机接管先停车，显式使能后推杆，电脑仍可停车');
  await desktop.screenshot('desktop');await phone.screenshot('phone');
  assert(await phone.read('document.documentElement.scrollWidth<=document.documentElement.clientWidth+1'),'手机页面横向溢出');
  for(const client of [desktop,phone])assert.deepEqual(client.exceptions,[],'真实页面有脚本异常');
  fs.writeFileSync(path.join(directory,'result.json'),JSON.stringify({success:true,hardware:false,domain:219,checks,
    desktop:await snapshot(desktop),phone:await snapshot(phone)},null,2));
  console.log('DUAL_BROWSER_RESULT '+JSON.stringify({success:true,hardware:false,checks}));
}catch(error){
  const failure={success:false,error:error.stack,checks};
  for(const [name,client] of [['desktop',desktop],['phone',phone]])if(client){try{failure[name]=await snapshot(client);await client.screenshot(name+'-failure');}catch(_){}failure[name+'_exceptions']=client.exceptions;}
  fs.writeFileSync(path.join(directory,'result.json'),JSON.stringify(failure,null,2));throw error;
}finally{for(const client of clients)await client.close();}})().catch(error=>{console.error(error.stack);process.exitCode=1;});
