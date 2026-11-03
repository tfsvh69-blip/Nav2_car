'use strict';
// 隔离D219的真实Chromium与真实网关/guard；Windows浏览器标识和桌面视口，不冒充原生Windows实机验收。
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
    if(!mobile)await this.call('Network.setUserAgentOverride',{userAgent:'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 Chrome/120.0 Safari/537.36'});
    if(mobile)await this.call('Network.setUserAgentOverride',{userAgent:'Mozilla/5.0 (Linux; Android 13) AppleWebKit/537.36 Chrome/120.0 Mobile Safari/537.36'});
    await this.call('Emulation.setDeviceMetricsOverride',{width:mobile?390:1280,height:mobile?844:900,deviceScaleFactor:1,mobile});
    // 默认用WebSocket，RTC与混合端的真实UDP另由transport套件验证。
    await this.call('Page.addScriptToEvaluateOnNewDocument',{source:'window.RTCPeerConnection=undefined;'});
    await this.call('Page.navigate',{url:base});
    await until(()=>this.read("typeof session==='string'&&!!session&&typeof operatorClientId==='string'&&!!operatorClientId&&['CURRENT','FAILED'].includes(state.phase)"),'页面会话/任务未就绪');
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

(async()=>{let browser;try{
  browser=await open(false);
  await until(()=>browser.read("state.autonomy?.phase==='RECOVERING'&&!document.getElementById('auto-pause').disabled"),'恢复期间未开放中止');
  assert.equal(await browser.read("document.getElementById('auto-pause').textContent"),'中止恢复');
  assert.match(await browser.read("document.getElementById('recovery-status').textContent"),/已验证 0\/6 帧/);
  assert.match(await browser.read("document.getElementById('recovery-status').textContent"),/多解竞争/);
  assert(await browser.read("!document.getElementById('recovery-progress').hidden&&document.getElementById('recovery-progress').getBoundingClientRect().height>0"),'恢复进度藏在折叠面板中');
  await until(()=>browser.read('!!displayMap&&!!mapImage'),'地图画面未加载');
  await browser.screenshot('windows-recovery-progress');checks.push('恢复阶段、验证帧数、多解原因和中止入口');
  await browser.click('auto-pause');
  await until(()=>browser.read("state.autonomy.phase==='FAULT'&&!autonomyPending&&!document.getElementById('auto-manual').disabled"),'中止恢复后未开放受限接管');
  assert.equal(await browser.read("document.getElementById('auto-manual').textContent"),'受限手动接管');
  assert(await browser.read("document.getElementById('auto-start').disabled&&!document.getElementById('auto-recheck').disabled"));
  await sleep(600);assert(await browser.read('state.scan_age<.5&&state.odom_age<.5'),'冻结期间只读观测停止更新');
  await browser.screenshot('windows-fault-actions');checks.push('冻结期间继续接收扫描和轮式，继续建图保持禁止');
  await browser.click('auto-manual');
  await until(()=>browser.read("state.autonomy.phase==='MANUAL'&&!autonomyPending&&!document.getElementById('enable').disabled"),'受限人工接管未形成真实资格');
  assert.match(await browser.read("document.getElementById('status-caption').textContent"),/地图冻结/);
  assert(await browser.read("document.getElementById('gear').disabled"));
  await browser.click('enable');await until(()=>browser.read('latestControl?.armed===true'),'受限遥控使能失败');
  await browser.pressStick();await until(()=>browser.read('Number(latestControl?.x)>0'),'按住摇杆没有通过guard');
  assert(await browser.read('Number(latestControl.x)<=.05'));
  await browser.releaseStick();await until(()=>browser.read('Number(latestControl?.x)===0'),'松手没有停车');
  await browser.screenshot('windows-restricted-manual');checks.push('受限低速、档位锁定、按住行驶及松手停车');
  await browser.pressStick();await until(()=>browser.read('Number(latestControl?.x)>0'),'第二次按住摇杆未产生速度');
  await browser.call('Page.navigate',{url:'about:blank'});await sleep(500);checks.push('页面离开断线，车端检查零输出');
  assert.equal(browser.exceptions.length,0,JSON.stringify(browser.exceptions));
  fs.writeFileSync(path.join(directory,'checks.json'),JSON.stringify({hardware:false,native_windows:false,real_chromium:true,checks},null,2));
}catch(error){if(browser){try{await browser.screenshot('failure');}catch(_){}fs.writeFileSync(path.join(directory,'failure.txt'),error.stack+'\n'+browser.errors);}throw error;}
finally{for(const client of clients)await client.close();}})().catch(error=>{console.error(error);process.exitCode=1;});
