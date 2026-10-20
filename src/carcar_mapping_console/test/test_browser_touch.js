'use strict';
// 可选实浏览器验收：由隔离C++夹具提供合成数据，真实DOM/CSS与触屏事件。
// MAPPING_TEST_CHROME仅指向独立浏览器副本，不使用用户配置，不连接真实车辆。
const assert=require('assert'),fs=require('fs'),os=require('os'),path=require('path'),{spawn}=require('child_process');
assert.equal(process.env.ROS_DOMAIN_ID,'219');assert.equal(Number(process.argv[3]),18089);
const latency=Number(process.argv[4]||0),profile=fs.mkdtempSync(path.join(os.tmpdir(),'carcar-touch-profile-'));
const chrome=spawn(process.env.MAPPING_TEST_CHROME,['--headless=new','--no-sandbox','--disable-gpu',
  '--no-proxy-server',
  '--disable-dev-shm-usage','--no-first-run','--disable-background-networking','--disable-extensions',
  '--disable-component-update','--remote-debugging-port=0','--user-data-dir='+profile,'--window-size=1280,800','about:blank'],
  {detached:true,env:{...process.env,XDG_CONFIG_HOME:path.join(profile,'config'),XDG_CACHE_HOME:path.join(profile,'cache'),
    LD_LIBRARY_PATH:process.env.MAPPING_TEST_CHROME_LIBS||process.env.LD_LIBRARY_PATH||''},stdio:['ignore','ignore','pipe']});
let errors='',socket,command=0;const pending=new Map(),exceptions=[];
chrome.stderr.on('data',data=>{errors=(errors+data).slice(-6000);});
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function until(test,label,ms=5000){const end=Date.now()+ms;while(Date.now()<end){if(await test())return;await sleep(10);}throw Error(label);}
function call(method,params={}){return new Promise((resolve,reject)=>{const id=++command;
  const timer=setTimeout(()=>{pending.delete(id);reject(Error('CDP超时 '+method));},5000);
  pending.set(id,{resolve,reject,timer});socket.send(JSON.stringify({id,method,params}));});}
async function evaluate(expression){const r=await call('Runtime.evaluate',{expression,returnByValue:true});if(r.exceptionDetails)throw Error(r.exceptionDetails.text);return r.result.value;}
async function point(id){return evaluate(`(()=>{const r=document.getElementById(${JSON.stringify(id)}).getBoundingClientRect();return {x:r.x+r.width/2,y:r.y+r.height/2,top:r.top,width:r.width};})()`);}
async function touch(type,x,y){await call('Input.dispatchTouchEvent',{type,touchPoints:type==='touchEnd'?[]:[{x,y,id:1,radiusX:8,radiusY:8,force:1}]});}
async function pressStick(){const p=await point('joystick');await touch('touchStart',p.x,p.y);
  for(let i=1;i<=8;i++){await touch('touchMove',p.x,p.y-(p.width/2-28)*i/8);await sleep(20);}
  await until(()=>evaluate('latestControl.armed&&latestControl.x>.04'),'真实触屏拖动没有运动输出');}
async function stayArmed(ms){const end=Date.now()+ms;while(Date.now()<end){assert(await evaluate('latestControl.armed'),'真实触屏持续按住被撤销使能：'+await evaluate('latestControl.reason'));await sleep(30);}}
(async()=>{try{
  const file=path.join(profile,'DevToolsActivePort');await until(()=>fs.existsSync(file),'Chromium启动失败');
  const port=fs.readFileSync(file,'utf8').split('\n')[0],pages=await(await fetch(`http://127.0.0.1:${port}/json/list`)).json();
  socket=new WebSocket(pages.find(p=>p.type==='page').webSocketDebuggerUrl.replace('ws://localhost:','ws://127.0.0.1:'));
  await new Promise((resolve,reject)=>{socket.onopen=resolve;socket.onerror=reject;});
  socket.onmessage=event=>{const v=JSON.parse(event.data);if(v.method==='Runtime.exceptionThrown')exceptions.push(v.params.exceptionDetails);
    if(!v.id)return;const p=pending.get(v.id);if(!p)return;clearTimeout(p.timer);pending.delete(v.id);v.error?p.reject(Error(v.error.message)):p.resolve(v.result);};
  await call('Runtime.enable');await call('Page.enable');await call('Network.enable');
  await call('Emulation.setDeviceMetricsOverride',{width:1280,height:800,deviceScaleFactor:1,mobile:false});
  await call('Emulation.setTouchEmulationEnabled',{enabled:true,maxTouchPoints:5});
  await call('Page.addScriptToEvaluateOnNewDocument',{source:`(()=>{const Native=WebSocket;class DelayedSocket{
    static OPEN=1;static CLOSED=3;constructor(url){this.native=new Native(url);this.native.onmessage=e=>setTimeout(()=>this.onmessage?.({data:e.data}),${latency});
    this.native.onclose=e=>this.onclose?.(e);this.native.onerror=e=>this.onerror?.(e);}
    get readyState(){return this.native.readyState;}get bufferedAmount(){return this.native.bufferedAmount;}
    send(data){setTimeout(()=>{if(this.native.readyState===1)this.native.send(data);},${latency});}close(){this.native.close();}}
    window.WebSocket=DelayedSocket;
    const NativePeer=window.RTCPeerConnection;window.carcarControlTrace=[];
    const trace=value=>{window.carcarControlTrace.push({at:performance.now(),...value});if(window.carcarControlTrace.length>512)window.carcarControlTrace.shift();};
    if(NativePeer)window.RTCPeerConnection=class extends NativePeer{
      createDataChannel(label,options){const channel=super.createDataChannel(label,options),send=channel.send.bind(channel),revisions=new Map();let message;
        Object.defineProperty(channel,'onmessage',{get:()=>message,set:value=>{message=value;}});
        channel.addEventListener('message',event=>{
          const value=JSON.parse(event.data);revisions.set(value.challenge,value.control_revision);if(revisions.size>32)revisions.delete(revisions.keys().next().value);
          trace({event:'received',revision:value.control_revision,armed:value.armed});
          if(window.carcarDropRtcUntil>performance.now())return;
          const extra=window.carcarDelayNextRtc||0;window.carcarDelayNextRtc=0;
          setTimeout(()=>{trace({event:'delivered',revision:value.control_revision});message?.({data:event.data});},${latency}+extra);
        });
        channel.send=text=>{const value=JSON.parse(text),revision=revisions.get(value.challenge);trace({event:'requested',action:value.action,sequence:value.sequence,revision});
          setTimeout(()=>{if(channel.readyState==='open'){trace({event:'sent',action:value.action,sequence:value.sequence,revision});send(text);}},${latency});};return channel;
      }
    };})()`});
  const navigation=await call('Page.navigate',{url:'http://127.0.0.1:18089/'});assert(!navigation.errorText,JSON.stringify(navigation));
  await until(()=>evaluate("typeof latestControl!=='undefined'&&!!session&&state.phase==='CURRENT'&&!document.getElementById('enable').disabled"),'真实页面使能入口未就绪');
  if(process.env.MAPPING_TEST_EXPECT_RTC==='1'){
    await until(()=>evaluate('rtcActive&&!transportAwaitingStop&&controlFresh()'),'真实浏览器没有建立WebRTC控制通道');
  }
  const button=await point('enable');await touch('touchStart',button.x,button.y);await touch('touchEnd');
  await until(()=>evaluate('latestControl.armed'),'真实触屏点击使能失败');await stayArmed(400);
  await evaluate("window.dispatchEvent(new Event('blur'))");await stayArmed(500);
  assert(await evaluate('window.carcarControlTrace.every(v=>v.action!=="heartbeat")'),'静止就绪仍在发送操纵心跳');
  await pressStick();await stayArmed(1800);await touch('touchEnd');await sleep(2*latency+150);
  assert(await evaluate('latestControl.armed&&latestControl.x===0'),'真实松手/失去捕获未保持使能停车');
  await stayArmed(300);await pressStick();await stayArmed(400);
  if(process.env.MAPPING_TEST_EXPECT_RTC==='1'&&latency===0){
    // 仅把一个旧状态延后450ms，后续新状态继续交付；验证UDP语义和网页乱序拒绝。
    await evaluate('window.carcarDelayNextRtc=450');await stayArmed(800);
    // 连续丢弃100ms的状态，仍须由后续新签发恢复；不改门控300ms期限。
    await evaluate('window.carcarDropRtcUntil=performance.now()+100');await stayArmed(700);
  }
  await evaluate("window.dispatchEvent(new Event('blur'))");
  await until(()=>evaluate('!latestControl.armed&&!pressed'),'推杆期间失焦未撤销按压');
  await touch('touchEnd');
  await until(()=>evaluate('!document.getElementById("enable").disabled'),'失焦停车后使能入口未恢复');
  await touch('touchStart',button.x,button.y);await touch('touchEnd');
  await until(()=>evaluate('latestControl.armed'),'失焦后重新使能失败');
  await pressStick();await stayArmed(400);
  // 冻结真实页面的JS，不清空其pressed/session，不关闭底层连接。
  await call('Page.setWebLifecycleState',{state:'frozen'});await sleep(500);
  const state=await(await fetch('http://127.0.0.1:18089/api/state')).json();
  assert.equal(state.control.armed,false,'冻结真实页面后未撤销使能');assert.equal(state.control.x,0);
  await call('Page.setWebLifecycleState',{state:'active'});await sleep(250);
  assert(await evaluate('!latestControl.armed&&!pressed'),'恢复页面后自动继续旧按压');
  assert.equal(exceptions.length,0,JSON.stringify(exceptions));
  console.log(`真实Chromium DOM/CSS触屏：静止无心跳/静止失焦保持就绪/推杆失焦停车/点击使能/拖动/松手/再推/冻结停车/恢复锁停通过，${latency}ms/单程`);
}catch(error){
  try{console.error('BROWSER_TRACE '+JSON.stringify(await evaluate('({armed:latestControl.armed,reason:latestControl.reason,revision:lastControlRevision,pressed:!!pressed,events:clientEvents.slice(-12),trace:window.carcarControlTrace})')));}catch(_){}
  throw error;
}finally{
  if(socket?.readyState===WebSocket.OPEN){try{await call('Browser.close');}catch(_){}socket.close();}
  try{process.kill(-chrome.pid,'SIGTERM');}catch(_){}await sleep(100);
  try{process.kill(-chrome.pid,'SIGKILL');}catch(_){}
  fs.rmSync(profile,{recursive:true,force:true,maxRetries:10,retryDelay:100});
}})().catch(error=>{console.error(error.stack);console.error(errors);process.exitCode=1;});
