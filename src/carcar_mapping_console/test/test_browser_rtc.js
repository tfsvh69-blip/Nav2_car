'use strict';
// 真实网页代码与传输模块，替换DOM/浏览器RTC API，验证用户可观察的时序和清理规则。
const assert=require('assert'),fs=require('fs'),vm=require('vm'),path=require('path');
let now=0;const elements={},sockets=[],peers=[],intervals=[],listeners={},fetches=[],pollDelays=[];
const drawing=new Proxy({},{get:()=>()=>{}});
const element=id=>elements[id]||=( {style:{},value:'slow',classList:{add(){},toggle(){}},getContext:()=>drawing,
  clientWidth:900,clientHeight:600,setPointerCapture(){},getBoundingClientRect:()=>({left:0,top:0,width:240,height:240})});
class Socket {
  static OPEN=1;static CLOSED=3;
  constructor(){this.readyState=1;this.bufferedAmount=0;this.sent=[];sockets.push(this);}
  send(text){this.sent.push(JSON.parse(text));}close(){this.readyState=3;}
}
class Channel {
  constructor(){this.readyState='connecting';this.bufferedAmount=0;this.sent=[];}
  send(text){this.sent.push(JSON.parse(text));}close(){this.readyState='closed';}
}
class Peer {
  constructor(config){assert.equal(config.iceServers.length,0);peers.push(this);this.candidates=[];}
  createDataChannel(label,options){assert.equal(label,'mapping-control');assert.equal(options.ordered,false);assert.equal(options.maxRetransmits,0);return this.channel=new Channel();}
  async createOffer(){return {type:'offer',sdp:'offline-test-sdp'};}
  async setLocalDescription(value){this.localDescription=value;
    this.onicecandidate?.({candidate:{candidate:'candidate:offline',sdpMid:'0'}});}
  async setRemoteDescription(value){this.remoteDescription=value;}
  async addIceCandidate(value){this.candidates.push(value);}close(){this.connectionState='closed';}
}
const document={hidden:false,getElementById:element,addEventListener:(name,fn)=>{listeners[name]=fn;}};
const context={document,window:{devicePixelRatio:1,addEventListener:(name,fn)=>{listeners[name]=fn;}},location:{host:'offline.test'},
  WebSocket:Socket,RTCPeerConnection:Peer,performance:{now:()=>now},console,AbortController,
  setInterval:fn=>{intervals.push(fn);},setTimeout:(fn,ms)=>{if(ms===250)pollDelays.push(fn);return 1;},clearTimeout(){},
  fetch:url=>new Promise((resolve,reject)=>fetches.push({url,resolve,reject})),requestAnimationFrame(){}};
vm.createContext(context);
const app=process.argv[2];vm.runInContext(fs.readFileSync(path.join(path.dirname(app),'rtc_control.js'),'utf8'),context);
vm.runInContext(fs.readFileSync(app,'utf8'),context);
const read=expr=>vm.runInContext(expr,context),socket=sockets[0];
const deliver=(channel,value)=>channel.onmessage?.({data:JSON.stringify(value)});
const state=(revision,armed)=>({control_epoch:'guard-a',control_revision:revision,armed,challenge:'token-'+revision,control_lease_ms:300,health:'',reason:armed?'按住行驶':'请使能'});
(async()=>{
  socket.onmessage({data:JSON.stringify({session:'rtc-owner',rtc_protocol:1,transport_protocol:1})});
  await new Promise(setImmediate);
  const actions=socket.sent.map(v=>v.action);assert(actions.indexOf('rtc_offer')>=0);
  assert(actions.indexOf('rtc_candidate')>actions.indexOf('rtc_offer'),'候选先于offer发出');
  const peer=peers[0],dc=peer.channel;
  socket.onmessage({data:JSON.stringify({rtc_candidate:true,candidate:'candidate:server',mid:'0'})});
  assert.equal(peer.candidates.length,0);
  socket.onmessage({data:JSON.stringify({rtc_description:true,type:'answer',sdp:'answer'})});
  await new Promise(setImmediate);assert.equal(peer.candidates.length,1);
  vm.runInContext("state={phase:'CURRENT',busy:false};",context);
  const beforeOpen=socket.sent.length;
  dc.readyState='open';dc.onopen();assert(read('rtcActive'));assert.equal(read('pressed'),'');
  assert(!socket.sent.slice(beforeOpen).some(v=>v.action==='stop'),'通道接通撤销了尚未推杆的使能');
  deliver(dc,state(1,false));assert.equal(dc.sent.length,0,'未使能仍发送无用操纵心跳');
  socket.onmessage({data:JSON.stringify(state(100,true))});assert.equal(read('latestControl.armed'),false,'旧TCP状态覆盖UDP通道');
  deliver(dc,state(2,true));assert.equal(dc.sent.at(-1).action,'heartbeat');
  const idleRtc=dc.sent.length,idleWs=socket.sent.length;
  listeners.blur();
  assert.equal(dc.sent.length,idleRtc,'静止时窗口失焦不应通过UDP撤销使能');
  assert.equal(socket.sent.length,idleWs,'静止时窗口失焦不应通过WebSocket撤销使能');
  assert(read('latestControl.armed'));
  // 旧50ms双节拍会在新挑战到达前用旧挑战再发一次，随后反而节流新挑战。
  now=50;intervals[0]();now=60;deliver(dc,state(3,true));
  assert.equal(dc.sent.at(-1).challenge,'token-3','新挑战被旧挑战定时回应的节流挡住');
  let revision=3;const heartbeatCount=dc.sent.length;
  for(let i=1;i<=20;i++){now=60+i*50;deliver(dc,state(++revision,true));intervals[0]();intervals[0]();}
  assert(dc.sent.length-heartbeatCount<=20,'定时器重复回应同一条挑战');
  const joystick=element('joystick'),event={pointerId:1,clientX:120,clientY:35,preventDefault(){}};
  const beforePress=dc.sent.length;joystick.onpointerdown(event);
  assert.equal(dc.sent.length,beforePress+1,'按住后没有立刻发送当前方向');
  assert.equal(dc.sent.at(-1).action,'drive');
  const drives=dc.sent.filter(v=>v.action==='drive').length;
  for(let i=1;i<=20;i++){now=1060+i*50;deliver(dc,state(++revision,true));intervals[0]();intervals[0]();}
  assert(dc.sent.filter(v=>v.action==='drive').length-drives<=20,'运动请求超出固定20Hz');
  const mirrored=socket.sent.filter(v=>v.action==='drive'&&v.backup===true);
  assert(mirrored.length>0,'推杆运动没有可靠副本');
  assert(mirrored.every(v=>dc.sent.some(u=>u.action==='drive'&&!u.backup&&u.sequence===v.sequence&&u.challenge===v.challenge)),'可靠副本与实时包不是同一条指令');
  joystick.onpointerup(event);assert.equal(dc.sent.at(-1).action,'release');
  assert.equal(socket.sent.at(-1).action,'release','松手未同时发送可靠通道');
  assert.equal(dc.sent.at(-1).sequence,socket.sent.at(-1).sequence);
  const freshAt=read('lastControlAt');now+=10;deliver(dc,state(revision-1,false));
  assert.equal(read('lastControlAt'),freshAt,'乱序旧状态刷新了新鲜度');assert(read('latestControl.armed'));
  dc.bufferedAmount=100;const sent=dc.sent.length;now+=50;intervals[0]();assert.equal(dc.sent.length,sent,'缓冲未空仍追加控制请求');dc.bufferedAmount=0;
  joystick.onpointerdown(event);
  const held=read('challenge');const heldDrives=()=>dc.sent.filter(v=>v.action==='drive'&&v.challenge===held).length;
  const first=heldDrives();now+=50;intervals[0]();
  assert.equal(heldDrives(),first+1,'同一条仍在许可内的挑战没有补发，丢包后会空等直到撤使能');
  const beforeStale=dc.sent.filter(v=>v.action==='drive').length;now=read('lastControlAt')+350;intervals[0]();
  assert.notEqual(read('pressed'),'','状态晚到时网页不应松开仍按住的摇杆');
  assert(dc.sent.filter(v=>v.action==='drive').length>beforeStale,'状态晚到时没有继续发送当前方向');
  deliver(dc,state(++revision,true));
  joystick.onpointerdown(event);listeners.blur();
  assert.equal(read('pressed'),'','推杆期间窗口失焦没有清除按压');
  assert.equal(dc.sent.at(-1).cause,'window_blur','推杆失焦没有通过UDP立即停车');
  assert.equal(socket.sent.at(-1).cause,'window_blur','推杆失焦没有通过可靠通道立即停车');
  deliver(dc,state(++revision,false));deliver(dc,state(++revision,true));
  joystick.onpointerdown(event);now+=350;intervals[0]();assert.notEqual(read('pressed'),'');assert.equal(dc.sent.at(-1).action,'drive');
  const lateMessage=dc.onmessage;
  const freshBeforeClose=read('lastControlAt');
  dc.onclose();assert.equal(read('rtcActive'),false);assert.equal(read('session'),'rtc-owner');
  assert.equal(read('lastControlAt'),freshBeforeClose,'数据通道关闭清除了仍有效的控制状态');
  assert.notEqual(socket.sent.at(-1).action,'stop','数据通道关闭把静止就绪撤成网页停车');
  lateMessage({data:JSON.stringify(state(revision+10,true))});assert.equal(read('lastControlAt'),freshBeforeClose,'关闭后迟到UDP恢复了使能显示');
  socket.onmessage({data:JSON.stringify(state(revision+20,false))});assert.equal(read('latestControl.armed'),false);
  // 同一网页任务重启会换门控进程，序号从1开始；旧进程的迟到高序号仍不可覆盖新进程。
  socket.onmessage({data:JSON.stringify({...state(1,false),control_epoch:'guard-b'})});
  assert.equal(read('lastControlRevision'),1,'门控重启后低序号被旧进程序号永久挡住');
  const epochAt=read('lastControlAt');now+=10;
  socket.onmessage({data:JSON.stringify(state(9999,true))});
  assert.equal(read('lastControlAt'),epochAt);assert.equal(read('latestControl.armed'),false,'旧进程迟到状态恢复使能');
  const readyCount=socket.sent.length;
  socket.onmessage({data:JSON.stringify({...state(2,true),control_epoch:'guard-b',idle_heartbeat_required:false})});
  for(let i=0;i<20;++i){now+=50;intervals[0]();}
  assert(!socket.sent.slice(readyCount).some(v=>v.action==='heartbeat'),'新门控静止就绪仍依赖操纵心跳');
  assert.equal(read('pressed'),'','静止等待产生了按压');
  socket.onmessage({data:JSON.stringify({...state(3,true),control_epoch:'guard-b',idle_heartbeat_required:false})});
  const firstPress=socket.sent.length;now+=40;joystick.onpointerdown(event);
  assert.equal(socket.sent.at(-1).action,'drive','按住后没有立刻重复当前方向');
  assert(socket.sent.length>firstPress);
  socket.onmessage({data:JSON.stringify({...state(4,true),control_epoch:'guard-b',idle_heartbeat_required:false})});
  assert.equal(socket.sent.at(-1).backup,undefined,'可靠通道本身的运动不标成副本');
  joystick.onpointerup(event);
  // 驱动真实poll的异常分支：HTTP失败且控制暂时过期，松手不能撤销静止就绪。
  const idleBeforeHttp=socket.sent.length;now+=350;
  fetches.find(v=>v.url==='/api/status').reject(Error('injected HTTP outage'));
  await new Promise(setImmediate);
  assert(!socket.sent.slice(idleBeforeHttp).some(v=>v.action==='stop'),'HTTP查询失败误撤销松手后的静止就绪');
  assert.equal(read('pressed'),'');
  // 同一异常发生在尚未释放的按压期间，仍须立即清除按压并停车。
  pollDelays.shift()();await new Promise(setImmediate);
  socket.onmessage({data:JSON.stringify({...state(5,true),control_epoch:'guard-b',idle_heartbeat_required:false})});
  joystick.onpointerdown(event);now+=350;
  fetches.filter(v=>v.url==='/api/status').at(-1).reject(Error('injected moving HTTP outage'));
  await new Promise(setImmediate);
  assert.notEqual(read('pressed'),'');assert.notEqual(socket.sent.at(-1).cause,'control_stale','HTTP查询失败把仍按住的摇杆撤成停车');
  element('transport').value='websocket';element('transport').onchange();assert.equal(peers.length,1);
  sockets.at(-1).onmessage({data:JSON.stringify({session:'old-path',rtc_protocol:1})});assert.equal(peers.length,1,'显式选择旧通道仍协商RTC');
  console.log('RTC无重传/信令顺序/候选等待/未使能静默/20Hz限频/旧TCP与乱序状态隔离/双通道松手/积压丢弃/关闭锁停/旧通道对照通过');
})().catch(error=>{console.error(error);process.exitCode=1;});
