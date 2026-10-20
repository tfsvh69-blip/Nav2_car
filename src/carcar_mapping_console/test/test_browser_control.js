'use strict';
// 运行真实网页脚本，仅替换DOM/网络/时钟；不连接机器人或发送真实指令。
const assert = require('assert'), fs = require('fs'), vm = require('vm');
let now = 0;
const intervals = [], listeners = {}, elements = {}, sockets = [];
const context2d = new Proxy({}, {get: () => () => {}});
function element(id) {
  return elements[id] ||= {id, style:{}, hidden:false, disabled:false, value:'slow',
    classList:{toggle(){},add(){},remove(){}}, getContext:()=>context2d,
    clientWidth:900,clientHeight:600,setPointerCapture(){},
    getBoundingClientRect:()=>({left:0,top:0,width:240,height:240})};
}
class Socket {
  static OPEN=1;static CLOSED=3;
  constructor(){this.readyState=1;this.bufferedAmount=0;this.sent=[];sockets.push(this);}
  send(value){this.sent.push(JSON.parse(value));}
  close(){this.readyState=3;}
}
const document={hidden:false,getElementById:element,querySelectorAll:()=>[],
  addEventListener:(name,fn)=>{listeners[name]=fn;}};
const context={document,window:{devicePixelRatio:1,addEventListener:(name,fn)=>{listeners[name]=fn;}},
  location:{host:'offline.test'},WebSocket:Socket,performance:{now:()=>now},
  setInterval:fn=>{intervals.push(fn);},setTimeout:()=>1,clearTimeout(){},
  AbortController,fetch:()=>new Promise(()=>{}),requestAnimationFrame(){},console};
vm.createContext(context);vm.runInContext(fs.readFileSync(process.argv[2],'utf8'),context);
const socket=sockets[0];
socket.onmessage({data:JSON.stringify({session:'offline-session'})});
socket.onmessage({data:JSON.stringify({armed:false,challenge:'enabling-old',reason:'请使能',health:''})});
assert.equal(socket.sent.at(-1).action,'heartbeat','使能确认途中也必须及时回应新挑战');
now+=5;socket.onmessage({data:JSON.stringify({armed:false,challenge:'enabling-new',reason:'请使能',health:''})});
assert.equal(socket.sent.at(-1).challenge,'enabling-new','新令牌不能被旧令牌的30ms节流挡住');
vm.runInContext("state={phase:'CURRENT',busy:false,control_age:0.02,control:{armed:true}};lastStateAt=0;",context);
const event={pointerId:1,clientX:120,clientY:35,preventDefault(){}};
const joystick=element('joystick');
socket.onmessage({data:JSON.stringify({armed:true,challenge:'token-0',reason:'按住行驶',health:''})});
joystick.onpointerdown(event);
// HTTP状态查询停滞3秒，但专用WebSocket持续返回新鲜门控状态。
for(now=50;now<=3000;now+=50){
  socket.onmessage({data:JSON.stringify({armed:true,challenge:'token-'+now,reason:'按住行驶',health:''})});
  intervals[0]();
  assert.equal(socket.sent.at(-1).action,'drive',`新鲜控制连接在${now}ms被HTTP查询延迟错误锁停`);
}
assert.equal(socket.readyState,Socket.OPEN);
joystick.onpointerup(event);
assert.equal(socket.sent.at(-1).action,'release');
joystick.onpointerdown(event);assert.equal(socket.sent.at(-1).action,'drive');
// 新门控报告300ms签发窗口时，220ms下行间隔不能仍按旧200ms本地撤销。
socket.onmessage({data:JSON.stringify({armed:true,challenge:'jitter',control_lease_ms:300,reason:'按住行驶',health:''})});
now+=220;intervals[0]();assert.equal(socket.sent.at(-1).action,'drive');
// 控制状态晚到时，手还按着就继续发当前方向，不在网页里撤使能。
now+=350;intervals[0]();assert.equal(socket.sent.at(-1).action,'drive');
assert.notEqual(vm.runInContext('activePointer',context),null);
const stopped=socket.sent.length;
socket.onmessage({data:JSON.stringify({armed:false,challenge:'after-stop',reason:'已停车',health:''})});
intervals[0]();assert(!socket.sent.slice(stopped).some(value=>value.action==='drive'));
document.hidden=true;listeners.visibilitychange();assert.equal(socket.readyState,Socket.CLOSED);
// 显式重连必须立即清空上一连接的使能/新鲜度，迟到旧socket消息不得恢复它。
document.hidden=false;
vm.runInContext("latestControl={armed:true,control_lease_ms:300};lastControlAt=performance.now();",context);
element('reconnect').onclick();
assert.equal(vm.runInContext('controlFresh()',context),false,'重连仍使用旧连接的新鲜度');
assert(!vm.runInContext('latestControl.armed',context),'重连保留旧使能显示');
socket.onmessage({data:JSON.stringify({armed:true,challenge:'late-old'})});
assert(!vm.runInContext('latestControl.armed',context),'旧连接迟到消息恢复了使能');
// 安卓Chrome触控事件边界：第二指不抢占，松手后的自然捕获释放不锁车。
const current=sockets.at(-1);current.onmessage({data:JSON.stringify({session:'android-events'})});
current.onmessage({data:JSON.stringify({armed:true,challenge:'android-token',control_lease_ms:300,health:''})});
now+=50;joystick.onpointerdown(event);assert.equal(socket.readyState,Socket.CLOSED);
const other={...event,pointerId:2};joystick.onpointerdown(other);joystick.onpointerup(other);
assert.equal(vm.runInContext('activePointer',context),1,'第二指抢占了摇杆');
now+=35;joystick.onpointermove({...event,clientX:500});
const outside=current.sent.at(-1);assert.equal(outside.action,'drive');
assert(Math.hypot(outside.x,outside.yaw)<=1.000001,'越界拖动突破摇杆范围');
joystick.onpointerup(event);const released=current.sent.length;
joystick.onlostpointercapture(event);assert.equal(current.sent.length,released,'正常松手后释放捕获误触发锁停');
now+=50;joystick.onpointerdown(event);joystick.onpointercancel(event);
assert.equal(current.sent.at(-1).cause,'pointer_cancel');
assert.equal(vm.runInContext('activePointer',context),null);
current.onmessage({data:JSON.stringify({armed:false,challenge:'idle-probe',control_lease_ms:300,health:''})});
const idleStart=current.sent.length;now+=350;intervals[0]();intervals[0]();
while(vm.runInContext('pendingClientEvents.length',context))intervals[1]();
assert.equal(current.sent.slice(idleStart).filter(v=>v.action==='client_event'&&v.event==='control_gap').length,1,'停车时断流应记录一次');
assert(!current.sent.slice(idleStart).some(v=>v.action==='drive'||v.action==='enable'),'停车诊断不得使能或运动');
// 旧门控未报告租期时，仍按其200ms窗口判断，不能默认增加到500ms。
current.onmessage({data:JSON.stringify({armed:false,challenge:'legacy-guard',health:''})});
now+=201;assert.equal(vm.runInContext('controlFresh()',context),false,'旧门控缺少租期字段时放宽了新鲜度');
// HTTP仍更新时，已冻结的激光预览不能被重新标成新鲜。
vm.runInContext("state={phase:'CURRENT'};scanPoints=[[1,2]];lastScanAt=performance.now();scanSourceAgeMs=100;",context);
now+=399;assert(vm.runInContext('scanFresh()',context));
now+=2;vm.runInContext('lastStateAt=performance.now()',context);
assert(!vm.runInContext('scanFresh()',context),'状态查询掩盖了冻结的激光预览');
const generation=vm.runInContext('previewGeneration',context);vm.runInContext('clearScan()',context);
assert.equal(vm.runInContext('previewGeneration',context),generation+1);
assert.equal(vm.runInContext('scanPoints.length',context),0);
// 显示属性更新前必须已送出新挑战的回应。
Object.defineProperty(element('motion'),'textContent',{configurable:true,set(){
  assert.equal(current.sent.at(-1).challenge,'response-first','显示更新先于心跳回应');
}});
now+=50;current.onmessage({data:JSON.stringify({armed:false,challenge:'response-first',health:''})});
// 握手失败有时先报error，不能只等close才清空会话和安排重连。
current.readyState=0;current.onerror();
assert.equal(vm.runInContext('session',context),'','连接错误仍保留旧会话');
assert.equal(vm.runInContext('ws',context),null,'连接错误后仍卡在旧socket');
console.log('持续摇杆/HTTP延迟/控制超时/重连/多指/松手捕获释放/触控取消/后台及停车断流诊断通过');
