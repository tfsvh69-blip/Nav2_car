'use strict';
// 真实app.js + 真实HTTP/WebSocket/门控；仅替换DOM与显示绘制。
// 必须由隔离域219的C++夹具启动，绝不接受真实车辆地址。
const assert=require('assert'),fs=require('fs'),vm=require('vm');
assert.equal(process.env.ROS_DOMAIN_ID,'219');
const port=Number(process.argv[3]);assert.equal(port,18089);
const base=`http://127.0.0.1:${port}`,latency=Number(process.argv[4]||0);
const burst=process.argv[5]==='burst';let networkBlockedUntil=0;
let suspended=false,closed=false;const elements={},listeners={},trace=[];
const context2d=new Proxy({},{get:()=>()=>{}});
function element(id){return elements[id]||=( {id,style:{},hidden:false,disabled:false,value:'slow',
  classList:{toggle(){},add(){},remove(){}},getContext:()=>context2d,
  clientWidth:900,clientHeight:600,setPointerCapture(){},replaceChildren(){},append(){},
  getBoundingClientRect:()=>({left:0,top:0,width:240,height:240})});}
class NetworkSocket {
  static OPEN=1;static CLOSED=3;
  constructor(url){this.native=new WebSocket(url);this.downAt=0;this.upAt=0;this.native.onmessage=event=>{
    this.schedule('downAt',()=>{if(!suspended&&!closed)this.onmessage?.({data:event.data});});
  };this.native.onclose=()=>this.onclose?.();this.native.onerror=()=>this.onerror?.();}
  get readyState(){return this.native.readyState;}
  get bufferedAmount(){return this.native.bufferedAmount;}
  schedule(direction,fn){const now=performance.now();const at=Math.max(now+latency,networkBlockedUntil,this[direction]);
    this[direction]=at+.01;setTimeout(fn,Math.max(0,at-now));}
  send(data){const v=JSON.parse(data);trace.push({at:performance.now(),action:v.action,sequence:v.sequence});
    this.schedule('upAt',()=>{if(!closed&&this.native.readyState===WebSocket.OPEN)this.native.send(data);});}
  close(){this.native.close();}
}
const document={hidden:false,getElementById:element,querySelectorAll:()=>[],createElement:()=>({}),
  addEventListener:(name,fn)=>{listeners[name]=fn;}};
const context={document,window:{devicePixelRatio:1,addEventListener:(name,fn)=>{listeners[name]=fn;}},
  location:{host:`127.0.0.1:${port}`},WebSocket:NetworkSocket,performance,setInterval,setTimeout,clearTimeout,
  AbortController,fetch:(url,options)=>fetch(new URL(url,base),options),requestAnimationFrame(){},
  createImageBitmap:async()=>({close(){}}),console};
vm.createContext(context);vm.runInContext(fs.readFileSync(process.argv[2],'utf8'),context);
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const read=expr=>vm.runInContext(expr,context);
async function wait(test,label,ms=6000){const end=performance.now()+ms;while(performance.now()<end){if(test())return;await sleep(10);}throw Error(label+': '+JSON.stringify(read('({control:latestControl,events:clientEvents.slice(-10)})')));}
async function hold(ms,label){const end=performance.now()+ms;while(performance.now()<end){
  assert(read('latestControl.armed'),label+': '+read('latestControl.reason'));await sleep(10);
}}
(async()=>{
  await wait(()=>read("!!session && state.phase==='CURRENT' && latestControl.health==='' && controlFresh()")&&!element('enable').disabled,'网页使能入口未就绪');
  element('enable').onclick();
  await wait(()=>read('latestControl.armed'),'点击使能未生效');
  const leaseMs=Number(read('latestControl.control_lease_ms'));
  assert(Number.isFinite(leaseMs)&&leaseMs>0&&leaseMs<=1000,'门控运动期限缺失或超出当前1秒上限');
  await hold(800,'待推杆时被撤销');
  const joystick=element('joystick'),e={pointerId:1,clientX:120,clientY:28,preventDefault(){}};
  joystick.onpointerdown(e);
  await wait(()=>read('Math.abs(latestControl.x||0)>.04'),'推杆没有输出',1000);
  await hold(2500,'持续推杆被撤销');
  joystick.onpointerup(e);
  await sleep(latency*2+100);
  assert(read('latestControl.armed'),'松手不应撤销使能');
  assert.equal(read('latestControl.x'),0,'松手未归零');
  await hold(600,'松手后的静止就绪被撤销');
  joystick.onpointerdown(e);await hold(800,'再次推杆被撤销');
  if(burst){
    // 模拟TCP保序的集中到达，禁止用可乱序的独立随机setTimeout代替TCP。
    for(const ms of [80,150,180]){networkBlockedUntil=performance.now()+ms;await hold(ms+250,'预算内突发延迟误撤销');}
    // 在网络仍阻塞时，覆盖当前收包租期、门控/状态发布周期和HTTP缓存后观察。
    networkBlockedUntil=performance.now()+leaseMs+500;await sleep(leaseMs+250);
    let stopped=await(await fetch(base+'/api/state')).json();
    assert.equal(stopped.control.armed,true,'运动断流只应归零，不应撤使能');assert.equal(stopped.control.x,0);
    assert.equal(stopped.control.reason,'操纵授权超时');
    joystick.onpointerup(e);
    networkBlockedUntil=performance.now()+550;await sleep(480);
    stopped=await(await fetch(base+'/api/state')).json();assert.equal(stopped.control.armed,true,'静止网络抖动错误撤销就绪');
    assert.equal(stopped.control.x,0,'静止断流产生了运动');
    await sleep(450);assert(read('latestControl.armed')&&!read('pressed'),'静止网络恢复产生了旧按压');
    element('reconnect').onclick();
    assert(!read('latestControl.armed')&&!read('pressed'),'显式重连保留了旧按压');
    await wait(()=>read("!!session&&controlFresh()&&!latestControl.armed&&latestControl.health===''")&&!element('enable').disabled,'显式重连未恢复未使能状态');
    element('enable').onclick();await wait(()=>read('latestControl.armed'),'重连后不能重新使能');
    joystick.onpointerdown(e);await hold(300,'挂起测试准备失败');
    console.log(`TCP保序80/150/180ms突发积压保持控制；超过${leaseMs}ms运动断流归零、静止断流保持零，恢复不重放旧动作`);
  }
  // 模拟JS挂起，但浏览器底层WebSocket仍然连着。旧指令必须到期锁停。
  suspended=true;const oldCount=trace.length;
  vm.runInContext("pressed='';session='';challenge='';lastControlAt=-Infinity",context);
  // 挂起前已经进入模拟上行队列的最后一包仍可在latency后到达；
  // 随后是门控报告的租期，再覆盖20ms门控/60ms状态发布/100ms HTTP缓存与调度余量。
  await sleep(leaseMs+latency+250);
  const state=await(await fetch(base+'/api/state')).json();
  assert.equal(state.control.armed,true,'脚本挂起后不应撤使能');
  assert.equal(state.control.x,0,'脚本挂起后旧运动持续');
  assert(!trace.slice(oldCount).some(v=>v.action==='drive'),'挂起测试意外继续发送运动');
  console.log(`网页点击使能→闲置→推杆→松手→再推→挂起停车通过，双向注入${latency}ms/单程`);
  closed=true;process.exit(0);
})().catch(error=>{console.error(error.stack);console.error(JSON.stringify({trace:trace.slice(-15),control:read('latestControl')},null,2));closed=true;process.exit(1);});
