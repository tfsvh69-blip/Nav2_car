'use strict';
// 回放现场自动保存后的真实状态组合，只替换DOM和网络，不连接或控制实车。
const assert = require('assert'), fs = require('fs'), vm = require('vm');
const source = fs.readFileSync(process.argv[2], 'utf8');
async function render(snapshot, {selection='manual'}={}) {
  const elements = {}, requests = [], confirmations=[],context2d = new Proxy({}, {get: () => () => {}});
  const element = id => elements[id] ||= {id, style:{}, hidden:false, disabled:false,
    checked:true, value:'manual', textContent:'', classList:{toggle(){},add(){},remove(){}},
    getContext:()=>context2d, clientWidth:900,clientHeight:600, setPointerCapture(){},focus(){},scrollIntoView(){},append(child){child.parentElement=this;},
    getBoundingClientRect:()=>({left:0,top:0,width:240,height:240})};
  class Socket {
    static OPEN=1;static CLOSED=3;
    constructor(){this.readyState=1;this.bufferedAmount=0;}
    send(){} close(){this.readyState=3;}
  }
  const context = {document:{hidden:false,getElementById:element,querySelectorAll:()=>[],addEventListener(){}},
    window:{devicePixelRatio:1,addEventListener(){},confirm:message=>{confirmations.push(message);return true;}}, location:{host:'offline.test'},
    WebSocket:Socket,performance:{now:()=>100},setInterval(){},setTimeout:()=>1,clearTimeout(){},
    AbortController,crypto:require('crypto').webcrypto,requestAnimationFrame(){},console,
    fetch:async (url,options={})=>{
      requests.push({url,options});
      return {ok:true,status:200,json:async()=>url==='/api/status'?snapshot:{accepted:true,operation:3}};
    }};
  vm.createContext(context);vm.runInContext(source,context);
  element('control-mode').value=selection;
  vm.runInContext("session='offline-observer';lastControlAt=100;latestControl={armed:false,health:''};",context);
  await new Promise(resolve=>setImmediate(resolve));
  return {elements,requests,context,confirmations};
}
// 仅回放前端协议，不连真实网关或车端。
(async()=>{
  const snapshot={phase:'FAILED',task_state:'RUNNING',managed:true,busy:false,dirty:false,
    autonomy_enabled:true,autonomy_age_ms:10,dataflow_protocol:1,control:{armed:false},
    autonomy:{task_id:'fault-fixture',phase:'FAULT',control_mode:'automatic',still:true,navigation_terminal:true,
      fault:{active:true,reason:'原始 /scan 超过0.5秒未更新'},healthy:false,
      actions:{recheck:{enabled:true,reason:''},resume:{enabled:false,reason:'请先重新检查'},return:{enabled:false,reason:'请先重新检查'},manual:{enabled:false,reason:'请先重新检查'},pause:{enabled:false,reason:'当前已停稳'}}}};
  const r=await render(snapshot),e=r.elements;
  assert.equal(typeof e['auto-recheck']?.onclick,'function','故障页面缺少重新检查操作');
  assert.equal(e['auto-recheck'].hidden,false);assert.equal(e['auto-recheck'].disabled,false);
  for(const id of ['auto-start','auto-return','auto-manual']){assert.equal(e[id].hidden,false);assert(e[id].disabled);assert(e[id].title.includes('重新检查'));}
  assert(!e['status-caption'].textContent.includes('独立运行'));
  await e['auto-recheck'].onclick();
  const post=r.requests.filter(x=>x.options.method==='POST');assert.equal(post.length,1);
  assert.equal(JSON.parse(post[0].options.body).action,'recheck');assert(e['auto-return'].disabled);
  const stale=await render({...snapshot,autonomy_age_ms:501});assert(stale.elements['auto-recheck'].disabled);
  const legacy=await render({phase:'CURRENT',managed:false,task_state:'STANDBY',autonomy_enabled:false,control:{armed:false}});
  assert.equal(legacy.elements.enable.hidden,false,'独立手动入口不能被托管任务的待命状态隐藏');
  assert.equal(legacy.elements['manual-controls'].hidden,false);
  const saved=await render({...snapshot,phase:'CURRENT',autonomy:{...snapshot.autonomy,phase:'PAUSED',healthy:true,
    operator_check:{phase:'READY'},fault:{active:false,reason:'历史超时'},actions:{...snapshot.autonomy.actions,
      resume:{enabled:true,reason:''},return:{enabled:true,reason:''},manual:{enabled:true,reason:''}}}});
  assert.equal(saved.elements['auto-start'].disabled,false);assert.equal(saved.elements['auto-return'].disabled,false);
  await saved.elements['auto-return'].onclick();assert(saved.requests.some(x=>x.options.body&&JSON.parse(x.options.body).action==='return'));
  console.log('R27故障重新检查、操作可用性、状态过期及独立返程协议通过');
})().catch(e=>{console.error(e);process.exitCode=1;});
