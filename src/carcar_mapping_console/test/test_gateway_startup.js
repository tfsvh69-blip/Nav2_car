'use strict';
// 网页启动瞬间的真实HTTP检查：隔离域218、固定localhost端口，不启动任何硬件。
const assert=require('assert'),{spawn}=require('child_process');
const child=spawn(process.argv[2],['--ros-args','-p','mode:=standby','-p','backend_hardware:=false',
  '-p','bind_address:=127.0.0.1','-p','port:=18090'],
  {env:{...process.env,ROS_DOMAIN_ID:'218',ROS_LOCALHOST_ONLY:'1'},stdio:['ignore','ignore','inherit']});
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
(async()=>{try{
  let state;const end=Date.now()+5000;
  while(Date.now()<end){try{state=await(await fetch('http://127.0.0.1:18090/api/state')).json();break;}catch(_){await sleep(2);}}
  assert(state,'网页入口未启动');assert.equal(state.phase,'STANDBY','首个HTTP状态必须完整，无需等下一次定时器');
  const r=await fetch('http://127.0.0.1:18090/api/diagnostics'),d=await r.json();
  assert(r.ok,JSON.stringify(d));assert.equal(d.phase,'STANDBY');assert.deepEqual(d.control,{});
  console.log('首个HTTP状态和诊断均完整，硬件未启动');
}finally{child.kill('SIGINT');await sleep(100);if(child.exitCode===null)child.kill('SIGKILL');}
})().catch(e=>{console.error(e);process.exitCode=1;});
