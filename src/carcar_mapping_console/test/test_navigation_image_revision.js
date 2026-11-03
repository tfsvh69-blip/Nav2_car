'use strict';
// 只执行正式图片刷新函数，模拟延迟、失败和任务切换，不连接车辆。
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../web/app.js'),'utf8');
const code=source.slice(source.indexOf('function refreshNavigationImages(){'),source.indexOf('function draw('));
async function run(){
  const requests=[];let closed=0;
  const context={state:{navigation:{global_costmap:{available:true,sequence:2}}},taskKey:'one',
    navImages:{global:{sequence:1,bitmap:{close(){closed++;}},meta:{sequence:1}}},
    performance:{now:()=>100},createImageBitmap:async bitmap=>bitmap,
    fetchTimed:()=>new Promise((resolve,reject)=>requests.push({resolve,reject})),console};
  vm.createContext(context);vm.runInContext(code,context);vm.runInContext('refreshNavigationImages()',context);
  assert.equal(context.navImages.global.sequence,1,'加载完成前不能把旧图标成新版本');
  requests[0].resolve({ok:true,headers:{get:()=>JSON.stringify({sequence:3})},blob:async()=>({close(){closed++;}})});
  await new Promise(setImmediate);
  assert.equal(context.navImages.global.sequence,3,'应使用实际图片携带的版本');assert.equal(closed,1);
  context.state.navigation.global_costmap.sequence=4;vm.runInContext('refreshNavigationImages()',context);
  requests[1].reject(Error('断线'));await new Promise(setImmediate);
  assert.equal(context.navImages.global.sequence,3);vm.runInContext('refreshNavigationImages()',context);
  assert.equal(requests.length,3,'失败后必须允许重试同一版本');
  context.taskKey='two';context.navImages={};requests[2].resolve({ok:true,headers:{get:()=>'{"sequence":4}'},blob:async()=>({close(){closed++;}})});
  await new Promise(setImmediate);assert.equal(context.navImages.global,undefined,'旧任务图片不得写入新任务');
  console.log('图片延迟／实际版本／失败重试／任务切换保护通过');
}
run().catch(error=>{console.error(error);process.exitCode=1;});
