'use strict';
// 执行正式绘制函数，验证开关、断流和错位不会改变数据或发出控制请求。
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
const source=fs.readFileSync(path.join(__dirname,'../web/app.js'),'utf8');
const code=source.slice(source.indexOf('function layerOn('),source.indexOf('function paintPath('));
let draws=0,requests=0;
const toggles={'layer-global-cost':{checked:true},'layer-keepout':{checked:false}};
const meta={frame:'map',width:3,height:2,resolution:.05,origin:{x:0,y:0,yaw:0}};
const image=()=>({bitmap:{},meta:structuredClone(meta),decodedAt:1000,sourceAge:0});
const context={state:{navigation:{obstacle_costmap:{available:true},keepout_mask:{available:true}}},
 navImages:{global:image(),keepout:image()},$:id=>toggles[id],performance:{now:()=>1000},
 ctx:{save(){},restore(){},translate(){},rotate(){},scale(){},drawImage(){draws++;}},fetch(){requests++;}};
vm.createContext(context);vm.runInContext(code,context);
const paint=()=>vm.runInContext("paintCost('keepout','keepout_mask','layer-keepout')",context);
paint();assert.equal(draws,0);assert.match(vm.runInContext('keepoutNote()',context),/隐藏.*仍生效/);
toggles['layer-keepout'].checked=true;paint();assert.equal(draws,1);
// 掩膜按变化发布，时间老不等于无效；必须仍有新鲜同几何基础图。
context.navImages.keepout.sourceAge=600;paint();assert.equal(draws,2);
context.navImages.global.sourceAge=2;paint();assert.equal(draws,2);
context.navImages.global.sourceAge=0;context.navImages.keepout.meta.origin.x=1;paint();assert.equal(draws,2);
context.navImages.keepout.meta.origin.x=0;context.state.navigation.obstacle_costmap.available=false;paint();assert.equal(draws,2);
context.state.navigation.obstacle_costmap.available=true;paint();assert.equal(draws,3);
assert.equal(requests,0,'显示开关不得请求车辆控制');
assert.equal(context.navImages.keepout.sourceAge,600,'显示不得伪造掩膜新鲜度');
assert(!/id="layer-keepout"[^>]*checked/.test(fs.readFileSync(path.join(__dirname,'../web/index.html'),'utf8')));
console.log('独立开关／默认隐藏／零控制请求／断流隐藏／错位隐藏／恢复显示通过');
