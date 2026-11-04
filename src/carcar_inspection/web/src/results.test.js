import {describe,it,expect} from 'vitest';
import {runResult,fieldTeachingAllowed} from './workflow.js';
describe('巡检完整性与现场示教',()=>{
  const run={status:'SUCCEEDED',snapshot:{points:[{id:'p1'},{id:'p2'},{id:'p3'}]},captures:{p1:{id:'a'},p2:{id:'b'},p3:{id:'c'}},missed:[]};
  it('三点全部有照片才显示全部完成',()=>{expect(runResult(run)).toBe('全部完成');expect(runResult({...run,captures:{p1:{id:'a'}}})).toContain('部分完成');expect(runResult({...run,captures:{}})).toContain('未采集到照片');});
  it('失败终态不被已有照片掩盖',()=>{expect(runResult({...run,status:'FAILED'})).not.toBe('全部完成');});
  it('现场录入无需伪造导航到达但仍要求定位停稳',()=>{const h={still:true,localization:true,map_matches:true};expect(fieldTeachingAllowed('current',false,h,true)).toBe(true);for(const key of Object.keys(h))expect(fieldTeachingAllowed('current',false,{...h,[key]:false},true)).toBe(false);expect(fieldTeachingAllowed('navigation',false,h,true)).toBe(false);expect(fieldTeachingAllowed('current',true,h,false)).toBe(false);});
});
