import { describe, it, expect } from "vitest";
import { createServoSlider } from "./servo-slider.js";

function fixture() {
  const state = { connected:true, visible:true, allowed:true, held:true, still:true, rgb:true,
    teaching:{ phase:"READY", revision:1, target_degrees:140.5 } };
  const commands=[];let clock=0,waitHook=()=>{};
  const slider=createServoSlider({read:()=>state,now:()=>clock,
    send:async (target,revision)=>{
      expect(revision).toBe(state.teaching.revision);
      expect(Math.abs(target-state.teaching.target_degrees)).toBeLessThanOrEqual(1);
      commands.push(target);state.teaching={phase:"WAITING_FRAME",revision:revision+1,target_degrees:target};
    },
    wait:async ()=>{clock+=50;waitHook();if(state.teaching.phase==="WAITING_FRAME")state.teaching={...state.teaching,phase:"AWAITING_CONFIRMATION",revision:state.teaching.revision+1};},
  });
  return {state,commands,slider,onWait:(hook)=>{waitHook=hook;}};
}
describe("舵机滑块调角",()=>{
  it("一次滑块目标自动分成1°小步，最后半度也能完成",async()=>{
    const {slider,commands}=fixture();
    expect(commands).toEqual([]);
    expect(await slider.start(137)).toEqual({completed:true,target:137});
    expect(commands).toEqual([139.5,138.5,137.5,137]);
  });
  it("移动中改目标只走最新方向，不排队执行旧终点",async()=>{
    const f=fixture();let retargeted=false;
    f.onWait(()=>{if(!retargeted){retargeted=true;f.slider.start(142);}});
    await f.slider.start(130);
    expect(f.commands).toEqual([139.5,140.5,141.5,142]);
  });
  it("停止后不追加下一步，保留已发出的一条",async()=>{
    const f=fixture();f.onWait(()=>f.slider.stop());
    expect(await f.slider.start(130)).toEqual({completed:false});
    expect(f.commands).toEqual([139.5]);
  });
  it("失联、后台或失去停稳后停止追加",async()=>{
    for(const key of ["connected","visible","still","allowed","rgb"]) {
      const f=fixture();f.onWait(()=>{f.state[key]=false;});
      await expect(f.slider.start(130)).rejects.toThrow(/停止/);
      expect(f.commands).toEqual([139.5]);
    }
  });
  it("越界不发送；新画面缺失时有期限，不反复重发",async()=>{
    const f=fixture();await expect(f.slider.start(161)).rejects.toThrow(/85/);expect(f.commands).toEqual([]);
    let clock=0,count=0;const state={...f.state,teaching:{phase:"READY",revision:1,target_degrees:140.5}};
    const slider=createServoSlider({read:()=>state,send:async()=>{count++;},now:()=>clock,wait:async()=>{clock+=1000;}});
    await expect(slider.start(130)).rejects.toThrow(/5秒/);expect(count).toBe(1);
  });
});
