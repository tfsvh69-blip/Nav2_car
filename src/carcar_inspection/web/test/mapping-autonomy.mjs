// 实际Chromium + 车端网页；仅隔离软件房间端口，不连接真实硬件。
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import {spawn} from 'node:child_process';
import {chromium} from 'playwright-core';
const [url,directory,runtime]=process.argv.slice(2);
assert(['http://127.0.0.1:18098','http://127.0.0.1:18229'].includes(url));
assert(runtime&&directory);
await fs.mkdir(directory,{recursive:true});
const profile=await fs.mkdtemp(path.join(os.tmpdir(),'carcar-mapping-browser-'));
const child=spawn(runtime,['--headless=new','--no-sandbox','--disable-gpu','--no-proxy-server','--disable-dev-shm-usage','--no-first-run','--disable-background-networking','--disable-extensions','--disable-component-update','--remote-debugging-port=0',`--user-data-dir=${profile}`,'about:blank'],{detached:true,env:{...process.env,ROS_DOMAIN_ID:'219'},stdio:['ignore','ignore','pipe']});
let browser,errors=[],stderr='';child.stderr.on('data',v=>stderr=(stderr+v).slice(-4000));
try{
  let port;
  for(let i=0;i<200&&!port;i++){try{port=(await fs.readFile(path.join(profile,'DevToolsActivePort'),'utf8')).split('\n')[0];}catch{await new Promise(r=>setTimeout(r,50));}}
  assert(port,stderr);browser=await chromium.connectOverCDP(`http://127.0.0.1:${port}`);
  const context=await browser.newContext({viewport:{width:1024,height:768}}),page=await context.newPage();
  page.on('pageerror',e=>errors.push(e.message));
  const state=async()=>await page.evaluate(async()=>await(await fetch('/api/status',{cache:'no-store'})).json());
  async function waitForPhase(phase){for(let i=0;i<(phase==='REVIEW_REQUIRED'?1000:200);i++){const s=await state();if(s.autonomy?.phase===phase)return s;await page.waitForTimeout(50);}assert.fail(`未进入${phase}：${JSON.stringify(await state())}`);}
  await page.goto(url);
  if(url.endsWith(':18229')){
    await page.waitForFunction(()=>!document.getElementById('publication-panel').hidden);
    await page.locator('#review-bundle').selectOption('candidate');
    await page.locator('#review-note').fill('隔离测试：尺量与墙线审核夹具；非实机场地');
    await page.locator('#review-approve').click();
    await page.waitForFunction(()=>document.getElementById('publication-result').textContent.includes('已发布该版本'));
    await page.locator('#review-bundle').selectOption('candidate');
    await page.locator('#map-activate').click();
    await page.waitForFunction(()=>document.getElementById('publication-result').textContent.includes('地图已加载'),null,{timeout:12000});
    const inspection=await page.evaluate(async()=>await(await fetch('/inspect/api/state')).json());
    assert.equal(inspection.map_activation.phase,'WAITING_LOCALIZATION');assert(inspection.motion_held);assert.equal(inspection.health.localization,false);
    assert.equal(inspection.map_activation.map_id,'candidate');assert(inspection.map_activation.request_id);
    await page.setViewportSize({width:390,height:844});await page.screenshot({path:path.join(directory,'phone-publication.png'),fullPage:true});
    assert.deepEqual(errors,[]);console.log('PUBLICATION_BROWSER '+JSON.stringify({success:true,review:true,activate:true,held:true}));
  }else{
  await page.waitForFunction(()=>document.getElementById('auto-pause')?.disabled===false);
  for(let i=0;i<400&&(await state()).autonomy.accepted_sequence<3;i++)await page.waitForTimeout(50);
  assert((await state()).autonomy.accepted_sequence>=3,'探索应产生新的接纳扫描，不能只原地提交目标');
  const initial=await waitForPhase('EXPLORING'),origin=initial.autonomy.origin;
  await page.reload();await page.waitForTimeout(800);
  let s=await state();assert.equal(s.autonomy.phase,'EXPLORING');assert.deepEqual(s.autonomy.origin,origin);
  assert(s.autonomy.elapsed_seconds>=initial.autonomy.elapsed_seconds,'刷新不能重置预算');
  await page.getByRole('button',{name:'暂停自动',exact:true}).click();await waitForPhase('PAUSED');
  s=await state();assert(s.autonomy.still&&s.autonomy.navigation_terminal);assert.deepEqual(s.autonomy.output,[0,0]);
  await page.getByRole('button',{name:'停稳后手动接管',exact:true}).click();await waitForPhase('MANUAL');
  await page.setViewportSize({width:390,height:844});await page.waitForTimeout(300);
  assert(await page.evaluate(()=>document.documentElement.scrollWidth<=innerWidth+1),'手机页面不应横向溢出');
  await page.screenshot({path:path.join(directory,'phone-manual.png'),fullPage:true});
  await page.locator('#site-confirmed').check();await page.locator('#auto-start').click();
  s=await waitForPhase('EXPLORING');assert.deepEqual(s.autonomy.origin,origin);
  await page.getByText('辅助工具与诊断',{exact:true}).click();
  await page.getByRole('button',{name:'重新连接',exact:true}).click();await page.waitForTimeout(700);
  assert.equal((await state()).autonomy.phase,'EXPLORING','自动任务应独立于网页连接');
  await page.getByRole('button',{name:'结束探索并返程',exact:true}).click();
  await waitForPhase('REVIEW_REQUIRED');s=await state();assert(s.last_saved_bundle,'结束后应保存独立候选');
  await page.screenshot({path:path.join(directory,'phone-candidate.png'),fullPage:true});
  assert.deepEqual(errors,[]);console.log('MAPPING_BROWSER '+JSON.stringify({success:true,refresh:true,pause:true,manual:true,resume:true,reconnect:true,candidate:s.last_saved_bundle,mobile:true}));
  }
}finally{
  if(browser)await browser.close();try{process.kill(-child.pid,'SIGTERM');}catch{}
  await fs.rm(profile,{recursive:true,force:true});
}
