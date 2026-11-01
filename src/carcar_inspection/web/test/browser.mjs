// APP-001真实浏览器与真实C++网页入口；只能连接本机隔离测试端口。
import assert from "node:assert/strict";
import fs from "node:fs/promises";
import os from "node:os";
import path from "node:path";
import { spawn } from "node:child_process";
import { chromium } from "playwright-core";
const [url, directory] = process.argv.slice(2);
assert.match(url || "", /^http:\/\/127\.0\.0\.1:18189$/);
assert(process.env.INSPECTION_BROWSER_RUNTIME, "缺少浏览器测试运行入口");
await fs.mkdir(directory, { recursive: true });
const profile = await fs.mkdtemp(
  path.join(os.tmpdir(), "carcar-inspection-browser-"),
);
// bwrap会关闭调试pipe的额外FD；用官方CDP连接本机随机调试端口。
const child = spawn(
  process.env.INSPECTION_BROWSER_RUNTIME,
  [
    "--headless=new",
    "--no-sandbox",
    "--disable-gpu",
    "--no-proxy-server",
    "--disable-dev-shm-usage",
    "--no-first-run",
    "--disable-background-networking",
    "--disable-extensions",
    "--disable-component-update",
    "--remote-debugging-port=0",
    `--user-data-dir=${profile}`,
    "about:blank",
  ],
  {
    detached: true,
    env: {
      ...process.env,
      ROS_DOMAIN_ID: "227",
      XDG_CONFIG_HOME: path.join(profile, "config"),
      XDG_CACHE_HOME: path.join(profile, "cache"),
    },
    stdio: ["ignore", "ignore", "pipe"],
  },
);
let stderr = "",
  browser,
  page;
async function waitForState(predicate, timeout=8000) {
  const until=Date.now()+timeout;
  while(Date.now()<until) {
    const state=await page.evaluate(async()=>await(await fetch("/api/state",{cache:"no-store"})).json());
    if(predicate(state))return state;
    await page.waitForTimeout(50);
  }
  assert.fail("车端状态在截止时间前未满足预期");
}
child.stderr.on("data", (value) => (stderr = (stderr + value).slice(-4000)));
try {
  const until = Date.now() + 8000;
  let port;
  while (Date.now() < until) {
    try {
      port = (
        await fs.readFile(path.join(profile, "DevToolsActivePort"), "utf8")
      ).split("\n")[0];
      break;
    } catch {
      await new Promise((r) => setTimeout(r, 25));
    }
  }
  assert(port, `Chromium启动失败：${stderr}`);
  browser = await chromium.connectOverCDP(`http://127.0.0.1:${port}`, {
    timeout: 8000,
  });
  page =
    browser.contexts()[0].pages()[0] || (await browser.contexts()[0].newPage());
  await page.bringToFront();
  await page.setViewportSize({ width: 1280, height: 900 });
  page.setDefaultTimeout(8000);
  const errors = [];
  page.on("pageerror", (error) => {
    errors.push(error.message);
    console.error("页面异常：", error.message);
  });
  page.on("requestfailed", (request) =>
    console.error("请求失败：", request.url(), request.failure()?.errorText),
  );
  await page.goto(url);
  console.log("浏览器已加载巡检网页");
  await page
    .locator(".aside-foot")
    .filter({ hasText: "操作者已连接" })
    .waitFor();
  await page.locator(".toolbar select").selectOption("testmap");
  await page.getByRole("button", { name: "载入地图", exact: true }).click();
  const speedControls=page.getByRole("region", { name: "导航速度档位" });
  for(const [label,mps] of [["4档 · 0.30 m/s（实验）",.30],["1档 · 0.10 m/s",.10],["2档 · 0.18 m/s",.18]]) {
    const button=speedControls.getByRole("button",{name:label,exact:true});
    await button.click();
    await waitForState((state)=>state.navigation_speed.ready&&state.navigation_speed.applied_mps===mps&&state.navigation_trial.phase==="IDLE");
    await page.waitForFunction((label)=>Array.from(document.querySelectorAll(".speed-options button"))
      .some((button)=>button.textContent.trim()===label&&button.getAttribute("aria-pressed")==="true"),label);
  }
  console.log("网页四档速度已验证升档、降档及控制器确认；切档未下发导航目标");
  const writes = [];
  page.on("request", (request) => {
    if (request.method() === "POST")
      writes.push(
        new URL(request.url()).pathname.replace(/^\/inspect(?=\/)/, ""),
      );
  });
  const name = page.locator('input[placeholder="例如：走廊仪表"]');
  await name.fill("地图选点草稿");
  await page
    .getByText("仅保存计划草稿（尚未实车确认）", { exact: true })
    .click();
  const saveDraft = page.getByRole("button", {
    name: "保存巡检点草稿",
    exact: true,
  });
  assert(
    await saveDraft.isDisabled(),
    "未选位置时不能把默认零坐标保存为巡检点",
  );
  const canvas = page.getByLabel("场地地图，点击设置停车位置并拖动设置朝向");
  // 断言主录入地图的真实像素，不能仅凭图层状态“收到数据”判断已绘制。
  await page.locator(".navigation-debug > summary").click();
  const layerOptions = page.locator(".navigation-layer-option input");
  for (let index=0;index<4;index++) await layerOptions.nth(index).uncheck();
  const samples = [
    {x:.25,y:.25,color:"purple"},
    {x:.45,y:.25,color:"orange"},
    {x:.5,y:1.5,color:"cyan"},
    {x:.5,y:1.7,color:"yellow"},
  ];
  for (let index=0;index<samples.length;index++) {
    await layerOptions.nth(index).check();
    await page.waitForFunction(({x,y,color})=>{
      const c=document.querySelector('canvas[aria-label="场地地图，点击设置停车位置并拖动设置朝向"]');
      if(!c?.width)return false;
      const [r,g,b]=c.getContext("2d").getImageData(Math.floor(c.width*x/2),Math.floor(c.height*(1-y/2)),1,1).data;
      return color==="purple"?b>r+15&&r>g+10:color==="orange"?r>g+15&&g>b+15:color==="cyan"?r<30&&g>130&&b>150:r>220&&g>180&&b<30;
    },samples[index],{timeout:4000});
    await layerOptions.nth(index).uncheck();
    await page.waitForFunction(({x,y})=>{
      const c=document.querySelector('canvas[aria-label="场地地图，点击设置停车位置并拖动设置朝向"]');
      const [r,g,b]=c.getContext("2d").getImageData(Math.floor(c.width*x/2),Math.floor(c.height*(1-y/2)),1,1).data;
      return r===255&&g===255&&b===255;
    },samples[index]);
  }
  for (let index=0;index<4;index++) await layerOptions.nth(index).check();
  console.log("主录入地图已验证两张代价图、两条路径及开关隐藏");
  await page
    .getByRole("button", { name: "在地图上选择巡检位置", exact: true })
    .click();
  const box = await canvas.boundingBox();
  assert(box && box.width > 0 && box.height > 0);
  await page.mouse.move(box.x + box.width * 0.4, box.y + box.height * 0.6);
  await page.mouse.down();
  await page.mouse.move(box.x + box.width * 0.55, box.y + box.height * 0.45, {
    steps: 5,
  });
  await page.mouse.up();
  await saveDraft.click();
  await page
    .locator(".point-list button")
    .filter({ hasText: "地图选点草稿" })
    .waitFor();
  const saved = await page.evaluate(async () =>
    (await (await fetch("/api/state")).json()).points.find(
      (p) => p.name === "地图选点草稿",
    ),
  );
  assert.equal(saved.verified, false);
  assert(
    Math.abs(saved.pose.x - 0.8) < 0.02 && Math.abs(saved.pose.y - 0.8) < 0.02,
    JSON.stringify(saved.pose),
  );
  assert(
    Math.abs(saved.pose.yaw - Math.PI / 4) < 0.03,
    "地图拖动方向没有保存为车头朝向",
  );
  await page.reload();
  await page
    .locator(".aside-foot")
    .filter({ hasText: "操作者已连接" })
    .waitFor();
  await page
    .locator(".point-list button")
    .filter({ hasText: "地图选点草稿" })
    .click();
  assert.equal(await name.inputValue(), "地图选点草稿");
  assert(
    Math.abs(
      Number(await page.getByLabel("X / 米", { exact: true }).inputValue()) -
        saved.pose.x,
    ) < 1e-6,
  );
  await page.setViewportSize({ width: 390, height: 844 });
  await page.getByRole("button", { name: "安排巡检", exact: true }).click();
  await page.getByLabel("添加巡检点").selectOption(saved.id);
  const saveTask = page.getByRole("button", {
    name: "保存巡检任务",
    exact: true,
  });
  assert(await saveTask.isDisabled(), "最终停车位尚未选择时不能保存任务");
  await page
    .getByRole("button", { name: "在地图上选择最终停车位置", exact: true })
    .click();
  await canvas.scrollIntoViewIfNeeded();
  const phoneBox = await canvas.boundingBox();
  await page.mouse.move(
    phoneBox.x + phoneBox.width * 0.65,
    phoneBox.y + phoneBox.height * 0.5,
  );
  await page.mouse.down();
  await page.mouse.move(
    phoneBox.x + phoneBox.width * 0.8,
    phoneBox.y + phoneBox.height * 0.5,
    { steps: 5 },
  );
  await page.mouse.up();
  await saveTask.click();
  await page.waitForFunction(() =>
    document.body.innerText.includes("巡检顺序与最终停车位已保存。"),
  );
  const task = await page.evaluate(
    async () => (await (await fetch("/api/state")).json()).tasks[0],
  );
  assert.deepEqual(task.point_ids, [saved.id]);
  assert(
    Math.abs(task.final_parking_pose.x - 1.3) < 0.02 &&
      Math.abs(task.final_parking_pose.y - 1) < 0.02,
  );
  assert(Math.abs(task.final_parking_pose.yaw) < 0.03);
  assert(
    writes.every((p) => ["/api/points/save", "/api/tasks/save"].includes(p)),
    `选点流程出现硬件请求：${writes}`,
  );
  console.log(
    "地图选择位置与朝向→保存草稿→刷新读回→手机选择最终停车位通过；无硬件请求",
  );
  await page.setViewportSize({ width: 1280, height: 900 });
  page.on("dialog", (dialog) => dialog.accept());
  async function waitEnabled(text) {
    await page.waitForFunction(
      (text) =>
        !Array.from(document.querySelectorAll("button")).find(
          (b) => b.textContent.trim() === text,
        )?.disabled,
      text,
    );
  }
  await page.getByRole("button", { name: "准备车辆", exact: true }).click();
  await page.getByText("启动时已经下发，不用再点确认。").waitFor();
  assert.equal(
    await page
      .getByRole("button", { name: "确认相机可用于行驶（94档）", exact: true })
      .count(),
    0,
    "准备车辆不再要求确认行驶镜头",
  );
  await page.getByRole("button", { name: "录入巡检点", exact: true }).click();
  await page.getByRole("button", { name: "新增点位", exact: true }).click();
  await page.getByRole("button", { name: "下发目标，让小车前往", exact: true }).click();
  await page.getByText(/^\s*尚未下发：请先在地图上选择停车位置/).waitFor();
  assert.equal((await (await fetch(`${url}/api/state`)).json()).navigation_trial.phase,"IDLE","目标未选择时必须明确提示，并且不能发送导航");
  await page
    .locator('input[placeholder="例如：走廊仪表"]')
    .fill("导航后实车确认点");
  await page.getByLabel("X / 米", { exact: true }).fill("1.3");
  await page.getByLabel("Y / 米", { exact: true }).fill("1.0");
  await page.getByLabel("车头朝向 / °", { exact: true }).fill("10");
  assert(
    await page
      .getByRole("button", { name: "开始停车调角", exact: true })
      .isDisabled(),
    "还没导航到点，不得直接确认拍摄点",
  );
  await waitEnabled("下发目标，让小车前往");
  await page
    .getByRole("button", { name: "下发目标，让小车前往", exact: true })
    .click();
  await waitForState((state)=>state.navigation_trial.phase==="ARRIVED"&&state.navigation_trial.target.x===1.3);
  await page.getByLabel("X / 米", { exact: true }).fill("1.35");
  assert(
    await page
      .getByRole("button", { name: "开始停车调角", exact: true })
      .isDisabled(),
    "修改候选坐标后必须再次导航验证",
  );
  const refinedResponse=page.waitForResponse((response)=>response.request().method()==="POST"&&new URL(response.url()).pathname.endsWith("/api/teach/navigate"));
  await page
    .getByRole("button", { name: "下发目标，让小车前往", exact: true })
    .click();
  const refinedTrial=await (await refinedResponse).json();
  await waitForState((state) => (
      state.navigation_trial.request_id === refinedTrial.request_id &&
      state.navigation_trial.phase === "ARRIVED" &&
      state.navigation_trial.target.x === 1.35
  ));
  await page.reload();
  await page.locator(".aside-foot").filter({hasText:"操作者已连接"}).waitFor();
  await page.waitForFunction(()=>Math.abs(Number(document.querySelector('.editor input[type="number"]')?.value)-1.35)<.001);
  assert(Math.abs(Number(await page.getByLabel("X / 米",{exact:true}).inputValue())-1.35)<.001,"刷新应恢复当前已到达候选，不能重发目标");
  await name.fill("导航后实车确认点");
  await page.getByRole("button", { name: "开始停车调角", exact: true }).click();
  await page.getByText("可以逐步调角", { exact: true }).waitFor();
  const slider=page.getByRole("slider",{name:"相机俯仰目标"});
  const angleRequests=[];
  page.on("request",(request)=>{if(request.method()==="POST"&&new URL(request.url()).pathname.endsWith("/api/teach/step"))angleRequests.push(request.postDataJSON().target_degrees);});
  await slider.evaluate((element)=>{element.value="136.5";element.dispatchEvent(new Event("input",{bubbles:true}));});
  assert.equal(angleRequests.length,0,"拖动预览不能提前发送角度");
  assert(await page.getByRole("button",{name:"4. 确认实车位置与画面，保存巡检点",exact:true}).isDisabled(),"滑块预览未应用时不能保存旧角度");
  await slider.dispatchEvent("change");
  await waitEnabled("4. 确认实车位置与画面，保存巡检点");
  assert.deepEqual(angleRequests,[139.5,138.5,137.5,136.5],"滑块必须逐步等待车端及新画面，不能一次大跳或重复发送");
  await page.getByRole("button", { name: "下发当前目标", exact: true }).click();
  await waitEnabled("4. 确认实车位置与画面，保存巡检点");
  await page
    .getByRole("button", {
      name: "4. 确认实车位置与画面，保存巡检点",
      exact: true,
    })
    .click();
  await page
    .locator(".point-list button")
    .filter({ hasText: "导航后实车确认点" })
    .waitFor();
  const confirmed = await page.evaluate(async () =>
    (await (await fetch("/api/state")).json()).points.find(
      (p) => p.name === "导航后实车确认点",
    ),
  );
  assert.equal(confirmed.verified, true);
  assert(Math.abs(confirmed.pose.x - 1.47) < 0.001,"导航成功后应保存偏差12厘米的实测位置，而不是候选坐标");
  assert.equal(confirmed.reference.navigation_target.x, 1.35);
  assert.equal(confirmed.reference.navigation_result,"SUCCEEDED");
  assert.equal(confirmed.target_degrees,136.5,"保存的应是滑块最终确认角度");
  assert(Math.abs(confirmed.reference.arrival_error.distance_m-.12)<.001);
  assert(Math.abs(confirmed.reference.arrival_error.yaw_rad-.16)<.001);
  await page
    .getByRole("button", { name: "查看此点参考照片", exact: true })
    .click();
  const reference = page.getByAltText("点位试拍参考照片");
  await reference.waitFor();
  await page.waitForFunction(() => {
    const image = document.querySelector('img[alt="点位试拍参考照片"]');
    return image?.complete && image.naturalWidth > 0;
  });
  await page.getByRole("button", { name: "录入巡检点", exact: true }).click();
  await page.evaluate(() => window.scrollTo(0, 0));
  await page.screenshot({ path: `${directory}/tablet.png`, fullPage: true });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.evaluate(() => window.scrollTo(0, 0));
  await page.screenshot({ path: `${directory}/phone.png`, fullPage: true });
  assert.equal(
    await page.evaluate(
      () => document.documentElement.scrollWidth > document.documentElement.clientWidth + 1,
    ),
    false,
    "手机页面出现横向溢出",
  );
  if (process.env.UI_REDESIGN_EVIDENCE) {
    const output = process.env.UI_REDESIGN_EVIDENCE;
    await fs.mkdir(output, { recursive: true });
    await fs.writeFile(`${output}/inspection-state.json`, JSON.stringify(await (await fetch(`${url}/api/state`)).json(), null, 2));
    await fs.copyFile(`${directory}/tablet.png`, `${output}/inspection-tablet.png`);
    await fs.copyFile(`${directory}/phone.png`, `${output}/inspection-phone.png`);
    // 保存正式候选的可见视口，避免超长截图缩小后难以阅读。
    for (const [name, width, height] of [["desktop", 1440, 1100], ["tablet", 1024, 900], ["phone", 390, 844]]) {
      await page.setViewportSize({ width, height });
      await page.evaluate(() => window.scrollTo(0, 0));
      await page.screenshot({ path: `${output}/inspection-${name}-viewport.png` });
    }
  }
  assert.deepEqual(errors, []);
  console.log(
    "真实浏览器完成导航到点→修改坐标再导航→调角新画面→按实车位置保存；平板与手机截图已保存",
  );
} catch (error) {
  if (page) {
    await page
      .screenshot({ path: `${directory}/failure.png`, fullPage: true })
      .catch(() => {});
    console.error(
      "页面状态：",
      await page
        .evaluate(() => ({
          hidden: document.hidden,
          text: document.body.innerText.slice(0, 2200),
        }))
        .catch(() => null),
    );
  }
  throw error;
} finally {
  const exited=new Promise((resolve)=>{
    if(child.exitCode!==null||child.signalCode!==null)resolve();
    else child.once("exit",resolve);
  });
  if (browser) await browser.close();
  try {
    process.kill(-child.pid, "SIGTERM");
  } catch {}
  await Promise.race([exited,new Promise((resolve)=>setTimeout(resolve,500))]);
  if(child.exitCode===null&&child.signalCode===null) {
    try{process.kill(-child.pid,"SIGKILL");}catch{}
    await Promise.race([exited,new Promise((resolve)=>setTimeout(resolve,500))]);
  }
  await fs.rm(profile, { recursive: true, force: true, maxRetries: 5, retryDelay: 100 });
}
