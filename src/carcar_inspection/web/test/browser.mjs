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
  const writes = [];
  page.on("request", (request) => {
    if (request.method() === "POST")
      writes.push(new URL(request.url()).pathname);
  });
  const name = page.locator('input[placeholder="例如：走廊仪表"]');
  await name.fill("地图选点草稿");
  const saveDraft = page.getByRole("button", {
    name: "保存巡检点草稿",
    exact: true,
  });
  assert(
    await saveDraft.isDisabled(),
    "未选位置时不能把默认零坐标保存为巡检点",
  );
  const canvas = page.getByLabel("场地地图，点击设置停车位置并拖动设置朝向");
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
  await page.getByRole("button", { name: "巡检任务", exact: true }).click();
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
  await page.getByRole("button", { name: "点位示教", exact: true }).click();
  await page.getByRole("button", { name: "新增点位", exact: true }).click();
  await page
    .locator('input[placeholder="例如：走廊仪表"]')
    .fill("网页示教仪表");
  await page
    .getByRole("button", { name: "1. 开始角度调整", exact: true })
    .click();
  await page.getByText("可以逐步调角", { exact: true }).waitFor();
  await page.getByRole("button", { name: "下发当前目标", exact: true }).click();
  const confirm = page.getByRole("button", {
    name: "3. 确认此画面，保存巡检点",
    exact: true,
  });
  await page.waitForFunction(
    () =>
      !Array.from(document.querySelectorAll("button")).find(
        (b) => b.textContent.trim() === "3. 确认此画面，保存巡检点",
      )?.disabled,
  );
  await confirm.click();
  await page
    .locator(".point-list button")
    .filter({ hasText: "网页示教仪表" })
    .waitFor();
  await page.getByText("点位已确认", { exact: true }).waitFor();
  await page.evaluate(() => window.scrollTo(0, 0));
  await page.screenshot({ path: `${directory}/tablet.png`, fullPage: true });
  await page.setViewportSize({ width: 390, height: 844 });
  await page.evaluate(() => window.scrollTo(0, 0));
  await page.screenshot({ path: `${directory}/phone.png`, fullPage: true });
  assert.equal(
    await page.evaluate(
      () => document.documentElement.scrollWidth > innerWidth + 1,
    ),
    false,
    "手机页面出现横向溢出",
  );
  assert.deepEqual(errors, []);
  console.log(
    "真实浏览器完成停车→下发目标→新画面→人工确认保存；平板与手机截图已保存",
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
  if (browser) await browser.close();
  try {
    process.kill(-child.pid, "SIGTERM");
  } catch {}
  await fs.rm(profile, { recursive: true, force: true });
}
