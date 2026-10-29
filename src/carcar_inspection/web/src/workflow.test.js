import { describe, it, expect } from "vitest";
import {
  stepTarget,
  pixelToMap,
  mapToPixel,
  canConfirm,
  approachCount,
  uploadLabel,
  planSlots,
  pointPhrase,
  tiltPhrase,
  confirmReason,
  sitePath,
} from "./workflow.js";
describe("示教显示约束", () => {
  it("一步目标不越过机械安全范围", () => {
    expect(stepTarget(159.5, 1)).toBe(160);
    expect(stepTarget(85, -1)).toBe(85);
  });
  it("新角度不能沿用旧画面的确认", () => {
    const t = { phase: "AWAITING_CONFIRMATION", revision: 9 };
    expect(canConfirm(t, { revision: 8, sequence: 10 }, true)).toBe(false);
    expect(canConfirm(t, { revision: 9, sequence: 10 }, false)).toBe(false);
    expect(canConfirm(t, { revision: 9, sequence: 10 }, true)).toBe(true);
  });
  it("缩放前的地图像素按原点旋转转换为米制坐标", () => {
    const m = {
      height: 100,
      resolution: 0.05,
      origin: { x: 2, y: 3, yaw: Math.PI / 2 },
    };
    const p = pixelToMap(20, 80, m);
    expect(p.x).toBeCloseTo(1);
    expect(p.y).toBeCloseTo(4);
    expect(mapToPixel(p, m).u).toBeCloseTo(20);
    expect(mapToPixel(p, m).v).toBeCloseTo(80);
  });
  it("备选停车位数量和上传状态分开显示", () => {
    expect(approachCount({ approaches: [{}, {}, {}] })).toBe(3);
    expect(approachCount({})).toBe(1);
    expect(uploadLabel("CONFIRMED")).toBe("已上传");
    expect(uploadLabel("PENDING")).toBe("待上传");
    expect(uploadLabel("PENDING_CONFIGURATION")).toBe("未配置上传");
  });
  it("四个观测点落在200平方米场地内，并用白话区分确认进度", () => {
    const slots = planSlots(4);
    expect(slots).toHaveLength(4);
    expect(planSlots(1)).toHaveLength(1);
    expect(planSlots(6)).toHaveLength(6);
    expect(planSlots(99)).toHaveLength(20);
    for (const slot of slots) {
      expect(slot.x).toBeGreaterThan(0);
      expect(slot.x).toBeLessThan(20);
      expect(slot.y).toBeGreaterThan(0);
      expect(slot.y).toBeLessThan(10);
    }
    expect(pointPhrase({}).title).toBe("还没到现场");
    expect(pointPhrase({ field_fixed: true }).title).toBe("车已经停在这里");
    expect(pointPhrase({ verified: true }).title).toBe("这个点好了");
    expect(tiltPhrase(140.5)).toBe("镜头大致平视");
    expect(tiltPhrase(150)).toContain("偏低");
    expect(tiltPhrase(100)).toContain("偏高");
    expect(confirmReason({ still: false, rgb: true }, "READY")).toBe(
      "请先停稳",
    );
    expect(confirmReason({ still: true, rgb: false }, "READY")).toBe(
      "画面还没更新",
    );
    expect(sitePath("/api/state", "/inspect/")).toBe("/inspect/api/state");
    expect(sitePath("/api/state", "/")).toBe("/api/state");
  });
});

describe("实车到点录入", () => {
  it("只有到达当前候选目标才允许继续试拍；坐标改动要求重新导航", async () => {
    const { trialMatchesTarget, trialLabel } = await import("./workflow.js");
    const target = { x: 1.2, y: 0.8, yaw: 0.2 };
    const trial = { phase: "ARRIVED", request_id: "trial1", target };
    expect(trialMatchesTarget(trial, target)).toBe(true);
    expect(trialMatchesTarget({ ...trial, phase: "NAVIGATING" }, target)).toBe(
      false,
    );
    expect(trialMatchesTarget({ ...trial, phase: "CANCELED" }, target)).toBe(
      false,
    );
    expect(trialMatchesTarget(trial, { ...target, x: 1.25 })).toBe(false);
    expect(trialMatchesTarget(trial, { ...target, yaw: 0.21 })).toBe(false);
    expect(trialLabel("STOPPING")).toContain("停车");
    expect(trialLabel("ARRIVED")).toBe("已到达并停稳");
  });
});
