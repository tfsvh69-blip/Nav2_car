export const labels = {
  IDLE: "尚未开始",
  ACQUIRING_HOLD: "正在保持停车",
  WAITING_STOP: "等待车体停稳",
  READY: "可以逐步调角",
  SENDING: "正在发送目标",
  WAITING_FRAME: "等待新画面",
  AWAITING_CONFIRMATION: "请检查画面并确认",
  CONFIRMED: "点位已确认",
  FAULT: "需要重新开始",
  RUNNING: "巡检中",
  PAUSING: "正在暂停停车",
  PAUSED: "已暂停",
  CANCELING: "正在取消停车",
  CANCELED: "已取消",
  FAILED: "执行失败",
  SUCCEEDED: "已完成",
  INTERRUPTED: "执行已中断",
};
export function stepTarget(current, delta) {
  return Math.round(Math.max(85, Math.min(160, current + delta)) * 10) / 10;
}
export function requestId() {
  const values = new Uint8Array(16);
  crypto.getRandomValues(values);
  return [...values].map((v) => v.toString(16).padStart(2, "0")).join("");
}
export function pixelToMap(u, v, map) {
  const x = u * map.resolution,
    y = (map.height - v) * map.resolution,
    c = Math.cos(map.origin.yaw),
    s = Math.sin(map.origin.yaw);
  return { x: map.origin.x + c * x - s * y, y: map.origin.y + s * x + c * y };
}
export function mapToPixel(p, map) {
  const x = p.x - map.origin.x,
    y = p.y - map.origin.y,
    c = Math.cos(map.origin.yaw),
    s = Math.sin(map.origin.yaw);
  return {
    u: (c * x + s * y) / map.resolution,
    v: map.height - (-s * x + c * y) / map.resolution,
  };
}
export function approachCount(point) {
  return Array.isArray(point?.approaches) && point.approaches.length
    ? point.approaches.length
    : 1;
}
export function uploadLabel(status) {
  if (status === "CONFIRMED") return "已上传";
  if (status === "PENDING") return "待上传";
  if (status === "PENDING_CONFIGURATION") return "未配置上传";
  return "未上传";
}
export const planPointLimit = 20;
function perimeter(distance) {
  const width = 14,
    height = 6,
    loop = 2 * (width + height);
  let rest = ((distance % loop) + loop) % loop;
  const x0 = 3,
    y0 = 2;
  if (rest <= width) return { x: x0 + rest, y: y0 };
  rest -= width;
  if (rest <= height) return { x: x0 + width, y: y0 + rest };
  rest -= height;
  if (rest <= width) return { x: x0 + width - rest, y: y0 + height };
  rest -= width;
  return { x: x0, y: y0 + height - rest };
}
export function planSlots(count = 4) {
  const total = Math.max(1, Math.min(planPointLimit, Math.round(Number(count) || 1)));
  const center = { x: 10, y: 5 };
  return Array.from({ length: total }, (_, index) => {
    const place = perimeter((index * 40) / total);
    return {
      name: `观测点 ${index + 1}`,
      sort_index: index,
      x: Math.round(place.x * 100) / 100,
      y: Math.round(place.y * 100) / 100,
      yaw: Math.atan2(center.y - place.y, center.x - place.x),
    };
  });
}
export function pointPhrase(point) {
  if (point?.verified)
    return {
      stage: "ready",
      title: "这个点好了",
      detail: "位置和画面都确认过，可以排进巡检",
    };
  if (point?.field_fixed)
    return {
      stage: "posed",
      title: "车已经停在这里",
      detail: "位置记下了，还要看镜头里的画面",
    };
  return {
    stage: "planned",
    title: "还没到现场",
    detail: "规划位置已有，等你把车开过去",
  };
}
export function tiltPhrase(degrees) {
  const value = Number(degrees);
  if (!Number.isFinite(value)) return "镜头角度还没定";
  if (Math.abs(value - 140.5) < 0.2) return "镜头大致平视";
  if (value > 140.5) return "镜头偏低，对着更低的地方";
  return "镜头偏高，对着更高的地方";
}
export function confirmReason(health, teachingPhase) {
  if (health && health.still === false) return "请先停稳";
  if (health && health.rgb === false) return "画面还没更新";
  if (teachingPhase !== "AWAITING_CONFIRMATION") return "先看一眼新画面";
  return "请再看一眼画面";
}
export function sitePath(path, base = "/") {
  const root = String(base || "/").replace(/\/$/, "");
  return `${root}${path}`;
}
export function canConfirm(teaching, rendered, connected) {
  return (
    connected &&
    teaching.phase === "AWAITING_CONFIRMATION" &&
    rendered.revision === teaching.revision &&
    rendered.sequence > 0
  );
}
