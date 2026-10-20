<script setup>
import ConsoleIcon from "./ConsoleIcon.vue";
import {
  ref,
  reactive,
  computed,
  onMounted,
  onBeforeUnmount,
  nextTick,
} from "vue";
import {
  labels,
  stepTarget,
  pixelToMap,
  mapToPixel,
  canConfirm,
  requestId,
  approachCount,
  uploadLabel,
  planSlots,
  planPointLimit,
  pointPhrase,
  tiltPhrase,
  confirmReason,
  sitePath,
} from "./workflow.js";
function site(path) {
  return sitePath(path, import.meta.env.BASE_URL);
}
const page = ref("points"),
  state = ref({
    maps: [],
    points: [],
    tasks: [],
    runs: [],
    health: {},
    teaching: { phase: "IDLE", revision: 0, target_degrees: 140.5 },
  });
const session = ref(""),
  connected = ref(false),
  notice = ref(""),
  busy = ref(false),
  selectedMap = ref(""),
  videoUrl = ref(""),
  rendered = reactive({ sequence: 0, revision: -1 });
const canvas = ref(null),
  planCanvas = ref(null),
  selectedId = ref(""),
  angleOpen = ref(false),
  alternateOpen = ref(false),
  planCount = ref(4),
  planReady = ref(false),
  mapSelecting = ref(false),
  draftPoseChosen = ref(false),
  routePoseChosen = ref(false),
  draft = reactive({
    id: "",
    revision: 0,
    name: "",
    x: 0,
    y: 0,
    yaw: 0,
    target_degrees: 140.5,
  });
const route = reactive({
  id: "",
  revision: 0,
  name: "日常巡检",
  point_ids: [],
  x: 0,
  y: 0,
  yaw: 0,
});
const navItems = [
  ["overview", "车辆概览"],
  ["points", "点位示教"],
  ["sketch", "规划草图"],
  ["tasks", "巡检任务"],
  ["running", "正在巡检"],
  ["records", "巡检记录"],
];
const pageDescriptions = {
  overview: "车辆、点位和任务，一目了然。",
  points: "标记停车位置，确认相机视角。",
  sketch: "先把想法放在图上，再到现场确认。",
  tasks: "安排巡检顺序，设定最终停车位置。",
  running: "跟随任务进度，查看采集结果。",
  records: "每次巡检，都有迹可循。",
};
const teaching = computed(() => state.value.teaching),
  confirmable = computed(
    () =>
      canConfirm(teaching.value, rendered, connected.value) &&
      state.value.health.still &&
      state.value.health.rgb,
  );
const points = computed(() =>
  state.value.points.filter((p) => p.map_id === state.value.map?.id),
);
const editPoseChosen = computed(() =>
  page.value === "tasks" ? routePoseChosen.value : draftPoseChosen.value,
);
const healthNames = {
  localization: "定位",
  map_matches: "地图一致",
  odom: "轮式反馈",
  scan: "雷达",
  rgb: "相机",
  hardware_interface: "角度接口",
};
const floorPoints = computed(() =>
  points.value
    .slice()
    .sort(
      (a, b) =>
        (a.sort_index ?? 0) - (b.sort_index ?? 0) ||
        a.name.localeCompare(b.name, "zh"),
    ),
);
const selectedPoint = computed(
  () =>
    floorPoints.value.find((item) => item.id === selectedId.value) ||
    floorPoints.value[0] ||
    null,
);
const allPointsReady = computed(
  () =>
    floorPoints.value.length > 0 &&
    floorPoints.value.every((item) => item.verified),
);
const primaryStep = computed(() => {
  const point = selectedPoint.value;
  if (!point) return { id: "none", label: "先放上观测点", enabled: false };
  const stage = pointPhrase(alternateOpen.value ? {} : point).stage;
  const planning = !!state.value.map?.planning;
  if (alternateOpen.value) {
    if (!state.value.hardware_enabled || planning)
      return {
        id: "wait",
        label: "接上小车并换上真地图后，才能增加另一个停靠点",
        enabled: false,
      };
    if (!angleOpen.value)
      return { id: "angle", label: "看镜头，调角度", enabled: true };
    return {
      id: "confirm-approach",
      label: "这个角度可以",
      enabled: confirmable.value,
    };
  }
  if (stage === "ready")
    return { id: "ready", label: "这个点好了", enabled: false };
  if (stage === "planned") {
    if (planning || !state.value.pose)
      return {
        id: "wait-pose",
        label: "先在图上摆好；等小车定位好了再来记。",
        enabled: false,
      };
    return { id: "record", label: "我已把车开到这里", enabled: true };
  }
  if (!angleOpen.value) {
    if (!state.value.hardware_enabled || planning)
      return {
        id: "wait-hardware",
        label: "接上小车并换上真地图后才能看镜头",
        enabled: false,
      };
    return { id: "angle", label: "看镜头，调角度", enabled: true };
  }
  return { id: "confirm", label: "这个角度可以", enabled: confirmable.value };
});
let socket = null,
  pollTimer = null,
  retryTimer = null,
  stopped = false,
  polling = false,
  videoLoading = false,
  mapImage = null,
  mapVersion = "",
  sequence = 0,
  challenge = "",
  intent = { x: 0, yaw: 0 },
  pressed = false,
  sourceStamp = ref("");
async function post(path, body = {}) {
  if (!session.value) throw new Error("请等待操作者连接建立");
  const response = await fetch(site(path), {
    method: "POST",
    headers: { "Content-Type": "application/json", "X-Mapping-Console": "1" },
    body: JSON.stringify({ ...body, session: session.value }),
  });
  const value = await response.json();
  if (!response.ok) throw new Error(value.error || "操作未完成");
  return value;
}
async function action(fn) {
  if (busy.value) return;
  busy.value = true;
  notice.value = "";
  try {
    await fn();
    await poll();
  } catch (error) {
    notice.value = error.message;
  } finally {
    busy.value = false;
  }
}
function connect() {
  if (stopped || socket || document.hidden) return;
  const ws = new WebSocket(
    `${location.protocol === "https:" ? "wss" : "ws"}://${location.host}${site("/ws/control")}`,
  );
  socket = ws;
  ws.onmessage = (event) => {
    if (socket !== ws) return;
    let data;
    try {
      data = JSON.parse(event.data);
    } catch {
      return;
    }
    if (data.session) {
      session.value = data.session;
      connected.value = true;
    }
    if (data.challenge) {
      challenge = data.challenge;
      state.value.manual = data;
      if (pressed) sendDrive();
    }
  };
  ws.onclose = () => {
    if (socket !== ws) return;
    socket = null;
    session.value = "";
    connected.value = false;
    pressed = false;
    intent = { x: 0, yaw: 0 };
    rendered.sequence = 0;
    if (!stopped && !document.hidden) retryTimer = setTimeout(connect, 1200);
  };
  ws.onerror = () => ws.close();
}
function send(data) {
  if (socket?.readyState === WebSocket.OPEN) socket.send(JSON.stringify(data));
}
function sendDrive() {
  if (challenge && connected.value)
    send({ action: "drive", sequence: ++sequence, challenge, ...intent });
}
function press(event, x, yaw) {
  event.preventDefault();
  if (!state.value.manual?.armed) return;
  pressed = true;
  intent = { x, yaw };
  event.currentTarget.setPointerCapture(event.pointerId);
  sendDrive();
}
function release() {
  pressed = false;
  intent = { x: 0, yaw: 0 };
  send({ action: "release", sequence: ++sequence });
}
function stopManual() {
  release();
  send({ action: "stop" });
}
function visibility() {
  if (document.hidden) {
    release();
    socket?.close();
  } else connect();
}
async function poll() {
  if (polling || stopped) return;
  polling = true;
  try {
    const response = await fetch(site("/api/state"));
    if (!response.ok) throw new Error("状态查询暂时不可用");
    const value = await response.json();
    if (value.teaching.revision !== state.value.teaching.revision) {
      rendered.sequence = 0;
      rendered.revision = -1;
    }
    state.value = { ...value, manual: state.value.manual };
    if (
      page.value === "sketch" &&
      !floorPoints.value.some((item) => item.id === selectedId.value)
    )
      selectedId.value = floorPoints.value[0]?.id || "";
    if (!value.health.rgb && videoUrl.value) {
      URL.revokeObjectURL(videoUrl.value);
      videoUrl.value = "";
      rendered.sequence = 0;
    }
    if (value.map && value.map.version !== mapVersion) {
      mapVersion = value.map.version;
      selectedMap.value = value.map.id;
      const image = new Image();
      image.onload = () => {
        mapImage = image;
        draw();
        planDraw();
      };
      image.src = site(`/api/map.png?v=${mapVersion}`);
    }
    await nextTick();
    draw();
    planDraw();
    if (
      page.value === "sketch" &&
      session.value &&
      !planReady.value &&
      !value.map
    ) {
      planReady.value = true;
      await post("/api/plan/floor");
    }
    if (
      page.value === "points" ||
      page.value === "sketch" ||
      page.value === "overview"
    )
      await preview();
  } catch (error) {
    if (!notice.value)
      notice.value = `${error.message}；车端自动任务继续独立执行。`;
  } finally {
    polling = false;
  }
}
async function preview() {
  if (videoLoading) return;
  videoLoading = true;
  let newUrl;
  try {
    const response = await fetch(site("/api/video.jpg"));
    if (!response.ok) {
      rendered.sequence = 0;
      return;
    }
    const frame = Number(response.headers.get("X-Frame-Sequence")),
      revision = teaching.value.revision;
    newUrl = URL.createObjectURL(await response.blob());
    const image = new Image();
    image.src = newUrl;
    await image.decode();
    if (stopped || revision !== teaching.value.revision) {
      URL.revokeObjectURL(newUrl);
      return;
    }
    const previous = videoUrl.value;
    videoUrl.value = newUrl;
    if (previous) URL.revokeObjectURL(previous);
    const stamp = response.headers.get("X-Source-Stamp");
    sourceStamp.value = stamp
      ? new Date(Number(stamp) / 1e6).toLocaleTimeString("zh-CN", {
          hour12: false,
          timeZone: "Asia/Shanghai",
        })
      : "";
    await nextTick();
    if (teaching.value.phase === "AWAITING_CONFIRMATION" && session.value) {
      await post("/api/teach/viewed", { revision, frame_sequence: frame });
      rendered.revision = revision;
      rendered.sequence = frame;
    }
  } catch {
    rendered.sequence = 0;
  } finally {
    videoLoading = false;
  }
}
async function chooseMap() {
  await post("/api/map/select", { map_id: selectedMap.value });
  mapVersion = "";
  draft.id = "";
  draft.revision = 0;
  route.id = "";
  route.revision = 0;
  route.point_ids = [];
  draftPoseChosen.value = false;
  routePoseChosen.value = false;
  mapSelecting.value = false;
  notice.value =
    page.value === "tasks"
      ? "地图已载入，请安排巡检点并选择最终停车位置。"
      : "地图已载入，点击“在地图上选择巡检位置”开始标点。";
}
function poseEdited(target) {
  if (target === route) routePoseChosen.value = true;
  else draftPoseChosen.value = true;
  draw();
}
function usePose(target) {
  const p = state.value.pose;
  if (!p) {
    notice.value = "当前位置尚未由地图定位得到";
    return;
  }
  Object.assign(target, { x: p.x, y: p.y, yaw: (p.yaw * 180) / Math.PI });
  poseEdited(target);
  mapSelecting.value = false;
  notice.value = "已记录小车当前的地图定位估计，请核对坐标和车头朝向。";
}
function poseOf(target) {
  return {
    x: Number(target.x),
    y: Number(target.y),
    yaw: (Number(target.yaw) * Math.PI) / 180,
  };
}
async function saveDraft() {
  const point = {
    name: draft.name,
    pose: poseOf(draft),
    target_degrees: Number(draft.target_degrees),
  };
  if (draft.id) point.id = draft.id;
  const p = await post("/api/points/save", {
    point,
    expected_revision: draft.revision,
  });
  draft.id = p.id;
  draft.revision = p.revision;
  notice.value = "草稿已保存，现场试拍确认后可用于巡检。";
}
async function begin() {
  await post("/api/teach/begin");
  rendered.sequence = 0;
  notice.value = "已请求停车保持；等待轮式反馈停稳后，再点击一步调整。";
}
async function step(delta) {
  await post("/api/teach/step", {
    revision: teaching.value.revision,
    target_degrees: stepTarget(teaching.value.target_degrees, delta),
  });
  rendered.sequence = 0;
}
async function confirm(purpose = "point") {
  const payload = {
    purpose,
    revision: teaching.value.revision,
    frame_sequence: rendered.sequence,
    name: draft.name,
    expected_revision: draft.revision,
  };
  if (draft.id) payload.id = draft.id;
  if (purpose === "approach") payload.point_id = draft.id;
  const p = await post("/api/teach/confirm", payload);
  if (purpose === "point" || purpose === "approach") {
    draft.id = p.id;
    draft.revision = p.revision;
    usePoint(p);
    notice.value =
      purpose === "approach"
        ? "已追加备选停车位。停车保持继续生效。"
        : "参考照片与巡检点已保存。停车保持继续生效。";
  } else
    notice.value =
      "导航参考已确认，保持停车；需要行驶时再点击遥控使能或启动任务。";
}
function uploadState(_run, photo) {
  const row = (state.value.uploads || []).find((item) => item.id === photo.id);
  return row?.status || photo.upload_status;
}
function usePoint(p) {
  Object.assign(draft, {
    id: p.id,
    revision: p.revision,
    name: p.name,
    x: p.pose.x,
    y: p.pose.y,
    yaw: (p.pose.yaw * 180) / Math.PI,
    target_degrees: p.target_degrees,
  });
  draftPoseChosen.value = true;
  mapSelecting.value = false;
}
function newPoint() {
  Object.assign(draft, {
    id: "",
    revision: 0,
    name: "",
    x: 0,
    y: 0,
    yaw: 0,
    target_degrees: teaching.value.target_degrees,
  });
  draftPoseChosen.value = false;
  mapSelecting.value = false;
  notice.value = "新点位：先载入地图，再选择巡检位置或记录小车当前位置。";
  draw();
}
function newTask() {
  Object.assign(route, {
    id: "",
    revision: 0,
    name: "新的巡检任务",
    point_ids: [],
    x: 0,
    y: 0,
    yaw: 0,
  });
  routePoseChosen.value = false;
  mapSelecting.value = false;
  draw();
}
async function localize() {
  if (
    window.confirm(
      "确认实物车辆就在地图编辑箭头的位置与朝向。将提交定位初值，车体须保持停车。",
    )
  )
    await post("/api/localize", { pose: poseOf(draft) });
}
function addPoint(id) {
  if (!route.point_ids.includes(id)) route.point_ids.push(id);
}
function move(index, delta) {
  const next = index + delta;
  if (next < 0 || next >= route.point_ids.length) return;
  [route.point_ids[index], route.point_ids[next]] = [
    route.point_ids[next],
    route.point_ids[index],
  ];
}
async function saveTask() {
  const task = {
    name: route.name,
    point_ids: route.point_ids,
    final_parking_pose: poseOf(route),
  };
  if (route.id) task.id = route.id;
  const value = await post("/api/tasks/save", {
    task,
    expected_revision: route.revision,
  });
  route.id = value.id;
  route.revision = value.revision;
  notice.value = "巡检顺序与最终停车位已保存。";
}
function useTask(t) {
  Object.assign(route, {
    id: t.id,
    revision: t.revision,
    name: t.name,
    point_ids: [...t.point_ids],
    x: t.final_parking_pose.x,
    y: t.final_parking_pose.y,
    yaw: (t.final_parking_pose.yaw * 180) / Math.PI,
  });
  routePoseChosen.value = true;
  mapSelecting.value = false;
}
async function startTask(t) {
  await post("/api/run/start", { task_id: t.id, request_id: requestId() });
  page.value = "running";
}
function draw() {
  const element = canvas.value,
    m = state.value.map;
  if (!element || !mapImage || !m) return;
  const ctx = element.getContext("2d");
  const density = window.devicePixelRatio || 1;
  const displayWidth = Math.min(
    element.parentElement.clientWidth,
    (540 * m.width) / m.height,
  );
  element.width = Math.max(1, Math.round(displayWidth * density));
  element.height = Math.max(
    1,
    Math.round((element.width * m.height) / m.width),
  );
  const scale = element.width / m.width;
  ctx.scale(scale, scale);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImage, 0, 0, m.width, m.height);
  const mark = (p, color, label) => {
    const q = mapToPixel(p, m),
      radius = (5 * density) / scale;
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = (1.5 * density) / scale;
    ctx.beginPath();
    ctx.arc(q.u, q.v, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(q.u, q.v);
    ctx.lineTo(
      q.u + Math.cos(p.yaw - m.origin.yaw) * radius * 3,
      q.v - Math.sin(p.yaw - m.origin.yaw) * radius * 3,
    );
    ctx.stroke();
    ctx.font = `${(12 * density) / scale}px sans-serif`;
    ctx.fillText(label, q.u + radius * 2, q.v - radius * 2);
  };
  points.value.forEach((p, i) =>
    mark(p.pose, p.verified ? "#117b69" : "#c08520", `${i + 1}`),
  );
  if (state.value.pose) mark(state.value.pose, "#2878d4", "车");
  if (editPoseChosen.value)
    mark(
      poseOf(page.value === "tasks" ? route : draft),
      "#c24c72",
      page.value === "tasks" ? "终点" : "编辑",
    );
}
let mapPointer = null;
function mapDown(event) {
  const m = state.value.map;
  if (!m || !mapSelecting.value) return;
  const rect = canvas.value.getBoundingClientRect(),
    u = ((event.clientX - rect.left) * m.width) / rect.width,
    v = ((event.clientY - rect.top) * m.height) / rect.height;
  mapPointer = { u, v };
  const p = pixelToMap(u, v, m);
  Object.assign(page.value === "tasks" ? route : draft, p);
  poseEdited(page.value === "tasks" ? route : draft);
  canvas.value.setPointerCapture(event.pointerId);
  draw();
}
function mapMove(event) {
  if (!mapPointer) return;
  const m = state.value.map,
    rect = canvas.value.getBoundingClientRect(),
    u = ((event.clientX - rect.left) * m.width) / rect.width,
    v = ((event.clientY - rect.top) * m.height) / rect.height;
  if (Math.hypot(u - mapPointer.u, v - mapPointer.v) > 2) {
    const angle =
      ((Math.atan2(mapPointer.v - v, u - mapPointer.u) + m.origin.yaw) * 180) /
      Math.PI;
    (page.value === "tasks" ? route : draft).yaw =
      ((((angle + 180) % 360) + 360) % 360) - 180;
  }
  draw();
}
function mapUp() {
  if (mapPointer) {
    mapSelecting.value = false;
    notice.value =
      page.value === "tasks"
        ? "最终停车位置与车头朝向已选好，安排巡检顺序后保存任务。选点不会让小车移动。"
        : "位置与车头朝向已选好，填写名称后保存草稿。选点不会让小车移动。";
  }
  mapPointer = null;
}
function shownPose(point) {
  return point.nudge || point.pose;
}
function clampPlan(x, y) {
  const map = state.value.map;
  const minX = map.origin.x + 0.4,
    minY = map.origin.y + 0.4;
  const maxX = map.origin.x + map.width * map.resolution - 0.4;
  const maxY = map.origin.y + map.height * map.resolution - 0.4;
  return {
    x: Math.min(maxX, Math.max(minX, x)),
    y: Math.min(maxY, Math.max(minY, y)),
  };
}
function planDraw() {
  const element = planCanvas.value,
    map = state.value.map;
  if (!element || !mapImage || !map || page.value !== "sketch") return;
  const ctx = element.getContext("2d");
  const density = window.devicePixelRatio || 1;
  const displayWidth = element.parentElement.clientWidth;
  element.width = Math.max(1, Math.round(displayWidth * density));
  element.height = Math.max(
    1,
    Math.round((element.width * map.height) / map.width),
  );
  const scale = element.width / map.width;
  ctx.setTransform(scale, 0, 0, scale, 0, 0);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImage, 0, 0, map.width, map.height);
  ctx.strokeStyle = "rgba(11,118,102,0.18)";
  ctx.lineWidth = 1 / (scale / density);
  const step = 1 / map.resolution;
  for (let x = 0; x <= map.width; x += step) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, map.height);
    ctx.stroke();
  }
  for (let y = 0; y <= map.height; y += step) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(map.width, y);
    ctx.stroke();
  }
  const mark = (pose, color, label, heavy) => {
    const pixel = mapToPixel(pose, map);
    const radius = ((heavy ? 9 : 6) * density) / scale;
    ctx.strokeStyle = color;
    ctx.fillStyle = color;
    ctx.lineWidth = ((heavy ? 3 : 1.5) * density) / scale;
    ctx.beginPath();
    ctx.arc(pixel.u, pixel.v, radius, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(pixel.u, pixel.v);
    ctx.lineTo(
      pixel.u + Math.cos(pose.yaw - map.origin.yaw) * radius * 3.2,
      pixel.v - Math.sin(pose.yaw - map.origin.yaw) * radius * 3.2,
    );
    ctx.stroke();
    ctx.font = `${((heavy ? 16 : 13) * density) / scale}px sans-serif`;
    ctx.fillText(label, pixel.u + radius * 1.4, pixel.v - radius * 1.4);
  };
  floorPoints.value.forEach((point, index) => {
    const pose = shownPose(point);
    (point.approaches || []).slice(1).forEach((approach) => {
      mark(approach.pose, "#8aa399", "", false);
    });
    mark(
      pose,
      point.id === selectedPoint.value?.id
        ? "#0b7666"
        : point.verified
          ? "#117b69"
          : "#c08520",
      String(index + 1),
      point.id === selectedPoint.value?.id,
    );
  });
  if (allPointsReady.value && floorPoints.value.length > 1) {
    ctx.strokeStyle = "#0b7666";
    ctx.lineWidth = (2 * density) / scale;
    ctx.beginPath();
    floorPoints.value.forEach((point, index) => {
      const pixel = mapToPixel(shownPose(point), map);
      if (index === 0) ctx.moveTo(pixel.u, pixel.v);
      else ctx.lineTo(pixel.u, pixel.v);
    });
    ctx.stroke();
  }
  if (state.value.pose) mark(state.value.pose, "#2878d4", "车", false);
}
function planHit(event) {
  const map = state.value.map,
    rect = planCanvas.value.getBoundingClientRect();
  const u = ((event.clientX - rect.left) * map.width) / rect.width;
  const v = ((event.clientY - rect.top) * map.height) / rect.height;
  const scale = rect.width / map.width;
  let found = null;
  for (const point of floorPoints.value) {
    const pose = shownPose(point);
    const pixel = mapToPixel(pose, map);
    const radius = 28 / scale;
    const tipU =
      pixel.u + Math.cos(pose.yaw - map.origin.yaw) * ((9 * 3.2) / scale);
    const tipV =
      pixel.v - Math.sin(pose.yaw - map.origin.yaw) * ((9 * 3.2) / scale);
    if (Math.hypot(u - tipU, v - tipV) < radius) found = { point, mode: "yaw" };
    else if (Math.hypot(u - pixel.u, v - pixel.v) < radius)
      found = { point, mode: "move" };
  }
  return found;
}
let planDrag = null;
function planDown(event) {
  if (!state.value.map) return;
  const hit = planHit(event);
  if (!hit) return;
  event.preventDefault();
  selectedId.value = hit.point.id;
  angleOpen.value = false;
  draft.name = hit.point.name;
  planDrag = { ...hit, pointerId: event.pointerId, moved: false };
  planCanvas.value.setPointerCapture(event.pointerId);
  planDraw();
}
function planMove(event) {
  if (!planDrag || planDrag.pointerId !== event.pointerId) return;
  const map = state.value.map,
    rect = planCanvas.value.getBoundingClientRect();
  const world = pixelToMap(
    ((event.clientX - rect.left) * map.width) / rect.width,
    ((event.clientY - rect.top) * map.height) / rect.height,
    map,
  );
  const current = shownPose(planDrag.point);
  planDrag.moved = true;
  if (planDrag.mode === "move") {
    const place = clampPlan(world.x, world.y);
    planDrag.point.nudge = { ...current, ...place };
  } else {
    planDrag.point.nudge = {
      ...current,
      yaw: Math.atan2(world.y - current.y, world.x - current.x),
    };
  }
  planDraw();
}
async function planUp(event) {
  if (!planDrag || (event && planDrag.pointerId !== event.pointerId)) return;
  const drag = planDrag;
  planDrag = null;
  const pose = drag.point.nudge;
  delete drag.point.nudge;
  if (!drag.moved || !pose) {
    planDraw();
    return;
  }
  await action(() =>
    savePlanPoint(drag.point, pose, {
      field_fixed: false,
      name: drag.point.name,
    }),
  );
}
async function savePlanPoint(point, pose, extra) {
  const saved = await post("/api/points/save", {
    point: {
      id: point.id,
      name: extra.name || point.name,
      pose,
      target_degrees: Number(point.target_degrees || 140.5),
      sort_index: point.sort_index ?? 0,
      field_fixed: !!extra.field_fixed,
    },
    expected_revision: point.revision,
  });
  selectedId.value = saved.id;
  notice.value = extra.field_fixed
    ? "位置记下了。接下来看镜头里的画面。"
    : "这个点的位置已经记下。";
}
function placeOnMap(slot) {
  const map = state.value.map;
  if (!map || map.planning) return slot;
  return {
    ...slot,
    x: map.origin.x + (slot.x / 20) * map.width * map.resolution,
    y: map.origin.y + (slot.y / 10) * map.height * map.resolution,
  };
}
async function layChosen() {
  const count = Math.max(
    1,
    Math.min(planPointLimit, Math.round(Number(planCount.value) || 4)),
  );
  if (!state.value.map) await post("/api/plan/floor");
  for (const slot of planSlots(count)) {
    const place = placeOnMap(slot);
    await post("/api/points/save", {
      point: {
        name: slot.name,
        pose: { x: place.x, y: place.y, yaw: place.yaw },
        target_degrees: 140.5,
        sort_index: slot.sort_index,
        field_fixed: false,
      },
      expected_revision: 0,
    });
  }
  notice.value = `已摆上 ${count} 个观测点。点一下就能改位置、名字或删掉。`;
}
async function addOnePoint() {
  if (floorPoints.value.length >= planPointLimit) return;
  const slot = placeOnMap(planSlots(floorPoints.value.length + 1).at(-1));
  await post("/api/points/save", {
    point: {
      name: `观测点 ${floorPoints.value.length + 1}`,
      pose: { x: slot.x, y: slot.y, yaw: slot.yaw },
      target_degrees: 140.5,
      sort_index: floorPoints.value.length,
      field_fixed: false,
    },
    expected_revision: 0,
  });
  notice.value = "又加了一个观测点，可以拖到你要的地方。";
}
async function clearPoints() {
  const ids = floorPoints.value.map((item) => item.id);
  for (const id of ids) await post("/api/points/remove", { id });
  selectedId.value = "";
  notice.value = "已清空。可以重新选要放几个点。";
}
async function removeSelected() {
  const point = selectedPoint.value;
  if (!point || floorPoints.value.length <= 1) return;
  await post("/api/points/remove", { id: point.id });
  selectedId.value = "";
  notice.value = "这个观测点已去掉。";
}
async function renameSelected() {
  const point = selectedPoint.value;
  const name = draft.name.trim();
  if (!point || !name || name === point.name) return;
  await post("/api/points/rename", { id: point.id, name });
  notice.value = "名字已改好。";
}
async function runPrimary() {
  const step = primaryStep.value;
  const point = selectedPoint.value;
  if (!step.enabled || !point) return;
  if (step.id === "record") {
    const pose = state.value.pose;
    await savePlanPoint(
      point,
      { x: pose.x, y: pose.y, yaw: pose.yaw },
      { field_fixed: true, name: point.name },
    );
    return;
  }
  if (step.id === "angle") {
    angleOpen.value = true;
    draft.name = point.name;
    draft.id = point.id;
    draft.revision = point.revision;
    if (["IDLE", "FAULT", "CONFIRMED"].includes(teaching.value.phase))
      await begin();
    return;
  }
  if (step.id === "confirm" || step.id === "confirm-approach") {
    draft.name = point.name;
    draft.id = point.id;
    draft.revision = point.revision;
    await confirm(step.id === "confirm" ? "point" : "approach");
    angleOpen.value = false;
    alternateOpen.value = false;
  }
}
async function backToSketch() {
  planReady.value = false;
  selectedId.value = "";
  await post("/api/plan/floor");
  planReady.value = true;
  notice.value = "又回到 20 米 × 10 米的规划草图。";
}
async function startOrdered() {
  if (!state.value.pose) {
    notice.value = "先把车停到巡完要停的地方，再按这个按钮。";
    return;
  }
  const existing = state.value.tasks.find((item) => item.id === "site_route");
  const saved = await post("/api/tasks/save", {
    task: {
      id: "site_route",
      name: "现场巡检",
      point_ids: floorPoints.value.map((item) => item.id),
      final_parking_pose: state.value.pose,
    },
    expected_revision: existing?.revision || 0,
  });
  if (state.value.navigation_enabled) await startTask(saved);
  else notice.value = "顺序已记下。自动出发还要等导航打开。";
}
function setPage(value) {
  stopManual();
  mapSelecting.value = false;
  mapPointer = null;
  page.value = value;
  nextTick(draw);
}
onMounted(() => {
  connect();
  poll();
  pollTimer = setInterval(poll, 250);
  document.addEventListener("visibilitychange", visibility);
  window.addEventListener("blur", release);
  window.addEventListener("pagehide", stopManual);
});
onBeforeUnmount(() => {
  stopped = true;
  clearInterval(pollTimer);
  clearTimeout(retryTimer);
  release();
  socket?.close();
  if (videoUrl.value) URL.revokeObjectURL(videoUrl.value);
  document.removeEventListener("visibilitychange", visibility);
  window.removeEventListener("blur", release);
  window.removeEventListener("pagehide", stopManual);
});
</script>

<template>
  <div class="shell">
    <aside>
      <a class="brand" href="/" aria-label="巡检首页"
        ><span class="brand-mark"><ConsoleIcon name="drive" /></span
        ><span>巡检<small>小车工作空间</small></span></a
      >
      <div class="workspace-switch" aria-label="工作空间">
        <a href="/"><ConsoleIcon name="map" />随行建图</a>
        <span class="workspace-current"
          ><ConsoleIcon name="points" />巡检工作台</span
        >
      </div>
      <p class="aside-label">工作空间</p>
      <nav aria-label="巡检页面">
        <button
          v-for="[key, title] in navItems"
          :key="key"
          :class="{ active: page === key }"
          :aria-current="page === key ? 'page' : undefined"
          @click="setPage(key)"
        >
          <ConsoleIcon :name="key" /><span>{{ title }}</span
          ><ConsoleIcon
            v-if="page === key"
            name="chevron"
            class="nav-chevron"
          />
        </button>
      </nav>
      <div class="aside-foot">
        <span :class="['dot', connected ? 'good' : '']"></span
        >{{ connected ? "操作者已连接" : "等待连接"
        }}<small>任务与照片保存在车端</small>
      </div>
    </aside>
    <main :aria-busy="busy">
      <header>
        <div>
          <p class="eyebrow">室内巡检工作台</p>
          <h1>{{ navItems.find((i) => i[0] === page)?.[1] }}</h1>
          <p class="page-description">{{ pageDescriptions[page] }}</p>
        </div>
        <button
          class="stop"
          @click="
            action(() =>
              state.run?.status === 'RUNNING'
                ? post('/api/run/cancel')
                : Promise.resolve(stopManual()),
            )
          "
        >
          <ConsoleIcon name="stop" />
          停车{{ state.run?.status === "RUNNING" ? "并取消任务" : "" }}
        </button>
      </header>
      <div v-if="notice || state.error" class="notice" role="status">
        {{ notice || state.error }}
      </div>
      <div v-if="!state.hardware_enabled" class="info">
        <span class="info-icon"><ConsoleIcon name="shield" /></span>
        <div>
          <strong>配置模式</strong>
          <p>查看地图、编辑点位与任务。实车调角和行驶尚未启用。</p>
        </div>
      </div>
      <section class="health-strip" aria-label="车辆就绪状态">
        <span
          v-for="(name, key) in healthNames"
          :key="key"
          :class="{ ready: state.health[key] }"
          ><i :class="['dot', state.health[key] ? 'good' : '']"></i
          ><strong>{{ name }}</strong
          ><small>{{ state.health[key] ? "已就绪" : "待就绪" }}</small></span
        >
      </section>
      <section v-if="page === 'overview'" class="grid">
        <article class="card">
          <h2>下一次巡检</h2>
          <p>选好地图、完成点位试拍，再启动已保存的巡检任务。</p>
          <button class="primary" @click="setPage('tasks')">
            选择巡检任务
          </button>
          <div class="stat">
            <strong>{{ state.points.length }}</strong> 个巡检点
            <strong>{{ state.tasks.length }}</strong> 个任务
          </div>
        </article>
        <article class="card">
          <h2>车辆状态</h2>
          <p>{{ state.motion_held ? "停车保持中" : "等待新的运动请求" }}</p>
          <p>自动任务下发后，关闭网页或断网仍继续。手动遥控失联停车。</p>
          <p>已保存照片与最终停车、上传结果分别记录。</p>
        </article>
      </section>
      <template v-if="page === 'points' || page === 'tasks'">
        <div class="toolbar">
          <label
            >场地地图<select v-model="selectedMap">
              <option value="">请选择</option>
              <option v-for="m in state.maps" :key="m.id" :value="m.id">
                {{ m.name }}
              </option>
            </select></label
          ><button
            :disabled="busy || !connected || !selectedMap"
            @click="action(chooseMap)"
          >
            <ConsoleIcon name="folder" />载入地图</button
          ><button v-if="page === 'points'" @click="newPoint">
            <ConsoleIcon name="plus" />新增点位</button
          ><button v-else @click="newTask">
            <ConsoleIcon name="plus" />新建任务</button
          ><span>当前图 {{ state.map?.id || "未选择" }}</span>
        </div>
        <p class="position-guide">
          {{
            page === "points"
              ? "① 载入地图 → ② 选择巡检位置并拖出车头方向 → ③ 命名、保存草稿 → ④ 实车调角与试拍确认"
              : "① 添加巡检点并排序 → ② 选择最终停车位置与朝向 → ③ 保存巡检任务"
          }}
        </p>
        <div class="workspace">
          <article class="card map-card">
            <div class="card-title">
              <h2>
                <ConsoleIcon :name="page === 'tasks' ? 'tasks' : 'points'" />{{
                  page === "tasks" ? "巡检顺序与最终停车" : "巡检停车位"
                }}
              </h2>
              <span>地图坐标 · 米</span>
            </div>
            <button
              class="primary map-pick"
              :disabled="!state.map || busy"
              :aria-pressed="mapSelecting"
              @click="mapSelecting = !mapSelecting"
            >
              {{
                mapSelecting
                  ? "取消地图选点"
                  : page === "tasks"
                    ? "在地图上选择最终停车位置"
                    : "在地图上选择巡检位置"
              }}
            </button>
            <p class="pick-help" role="status">
              {{
                !state.map
                  ? "先在上方选择场地地图，再点击“载入地图”。"
                  : mapSelecting
                    ? "按住地图上的停车位置，向车头朝向拖动，松开完成。也可直接填写右侧坐标。"
                    : editPoseChosen
                      ? "位置已选择；可修改右侧坐标，或再次点击按钮重新选点。"
                      : "点击上方选点按钮，或记录小车当前位置与朝向。选点只编辑配置，不会移动车辆。"
              }}
            </p>
            <div :class="['map-surface', { picking: mapSelecting }]">
              <canvas
                ref="canvas"
                aria-label="场地地图，点击设置停车位置并拖动设置朝向"
                @pointerdown="mapDown"
                @pointermove="mapMove"
                @pointerup="mapUp"
                @pointercancel="mapUp"
              ></canvas>
              <div v-if="!state.map" class="surface-empty">
                <ConsoleIcon name="map" />
                <h3>从一张地图开始</h3>
                <p>载入已保存的场地地图，规划巡检停车位。</p>
              </div>
            </div>
            <p class="caption">
              保存地图 · map坐标系{{
                state.map ? ` · ${state.map.resolution.toFixed(3)} 米/格` : ""
              }}。
              白：空闲　黑：占用　灰：未知　蓝：当前定位估计　粉：编辑目标　绿：已确认点　黄：草稿。箭头为车头朝向。
            </p>
            <p class="caption">
              地图按原比例显示，屏幕大小不改变米制坐标。请在白色区域选停车位，并现场核对车身空间；保存地图不代表当前障碍状态。
            </p>
            <details v-if="page === 'points'" class="localization-tools">
              <summary>小车初始定位（停车后使用）</summary>
              <p class="caption">
                仅当定位估计与实物不符时使用：先在地图选出小车实际所在位置和车头方向，再提交初值，随后核对定位结果。
              </p>
              <button
                :disabled="
                  busy ||
                  !connected ||
                  !state.hardware_enabled ||
                  !state.health.still ||
                  !draftPoseChosen
                "
                @click="action(localize)"
              >
                提交小车初始定位
              </button>
              <p v-if="!state.hardware_enabled" class="caption">
                配置模式未接入实车，初始定位功能暂不可用。
              </p>
            </details>
          </article>
          <article v-if="page === 'points'" class="card editor">
            <h2><ConsoleIcon name="points" />巡检点设置</h2>
            <label
              >点位名称<input
                v-model="draft.name"
                placeholder="例如：走廊仪表"
                maxlength="60"
            /></label>
            <div class="form-row">
              <label
                >X / 米<input
                  v-model.number="draft.x"
                  type="number"
                  step="0.01"
                  @input="poseEdited(draft)" /></label
              ><label
                >Y / 米<input
                  v-model.number="draft.y"
                  type="number"
                  step="0.01"
                  @input="poseEdited(draft)" /></label
              ><label
                >朝向 / °<input
                  v-model.number="draft.yaw"
                  type="number"
                  min="-180"
                  max="180"
                  step="1"
                  @input="poseEdited(draft)"
              /></label>
            </div>
            <button
              :disabled="!state.pose"
              @click="
                usePose(draft);
                draw();
              "
            >
              记录小车当前位置与朝向
            </button>
            <p v-if="!state.pose" class="caption">
              尚无小车定位数据，可先在地图上选点保存草稿。接入实车并完成定位后，才能记录当前位置。
            </p>
            <label
              >草稿俯仰目标 / °<input
                v-model.number="draft.target_degrees"
                type="number"
                min="85"
                max="160"
                step="1" /></label
            ><button
              class="primary"
              :disabled="
                busy ||
                !connected ||
                !draft.name.trim() ||
                !state.map ||
                !draftPoseChosen
              "
              @click="action(saveDraft)"
            >
              保存巡检点草稿
            </button>
            <p class="caption">草稿需实车试拍确认，才能加入自动巡检执行。</p>
            <div class="point-list">
              <button
                v-for="p in points"
                :key="p.id"
                :class="{ selected: draft.id === p.id }"
                @click="
                  usePoint(p);
                  draw();
                "
              >
                <strong>{{ p.name }}</strong
                ><span
                  >{{ p.verified ? "已试拍确认" : "待现场确认" }} ·
                  {{ approachCount(p) }}个停车位 ·
                  {{ p.target_degrees }}°目标</span
                >
              </button>
            </div>
          </article>
          <article v-else class="card editor">
            <h2>安排任务</h2>
            <label>任务名称<input v-model="route.name" maxlength="60" /></label
            ><label
              >添加巡检点<select
                @change="
                  addPoint($event.target.value);
                  $event.target.value = '';
                "
              >
                <option value="">选择点位</option>
                <option v-for="p in points" :key="p.id" :value="p.id">
                  {{ p.name }}{{ p.verified ? "" : "（待确认）" }}
                </option>
              </select></label
            >
            <ol class="route-list">
              <li v-for="(id, index) in route.point_ids" :key="id">
                <span>{{
                  state.points.find((p) => p.id === id)?.name || id
                }}</span
                ><button aria-label="向前排序" @click="move(index, -1)">
                  <ConsoleIcon name="arrow" /></button
                ><button aria-label="向后排序" @click="move(index, 1)">
                  <ConsoleIcon name="arrow" class="rotate-down" /></button
                ><button @click="route.point_ids.splice(index, 1)">移除</button>
              </li>
            </ol>
            <h3>最终停车位置</h3>
            <div class="form-row">
              <label
                >X / 米<input
                  v-model.number="route.x"
                  type="number"
                  step="0.01"
                  @input="poseEdited(route)" /></label
              ><label
                >Y / 米<input
                  v-model.number="route.y"
                  type="number"
                  step="0.01"
                  @input="poseEdited(route)" /></label
              ><label
                >朝向 / °<input
                  v-model.number="route.yaw"
                  type="number"
                  min="-180"
                  max="180"
                  @input="poseEdited(route)"
              /></label>
            </div>
            <button
              :disabled="!state.pose"
              @click="
                usePose(route);
                draw();
              "
            >
              使用车辆当前位置</button
            ><button
              class="primary"
              :disabled="
                busy ||
                !connected ||
                !route.point_ids.length ||
                !state.map ||
                !routePoseChosen
              "
              @click="action(saveTask)"
            >
              保存巡检任务
            </button>
          </article>
        </div>
      </template>
      <section v-if="page === 'sketch'" class="plan-page">
        <p class="plan-banner">
          规划草图 ·
          用于预排点位。实车巡检须载入激光地图，并在现场确认位置与视角。
        </p>
        <div class="plan-map">
          <canvas
            ref="planCanvas"
            aria-label="200平方米规划草图。点选观测点，拖动圆点改位置，拖动箭头改车头。"
            @pointerdown="planDown"
            @pointermove="planMove"
            @pointerup="planUp"
            @pointercancel="planUp"
          ></canvas>
          <p v-if="!state.map">正在准备 20 米 × 10 米的场地…</p>
        </div>
        <p class="caption">
          点一下选中。按住圆点拖到想停的地方，按住箭头拖出车头。松手就记下，不用再找保存。
        </p>
        <div v-if="state.map && !floorPoints.length" class="chooser">
          <h2>先放几个观测点？</h2>
          <p>4 个只是建议，数量可以改，之后也能再加或去掉。</p>
          <div class="count-choices">
            <button
              v-for="count in [1, 2, 4, 6, 8]"
              :key="count"
              :class="{ selected: planCount === count }"
              @click="planCount = count"
            >
              {{ count }} 个{{ count === 4 ? " · 建议" : "" }}
            </button>
          </div>
          <label
            >自己填，1 到 {{ planPointLimit }} 个
            <input
              v-model.number="planCount"
              type="number"
              min="1"
              :max="planPointLimit"
          /></label>
          <button
            class="primary big"
            :disabled="busy"
            @click="action(layChosen)"
          >
            放置观测点
          </button>
        </div>
        <template v-else-if="floorPoints.length">
          <button
            v-for="(point, index) in floorPoints"
            :key="point.id"
            :class="['plan-item', { selected: point.id === selectedPoint?.id }]"
            @click="
              selectedId = point.id;
              angleOpen = false;
              alternateOpen = false;
              draft.name = point.name;
              planDraw();
            "
          >
            <strong>{{ index + 1 }}. {{ point.name }}</strong>
            <span
              >{{ pointPhrase(point).title }}。{{
                pointPhrase(point).detail
              }}</span
            >
          </button>
          <div class="custom-row">
            <button
              :disabled="busy || floorPoints.length >= planPointLimit"
              @click="action(addOnePoint)"
            >
              再加一个观测点
            </button>
            <button
              :disabled="busy || floorPoints.length <= 1"
              @click="action(removeSelected)"
            >
              去掉当前这个点
            </button>
            <button :disabled="busy" @click="action(clearPoints)">
              清空后重新选数量
            </button>
            <label
              >名字
              <input
                id="point-name"
                v-model="draft.name"
                maxlength="60"
                @change="action(renameSelected)"
            /></label>
          </div>
          <article v-if="selectedPoint" class="card now-card">
            <h2>{{ pointPhrase(selectedPoint).title }}</h2>
            <p>
              {{ pointPhrase(selectedPoint).detail }}
              {{ tiltPhrase(selectedPoint.target_degrees) }}
            </p>
            <div v-if="angleOpen" class="angle-panel">
              <div class="video-surface">
                <img v-if="videoUrl" :src="videoUrl" alt="相机实时画面" />
                <p v-else>等待相机图像</p>
              </div>
              <div class="step-buttons">
                <button
                  class="big"
                  :disabled="busy"
                  @click="action(() => step(-1))"
                >
                  镜头抬一点
                </button>
                <button
                  class="big"
                  :disabled="busy"
                  @click="action(() => step(1))"
                >
                  镜头低一点
                </button>
              </div>
              <p v-if="!confirmable" class="status-wait">
                {{ confirmReason(state.health, teaching.phase) }}
              </p>
            </div>
            <button
              class="primary big"
              :disabled="busy || !primaryStep.enabled"
              @click="action(runPrimary)"
            >
              {{ primaryStep.label }}
            </button>
            <button
              v-if="
                pointPhrase(selectedPoint).stage === 'ready' &&
                approachCount(selectedPoint) < 3
              "
              class="quiet"
              @click="
                alternateOpen = true;
                angleOpen = false;
              "
            >
              这个点还有另一个可停的地方
            </button>
          </article>
          <p v-if="!allPointsReady" class="caption">
            还有点没确认完，车不会自己出发。数量不对就再加，或去掉不要的点。
          </p>
          <article v-else-if="state.map?.planning" class="card">
            <h2>这些点都确认好了</h2>
            <p>
              它们还在规划草图上。等真地图建好后，要在那张图上再记一次位置，车才会自己去。
            </p>
          </article>
          <article v-else class="card">
            <h2>按这个顺序去巡检</h2>
            <p>
              从上到下就是出发顺序。巡完以后停在单独选的地方，不占用这些观测点。
            </p>
            <button
              class="primary big"
              :disabled="busy || !state.navigation_enabled"
              @click="action(startOrdered)"
            >
              按这个顺序去巡检
            </button>
          </article>
        </template>
        <details class="extra-tools">
          <summary>手动遥控与场地地图</summary>
          <div class="drive-buttons">
            <button
              :disabled="busy || !connected || !state.hardware_enabled"
              @click="action(() => post('/api/manual/enable'))"
            >
              启用低速遥控
            </button>
            <button
              aria-label="前进"
              @pointerdown="press($event, 1, 0)"
              @pointerup="release"
              @pointercancel="release"
            >
              <ConsoleIcon name="arrow" />
            </button>
            <button
              aria-label="左转"
              @pointerdown="press($event, 0, 1)"
              @pointerup="release"
              @pointercancel="release"
            >
              <ConsoleIcon name="left" />
            </button>
            <button
              aria-label="右转"
              @pointerdown="press($event, 0, -1)"
              @pointerup="release"
              @pointercancel="release"
            >
              <ConsoleIcon name="right" />
            </button>
            <button
              aria-label="后退"
              @pointerdown="press($event, -1, 0)"
              @pointerup="release"
              @pointercancel="release"
            >
              <ConsoleIcon name="arrow" class="rotate-down" />
            </button>
          </div>
          <label
            >已经扫好的地图
            <select v-model="selectedMap">
              <option value="">请选择</option>
              <option
                v-for="item in state.maps"
                :key="item.id"
                :value="item.id"
              >
                {{ item.name }}
              </option>
            </select></label
          >
          <button :disabled="busy || !selectedMap" @click="action(chooseMap)">
            用这张地图
          </button>
          <button :disabled="busy" @click="action(backToSketch)">
            回到 200 平方米草图
          </button>
        </details>
      </section>
      <section v-if="page === 'points'" class="workspace teaching">
        <article class="card">
          <div class="card-title">
            <h2><ConsoleIcon name="camera" />相机实时画面</h2>
            <span :class="state.health.rgb ? 'status-good' : 'status-wait'">{{
              state.health.rgb ? "当前源帧有效" : "等待有效图像"
            }}</span>
          </div>
          <div class="video-surface">
            <img v-if="videoUrl" :src="videoUrl" alt="相机实时RGB画面" />
            <div v-else class="surface-empty">
              <ConsoleIcon name="camera" />
              <h3>等待相机画面</h3>
              <p>接入有效 RGB 图像后在此预览</p>
            </div>
          </div>
          <p class="caption">
            RGB原始流预览；窗口大小不改变镜头视场。图像时间戳：{{
              sourceStamp || "尚无"
            }}。参考照片是历史画面。
          </p>
        </article>
        <article class="card">
          <h2>一步一步确认角度</h2>
          <ol class="steps">
            <li :class="{ done: state.motion_held }">
              保持停车并等待轮式反馈停稳
            </li>
            <li
              :class="{
                done: [
                  'WAITING_FRAME',
                  'AWAITING_CONFIRMATION',
                  'CONFIRMED',
                ].includes(teaching.phase),
              }"
            >
              每次调整目标1°，等待发送结果和新画面
            </li>
            <li :class="{ done: teaching.phase === 'CONFIRMED' }">
              确认画面合适，保存固定角度与参考照片
            </li>
          </ol>
          <p class="phase">{{ labels[teaching.phase] || teaching.phase }}</p>
          <p v-if="teaching.error" class="status-wait">{{ teaching.error }}</p>
          <button
            :disabled="
              busy ||
              !connected ||
              !state.hardware_enabled ||
              !['IDLE', 'FAULT', 'CONFIRMED'].includes(teaching.phase)
            "
            @click="action(begin)"
          >
            1. 开始角度调整
          </button>
          <div class="angle-readout">
            <strong
              >{{ teaching.target_degrees?.toFixed(1) }}<small>°</small></strong
            ><span
              >S1 俯仰目标 · 板端命令
              {{ teaching.board_command ?? "—" }} 档</span
            >
          </div>
          <div class="step-buttons">
            <button
              :disabled="
                busy ||
                !['READY', 'AWAITING_CONFIRMATION'].includes(teaching.phase)
              "
              @click="action(() => step(-1))"
            >
              目标 −1°</button
            ><button
              :disabled="
                busy ||
                !['READY', 'AWAITING_CONFIRMATION'].includes(teaching.phase)
              "
              @click="action(() => step(1))"
            >
              目标 +1°</button
            ><button
              :disabled="
                busy ||
                !['READY', 'AWAITING_CONFIRMATION'].includes(teaching.phase)
              "
              @click="action(() => step(0))"
            >
              下发当前目标
            </button>
          </div>
          <p class="caption">
            历史方向为目标减小抬头、增大低头，首次使用以实物验证。板端整数档会量化；没有实际位置反馈。每步人工点击，不连续扫角。
          </p>
          <button
            class="primary"
            :disabled="busy || !confirmable || !draft.name"
            @click="action(() => confirm('point'))"
          >
            3. 确认此画面，保存巡检点</button
          ><button
            :disabled="busy || !confirmable || !draft.id"
            @click="action(() => confirm('approach'))"
          >
            把当前位置存为备选停车位</button
          ><button
            :disabled="busy || !confirmable || teaching.board_command !== 94"
            @click="action(() => confirm('navigation'))"
          >
            确认当前为导航参考姿态（94档）
          </button>
          <p class="caption">
            每个拍摄目标最多3个停车位，各自保存朝向和俯仰。某一位到不了就试下一位；三位都失败则记为未拍并继续后面的目标。保存后继续保持停车。支架异常时立即断开执行器电源。
          </p>
        </article>
      </section>
      <section v-if="page === 'points'" class="card remote">
        <div>
          <h2>示教遥控</h2>
          <p class="caption">
            低速、按住行驶，松手停车；遥控不提供自动避障。调角期间禁止行驶。
          </p>
          <button
            :disabled="busy || !connected || !state.hardware_enabled"
            @click="action(() => post('/api/manual/enable'))"
          >
            <ConsoleIcon name="drive" />启用遥控</button
          ><span>{{ state.manual?.reason || "尚未使能" }}</span>
        </div>
        <div class="drive-buttons">
          <button
            aria-label="前进"
            @pointerdown="press($event, 1, 0)"
            @pointerup="release"
            @pointercancel="release"
            @lostpointercapture="release"
          >
            <ConsoleIcon name="arrow" /></button
          ><button
            aria-label="左转"
            @pointerdown="press($event, 0, 1)"
            @pointerup="release"
            @pointercancel="release"
            @lostpointercapture="release"
          >
            <ConsoleIcon name="left" /></button
          ><button class="drive-stop" aria-label="手动停车" @click="stopManual">
            <ConsoleIcon name="stop" /></button
          ><button
            aria-label="右转"
            @pointerdown="press($event, 0, -1)"
            @pointerup="release"
            @pointercancel="release"
            @lostpointercapture="release"
          >
            <ConsoleIcon name="right" /></button
          ><button
            aria-label="后退"
            @pointerdown="press($event, -1, 0)"
            @pointerup="release"
            @pointercancel="release"
            @lostpointercapture="release"
          >
            <ConsoleIcon name="arrow" class="rotate-down" />
          </button>
        </div>
      </section>
      <section v-if="page === 'tasks'" class="task-cards">
        <article v-for="t in state.tasks" :key="t.id" class="card">
          <h2>{{ t.name }}</h2>
          <p>{{ t.point_ids.length }} 个点位 · {{ t.map_id }}</p>
          <p class="caption">采集完成后前往独立的最终停车位。</p>
          <div class="buttons">
            <button
              @click="
                useTask(t);
                draw();
              "
            >
              编辑任务</button
            ><button
              class="primary"
              :disabled="busy || !connected || !state.navigation_enabled"
              @click="action(() => startTask(t))"
            >
              开始巡检
            </button>
          </div>
        </article>
      </section>
      <section v-if="page === 'running'" class="card">
        <h2>
          {{
            state.run
              ? labels[state.run.status] || state.run.status
              : "当前没有巡检任务"
          }}
        </h2>
        <template v-if="state.run"
          ><p>
            {{
              state.run.parking
                ? "正在前往最终停车位"
                : `当前观察点 ${Math.min(state.run.point_index + 1, state.run.snapshot.points.length)} / ${state.run.snapshot.points.length}`
            }}
            ·
            {{
              {
                restore: "恢复导航姿态",
                navigate: "导航到目标",
                stop: "确认停车",
                adjust: "调整相机",
                capture: "拍照保存",
              }[state.run.step] || "准备中"
            }}
          </p>
          <p v-if="state.run.error" class="status-wait">
            {{ state.run.error }}
          </p>
          <div class="buttons">
            <button
              :disabled="busy || state.run.status !== 'RUNNING'"
              @click="action(() => post('/api/run/pause'))"
            >
              暂停</button
            ><button
              class="primary"
              :disabled="busy || state.run.status !== 'PAUSED'"
              @click="action(() => post('/api/run/resume'))"
            >
              继续</button
            ><button
              :disabled="
                busy || !['RUNNING', 'PAUSED'].includes(state.run.status)
              "
              @click="action(() => post('/api/run/cancel'))"
            >
              取消本轮
            </button>
          </div>
          <p>
            网页关闭或WiFi断开，已接受的自动巡检继续执行。重连后读取车端进度。
          </p>
          <div class="photos">
            <figure v-for="photo in state.run.captures" :key="photo.id">
              <img :src="site(`/api/media/${photo.id}`)" alt="本轮巡检照片" />
              <figcaption>
                {{
                  state.points.find((p) => p.id === photo.point_id)?.name ||
                  photo.point_id
                }}
                · {{ uploadLabel(photo.upload_status) }}
              </figcaption>
            </figure>
          </div></template
        >
      </section>
      <section v-if="page === 'records'" class="record-list">
        <article
          v-for="run in [...state.runs].reverse()"
          :key="run.id"
          class="card"
        >
          <div class="card-title">
            <h2>{{ run.snapshot.name }}</h2>
            <span>{{ labels[run.status] || run.status }}</span>
          </div>
          <p>
            {{ Object.keys(run.captures || {}).length }} /
            {{ run.snapshot.points.length }} 个点已采集 · 未到达
            {{ (run.missed || []).length }} · 最终停车：{{
              run.parking_status === "SUCCEEDED"
                ? "已完成"
                : run.parking_status === "FAILED"
                  ? "失败"
                  : "未完成"
            }}
          </p>
          <p class="caption">
            {{
              run.error ||
              "照片先保存在车端。未配置上传地址时保持待配置；上传失败不会重拍或重新开车。"
            }}
          </p>
          <div class="photos">
            <figure v-for="photo in run.captures" :key="photo.id">
              <a :href="site(`/api/media/${photo.id}`)" download
                ><img :src="site(`/api/media/${photo.id}`)" alt="历史巡检照片"
              /></a>
              <figcaption>
                {{
                  run.snapshot.points.find((p) => p.id === photo.point_id)?.name
                }}
                · {{ uploadLabel(uploadState(run, photo)) }} · 点击下载
              </figcaption>
            </figure>
          </div>
        </article>
        <p v-if="!state.runs.length" class="empty">
          <ConsoleIcon name="records" />
          完成第一轮巡检后，照片和结果会显示在这里。
        </p>
      </section>
      <footer>
        <ConsoleIcon name="shield" />
        每个目标最多三个停车位、成功后一张照片 · 车端保存后排队上传 ·
        地图与点位按版本关联
      </footer>
    </main>
  </div>
</template>
