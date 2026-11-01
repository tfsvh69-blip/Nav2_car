<script setup>
import ConsoleIcon from "./ConsoleIcon.vue";
import { createServoSlider } from "./servo-slider.js";
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
  trialLabel,
  trialMatchesTarget,
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
const initialPose = reactive({ x: 0, y: 0, yaw: 0 });
const sliderTarget = ref(null),
  sliderDirty = ref(false),
  sliderRunning = ref(false);
const initialPoseChosen = ref(false);
const editTarget = computed(() =>
  page.value === "overview"
    ? initialPose
    : page.value === "tasks"
      ? route
      : draft,
);
const trial = computed(() => state.value.navigation_trial || { phase: "IDLE" });
const trialArrived = computed(
  () => draftPoseChosen.value && trialMatchesTarget(trial.value, poseOf(draft)),
);
const trialActive = computed(() => !!trial.value.active);
const goalReason = computed(
  () =>
    state.value.goal_navigation_block_reason ||
    (!state.value.goal_navigation_enabled
      ? "当前是静止测试，尚未启用单点导航"
      : ""),
);
const navigationRequest = reactive({ phase: "IDLE", message: "" });
const navigationLayers = reactive({
  global_costmap: true,
  local_costmap: true,
  global_path: true,
  local_path: true,
});
const navigationLayerNames = {
  global_costmap: "全局障碍与膨胀（紫）",
  local_costmap: "局部障碍与膨胀（橙）",
  global_path: "全局路径（青）",
  local_path: "局部路径（亮黄）",
};
const navigationImages = {};
const navigationDownloads = new Set();
const navigateReason = computed(() =>
  !connected.value
    ? "请等待控制连接建立"
    : !draftPoseChosen.value
      ? "请先在地图上选择停车位置与车头方向，或填写候选坐标"
      : trialActive.value
        ? "已有导航正在执行，请等待或取消"
        : goalReason.value,
);
function navigationLayerStatus(layer) {
  const info = state.value.navigation_debug?.[layer];
  if (!info?.available) return info?.error || "尚未收到数据";
  const seconds = Math.max(0, info.source_age_seconds || 0);
  return `${info.source_frame}→map · ${seconds.toFixed(1)}秒前${seconds > 2 ? " · 历史数据" : ""}${layer.endsWith("path") ? ` · ${info.points?.length || 0}个显示点` : ""}`;
}
async function loadNavigationImages(value) {
  for (const key of ["global_costmap", "local_costmap"]) {
    const meta = value.navigation_debug?.[key];
    if (
      !navigationLayers[key] ||
      !meta?.available ||
      navigationDownloads.has(key) ||
      navigationImages[key]?.meta.sequence === meta.sequence
    )
      continue;
    navigationDownloads.add(key);
    try {
      const response = await fetch(
        site(`/api/navigation/${key.replaceAll("_", "-")}.png`),
      );
      if (!response.ok) continue;
      const paired = JSON.parse(
        response.headers.get("X-Navigation-Layer") || "{}",
      );
      const url = URL.createObjectURL(await response.blob());
      const image = new Image();
      image.onload = () => {
        navigationImages[key] = { image, meta: paired };
        URL.revokeObjectURL(url);
        draw();
      };
      image.onerror = () => URL.revokeObjectURL(url);
      image.src = url;
    } catch {
      /* 状态栏保留实际消息更新时间；不伪造空闲栅格。 */
    } finally {
      navigationDownloads.delete(key);
    }
  }
}
const pointSaved = computed(
  () => !!draft.id && points.value.some((p) => p.id === draft.id && p.verified),
);
const pointCanConfirm = computed(
  () =>
    confirmable.value &&
    !sliderRunning.value &&
    !sliderDirty.value &&
    trialArrived.value &&
    state.value.health.localization &&
    state.value.health.map_matches,
);
const navItems = [
  ["overview", "准备车辆"],
  ["points", "录入巡检点"],
  ["tasks", "安排巡检"],
  ["running", "任务进度"],
  ["records", "照片与记录"],
];
const pageDescriptions = {
  overview: "先确认车在地图中的位置。行驶时镜头固定为 140.5°，不用再确认。",
  points: "先导航到候选停车位，再以实车停靠结果调整位置和拍摄角度。",
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
const servoSlider = createServoSlider({
  read: () => ({
    connected: connected.value,
    visible: !document.hidden,
    allowed:
      !stopped &&
      page.value === "points" &&
      trialArrived.value &&
      !trialActive.value,
    held: state.value.motion_held,
    still: state.value.health.still,
    rgb: state.value.health.rgb,
    teaching: teaching.value,
  }),
  send: async (target, revision) => {
    rendered.sequence = 0;
    await post("/api/teach/step", { revision, target_degrees: target });
  },
  onRunning: (value) => {
    sliderRunning.value = value;
  },
});
function cancelSlider() {
  servoSlider.stop();
  sliderTarget.value = null;
  sliderDirty.value = false;
  if (sliderRunning.value)
    notice.value =
      "已停止追加调角命令；已发送的一小步可能完成，停车保持继续生效。";
}
function previewSlider(event) {
  sliderTarget.value = Number(event.target.value);
  sliderDirty.value = true;
}
async function commitSlider() {
  const target = Number(sliderTarget.value ?? teaching.value.target_degrees);
  sliderDirty.value = true;
  rendered.sequence = 0;
  notice.value =
    "正在按滑块目标逐步调角；可改目标或停止，完成前不能保存旧画面。";
  try {
    const result = await servoSlider.start(target);
    if (result.completed && Number(sliderTarget.value) === result.target) {
      sliderTarget.value = null;
      sliderDirty.value = false;
      notice.value = "滑块调角已完成，请检查本次新画面后确认保存。";
    }
  } catch (error) {
    notice.value = error.message;
  }
}
const points = computed(() =>
  state.value.points.filter((p) => p.map_id === state.value.map?.id),
);
const editPoseChosen = computed(() =>
  page.value === "overview"
    ? initialPoseChosen.value
    : page.value === "tasks"
      ? routePoseChosen.value
      : draftPoseChosen.value,
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
let restoreArrivalOnLoad = true;
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
    cancelSlider();
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
  cancelSlider();
  release();
  send({ action: "stop" });
}
function visibility() {
  if (document.hidden) {
    cancelSlider();
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
    // 刷新只恢复当前实车已到达的候选编辑值，不重发目标或角度。
    if (restoreArrivalOnLoad) {
      restoreArrivalOnLoad = false;
      if (
        !draftPoseChosen.value &&
        page.value === "points" &&
        value.navigation_trial?.phase === "ARRIVED" &&
        value.navigation_trial.map_id === value.map?.id &&
        value.navigation_trial.target
      ) {
        const target = value.navigation_trial.target;
        Object.assign(draft, {
          x: target.x,
          y: target.y,
          yaw: (target.yaw * 180) / Math.PI,
        });
        draftPoseChosen.value = true;
      }
    }
    void loadNavigationImages(value);
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
  if (target === initialPose) initialPoseChosen.value = true;
  else if (target === route) routePoseChosen.value = true;
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
  cancelSlider();
  await post("/api/teach/begin");
  rendered.sequence = 0;
  notice.value = "已请求停车保持；等待轮式反馈停稳后，再点击一步调整。";
}
async function startTrialCapture() {
  if (["IDLE", "FAULT", "CONFIRMED"].includes(teaching.value.phase))
    await begin();
  await nextTick();
  document
    .querySelector(".workspace.teaching")
    ?.scrollIntoView({ behavior: "smooth", block: "start" });
}
async function navigateDraft() {
  if (navigateReason.value) {
    navigationRequest.phase = "REJECTED";
    navigationRequest.message = navigateReason.value;
    throw new Error(navigateReason.value);
  }
  if (
    !window.confirm(
      `让小车前往候选停车位：X ${Number(draft.x).toFixed(2)}米、Y ${Number(draft.y).toFixed(2)}米、朝向 ${Number(draft.yaw).toFixed(1)}°。请确认行驶区域安全，并在旁看护。`,
    )
  ) {
    navigationRequest.phase = "CANCELED";
    navigationRequest.message = "已取消确认，尚未向车端发送目标";
    return;
  }
  navigationRequest.phase = "SUBMITTING";
  navigationRequest.message = "正在向车端提交目标……";
  try {
    await post("/api/teach/navigate", {
      request_id: requestId(),
      pose: poseOf(draft),
    });
    navigationRequest.phase = "ACCEPTED";
    navigationRequest.message =
      "车端已接受目标，正在准备导航；请查看下面的实时阶段";
  } catch (error) {
    navigationRequest.phase = "REJECTED";
    navigationRequest.message = `目标未被接受：${error.message}`;
    throw error;
  }
  notice.value = "目标已提交。到达并停稳后，核对实际位置，再开始调整相机。";
}
function nudgeCoordinate(key, delta) {
  draft[key] = Math.round((Number(draft[key]) + delta) * 100) / 100;
  poseEdited(draft);
  notice.value = "只修改了候选目标；请重新下发导航，让实车验证这次调整。";
}
async function stopEverything() {
  stopManual();
  if (trialActive.value) await post("/api/teach/cancel-navigation");
  else if (["RUNNING", "PAUSED"].includes(state.value.run?.status))
    await post("/api/run/cancel");
}
async function step(delta) {
  cancelSlider();
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
  if (purpose !== "navigation" && trialArrived.value)
    payload.trial_id = trial.value.request_id;
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
  notice.value = "先选择候选停车位和车头朝向，再点击“下发目标，让小车前往”。";
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
    await post("/api/localize", { pose: poseOf(initialPose) });
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
function drawNavigationLayers(ctx, map, density, scale) {
  for (const key of ["global_costmap", "local_costmap"]) {
    const layer = navigationImages[key];
    if (
      !navigationLayers[key] ||
      !layer?.meta.available ||
      !state.value.navigation_debug?.[key]?.available
    )
      continue;
    const meta = layer.meta,
      origin = mapToPixel(meta.origin, map),
      ratio = meta.resolution / map.resolution;
    ctx.save();
    ctx.translate(origin.u, origin.v);
    ctx.rotate(-(meta.origin.yaw - map.origin.yaw));
    ctx.drawImage(
      layer.image,
      0,
      -meta.height * ratio,
      meta.width * ratio,
      meta.height * ratio,
    );
    ctx.restore();
  }
  for (const [key, color] of [
    ["global_path", "#00a8bd"],
    ["local_path", "#ffe000"],
  ]) {
    const layer = state.value.navigation_debug?.[key];
    if (!navigationLayers[key] || !layer?.available || !layer.points?.length)
      continue;
    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = ((key === "local_path" ? 4 : 3) * density) / scale;
    if (layer.source_age_seconds > 2)
      ctx.setLineDash([(6 * density) / scale, (4 * density) / scale]);
    ctx.beginPath();
    layer.points.forEach(([x, y], index) => {
      const p = mapToPixel({ x, y }, map);
      if (index) ctx.lineTo(p.u, p.v);
      else ctx.moveTo(p.u, p.v);
    });
    ctx.stroke();
    ctx.restore();
  }
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
  drawNavigationLayers(ctx, m, density, scale);
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
      poseOf(editTarget.value),
      "#c24c72",
      page.value === "overview"
        ? "定位初值"
        : page.value === "tasks"
          ? "最终停车"
          : "候选目标",
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
  Object.assign(editTarget.value, p);
  poseEdited(editTarget.value);
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
    editTarget.value.yaw = ((((angle + 180) % 360) + 360) % 360) - 180;
  }
  draw();
}
function mapUp() {
  if (mapPointer) {
    mapSelecting.value = false;
    notice.value =
      page.value === "overview"
        ? "初始定位候选已标出。请核对这就是实物车当前的位置与朝向，再提交初值。"
        : page.value === "tasks"
          ? "最终停车位置与车头朝向已选好，安排巡检顺序后保存任务。选点不会让小车移动。"
          : "候选停车位与车头朝向已选好。点击“下发目标，让小车前往”才会开始行驶。";
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
  drawNavigationLayers(ctx, map, density, scale);
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
  window.addEventListener("blur", cancelSlider);
  window.addEventListener("pagehide", stopManual);
});
onBeforeUnmount(() => {
  stopped = true;
  cancelSlider();
  clearInterval(pollTimer);
  clearTimeout(retryTimer);
  release();
  socket?.close();
  if (videoUrl.value) URL.revokeObjectURL(videoUrl.value);
  document.removeEventListener("visibilitychange", visibility);
  window.removeEventListener("blur", release);
  window.removeEventListener("blur", cancelSlider);
  window.removeEventListener("pagehide", stopManual);
});
</script>

<template>
  <a class="skip-link" href="#inspection-main">跳到巡检工作区</a>
  <div class="shell">
    <aside>
      <a class="brand" href="/" aria-label="巡检首页"
        ><span class="brand-mark"><ConsoleIcon name="drive" /></span
        ><span>巡检<small>机器人工作空间</small></span></a
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
    <main id="inspection-main" tabindex="-1" :aria-busy="busy">
      <header class="topbar">
        <div class="breadcrumb">
          <span>工作空间</span><span aria-hidden="true">/</span
          ><strong>巡检工作台</strong>
        </div>
        <button class="stop" @click="action(stopEverything)">
          <ConsoleIcon name="stop" />停车并取消当前行驶
        </button>
      </header>
      <div class="page-intro">
        <h1>
          {{ navItems.find((i) => i[0] === page)?.[1] || "高级规划草图" }}
        </h1>
        <p class="page-description">{{ pageDescriptions[page] }}</p>
      </div>
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
          <p>
            先确认当前定位，再到“录入巡检点”下发单点导航。行驶时镜头固定为
            140.5°（板端 94 档）。所有点位实车试拍确认后，才安排整轮巡检。
          </p>
          <button class="primary" @click="setPage('tasks')">
            安排整轮巡检
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
          <p>点位参考照片与正式巡检照片分开标明，均可在网页查看、下载。</p>
        </article>
      </section>
      <template
        v-if="page === 'points' || page === 'tasks' || page === 'overview'"
      >
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
          ><button
            v-if="page === 'points'"
            :disabled="trialActive"
            @click="newPoint"
          >
            <ConsoleIcon name="plus" />新增点位</button
          ><button v-else-if="page === 'tasks'" @click="newTask">
            <ConsoleIcon name="plus" />新建任务</button
          ><span>当前图 {{ state.map?.id || "未选择" }}</span>
        </div>
        <section
          v-if="state.navigation_speed?.enabled"
          class="card speed-controls"
          aria-label="导航速度档位"
        >
          <h2>导航速度</h2>
          <div class="speed-options">
            <button
              v-for="option in state.navigation_speed.options"
              :key="option.mps"
              :aria-pressed="state.navigation_speed.applied_mps === option.mps"
              :class="{
                primary: state.navigation_speed.applied_mps === option.mps,
              }"
              :disabled="
                busy ||
                !connected ||
                trialActive ||
                !state.health.still ||
                state.navigation_speed.pending ||
                ['RUNNING', 'PAUSED'].includes(state.run?.status)
              "
              @click="
                action(() =>
                  post('/api/navigation/speed', { max_linear_mps: option.mps }),
                )
              "
            >
              {{ option.label }}
            </button>
          </div>
          <p
            role="status"
            :class="
              state.navigation_speed.ready ? 'status-good' : 'status-wait'
            "
          >
            {{ state.navigation_speed.message }}
          </p>
          <p class="caption">
            前进上限
            {{ state.navigation_speed.applied_mps?.toFixed(2) ?? "待确认" }} m/s
            · 当前下发
            {{ state.motion?.command_linear_mps?.toFixed(2) ?? "—" }} m/s ·
            轮式反馈 {{ state.motion?.wheel_linear_mps?.toFixed(2) ?? "—" }} m/s
          </p>
          <p class="caption">
            停车后点击切档，确认后用于下一次导航。上限不是匀速命令；转向、避障会减速。3、4档供现场实验，注意观察抖动和停车距离。
          </p>
        </section>
        <p class="position-guide">
          {{
            page === "points"
              ? "选候选目标 → 下发导航到点 → 实车坐标与相机微调 → 试拍确认并保存"
              : page === "overview"
                ? "本页只准备车辆：确定当前所在位置。行驶镜头固定为 140.5°，下一页再设置巡检目标。"
                : "选择已确认的巡检点 → 排列顺序 → 设置独立最终停车位 → 保存任务"
          }}
        </p>
        <ol
          v-if="page === 'points'"
          class="workflow-track"
          aria-label="巡检点录入步骤"
        >
          <li :class="{ current: !draftPoseChosen }">
            <strong>1</strong>选候选目标
          </li>
          <li
            :class="{
              current: draftPoseChosen && !trialArrived && !pointSaved,
            }"
          >
            <strong>2</strong>导航到点
          </li>
          <li :class="{ current: trialArrived && !pointSaved }">
            <strong>3</strong>实车微调与试拍
          </li>
          <li :class="{ current: pointSaved }">
            <strong>4</strong>保存确认结果
          </li>
        </ol>
        <div class="workspace">
          <article class="card map-card">
            <div class="card-title">
              <h2>
                <ConsoleIcon :name="page === 'tasks' ? 'tasks' : 'points'" />{{
                  page === "overview"
                    ? "小车当前所在位置"
                    : page === "tasks"
                      ? "巡检顺序与最终停车"
                      : "1. 选择候选停车位"
                }}
              </h2>
              <span>地图坐标 · 米</span>
            </div>
            <button
              class="primary map-pick"
              :disabled="!state.map || busy || trialActive"
              :aria-pressed="mapSelecting"
              @click="mapSelecting = !mapSelecting"
            >
              {{
                mapSelecting
                  ? "取消地图选点"
                  : page === "overview"
                    ? "标出小车当前真实位置与朝向"
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
            <details class="map-reading">
              <summary>地图坐标与颜色说明</summary>
              <p class="caption">
                保存地图 · map坐标系{{
                  state.map
                    ? ` · ${state.map.resolution.toFixed(3)} 米/格`
                    : ""
                }}。
                白：空闲　黑：占用　灰：未知　蓝：当前定位估计　粉：编辑目标　绿：已确认点　黄：草稿。箭头为车头朝向。
              </p>
              <p class="caption">
                地图按原比例显示，屏幕大小不改变米制坐标。请在白色区域选停车位，并现场核对车身空间；保存地图不代表当前障碍状态。
              </p>
            </details>
            <details class="navigation-debug">
              <summary>导航调试图层与数据状态</summary>
              <p class="caption">
                来自车端真实话题；全局／局部代价图包含障碍与膨胀安全余量，红色为高代价障碍，透明部分不表示已确认安全。局部数据按当前TF转换到map；显示刷新最多2Hz，路径仅抽样显示，不修改规划。旧路径用虚线。
              </p>
              <label
                v-for="(name, key) in navigationLayerNames"
                :key="key"
                class="navigation-layer-option"
              >
                <input
                  type="checkbox"
                  v-model="navigationLayers[key]"
                  @change="draw"
                />
                <span
                  >{{ name
                  }}<small>{{ navigationLayerStatus(key) }}</small></span
                >
              </label>
              <p
                :class="state.health.perception ? 'status-good' : 'status-wait'"
              >
                深度避障：{{
                  state.health.perception
                    ? "当前有效"
                    : "当前未放行；下发后仍会检查新点云与图层重建"
                }}
              </p>
            </details>
          </article>
          <article
            v-if="page === 'points'"
            class="card editor"
            aria-label="巡检点位编辑"
          >
            <h2>2. 下发目标，让小车前往</h2>
            <label
              >点位名称<input
                v-model="draft.name"
                placeholder="例如：走廊仪表"
                maxlength="60"
                :disabled="trialActive"
            /></label>
            <p class="caption">
              下面是候选导航目标。修改数字不会立即移动车辆，点击下发后才行驶。
            </p>
            <div class="form-row">
              <label
                >X / 米<input
                  v-model.number="draft.x"
                  type="number"
                  step="0.05"
                  :disabled="trialActive"
                  @input="poseEdited(draft)"
              /></label>
              <label
                >Y / 米<input
                  v-model.number="draft.y"
                  type="number"
                  step="0.05"
                  :disabled="trialActive"
                  @input="poseEdited(draft)"
              /></label>
              <label
                >车头朝向 / °<input
                  v-model.number="draft.yaw"
                  type="number"
                  step="1"
                  :disabled="trialActive"
                  @input="poseEdited(draft)"
              /></label>
            </div>
            <details class="extra-tools">
              <summary>微调候选坐标与方向</summary>
              <div class="coordinate-nudges">
                <button
                  v-for="[key, delta, text] in [
                    ['x', -0.05, 'X −5cm'],
                    ['x', 0.05, 'X +5cm'],
                    ['y', -0.05, 'Y −5cm'],
                    ['y', 0.05, 'Y +5cm'],
                    ['yaw', -1, '朝向 −1°'],
                    ['yaw', 1, '朝向 +1°'],
                  ]"
                  :key="text"
                  :disabled="trialActive"
                  @click="nudgeCoordinate(key, delta)"
                >
                  {{ text }}
                </button>
              </div>
              <p class="caption">
                X/Y沿地图坐标轴调整；改好后再次下发导航验证。导航有允许误差，修改5cm不保证实车移动5cm，以实车反馈和现场观察确认。
              </p>
            </details>
            <p v-if="goalReason" class="status-wait">
              {{ goalReason }}。<button
                class="quiet"
                @click="setPage('overview')"
              >
                去准备车辆
              </button>
            </p>
            <button
              class="primary big"
              :disabled="busy || trialActive"
              @click="action(navigateDraft)"
            >
              {{
                trialArrived ? "按调整后的目标重新导航" : "下发目标，让小车前往"
              }}
            </button>
            <p
              v-if="navigateReason && !trialActive"
              class="status-wait"
              role="status"
            >
              尚未下发：{{ navigateReason }}
            </p>
            <p
              v-if="
                navigationRequest.message &&
                (trialActive || navigationRequest.phase !== 'ACCEPTED')
              "
              :class="
                navigationRequest.phase === 'ACCEPTED'
                  ? 'status-good'
                  : 'status-wait'
              "
              role="status"
            >
              {{ navigationRequest.message }}
            </p>
            <p class="phase" role="status">{{ trialLabel(trial.phase) }}</p>
            <p v-if="trial.error" class="status-wait">{{ trial.error }}</p>
            <p v-if="trial.nav2_result" class="caption">
              导航器结果：{{
                trial.nav2_result
              }}。到达后还需停车、核对实物位置并试拍确认。
            </p>
            <p v-if="trial.arrival_error" class="caption">
              停车时与候选目标的定位估计差：{{
                (trial.arrival_error.distance_m * 100).toFixed(1)
              }}厘米／{{
                ((trial.arrival_error.yaw_rad * 180) / Math.PI).toFixed(1)
              }}°。此数值用于对照，保存的是确认时的实际定位估计。
            </p>
            <button
              v-if="trialActive"
              class="stop"
              :disabled="busy"
              @click="action(() => post('/api/teach/cancel-navigation'))"
            >
              取消本次导航并停车
            </button>
            <table class="pose-comparison">
              <caption>
                候选目标与实车定位估计（map坐标）
              </caption>
              <thead>
                <tr>
                  <th>来源</th>
                  <th>X / 米</th>
                  <th>Y / 米</th>
                  <th>朝向 / °</th>
                </tr>
              </thead>
              <tbody>
                <tr>
                  <th>候选目标</th>
                  <td>{{ Number(draft.x).toFixed(2) }}</td>
                  <td>{{ Number(draft.y).toFixed(2) }}</td>
                  <td>{{ Number(draft.yaw).toFixed(1) }}</td>
                </tr>
                <tr>
                  <th>实车当前估计</th>
                  <td>{{ state.pose?.x.toFixed(2) ?? "—" }}</td>
                  <td>{{ state.pose?.y.toFixed(2) ?? "—" }}</td>
                  <td>
                    {{
                      state.pose
                        ? ((state.pose.yaw * 180) / Math.PI).toFixed(1)
                        : "—"
                    }}
                  </td>
                </tr>
              </tbody>
            </table>
            <p v-if="pointSaved" class="status-good">
              此点已保存实车确认结果和参考照片，可到“安排巡检”添加进任务。继续修改时须重新导航、调角并确认。
            </p>
            <p v-else-if="trialArrived" class="status-good">
              导航器已确认到达并停稳，请核对实物停车位置。位置或朝向不合适就修改候选坐标、再导航一次；合适后到下方调整镜头并试拍，保存当前实车定位估计。
            </p>
            <button
              v-if="trialArrived && !pointSaved"
              class="primary"
              :disabled="busy"
              @click="action(startTrialCapture)"
            >
              下一步：调镜头并试拍
            </button>
            <p class="caption">
              最终确认保存的是实车当时的定位估计、舵机命令和新照片；候选坐标保留作对照，不当作实际到达值。
            </p>
            <details class="extra-tools">
              <summary>仅保存计划草稿（尚未实车确认）</summary>
              <button
                :disabled="
                  busy ||
                  !connected ||
                  !draft.name.trim() ||
                  !state.map ||
                  !draftPoseChosen ||
                  trialActive
                "
                @click="action(saveDraft)"
              >
                保存巡检点草稿
              </button>
            </details>
            <h3>已保存点位</h3>
            <div class="point-list">
              <button
                v-for="p in points"
                :key="p.id"
                :class="{ selected: draft.id === p.id }"
                :disabled="trialActive"
                @click="
                  usePoint(p);
                  draw();
                "
              >
                <strong>{{ p.name }}</strong
                ><span
                  >{{ p.verified ? "实车已确认" : "计划草稿，待导航验证" }} ·
                  {{ approachCount(p) }}个停车位</span
                >
              </button>
            </div>
            <button
              v-if="points.find((p) => p.id === draft.id)?.reference"
              @click="setPage('records')"
            >
              查看此点参考照片
            </button>
          </article>
          <article v-else-if="page === 'overview'" class="card editor">
            <h2>告诉系统：小车现在在哪里</h2>
            <p>
              只在定位尚未就绪或蓝色车标与实物不符时操作。这里填写当前实物位置，不是让车去的目标。
            </p>
            <div class="form-row">
              <label
                >当前 X / 米<input
                  v-model.number="initialPose.x"
                  type="number"
                  step="0.01"
                  @input="poseEdited(initialPose)"
              /></label>
              <label
                >当前 Y / 米<input
                  v-model.number="initialPose.y"
                  type="number"
                  step="0.01"
                  @input="poseEdited(initialPose)"
              /></label>
              <label
                >当前车头 / °<input
                  v-model.number="initialPose.yaw"
                  type="number"
                  step="1"
                  @input="poseEdited(initialPose)"
              /></label>
            </div>
            <button
              :disabled="
                busy ||
                !connected ||
                !state.hardware_enabled ||
                !state.health.still ||
                !initialPoseChosen ||
                trialActive
              "
              @click="action(localize)"
            >
              提交小车初始定位
            </button>
            <p
              :class="state.health.localization ? 'status-good' : 'status-wait'"
            >
              {{
                state.health.localization
                  ? "定位已就绪，请对照实物检查蓝色车标。"
                  : "定位待就绪；先标出真实位置并提交初值。"
              }}
            </p>
            <button class="primary" @click="setPage('points')">
              下一步：录入巡检点
            </button>
            <details class="extra-tools">
              <summary>高级：规划草图</summary>
              <p>只用于预排想法，不能导航；实际点位在实测地图上录入。</p>
              <button @click="setPage('sketch')">打开规划草图</button>
            </details>
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
      <section
        v-if="page === 'points' || page === 'overview'"
        class="workspace teaching"
      >
        <article class="card">
          <div class="card-title">
            <h2><ConsoleIcon name="camera" />相机实时画面</h2>
            <span :class="state.health.rgb ? 'status-good' : 'status-wait'">{{
              state.health.rgb ? "当前源帧有效" : "等待有效图像"
            }}</span>
          </div>
          <div class="video-surface">
            <img
              v-if="videoUrl"
              :src="videoUrl"
              alt="相机实时RGB画面"
              width="640"
              height="480"
            />
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
          <p v-if="page === 'overview'" class="caption">
            行驶时镜头固定为 140.5°（板端 94
            档）。启动时已经下发，不用再点确认。到点后如果要改拍摄角度，再到「录入巡检点」里调。
          </p>
        </article>
        <article v-if="page === 'points'" class="card">
          <h2>3. 实车到点后，调整镜头并试拍</h2>
          <p v-if="!trialArrived" class="status-wait">
            请先完成上方单点导航并停稳。修改候选坐标后，需要重新导航验证，才能确认保存。
          </p>
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
              拖动滑块选角度，松手后自动小步调整并等待新画面
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
              trialActive ||
              (page === 'points' && !trialArrived) ||
              !['IDLE', 'FAULT', 'CONFIRMED'].includes(teaching.phase)
            "
            @click="action(begin)"
          >
            开始停车调角
          </button>
          <div class="angle-readout">
            <strong
              >{{ (sliderTarget ?? teaching.target_degrees)?.toFixed(1)
              }}<small>°</small></strong
            ><span
              >{{ sliderDirty || sliderRunning ? "滑块目标" : "S1俯仰目标" }} ·
              已下发目标 {{ teaching.target_degrees?.toFixed(1) }}° · 板端命令
              {{ teaching.board_command ?? "—" }} 档</span
            >
          </div>
          <label class="servo-slider-label" for="servo-angle-slider"
            >相机俯仰目标</label
          >
          <input
            id="servo-angle-slider"
            class="servo-angle-slider"
            type="range"
            min="85"
            max="160"
            step="0.5"
            :value="sliderTarget ?? teaching.target_degrees"
            :disabled="
              busy ||
              !connected ||
              !trialArrived ||
              !state.motion_held ||
              !state.health.still ||
              ![
                'READY',
                'SENDING',
                'WAITING_FRAME',
                'AWAITING_CONFIRMATION',
              ].includes(teaching.phase)
            "
            @input="previewSlider"
            @change="commitSlider"
            @pointercancel="cancelSlider"
            @keydown.esc.prevent="cancelSlider"
          />
          <div class="servo-slider-scale">
            <span>85° 抬头方向</span><span>140.5° 平视参考</span
            ><span>160° 低头方向</span>
          </div>
          <button
            :disabled="!sliderRunning && !sliderDirty"
            @click="cancelSlider"
          >
            停止滑块调角
          </button>
          <p v-if="sliderRunning" class="status-wait" role="status">
            正在逐步调到滑块目标；可拖动改目标。等待本次发送及新画面，完成前不能保存。
          </p>
          <p v-else-if="sliderDirty" class="status-wait" role="status">
            滑块目标尚未完成；松手提交，或停止调角后按当前画面重新确认。
          </p>
          <div class="step-buttons">
            <button
              :disabled="
                busy ||
                sliderRunning ||
                !['READY', 'AWAITING_CONFIRMATION'].includes(teaching.phase)
              "
              @click="action(() => step(-1))"
            >
              目标 −1°</button
            ><button
              :disabled="
                busy ||
                sliderRunning ||
                !['READY', 'AWAITING_CONFIRMATION'].includes(teaching.phase)
              "
              @click="action(() => step(1))"
            >
              目标 +1°</button
            ><button
              :disabled="
                busy ||
                sliderRunning ||
                !['READY', 'AWAITING_CONFIRMATION'].includes(teaching.phase)
              "
              @click="action(() => step(0))"
            >
              下发当前目标
            </button>
          </div>
          <p class="caption">
            拖动只预览目标，松手后自动分成最多1°的小步，每步等待车端结果与新画面；停止、断连或离开页面后不再追加，已发送的一小步可能完成。±1°按钮保留作微调。目标减小为抬头、增大为低头，以实物核对；板端整数档会量化，没有实际角度反馈。
          </p>
          <button
            class="primary"
            :disabled="busy || !pointCanConfirm || !draft.name.trim()"
            v-if="page === 'points'"
            @click="action(() => confirm('point'))"
          >
            4. 确认实车位置与画面，保存巡检点</button
          ><button
            v-if="draft.id"
            :disabled="busy || !pointCanConfirm || !draft.id"
            @click="action(() => confirm('approach'))"
          >
            把当前位置存为备选停车位
          </button>
          <p class="caption">
            每个拍摄目标最多3个停车位，各自保存朝向和俯仰。某一位到不了就试下一位；三位都失败则记为未拍并继续后面的目标。保存后继续保持停车。支架异常时立即断开执行器电源。
          </p>
        </article>
      </section>
      <details v-if="page === 'points'" class="card remote">
        <summary>高级：手动遥控（首次录入优先使用单点导航）</summary>
        <div>
          <h2>手动遥控</h2>
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
      </details>
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
                · 已保存到小车
              </figcaption>
            </figure>
          </div></template
        >
      </section>
      <section v-if="page === 'records'" class="record-list">
        <article
          v-for="point in state.points.filter((p) => p.reference)"
          :key="'reference-' + point.id"
          class="card"
        >
          <h2>{{ point.name }} · 点位参考照片</h2>
          <p class="caption">
            人工录入点位时保存的历史画面，不计入正式巡检采集。坐标来自当时实车定位估计。
          </p>
          <div class="photos">
            <figure>
              <a :href="site('/api/media/' + point.reference.id)" download
                ><img
                  :src="site('/api/media/' + point.reference.id)"
                  alt="点位试拍参考照片"
              /></a>
              <figcaption>已保存到小车 · 点击照片下载</figcaption>
            </figure>
          </div>
        </article>
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
            {{ run.error || "照片已保存在小车，可直接在此查看和下载。" }}
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
                · 已保存到小车 · 点击下载
              </figcaption>
              <details>
                <summary>外部上传状态（可选）</summary>
                {{ uploadLabel(uploadState(run, photo)) }}
              </details>
            </figure>
          </div>
        </article>
        <p
          v-if="!state.runs.length && !state.points.some((p) => p.reference)"
          class="empty"
        >
          <ConsoleIcon name="records" />
          实车到点试拍后，参考照片会显示在这里；启动正式巡检后再追加每轮采集记录。
        </p>
      </section>
      <footer>
        <ConsoleIcon name="shield" />
        实车确认点位 · 照片保存在小车，可网页查看和下载 · 地图与点位按版本关联
      </footer>
    </main>
  </div>
</template>
