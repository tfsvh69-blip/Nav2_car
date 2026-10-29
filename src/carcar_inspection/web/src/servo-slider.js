// 滑块只选最终目标；继续复用车端1°单步及新帧检查，最多一条请求在途。
export function createServoSlider({ read, send, wait = (ms) => new Promise((r) => setTimeout(r, ms)), now = Date.now, onRunning = () => {} }) {
  let target, task = null, generation = 0;
  function checked() {
    const state = read();
    if (!state.connected || !state.visible || !state.allowed)
      throw new Error("调角已停止：连接、页面或当前到点状态已变化；没有继续追加角度命令");
    if (!state.held || !state.still || !state.rgb)
      throw new Error("调角已停止：停车或相机反馈尚未就绪；没有继续追加角度命令");
    if (!["READY", "SENDING", "WAITING_FRAME", "AWAITING_CONFIRMATION"].includes(state.teaching.phase))
      throw new Error(state.teaching.error || "请先开始停车调角，再使用滑块");
    return state.teaching;
  }
  async function run(id) {
    let deadline = now() + 5000;
    while (id === generation) {
      const state = checked();
      if (!["READY", "AWAITING_CONFIRMATION"].includes(state.phase)) {
        if (now() > deadline) throw new Error("等待本次调角结果或新画面超过5秒，已停止追加命令；请查看相机与车端状态");
        await wait(50); continue;
      }
      if (Math.abs(state.target_degrees - target) < 1e-6 && state.phase === "AWAITING_CONFIRMATION")
        return { completed: true, target };
      const next = Math.round((state.target_degrees + Math.max(-1, Math.min(1, target - state.target_degrees))) * 10) / 10;
      const revision = state.revision;
      await send(next, revision);
      deadline = now() + 5000;
      // POST应答不代表新画面已到；不能按旧缓存重复下发或提前确认保存。
      while (id === generation) {
        const update = checked();
        if (update.revision > revision && update.phase === "AWAITING_CONFIRMATION" && Math.abs(update.target_degrees - next) < 1e-6) break;
        if (now() > deadline) throw new Error("等待本次调角结果或新画面超过5秒，已停止追加命令；请查看相机与车端状态");
        await wait(50);
      }
    }
    return { completed: false };
  }
  return {
    start(value) {
      if (!Number.isFinite(value) || value < 85 || value > 160)
        return Promise.reject(new Error("滑块目标须在85°～160°，没有发送角度命令"));
      target = Math.round(value * 2) / 2;
      if (task) return task; // 新目标覆盖旧目标，不累积整条轨迹。
      const id = ++generation;
      onRunning(true);
      task = run(id).finally(() => { task = null; onRunning(false); });
      return task;
    },
    stop() { ++generation; },
  };
}
