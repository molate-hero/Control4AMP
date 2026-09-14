/* 网络控制台前端逻辑：只通过后端 API 发送控制命令。 */
const $ = (id) => document.getElementById(id);
const state = {
  groundSources: new Map(["forward", "backward", "left", "right"].map((direction) => [direction, new Set()])),
  groundTimer: null,
  sprint: false,
  lastForwardTap: 0,
  autonomyEnabled: false,
  flightActive: false,
  flightTimer: null,
  pendingAction: null,
  preflightDisabled: false,
};

// 统一显示短时操作反馈，避免用户需要查看浏览器控制台判断结果。
function showMessage(message) {
  const bar = $("snackbar"); bar.textContent = message; bar.classList.add("show");
  window.clearTimeout(showMessage.timer); showMessage.timer = window.setTimeout(() => bar.classList.remove("show"), 2600);
}
async function api(path, options = {}) {
  const response = await fetch(path, { headers: { "Content-Type": "application/json" }, ...options });
  const data = await response.json().catch(() => ({}));
  if (!response.ok || data.ok === false) throw new Error(data.error || `请求失败（${response.status}）`);
  return data;
}
function fixed(value) { return Number(value).toFixed(2); }

// 根据当前按下的方向生成归一化差速输入。普通行驶限速，疾跑只作用于持续前进。
function groundValues() {
  const active = (direction) => state.groundSources.get(direction).size > 0;
  const forward = active("forward"), backward = active("backward");
  const left = active("left"), right = active("right");
  const speed = state.sprint && forward && !backward ? 1 : 0.65;
  return {
    throttle: (forward ? speed : 0) - (backward ? 0.65 : 0),
    steering: (right ? 0.65 : 0) - (left ? 0.65 : 0),
  };
}
function groundHasInput() { return [...state.groundSources.values()].some((sources) => sources.size > 0); }
function refreshGroundVisuals() {
  const names = { forward: "前进", backward: "后退", left: "左转", right: "右转" };
  const actions = Object.keys(names).filter((direction) => state.groundSources.get(direction).size > 0).map((direction) => names[direction]);
  $("groundAction").textContent = actions.length ? actions.join(" + ") : "停车";
  $("groundSprintState").textContent = state.sprint && state.groundSources.get("forward").size ? "疾跑中" : "疾跑未启用";
  document.querySelectorAll(".direction-button").forEach((button) => button.classList.toggle("is-active", state.groundSources.get(button.dataset.direction).size > 0));
}
async function sendGroundInput() {
  try {
    if (!groundHasInput()) return;
    await api("/api/ground/control", { method: "POST", body: JSON.stringify(groundValues()) });
  } catch (error) { showMessage(error.message); clearGroundInputs(); }
}
async function sendGroundStop() {
  try { await api("/api/ground/stop", { method: "POST", body: "{}" }); }
  catch (error) { showMessage(error.message); }
}
function ensureGroundTimer() { if (!state.groundTimer) state.groundTimer = window.setInterval(sendGroundInput, 100); }
function clearGroundTimer() { window.clearInterval(state.groundTimer); state.groundTimer = null; }
function registerForwardTap() {
  const now = performance.now();
  // 两次独立的前进按下间隔足够短时，按 Minecraft 风格进入疾跑。
  state.sprint = now - state.lastForwardTap <= 350;
  state.lastForwardTap = now;
}
function setGroundSource(direction, source, active) {
  if (state.autonomyEnabled) return;
  const sources = state.groundSources.get(direction);
  if (active) sources.add(source); else sources.delete(source);
  if (!active && direction === "forward" && sources.size === 0) state.sprint = false;
  refreshGroundVisuals();
  if (groundHasInput()) { ensureGroundTimer(); sendGroundInput(); }
  else { clearGroundTimer(); sendGroundStop(); }
}
function clearGroundInputs() {
  state.groundSources.forEach((sources) => sources.clear());
  state.sprint = false; refreshGroundVisuals(); clearGroundTimer();
  // 浏览器失焦时只清理手动输入；自主运行不应因切换窗口而被停车。
  if (!state.autonomyEnabled) sendGroundStop();
}
function bindGroundButtons() {
  document.querySelectorAll(".direction-button").forEach((button) => {
    const direction = button.dataset.direction;
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault(); button.setPointerCapture?.(event.pointerId);
      if (direction === "forward") registerForwardTap();
      setGroundSource(direction, `pointer-${event.pointerId}`, true);
    });
    ["pointerup", "pointercancel", "lostpointercapture"].forEach((eventName) => button.addEventListener(eventName, (event) => {
      setGroundSource(direction, `pointer-${event.pointerId}`, false);
    }));
  });
}
const keyboardDirections = { ArrowUp: "forward", ArrowDown: "backward", ArrowLeft: "left", ArrowRight: "right", KeyW: "forward", KeyS: "backward", KeyA: "left", KeyD: "right" };
document.addEventListener("keydown", (event) => {
  const direction = keyboardDirections[event.code]; if (!direction) return;
  if (state.autonomyEnabled) return;
  event.preventDefault(); if (event.repeat) return;
  if (direction === "forward") registerForwardTap();
  setGroundSource(direction, `key-${event.code}`, true);
});
document.addEventListener("keyup", (event) => {
  const direction = keyboardDirections[event.code]; if (!direction) return;
  event.preventDefault(); setGroundSource(direction, `key-${event.code}`, false);
});
window.addEventListener("blur", () => { if (!state.autonomyEnabled) clearGroundInputs(); });

// 读取页面中的飞行目标滑块，并按固定频率保持网络 setpoint 新鲜。
function flightValues() { return { roll: $("rollSlider").value, pitch: $("pitchSlider").value, yaw: $("yawSlider").value, thrust: $("thrustSlider").value }; }
function refreshSliderLabels() { ["roll", "pitch", "yaw", "thrust"].forEach((name) => { $(`${name}Output`).textContent = fixed($(`${name}Slider`).value); }); }
async function sendFlightSetpoint() { try { await api("/api/fc/setpoint", { method: "POST", body: JSON.stringify(flightValues()) }); } catch (error) { showMessage(error.message); if (state.flightActive) await stopFlightControl(); throw error; } }
async function startFlightControl() {
  try {
    // 先通知后端建立 setpoint 流，再开始发送页面目标。
    await api("/api/fc/control", { method: "POST", body: JSON.stringify({ enabled: true }) });
    state.flightActive = true; await sendFlightSetpoint(); state.flightTimer = window.setInterval(sendFlightSetpoint, 100);
    showMessage("已建立 Offboard setpoint 流");
  } catch (error) { $("flightControlSwitch").checked = false; showMessage(error.message); }
}
async function stopFlightControl() {
  if (!state.flightActive) return; state.flightActive = false; window.clearInterval(state.flightTimer); state.flightTimer = null;
  // 关闭前先将页面目标归零，再通知后端停止网络控制流。
  ["rollSlider", "pitchSlider", "yawSlider", "thrustSlider"].forEach((id) => $(id).value = 0); refreshSliderLabels();
  try { await api("/api/fc/setpoint", { method: "POST", body: JSON.stringify(flightValues()) }); await api("/api/fc/control", { method: "POST", body: JSON.stringify({ enabled: false }) }); showMessage("网络飞行控制已关闭"); }
  catch (error) { showMessage(error.message); }
}

// 半秒轮询后端状态，页面只负责展示，安全判断始终在服务端执行。
async function refreshStatus() {
  try {
    const data = await api("/api/status"); const flight = data.flight, ground = data.ground;
    $("runtimeMode").textContent = data.hardware ? "LIVE" : "SIMULATION"; $("serverMessage").textContent = data.hardware ? "真实硬件已授权" : "仅仿真，不操作串口";
    $("connectionChip").classList.toggle("offline", data.hardware && !flight.connected); $("connectionText").textContent = data.hardware && !flight.connected ? "飞控未连接" : "服务在线";
    $("groundState").textContent = ground.mode === "LIVE" ? "在线" : "仿真"; $("flightState").textContent = flight.connected ? "在线" : "未连接";
    state.autonomyEnabled = Boolean(ground.autonomy_enabled);
    $("autonomyButton").textContent = state.autonomyEnabled ? "关闭自主运行" : "自主运行";
    $("autonomyButton").classList.toggle("autonomy-active", state.autonomyEnabled);
    $("autonomyHint").textContent = ground.autonomy_error || (state.autonomyEnabled ? `自主运行中：${ground.autonomy_reason || "正在读取深度"}` : "自主运行会使用深度相机自动避障；启动时会暂时暂停深度预览。");
    document.querySelectorAll(".direction-button").forEach((button) => { button.disabled = state.autonomyEnabled; });
    $("flightMode").textContent = flight.mode || "—"; $("flightArmed").textContent = flight.armed ? "是" : "否";
    $("flightState").textContent = flight.offboard_ready ? "Offboard 就绪" : (flight.connected ? "在线" : "未连接");
    $("flightArmedBadge").classList.toggle("armed", flight.armed); $("flightArmedBadge").classList.toggle("disarmed", !flight.armed);
    $("flightArmedText").textContent = flight.armed ? "已解锁" : "已上锁"; $("armButton").disabled = flight.armed; $("disarmButton").disabled = !flight.armed;
    state.preflightDisabled = Boolean(flight.preflight_disarm_disabled);
    $("preflightDisarmState").textContent = state.preflightDisabled ? "已关闭，PX4 不会因未起飞自动上锁" : "当前启用，约 10 秒未起飞会自动上锁";
    $("debugPreflightButton").textContent = state.preflightDisabled ? "恢复自动上锁" : "关闭自动上锁";
    $("batteryValue").textContent = flight.battery == null ? "—" : `${flight.battery}%`; $("altitudeValue").textContent = flight.altitude_relative == null ? "—" : `${Number(flight.altitude_relative).toFixed(2)} m`;
  } catch (error) { $("connectionChip").classList.add("offline"); $("connectionText").textContent = "服务离线"; }
}
// 解锁、上锁和紧急降落等高风险动作必须经过二次确认。
function openConfirm(title, message, action) { state.pendingAction = action; $("dialogTitle").textContent = title; $("dialogMessage").textContent = message; $("confirmDialog").showModal(); }
async function runAction(action) {
  try {
    if (action === "arm") await api("/api/fc/arm", { method: "POST", body: JSON.stringify({ confirm: true }) });
    if (action === "disarm") await api("/api/fc/disarm", { method: "POST", body: "{}" });
    if (action === "land") await api("/api/fc/emergency", { method: "POST", body: "{}" });
    if (action === "disable-preflight") await api("/api/fc/debug/preflight-disarm", { method: "POST", body: JSON.stringify({ disabled: true, confirm: true }) });
    if (action === "restore-preflight") await api("/api/fc/debug/preflight-disarm", { method: "POST", body: JSON.stringify({ disabled: false }) });
    if (action === "autonomy-start") {
      await api("/api/ground/autonomy", { method: "POST", body: JSON.stringify({ enabled: true }) });
      // 立即更新本地状态，避免启动请求返回到状态轮询之间被失焦逻辑误判为手动模式。
      state.autonomyEnabled = true;
    }
    showMessage(action === "autonomy-start" ? "自主运行已启动" : "命令已发送");
    await refreshStatus();
  } catch (error) { showMessage(error.message); }
}

bindGroundButtons();
$("groundStopButton").addEventListener("click", async () => {
  clearGroundInputs();
  if (state.autonomyEnabled) {
    try { await api("/api/ground/autonomy", { method: "POST", body: JSON.stringify({ enabled: false }) }); state.autonomyEnabled = false; }
    catch (error) { showMessage(error.message); return; }
  }
  showMessage("已发送停车命令");
});
$("autonomyButton").addEventListener("click", async () => {
  if (state.autonomyEnabled) {
    try { await api("/api/ground/autonomy", { method: "POST", body: JSON.stringify({ enabled: false }) }); showMessage("自主运行已停止"); await refreshStatus(); }
    catch (error) { showMessage(error.message); }
  } else {
    openConfirm("启动自主运行？", "地面车将根据 RealSense 深度相机自动行驶并避障；启动时会暂停深度预览。请确认车辆周围安全。", "autonomy-start");
  }
});
["roll", "pitch", "yaw", "thrust"].forEach((name) => $(`${name}Slider`).addEventListener("input", () => { refreshSliderLabels(); if (state.flightActive) sendFlightSetpoint(); }));
$("flightControlSwitch").addEventListener("change", (event) => event.target.checked ? startFlightControl() : stopFlightControl());
$("debugPreflightButton").addEventListener("click", () => { const action = state.preflightDisabled ? "restore-preflight" : "disable-preflight"; const message = state.preflightDisabled ? "恢复 PX4 原来的未起飞自动上锁设置。" : "仅用于拆桨调试，将把 COM_DISARM_PRFLT 设为 -1，关闭未起飞自动上锁。"; openConfirm(state.preflightDisabled ? "恢复自动上锁？" : "关闭未起飞自动上锁？", message, action); });
$("modeButton").addEventListener("click", async () => { try { await api("/api/fc/mode", { method: "POST", body: JSON.stringify({ mode: $("modeSelect").value }) }); showMessage("模式切换命令已发送"); } catch (error) { showMessage(error.message); } });
$("armButton").addEventListener("click", () => openConfirm("确认解锁？", "解锁会使飞行器进入可运行状态，请确认周围环境安全。", "arm"));
$("disarmButton").addEventListener("click", () => openConfirm("确认上锁？", "将向飞控发送上锁命令。", "disarm"));
$("emergencyButton").addEventListener("click", () => openConfirm("确认紧急降落？", "这会立即向飞控发送 LAND 命令。", "land"));
$("dialogConfirm").addEventListener("click", (event) => { event.preventDefault(); $("confirmDialog").close(); runAction(state.pendingAction); });
window.addEventListener("pagehide", () => { fetch("/api/ground/stop", { method: "POST", headers: { "Content-Type": "application/json" }, body: "{}", keepalive: true }); });
refreshGroundVisuals();
refreshSliderLabels(); refreshStatus(); window.setInterval(refreshStatus, 500);
