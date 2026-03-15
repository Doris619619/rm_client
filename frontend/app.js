const API_URL = "http://127.0.0.1:8000/api/state";
const POLL_MS = 400;

function byId(id) {
  return document.getElementById(id);
}

function show(value, fallback = "--") {
  if (value === null || value === undefined || value === "") return fallback;
  return String(value);
}

function yesNo(value) {
  if (value === true) return "是";
  if (value === false) return "否";
  return "--";
}

function renderState(data) {
  const gs = data?.game_status || null;
  const gu = data?.global_unit_status || null;
  const rd = data?.robot_dynamic_status || null;
  const gc = data?.guard_ctrl_result || null;
  const meta = data?.meta || {};

  byId("currentStage").textContent = show(gs?.current_stage);
  byId("elapsedSec").textContent = show(gs?.stage_elapsed_sec);
  byId("countdownSec").textContent = show(gs?.stage_countdown_sec);

  const currentRound = show(gs?.current_round);
  const totalRounds = show(gs?.total_rounds);
  byId("roundInfo").textContent = `${currentRound} / ${totalRounds}`;

  const bridgeAlive = meta?.bridge_alive === true;
  const hasCore = !!(gs || gu || rd || gc);
  const statusText = bridgeAlive ? (hasCore ? "已连接并接收数据" : "已连接，等待数据") : "桥接离线";
  const statusEl = byId("systemStatus");
  statusEl.textContent = statusText;
  statusEl.className = `v ${bridgeAlive ? "status-ok" : "status-bad"}`;

  const pill = byId("bridgeStatus");
  pill.textContent = bridgeAlive ? "BRIDGE ONLINE" : "BRIDGE OFFLINE";
  pill.className = `pill ${bridgeAlive ? "online" : "offline"}`;

  byId("health").textContent = show(rd?.current_health);
  byId("ammo").textContent = show(rd?.remaining_ammo);
  byId("canAmmo").textContent = yesNo(rd?.can_remote_ammo);
  byId("canHeal").textContent = yesNo(rd?.can_remote_heal);

  const globalEl = byId("globalStatus");
  if (!gu) {
    globalEl.textContent = "暂无数据";
  } else {
    globalEl.textContent = JSON.stringify(gu, null, 2);
  }

  byId("cmdId").textContent = show(gc?.command_id);
  byId("resultCode").textContent = show(gc?.result_code);
  byId("ackTime").textContent = show(meta?.last_update_guard_ctrl_result);
}

async function poll() {
  try {
    const resp = await fetch(API_URL, { cache: "no-store" });
    if (!resp.ok) throw new Error(`HTTP ${resp.status}`);
    const data = await resp.json();
    renderState(data);
  } catch (_err) {
    renderState({
      game_status: null,
      global_unit_status: null,
      robot_dynamic_status: null,
      guard_ctrl_result: null,
      meta: { bridge_alive: false },
    });
  }
}

poll();
setInterval(poll, POLL_MS);
