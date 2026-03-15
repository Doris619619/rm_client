const API_URL = "http://127.0.0.1:8000/api/state";
const POLL_MS = 400;

const FRESHNESS_THRESHOLDS = {
  game: 2,
  global: 3,
  robot: 1,
  ack: 5,
};

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

function toNumber(value) {
  if (typeof value === "number" && Number.isFinite(value)) return value;
  if (typeof value === "string" && value.trim() !== "") {
    const n = Number(value);
    if (Number.isFinite(n)) return n;
  }
  return null;
}

function toSecondsText(value) {
  const n = toNumber(value);
  if (n === null) return "--";
  return `${Math.max(0, Math.floor(n))}s`;
}

function parseIsoTime(isoText) {
  if (!isoText || typeof isoText !== "string") return null;
  const ms = Date.parse(isoText);
  if (Number.isNaN(ms)) return null;
  return ms;
}

function freshnessFromIso(isoText, thresholdSec) {
  const ts = parseIsoTime(isoText);
  if (ts === null) {
    return { text: "--", className: "fresh unknown" };
  }
  const ageSec = (Date.now() - ts) / 1000;
  if (ageSec <= thresholdSec) {
    return { text: "ONLINE", className: "fresh online" };
  }
  return { text: "STALE", className: "fresh stale" };
}

function setFreshness(elId, isoText, thresholdSec) {
  const el = byId(elId);
  const result = freshnessFromIso(isoText, thresholdSec);
  el.textContent = result.text;
  el.className = result.className;
}

function buildGlobalSummary(gu) {
  if (!gu || typeof gu !== "object") return null;
  return {
    baseHealth: gu.base_health,
    baseStatus: gu.base_status,
    outpostHealth: gu.outpost_health,
    damageRed: gu.total_damage_red,
    damageBlue: gu.total_damage_blue,
  };
}

function renderGlobalSummary(gu) {
  const noData = byId("globalNoData");
  const summary = byId("globalSummary");
  const data = buildGlobalSummary(gu);

  if (!data) {
    noData.classList.remove("hidden");
    summary.classList.add("hidden");
    byId("globalBaseHealth").textContent = "--";
    byId("globalBaseStatus").textContent = "--";
    byId("globalOutpostHealth").textContent = "--";
    byId("globalDamageRed").textContent = "--";
    byId("globalDamageBlue").textContent = "--";
    return;
  }

  noData.classList.add("hidden");
  summary.classList.remove("hidden");
  byId("globalBaseHealth").textContent = show(data.baseHealth);
  byId("globalBaseStatus").textContent = show(data.baseStatus);
  byId("globalOutpostHealth").textContent = show(data.outpostHealth);
  byId("globalDamageRed").textContent = show(data.damageRed);
  byId("globalDamageBlue").textContent = show(data.damageBlue);
}

function renderState(data) {
  const gs = data?.game_status || null;
  const gu = data?.global_unit_status || null;
  const rd = data?.robot_dynamic_status || null;
  const gc = data?.guard_ctrl_result || null;
  const meta = data?.meta || {};

  byId("redScore").textContent = show(gs?.red_score);
  byId("blueScore").textContent = show(gs?.blue_score);
  byId("stageDisplay").textContent = show(gs?.current_stage);
  byId("remainTime").textContent = toSecondsText(gs?.stage_countdown_sec);

  const currentRound = show(gs?.current_round);
  const totalRounds = show(gs?.total_rounds);
  byId("roundDisplay").textContent = `${currentRound} / ${totalRounds}`;

  const bridgeAlive = meta?.bridge_alive === true;
  const statusText = bridgeAlive ? "ONLINE" : "OFFLINE";
  const statusEl = byId("connStatus");
  statusEl.textContent = statusText;
  statusEl.className = `meta-v ${bridgeAlive ? "status-ok" : "status-bad"}`;

  const badge = byId("bridgeBadge");
  badge.textContent = bridgeAlive ? "BRIDGE ONLINE" : "BRIDGE OFFLINE";
  badge.className = `bridge-badge ${bridgeAlive ? "online" : "offline"}`;

  byId("health").textContent = show(rd?.current_health);
  byId("ammo").textContent = show(rd?.remaining_ammo);
  byId("canAmmo").textContent = yesNo(rd?.can_remote_ammo);
  byId("canHeal").textContent = yesNo(rd?.can_remote_heal);

  renderGlobalSummary(gu);

  byId("cmdId").textContent = show(gc?.command_id);
  byId("resultCode").textContent = show(gc?.result_code);
  byId("ackTime").textContent = show(meta?.last_update_guard_ctrl_result);

  setFreshness("freshGame", meta?.last_update_game_status, FRESHNESS_THRESHOLDS.game);
  setFreshness("freshGlobal", meta?.last_update_global_unit_status, FRESHNESS_THRESHOLDS.global);
  setFreshness("freshRobot", meta?.last_update_robot_dynamic_status, FRESHNESS_THRESHOLDS.robot);
  setFreshness("freshAck", meta?.last_update_guard_ctrl_result, FRESHNESS_THRESHOLDS.ack);
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
