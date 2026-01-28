const defaultHost = () => {
  const params = new URLSearchParams(window.location.search);
  return params.get("host") || window.location.hostname || "kilo-dev.local";
};

const defaultPort = () => {
  const params = new URLSearchParams(window.location.search);
  return params.get("port") || "9090";
};

const elements = {
  host: document.getElementById("host"),
  port: document.getElementById("port"),
  connect: document.getElementById("connect"),
  connStatus: document.getElementById("connStatus"),
  motionAllowed: document.getElementById("motionAllowed"),
  emotion: document.getElementById("emotion"),
  uiLock: document.getElementById("uiLock"),
  uiEmotion: document.getElementById("uiEmotion"),
  safeToMove: document.getElementById("safeToMove"),
  safetyReason: document.getElementById("safetyReason"),
  safetyLatched: document.getElementById("safetyLatched"),
  safetyOverride: document.getElementById("safetyOverride"),
  safetyRx: document.getElementById("safetyRx"),
  safetyAge: document.getElementById("safetyAge"),
  controlLocked: document.getElementById("controlLocked"),
  controlLockedReason: document.getElementById("controlLockedReason"),
  controlRelay: document.getElementById("controlRelay"),
  controlRelayReason: document.getElementById("controlRelayReason"),
  controlThrottle: document.getElementById("controlThrottle"),
  controlRx: document.getElementById("controlRx"),
  controlAge: document.getElementById("controlAge"),
  safetyRaw: document.getElementById("safetyRaw"),
  controlRaw: document.getElementById("controlRaw"),
  motionCard: document.getElementById("motionCard"),
};

const state = {
  socket: null,
  safety: {},
  control: {},
  safetyRxMs: 0,
  controlRxMs: 0,
  connected: false,
};

const fmtBool = (value) => (value ? "TRUE" : "FALSE");
const fmtAge = (ageMs) => {
  if (!ageMs) return "—";
  const s = Math.max(0, Math.round(ageMs / 100) / 10);
  return `${s.toFixed(1)}s`;
};

const fmtTime = (tsMs) => {
  if (!tsMs) return "—";
  const dt = new Date(tsMs);
  return dt.toLocaleTimeString();
};

const setStatus = (text, level) => {
  elements.connStatus.textContent = text;
  elements.connStatus.classList.remove("ok", "warn", "bad");
  if (level) elements.connStatus.classList.add(level);
};

const deriveUi = () => {
  const safe = Boolean(state.safety.safe_to_move);
  const reason = String(state.safety.reason || "UNKNOWN");
  const locked = Boolean(state.control.locked ?? true);
  const lockedReason = String(state.control.locked_reason || "");
  const relayKilled = Boolean(state.control.relay_killed ?? true);

  const uiLock = !safe || locked || relayKilled;
  let uiEmotion = "OK";
  if (!safe) uiEmotion = reason;
  else if (locked) uiEmotion = lockedReason || "LOCKED";
  else if (relayKilled) uiEmotion = "RELAY_KILLED";

  const uiMotionAllowed = safe && !locked && !relayKilled;

  return { uiLock, uiEmotion, uiMotionAllowed };
};

const updateUi = () => {
  const { uiLock, uiEmotion, uiMotionAllowed } = deriveUi();

  elements.motionAllowed.textContent = uiMotionAllowed ? "YES" : "NO";
  elements.emotion.textContent = uiEmotion;
  elements.uiLock.textContent = fmtBool(uiLock);
  elements.uiEmotion.textContent = uiEmotion;

  elements.safeToMove.textContent = fmtBool(Boolean(state.safety.safe_to_move));
  elements.safetyReason.textContent = String(state.safety.reason || "—");
  elements.safetyLatched.textContent = fmtBool(Boolean(state.safety.latched));
  elements.safetyOverride.textContent = fmtBool(Boolean(state.safety.override_required));
  elements.safetyRx.textContent = fmtTime(state.safetyRxMs);
  elements.safetyAge.textContent = fmtAge(Date.now() - state.safetyRxMs);

  elements.controlLocked.textContent = fmtBool(Boolean(state.control.locked));
  elements.controlLockedReason.textContent = String(state.control.locked_reason || "—");
  elements.controlRelay.textContent = fmtBool(Boolean(state.control.relay_killed));
  elements.controlRelayReason.textContent = String(state.control.relay_reason || "—");
  const throttle = state.control.applied?.throttle;
  elements.controlThrottle.textContent = Number.isFinite(throttle) ? throttle.toFixed(3) : "—";
  elements.controlRx.textContent = fmtTime(state.controlRxMs);
  elements.controlAge.textContent = fmtAge(Date.now() - state.controlRxMs);

  elements.safetyRaw.textContent = JSON.stringify(state.safety, null, 2);
  elements.controlRaw.textContent = JSON.stringify(state.control, null, 2);

  if (uiMotionAllowed) {
    elements.motionCard.style.borderColor = "#1f8f5f";
    elements.motionAllowed.className = "badge-ok";
  } else if (uiLock) {
    elements.motionCard.style.borderColor = "#c0392b";
    elements.motionAllowed.className = "badge-bad";
  } else {
    elements.motionAllowed.className = "badge-warn";
  }
};

const subscribe = () => {
  if (!state.socket || state.socket.readyState !== WebSocket.OPEN) return;
  const sub = (topic) => {
    state.socket.send(JSON.stringify({ op: "subscribe", topic }));
  };
  sub("/kilo/state/safety_json");
  sub("/kilo/state/control_json");
};

const connect = () => {
  if (state.socket) {
    state.socket.close();
  }

  const host = elements.host.value.trim();
  const port = elements.port.value.trim();
  const url = `ws://${host}:${port}`;

  setStatus("Connecting", "warn");

  const socket = new WebSocket(url);
  state.socket = socket;

  socket.onopen = () => {
    state.connected = true;
    setStatus("Connected", "ok");
    subscribe();
  };

  socket.onclose = () => {
    state.connected = false;
    setStatus("Disconnected", "bad");
  };

  socket.onerror = () => {
    state.connected = false;
    setStatus("Socket Error", "bad");
  };

  socket.onmessage = (event) => {
    let payload;
    try {
      payload = JSON.parse(event.data);
    } catch (err) {
      return;
    }

    if (payload.op !== "publish" || !payload.topic || !payload.msg) return;

    const raw = payload.msg.data;
    if (typeof raw !== "string") return;

    let parsed;
    try {
      parsed = JSON.parse(raw);
    } catch (err) {
      return;
    }

    if (payload.topic === "/kilo/state/safety_json") {
      if (parsed.schema_version !== "state_safety_v1") return;
      state.safety = parsed;
      state.safetyRxMs = Date.now();
    }

    if (payload.topic === "/kilo/state/control_json") {
      if (parsed.schema_version !== "state_control_v1") return;
      state.control = parsed;
      state.controlRxMs = Date.now();
    }

    updateUi();
  };
};

const init = () => {
  elements.host.value = defaultHost();
  elements.port.value = defaultPort();
  elements.connect.addEventListener("click", connect);
  connect();
  updateUi();
  setInterval(updateUi, 500);
};

init();
