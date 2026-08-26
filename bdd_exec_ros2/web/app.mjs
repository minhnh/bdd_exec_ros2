import {
  buildLanes,
  buildScenarios,
  detailEntries,
  displayKind,
  formatStamp,
  isTimelineTrinary,
  laneStateAt,
  recordSeconds,
  selectContextId,
} from "./timeline.mjs";

const NS = "http://www.w3.org/2000/svg";
const records = new Map();
const connection = document.querySelector("#connection");
const connectionDot = document.querySelector("#connection-dot");
const lastUpdate = document.querySelector("#last-update");
const empty = document.querySelector("#empty");
const timeline = document.querySelector("#timeline");
const scenarioList = document.querySelector("#scenario-list");
const laneLabels = document.querySelector("#lane-labels");
const followButton = document.querySelector("#follow");
const playheadInput = document.querySelector("#playhead");
const playheadTime = document.querySelector("#playhead-time");
const details = document.querySelector("#details");
const detailsEmpty = document.querySelector("#details-empty");

let followLive = true;
let selectedContextId = null;
let selectedTime = 0;
let selectedRecord = null;
let latestSnapshot = null;
let latestStamp = null;
let useSimTime = false;
let renderFrame = null;

function formatSeconds(value) {
  let sec = Math.floor(value);
  let nanosec = Math.round((value - sec) * 1e9);
  if (nanosec === 1e9) {
    sec += 1;
    nanosec = 0;
  }
  return formatStamp({ sec, nanosec }, useSimTime);
}

function formatDuration(value) {
  const milliseconds = Math.max(0, Math.round(value * 1000));
  const hours = Math.floor(milliseconds / 3600000);
  const minutes = Math.floor((milliseconds % 3600000) / 60000);
  const seconds = Math.floor((milliseconds % 60000) / 1000);
  const fraction = String(milliseconds % 1000).padStart(3, "0");
  return [hours, minutes, seconds]
    .map((part) => String(part).padStart(2, "0"))
    .join(":") + "." + fraction;
}

function setConnection(text, state) {
  connection.textContent = text;
  connectionDot.className = state;
}

function addRecord(record) {
  records.set(record.id, record);
}

function scheduleRender() {
  if (renderFrame !== null) return;
  renderFrame = requestAnimationFrame(() => {
    renderFrame = null;
    render();
  });
}

function receive(message) {
  if (message.type === "hello") {
    useSimTime = Boolean(message.use_sim_time);
    setConnection("Connected · " + message.mode, "connected");
  } else if (message.type === "snapshot") {
    if (Array.isArray(message.history)) {
      records.clear();
      message.history.forEach(addRecord);
    }
    latestSnapshot = message.snapshot;
    latestStamp = latestSnapshot.stamp;
    lastUpdate.textContent = "Last update " + formatStamp(latestStamp, useSimTime);
  } else if (message.type === "timeline_record") {
    addRecord(message.record);
  }
  scheduleRender();
}

function connect() {
  const scheme = location.protocol === "https:" ? "wss:" : "ws:";
  const socket = new WebSocket(scheme + "//" + location.host + "/ws");
  socket.addEventListener("open", () => setConnection("Connected", "connected"));
  socket.addEventListener("message", (event) => receive(JSON.parse(event.data)));
  socket.addEventListener("close", () => {
    setConnection("Disconnected · retrying", "disconnected");
    setTimeout(connect, 1000);
  });
  socket.addEventListener("error", () => socket.close());
}

function svg(name, attributes = {}, text = "") {
  const element = document.createElementNS(NS, name);
  for (const [key, value] of Object.entries(attributes)) {
    element.setAttribute(key, String(value));
  }
  if (text) element.textContent = text;
  return element;
}

function selectable(element, record) {
  element.setAttribute("tabindex", "0");
  element.setAttribute("role", "button");
  element.setAttribute(
    "aria-label",
    displayKind(record) + " " + (record.label || "") + (record.discarded ? " discarded" : ""),
  );
  const select = () => {
    selectedRecord = record;
    selectedTime = recordSeconds(record);
    followLive = false;
    render();
  };
  element.addEventListener("click", select);
  element.addEventListener("keydown", (event) => {
    if (event.key === "Enter" || event.key === " ") {
      event.preventDefault();
      select();
    }
  });
}

function drawTrinary(root, record, x, y) {
  const group = svg("g", {
    class: "marker " + record.value + (record.discarded ? " discarded" : ""),
  });
  if (record.value === "true") {
    group.append(svg("circle", { cx: x, cy: y, r: 6 }));
  } else if (record.value === "false") {
    group.append(svg("line", { x1: x - 5, y1: y - 5, x2: x + 5, y2: y + 5 }));
    group.append(svg("line", { x1: x + 5, y1: y - 5, x2: x - 5, y2: y + 5 }));
  } else {
    group.append(svg("polygon", {
      points: x + "," + (y - 7) + " " + (x + 7) + "," + y + " " +
        x + "," + (y + 7) + " " + (x - 7) + "," + y,
    }));
  }
  if (record.discarded) {
    group.append(svg("circle", { class: "discarded-ring", cx: x, cy: y, r: 10 }));
  }
  selectable(group, record);
  root.append(group);
}

function renderDetails() {
  details.replaceChildren();
  detailsEmpty.hidden = Boolean(selectedRecord);
  if (!selectedRecord) return;
  for (const [key, value] of detailEntries(selectedRecord, useSimTime)) {
    const term = document.createElement("dt");
    term.textContent = key;
    const description = document.createElement("dd");
    description.textContent = String(value);
    details.append(term, description);
  }
}

function renderColumns(scenarios, lanes) {
  scenarioList.replaceChildren();
  for (const scenario of scenarios) {
    const item = document.createElement("li");
    const button = document.createElement("button");
    button.type = "button";
    button.className = "scenario-option";
    if (scenario.contextId === selectedContextId) button.setAttribute("aria-current", "true");

    const title = document.createElement("strong");
    title.textContent = scenario.label;
    const context = document.createElement("small");
    context.textContent = scenario.contextId.slice(0, 8);
    const summary = document.createElement("span");
    summary.className = "scenario-summary";
    const status = document.createElement("span");
    status.className = "status " + scenario.value;
    status.textContent = scenario.value;
    const duration = document.createElement("span");
    duration.textContent = formatDuration(scenario.endSeconds - scenario.startSeconds);
    summary.append(status, duration);
    button.append(title, context, summary);
    button.addEventListener("click", () => {
      selectedContextId = scenario.contextId;
      selectedRecord = null;
      selectedTime = scenario.endSeconds;
      followLive = true;
      render();
    });
    item.append(button);
    scenarioList.append(item);
  }

  laneLabels.replaceChildren();
  for (const lane of lanes) {
    const item = document.createElement("li");
    item.title = lane.label;
    const text = document.createElement("strong");
    text.textContent = lane.label;
    item.append(text);
    if (lane.type === "trinary") {
      const value = laneStateAt(lane, selectedTime);
      const status = document.createElement("span");
      status.className = "status " + value;
      status.textContent = value === "pending" ? "not evaluated" : value;
      item.append(status);
    }
    laneLabels.append(item);
  }
}

function render() {
  const all = [...records.values()];
  const scenarios = buildScenarios(all, latestStamp);
  selectedContextId = selectContextId(scenarios, selectedContextId);

  const scenario = scenarios.find((item) => item.contextId === selectedContextId);
  timeline.replaceChildren();
  empty.hidden = Boolean(scenario);
  if (!scenario) {
    renderColumns(scenarios, []);
    followButton.disabled = true;
    playheadInput.disabled = true;
    playheadTime.value = "—";
    renderDetails();
    return;
  }

  const minimum = scenario.startSeconds;
  const maximum = scenario.endSeconds;
  const visible = scenario.records.filter((record) => {
    const seconds = recordSeconds(record);
    return seconds >= minimum && seconds <= maximum;
  });
  const snapshotScenario = latestSnapshot?.scenarios?.find(
    (item) => item.context_id === scenario.contextId,
  );
  const definitions = snapshotScenario ? [
    { lane_type: "behaviour", label: snapshotScenario.behaviour.representation },
    ...snapshotScenario.fluents.map(
      (fluent) => ({ lane_type: "policy", label: fluent.representation }),
    ),
  ] : [];
  const lanes = buildLanes(visible, scenario.contextId, definitions);
  if (followLive) selectedTime = maximum;
  selectedTime = Math.min(maximum, Math.max(minimum, selectedTime));
  renderColumns(scenarios, lanes);

  const width = Math.max(timeline.clientWidth, 760);
  const left = 12;
  const right = 24;
  const rowHeight = 52;
  const top = 34;
  const height = Math.max(top + lanes.length * rowHeight + 20, timeline.parentElement.clientHeight);
  const plotWidth = width - left - right;
  const duration = maximum - minimum;
  const x = (time) => left + ((time - minimum) / Math.max(duration, Number.EPSILON)) * plotWidth;
  const root = svg("svg", {
    viewBox: "0 0 " + width + " " + height,
    width,
    height,
    "aria-label": "BDD execution timeline",
  });

  for (let tick = 0; tick <= 2; tick += 1) {
    const value = minimum + ((maximum - minimum) * tick) / 2;
    const position = x(value);
    root.append(svg("line", {
      class: "tick",
      x1: position,
      y1: top - 12,
      x2: position,
      y2: height,
    }));
    const anchor = tick === 0 ? "start" : tick === 2 ? "end" : "middle";
    root.append(svg("text", {
      class: "tick-label",
      x: position,
      y: 14,
      "text-anchor": anchor,
    }, formatDuration(value - minimum)));
  }

  for (const [index, lane] of lanes.entries()) {
    const y = top + index * rowHeight + rowHeight / 2;
    root.append(svg("line", {
      class: "lane-rule",
      x1: left,
      y1: y,
      x2: width - right,
      y2: y,
    }));

    if (lane.type === "scenario") {
      const start = lane.records.find((record) => record.kind === "scenario_start");
      const end = lane.records.find((record) => record.kind === "scenario_end");
      if (start) {
        const element = svg("rect", {
          class: "scenario " + (end?.value || "running"),
          x: x(recordSeconds(start)),
          y: y - 8,
          width: Math.max(3, x(end ? recordSeconds(end) : maximum) - x(recordSeconds(start))),
          height: 16,
          rx: 4,
        });
        selectable(element, end || start);
        root.append(element);
      }
    } else if (lane.type === "event") {
      for (const record of lane.records) {
        const element = svg("circle", {
          class: "event",
          cx: x(recordSeconds(record)),
          cy: y,
          r: 6,
        });
        selectable(element, record);
        root.append(element);
      }
    } else {
      for (const record of lane.records.filter(isTimelineTrinary)) {
        drawTrinary(root, record, x(recordSeconds(record)), y);
      }
    }
  }

  root.append(svg("line", {
    class: "playhead",
    x1: x(Math.min(maximum, Math.max(minimum, selectedTime))),
    y1: top - 12,
    x2: x(Math.min(maximum, Math.max(minimum, selectedTime))),
    y2: height,
  }));

  root.addEventListener("click", (event) => {
    if (event.target !== root) return;
    const bounds = root.getBoundingClientRect();
    const cursor = ((event.clientX - bounds.left) / bounds.width) * width;
    selectedTime = minimum + ((cursor - left) / plotWidth) * (maximum - minimum);
    selectedTime = Math.min(maximum, Math.max(minimum, selectedTime));
    followLive = false;
    render();
  });

  timeline.append(root);
  playheadInput.min = "0";
  playheadInput.max = String(duration);
  playheadInput.value = String(selectedTime - minimum);
  playheadInput.dataset.start = String(minimum);
  playheadInput.disabled = duration === 0;
  playheadTime.value =
    formatDuration(selectedTime - minimum) + " · " +
    (useSimTime ? "Sim time " : "") + formatSeconds(selectedTime);
  followButton.textContent = followLive
    ? (scenario.finished ? "At end" : "Pause")
    : "Go to end";
  followButton.disabled = scenario.finished && followLive;
  renderDetails();
}

followButton.addEventListener("click", () => {
  followLive = !followLive;
  render();
});

playheadInput.addEventListener("input", () => {
  followLive = false;
  selectedTime = Number(playheadInput.dataset.start) + Number(playheadInput.value);
  render();
});

document.querySelector("#clear").addEventListener("click", () => {
  records.clear();
  selectedContextId = null;
  selectedRecord = null;
  latestSnapshot = null;
  latestStamp = null;
  followLive = true;
  render();
});

window.addEventListener("resize", render);
connect();
render();
