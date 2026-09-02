export function recordSeconds(record) {
  return Number(record.stamp.sec) + Number(record.stamp.nanosec) / 1e9;
}

export function materializeRecords(records) {
  const discarded = new Set(
    records
      .filter((record) => record.kind === "trinary_discarded")
      .map((record) => record.target_id),
  );
  return records
    .filter((record) => record.kind !== "trinary_discarded")
    .map((record) => ({
      ...record,
      discarded: Boolean(record.discarded || discarded.has(record.id)),
    }))
    .sort((left, right) => left.sequence - right.sequence);
}

export function timeExtent(records) {
  if (!records.length) return [0, 1];
  const values = records.map(recordSeconds);
  const minimum = Math.min(...values);
  const maximum = Math.max(...values);
  return minimum === maximum ? [minimum, minimum + 1] : [minimum, maximum];
}

export function buildScenarios(input, currentStamp = null) {
  const scenarios = new Map();
  for (const record of materializeRecords(input)) {
    if (!record.context_id) continue;
    if (!scenarios.has(record.context_id)) {
      scenarios.set(record.context_id, {
        contextId: record.context_id,
        label: record.context_id,
        sequence: record.sequence,
        records: [],
      });
    }
    const scenario = scenarios.get(record.context_id);
    scenario.records.push(record);
    if (record.kind.startsWith("scenario_") && record.label) scenario.label = record.label;
  }

  const current = currentStamp ? recordSeconds({ stamp: currentStamp }) : null;
  return [...scenarios.values()]
    .sort((left, right) => left.sequence - right.sequence)
    .map((scenario) => {
      const start = scenario.records.find((record) => record.kind === "scenario_start");
      const end = scenario.records.findLast((record) => record.kind === "scenario_end");
      const first = recordSeconds(scenario.records[0]);
      const last = recordSeconds(scenario.records.at(-1));
      const startSeconds = start ? recordSeconds(start) : first;
      const endSeconds = end
        ? recordSeconds(end)
        : Math.max(startSeconds, last, current ?? last);
      return {
        ...scenario,
        startSeconds,
        endSeconds,
        finished: Boolean(end),
        value: end?.value || "running",
      };
    });
}

export function selectContextId(scenarios, currentContextId) {
  return scenarios.some((scenario) => scenario.contextId === currentContextId)
    ? currentContextId : scenarios[0]?.contextId || null;
}

export function clauseStateAt(lane, seconds) {
  return lane.records.findLast(
    (record) => record.role === "result" && recordSeconds(record) <= seconds,
  ) || null;
}

export function laneStateAt(lane, seconds) {
  return clauseStateAt(lane, seconds)?.value ||
    (lane.laneType === "behaviour" ? "running" : "pending");
}

export function isTimelineTrinary(record) {
  return record.kind === "trinary" &&
    (record.lane_type === "behaviour" || record.role === "assertion");
}

export function displayKind(record) {
  if (record.kind === "trinary") {
    return record.lane_type === "behaviour" ? "Behaviour trinary" : "Fluent trinary";
  }
  if (record.kind.startsWith("scenario_")) return "Scenario";
  if (record.kind === "event") return "Event";
  return record.kind;
}

export function detailEntries(record, useSimTime = false) {
  return [
    ["Kind", displayKind(record)],
    [useSimTime ? "Sim time" : "Time", formatStamp(record.stamp, useSimTime)],
    ["Label", record.label],
    ["URI", record.uri],
    ["Value", record.value],
    ["Reason", record.reason],
    ["Discarded", record.discarded],
  ].filter(([, value]) => value !== undefined && value !== "" && value !== false);
}

export function buildLanes(input, contextId, definitions = []) {
  const records = materializeRecords(input);
  const events = {
    id: "events",
    label: "Events",
    type: "event",
    records: records.filter(
      (record) => record.context_id === contextId && record.kind === "event",
    ),
  };
  const scenario = {
    id: "scenario:" + contextId,
    label: contextId,
    type: "scenario",
    records: [],
  };
  const children = new Map();

  function clause(laneType, label, uri = "") {
    const key = laneType + ":" + (uri || label);
    if (!children.has(key)) {
      children.set(key, {
        id: contextId + ":" + key,
        label,
        laneType,
        type: "trinary",
        records: [],
      });
    }
    return children.get(key);
  }

  for (const definition of definitions) {
    clause(definition.lane_type, definition.label, definition.uri);
  }
  for (const record of records) {
    if (record.context_id !== contextId || record.kind === "event") continue;
    if (record.kind === "scenario_start" || record.kind === "scenario_end") {
      scenario.records.push(record);
      if (record.label) scenario.label = record.label;
      continue;
    }
    if (record.kind !== "trinary") continue;
    const child = clause(record.lane_type, record.label, record.uri);
    child.records.push(record);
    if (record.role === "assertion" && record.reason) child.message = record.reason;
  }
  return [scenario, events, ...children.values()];
}

export function formatStamp(stamp, useSimTime = false) {
  const fraction = String(stamp.nanosec).padStart(9, "0").slice(0, 3);
  if (useSimTime) {
    const hours = Math.floor(stamp.sec / 3600);
    const minutes = Math.floor((stamp.sec % 3600) / 60);
    const seconds = stamp.sec % 60;
    return [hours, minutes, seconds].map(
      (part) => String(part).padStart(2, "0"),
    ).join(":") + "." + fraction;
  }
  return new Date(stamp.sec * 1000 + Number(fraction))
    .toISOString().replace("T", " ").replace("Z", " UTC");
}
