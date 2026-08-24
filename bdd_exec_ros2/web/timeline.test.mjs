import assert from "node:assert/strict";
import test from "node:test";

import {
  buildLanes,
  buildScenarios,
  clauseStateAt,
  formatStamp,
  materializeRecords,
  timeExtent,
} from "./timeline.mjs";

const stamp = (sec) => ({ sec, nanosec: 0 });

test("builds compact lanes for only the selected scenario", () => {
  const records = [
    { id: "r1", sequence: 1, kind: "scenario_start", context_id: "ctx", stamp: stamp(10), label: "Pick" },
    { id: "r2", sequence: 2, kind: "event", context_id: "ctx", stamp: stamp(11), label: "grasp" },
    { id: "r3", sequence: 3, kind: "trinary", context_id: "ctx", stamp: stamp(12), lane_type: "policy", label: "held", role: "assertion", value: "true", reason: "force norm is less than 5 N" },
    { id: "r4", sequence: 4, kind: "trinary", context_id: "ctx", stamp: stamp(12), lane_type: "policy", label: "held", role: "result", value: "true", reason: "all assertions are true" },
    { id: "r5", sequence: 5, kind: "event", context_id: "other", stamp: stamp(13), label: "ignored" },
  ];

  const lanes = buildLanes(records, "ctx", [
    { lane_type: "behaviour", label: "Move arm" },
    { lane_type: "policy", label: "held" },
  ]);
  assert.deepEqual(
    lanes.map((lane) => [lane.type, lane.label]),
    [
      ["scenario", "Pick"],
      ["event", "Events"],
      ["trinary", "Move arm"],
      ["trinary", "held"],
    ],
  );
  assert.equal(lanes[0].label, "Pick");
  assert.equal(lanes[1].records.length, 1);
  assert.equal(clauseStateAt(lanes[2], 12), null);
  assert.equal(lanes[3].message, "force norm is less than 5 N");
  assert.deepEqual(timeExtent(materializeRecords(records.slice(0, 4))), [10, 12]);
});

test("keeps scenario clocks independent and freezes completed scenarios", () => {
  const records = [
    { id: "a1", sequence: 1, kind: "scenario_start", context_id: "a", stamp: stamp(10), label: "A" },
    { id: "a2", sequence: 2, kind: "scenario_end", context_id: "a", stamp: stamp(12), label: "A", value: "true" },
    { id: "b1", sequence: 3, kind: "scenario_start", context_id: "b", stamp: stamp(20), label: "B" },
    { id: "z1", sequence: 4, kind: "scenario_end", context_id: "zero", stamp: stamp(25), label: "Zero", value: "unknown" },
  ];

  const scenarios = buildScenarios(records, stamp(30));
  assert.deepEqual(scenarios.map((scenario) => scenario.contextId), ["a", "b", "zero"]);
  assert.deepEqual(
    scenarios.map(({ startSeconds, endSeconds, finished }) => ({
      startSeconds,
      endSeconds,
      finished,
    })),
    [
      { startSeconds: 10, endSeconds: 12, finished: true },
      { startSeconds: 20, endSeconds: 30, finished: false },
      { startSeconds: 25, endSeconds: 25, finished: true },
    ],
  );
});

test("derives clause result at the playhead using sequence order", () => {
  const lane = {
    records: [
      { sequence: 1, role: "result", value: "unknown", stamp: stamp(11) },
      { sequence: 2, role: "assertion", value: "false", stamp: stamp(11) },
      { sequence: 3, role: "result", value: "true", stamp: stamp(11) },
    ],
  };

  assert.equal(clauseStateAt(lane, 10), null);
  assert.equal(clauseStateAt(lane, 11).value, "true");
});

test("formats wall and simulation timestamps like the desktop visualizer", () => {
  assert.equal(formatStamp({ sec: 0, nanosec: 123456789 }), "1970-01-01 00:00:00.123 UTC");
  assert.equal(formatStamp({ sec: 3661, nanosec: 123456789 }, true), "01:01:01.123");
});

test("marks discarded trinaries without removing their history", () => {
  const records = [
    { id: "r1", sequence: 1, kind: "trinary", context_id: "ctx", stamp: stamp(2), lane_type: "policy", label: "held", value: "unknown" },
    { id: "r2", sequence: 2, kind: "trinary_discarded", context_id: "ctx", stamp: stamp(3), target_id: "r1" },
  ];

  const materialized = materializeRecords(records);
  assert.equal(materialized.length, 1);
  assert.equal(materialized[0].discarded, true);
  assert.equal(materializeRecords(materialized)[0].discarded, true);
});
