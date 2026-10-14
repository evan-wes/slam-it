// The frame slider's timeline: the golden bundle's part of its sequence, and runs of states and
// maps.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBundle } from "../src/bundle.js";
import { STATE_WORDS, notRunText, placement, spanHint, stateSpans } from "../src/timeline.js";
import { fetchGolden } from "./golden.js";

/**
 * A track as the timeline reads it.
 * @param {number} first Its first frame.
 * @param {string[]} states Each of its frames' state.
 * @returns {{info: {frames: [number, number]}, records: {frame: number[], state: string[]}}}
 */
function track(first, states) {
  return {
    info: { frames: [first, first + states.length] },
    records: { frame: states.map((_, k) => first + k), state: states },
  };
}

test("a backend's frames are grouped into runs of one state", () => {
  const states = ["not_initialized", "not_initialized", "tracking", "tracking", "lost"];
  const none = { map: null, reset: false };
  assert.deepEqual(stateSpans(track(3, states).records), [
    { state: "not_initialized", ...none, first: 3, end: 5 },
    { state: "tracking", ...none, first: 5, end: 7 },
    { state: "lost", ...none, first: 7, end: 8 },
  ]);
  assert.deepEqual(stateSpans({ frame: [] }), []);
  // A gap in the frames (an RGB-D frame without depth, skipped) starts a new run.
  const gapped = { frame: [0, 1, 3], state: ["tracking", "tracking", "tracking"] };
  assert.deepEqual(stateSpans(gapped), [
    { state: "tracking", ...none, first: 0, end: 2 },
    { state: "tracking", ...none, first: 3, end: 4 },
  ]);
});

test("a new map starts a new run of frames, and a reset map's frames say so", () => {
  const records = {
    frame: [0, 1, 2, 3, 4, 5, 6, 7],
    state: [
      "tracking",
      "tracking",
      "lost",
      "not_initialized",
      "tracking",
      "lost",
      "tracking",
      "tracking",
    ],
    map: [0, 0, 0, null, 1, 1, 2, 2],
  };
  const maps = [
    { id: 0, reset: false },
    { id: 1, reset: true },
    { id: 2, reset: false },
  ];
  assert.deepEqual(stateSpans(records, maps), [
    { state: "tracking", map: 0, reset: false, first: 0, end: 2 },
    { state: "lost", map: 0, reset: false, first: 2, end: 3 },
    { state: "not_initialized", map: null, reset: false, first: 3, end: 4 },
    { state: "tracking", map: 1, reset: true, first: 4, end: 5 },
    { state: "lost", map: 1, reset: true, first: 5, end: 6 },
    { state: "tracking", map: 2, reset: false, first: 6, end: 8 },
  ]);
  // Two maps back to back, without a frame between, are two runs.
  const touching = { frame: [0, 1], state: ["tracking", "tracking"], map: [0, 1] };
  assert.equal(stateSpans(touching).length, 2);
});

test("each run of states is hinted by its words, map and frames", () => {
  const none = { map: null, reset: false };
  assert.equal(
    spanHint("cpp", { state: "tracking", map: 1, reset: false, first: 61, end: 119 }),
    "cpp: tracking in map 1, frames 61–118",
  );
  assert.equal(
    spanHint("cpp", { state: "lost", map: 1, reset: false, first: 119, end: 120 }),
    "cpp: lost the track of map 1, frame 119",
  );
  assert.equal(
    spanHint("cpp", { state: "tracking", map: 2, reset: true, first: 120, end: 125 }),
    "cpp: tracking in map 2, reset, frames 120–124",
  );
  assert.equal(
    spanHint("rust", { state: "not_initialized", ...none, first: 0, end: 61 }),
    "rust: no map yet, frames 0–60",
  );
  assert.deepEqual(Object.keys(STATE_WORDS), ["not_initialized", "tracking", "lost"]);
});

test("the golden runs stop short of the room's end by count: the rest is not run", async () => {
  const bundle = await loadBundle(fetchGolden);
  assert.deepEqual(bundle.info.extent, { frames: 40, start: 0, end: 1.3 });
  const placed = placement(bundle.info.extent, bundle.times);
  assert.equal(placed.before, 0);
  assert.equal(placed.ran, bundle.times[5]);
  assert.ok(Math.abs(placed.after - (1.3 - 5 / 30)) < 1e-6);
  assert.deepEqual(notRunText(placed).after, ["not run", "0.2 s to 1.3 s: no run went further"]);
});
test("the part before the bundle's frames, and parts under half a frame, are placed", () => {
  const times = [0, 0.1, 0.2];
  const placed = placement({ frames: 30, start: -0.5, end: 0.2 + 1e-7 }, times);
  assert.deepEqual(placed, { before: 0.5, ran: 0.2, after: 0 });
  assert.deepEqual(notRunText(placed).before, [
    "not run",
    "the sequence's first 0.5 s: no run started there",
  ]);
  // A one-frame sequence: no part before or after, whatever its rounding.
  assert.deepEqual(placement({ frames: 1, start: 0, end: 0 }, [0]), {
    before: 0,
    ran: 0,
    after: 0,
  });
});
