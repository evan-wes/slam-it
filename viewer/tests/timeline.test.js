// The frame slider's timeline: the golden bundle's part of its sequence, and runs of states.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBundle, loadTrack } from "../src/bundle.js";
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
  assert.deepEqual(stateSpans(track(3, states).records), [
    { state: "not_initialized", first: 3, end: 5 },
    { state: "tracking", first: 5, end: 7 },
    { state: "lost", first: 7, end: 8 },
  ]);
  assert.deepEqual(stateSpans({ frame: [] }), []);
  // A gap in the frames (an RGB-D frame without depth, skipped) starts a new run.
  const gapped = { frame: [0, 1, 3], state: ["tracking", "tracking", "tracking"] };
  assert.deepEqual(stateSpans(gapped), [
    { state: "tracking", first: 0, end: 2 },
    { state: "tracking", first: 3, end: 4 },
  ]);
});

test("each run of states is hinted by its words and frames", () => {
  assert.equal(
    spanHint("cpp", { state: "tracking", first: 61, end: 119 }),
    "cpp: tracking, frames 61–118",
  );
  assert.equal(
    spanHint("cpp", { state: "lost", first: 119, end: 120 }),
    "cpp: lost the track, frame 119",
  );
  assert.equal(
    spanHint("rust", { state: "not_initialized", first: 0, end: 61 }),
    "rust: no map yet, frames 0–60",
  );
  assert.deepEqual(Object.keys(STATE_WORDS), ["not_initialized", "tracking", "lost"]);
});

test("the golden runs stop short of the room's end by count: the rest is not run", async () => {
  const bundle = await loadBundle(fetchGolden);
  assert.deepEqual(bundle.info.extent, { frames: 40, start: 0, end: 1.3 });
  const names = Object.keys(bundle.info.backends);
  const tracks = await Promise.all(names.map((name) => loadTrack(fetchGolden, bundle, name)));
  const placed = placement(bundle.info.extent, bundle.times, tracks);
  assert.equal(placed.before, 0);
  assert.equal(placed.ran, bundle.times[5]);
  assert.ok(Math.abs(placed.after - (1.3 - 5 / 30)) < 1e-6);
  assert.equal(placed.lost, false);
  assert.deepEqual(notRunText(placed).after, ["not run", "0.2 s to 1.3 s: no run went further"]);
});

test("a run that lost the track at its last frame says the rest was not processed", () => {
  const times = Array.from({ length: 150 }, (_, k) => k / 30);
  const states = [...Array(75).fill("not_initialized"), ...Array(74).fill("tracking"), "lost"];
  const placed = placement({ frames: 360, start: 0, end: 359 / 30 }, times, [track(0, states)]);
  assert.equal(placed.lost, true);
  assert.ok(Math.abs(placed.after - 210 / 30) < 1e-9);
  assert.deepEqual(notRunText(placed).after, [
    "not processed: tracking was lost",
    "tracking was lost at 5.0 s of 12.0 s, which ends a run, so the rest was not processed",
  ]);
  // Another backend that reached the same last frame still tracking: not every run was lost.
  const tracking = track(0, Array(150).fill("tracking"));
  assert.equal(
    placement({ frames: 360, start: 0, end: 12 }, times, [track(0, states), tracking]).lost,
    false,
  );
  // A run that lost the track earlier, and so did not reach the last frame, is not asked.
  const earlier = track(0, [...Array(9).fill("tracking"), "lost"]);
  assert.equal(
    placement({ frames: 360, start: 0, end: 12 }, times, [earlier, tracking]).lost,
    false,
  );
});

test("the part before the bundle's frames, and parts under half a frame, are placed", () => {
  const times = [0, 0.1, 0.2];
  const placed = placement({ frames: 30, start: -0.5, end: 0.2 + 1e-7 }, times, [
    track(0, ["tracking", "tracking", "tracking"]),
  ]);
  assert.deepEqual(placed, { before: 0.5, ran: 0.2, after: 0, lost: false });
  assert.deepEqual(notRunText(placed).before, [
    "not run",
    "the sequence's first 0.5 s: no run started there",
  ]);
  // A one-frame sequence: no part before or after, whatever its rounding.
  assert.deepEqual(placement({ frames: 1, start: 0, end: 0 }, [0], [track(0, ["tracking"])]), {
    before: 0,
    ran: 0,
    after: 0,
    lost: false,
  });
});
