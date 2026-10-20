// The loops a run closed and the "seen before" inset's earlier frames: on the C++ runs of the
// rooms' golden records that close loops (tests/fixtures/edge/room_runs_cpp.json), held to what
// those records say of themselves, and on runs made up here.

import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { test } from "node:test";

import {
  EFFECT_FADE,
  EFFECT_HOLD,
  INSET_AFTER,
  INSET_BEFORE,
  cycleRuns,
  effectChange,
  effectFrame,
  effectOpacity,
  effectOver,
  insetAt,
  insets,
  loopBeside,
  loopHint,
  loopsOf,
  seenText,
} from "../src/loops.js";

const ROOMS = new URL("../../tests/fixtures/edge/room_runs_cpp.json", import.meta.url);

/**
 * A table held as columns, as rows.
 * @param {number[][]} columns The columns, of equal length.
 * @returns {Int32Array} The rows, one after another.
 */
function rows(columns) {
  return Int32Array.from(columns[0].flatMap((_, i) => columns.map((column) => column[i])));
}

/**
 * A golden record's case, as the fixture holds it: columns a frame, a keyframe or a loop.
 * @typedef {{summaries: {loop?: {correction?: {matched: number, current: number} | null} | null}[],
 *     maps: (number | null)[], keyframe_maps: number[], keyframe_lives: number[][],
 *     keyframe_loops: number[][], poses: number[][], pose_maps: number[], timestamps: number[]}}
 *     Case
 */

/**
 * The rooms' cases that close a loop, each as a bundle gives its run (`Backend`'s records and
 * keyframes; its poses and times where every frame is tracked, a pose a frame).
 */
async function rooms() {
  const doc = JSON.parse(await readFile(ROOMS, "utf8"));
  /** @type {[string, Case][]} */
  const cases = Object.entries(doc.cases);
  const closing = cases.filter(([, c]) => c.keyframe_loops[0].length > 0);
  assert.ok(closing.length > 0, "the golden records close loops");
  return closing.map(([name, c]) => {
    const frames = c.summaries.length;
    const every = Array.from({ length: frames }, (_, i) => i);
    return {
      name,
      c,
      run: {
        records: { frame: every, map: c.maps, loop: c.summaries.map((s) => s.loop ?? null) },
        keyframeMaps: Int32Array.from(c.keyframe_maps),
        keyframeLives: rows(c.keyframe_lives),
        keyframeLoops: rows(c.keyframe_loops),
      },
      tracked:
        c.poses.length === frames
          ? {
              poses: Float32Array.from(c.poses.flatMap((p) => p.slice(0, 12))),
              poseFrames: Int32Array.from(every),
              poseMaps: Int32Array.from(c.pose_maps),
              times: Float32Array.from(c.timestamps, (t) => t - c.timestamps[0]),
            }
          : null,
    };
  });
}

/**
 * The loops a case's record says its run closed, from its own columns: each frame whose summary
 * holds a correction, the record's keyframes counted from its map's first, which must be the
 * `keyframe_loops` row of the same order.
 * @param {Case} c The case.
 * @returns {import("../src/loops.js").Loop[]} The loops.
 */
function recordedLoops(c) {
  const corrections = c.summaries.flatMap((summary, frame) => {
    const correction = summary.loop?.correction ?? null;
    return correction === null ? [] : [{ frame, correction }];
  });
  assert.equal(corrections.length, c.keyframe_loops[0].length, "a correction for each loop");
  return corrections.map(({ frame, correction }, e) => {
    const map = /** @type {number} */ (c.maps[frame]);
    const first = c.keyframe_maps.indexOf(map);
    const [older, newer] = [first + correction.matched, first + correction.current];
    assert.deepEqual(
      [Math.min(older, newer), Math.max(older, newer)],
      [c.keyframe_loops[0][e], c.keyframe_loops[1][e]],
      `loop ${e}: the record's keyframes are keyframe_loops.npy's`,
    );
    return {
      frame,
      map,
      older,
      newer,
      matched: correction.matched,
      current: correction.current,
      olderMade: c.keyframe_lives[0][older],
      newerMade: c.keyframe_lives[0][newer],
    };
  });
}

test("a loop is the frame whose record corrects it, its keyframes the loop the map holds", async () => {
  for (const { name, c, run, tracked } of await rooms()) {
    const loops = loopsOf(run);
    assert.deepEqual(loops, recordedLoops(c), name);
    if (tracked !== null) {
      const { times } = tracked;
      for (const loop of loops) {
        const apart = (times[loop.newerMade] - times[loop.olderMade]).toFixed(1);
        assert.equal(
          loopHint(loop, times),
          `loop closed in map ${loop.map}: keyframes ${loop.matched} ↔ ${loop.current}, ` +
            `${apart} s apart`,
        );
      }
    }
  }
});

test("a run without a loop column closed none", () => {
  const run = {
    records: { frame: [0, 1, 2], map: [0, 0, 0] },
    keyframeMaps: Int32Array.from([0, 0]),
    keyframeLives: Int32Array.from([0, -1, -1, 2, -1, -1]),
    keyframeLoops: new Int32Array(0),
  };
  assert.deepEqual(loopsOf(run), []);
});

/**
 * A made-up run of two maps, each record's keyframes counted in its map, as the pipeline counts
 * them: map 0 has keyframes 0 to 2, map 1 keyframes 3 to 7, made at frames 0, 2, 4 and 10 to 18.
 * Map 1 closes a loop at frame 20, between its keyframes 1 and 4 (the run's 4 and 7).
 * @param {number[]} loops The run's loops, two ids a loop.
 */
function twoMaps(loops) {
  const made = [0, 2, 4, 10, 12, 14, 16, 18];
  const correction = { matched: 1, current: 4 };
  return {
    records: {
      frame: [19, 20, 21],
      map: [1, 1, 1],
      loop: [null, { detection: null, correction }, { detection: null, correction: null }],
    },
    keyframeMaps: Int32Array.from([0, 0, 0, 1, 1, 1, 1, 1]),
    keyframeLives: Int32Array.from(made.flatMap((frame) => [frame, -1, -1])),
    keyframeLoops: Int32Array.from(loops),
  };
}

test("a record's keyframes count from its map's first: the run's ids are found from it", () => {
  assert.deepEqual(loopsOf(twoMaps([4, 7])), [
    {
      frame: 20,
      map: 1,
      older: 4,
      newer: 7,
      matched: 1,
      current: 4,
      olderMade: 12,
      newerMade: 18,
    },
  ]);
  // Keyframes that are no loop the run holds: the bundle does not say what it shows.
  assert.throws(() => loopsOf(twoMaps([3, 7])), /frame 20: .* not a loop the run closed/);
  assert.throws(() => loopsOf(twoMaps([])), /not a loop the run closed/);
});

test("the loop beside a frame is the nearest before it or after it, if there is one", () => {
  const loops = [100, 250, 400].map((frame) => ({ ...loopsOf(twoMaps([4, 7]))[0], frame }));
  assert.equal(loopBeside(loops, 250, -1)?.frame, 100);
  assert.equal(loopBeside(loops, 250, 1)?.frame, 400);
  assert.equal(loopBeside(loops, 251, -1)?.frame, 250);
  assert.equal(loopBeside(loops, 100, -1), null);
  assert.equal(loopBeside(loops, 400, 1), null);
  assert.equal(loopBeside([], 5, 1), null);
});

test("the window's frames each have the earlier frame at their place, never stepping back", () => {
  // One frame a second along x: the first pass at x = 0 to 9 (frames 0 to 9), elsewhere until
  // frame 19, then the same street again from frame 20, which closes the loop at frame 23 with
  // the keyframe made at frame 2. Frame 24 has no pose; frame 26 stands where frame 1 did.
  const xs = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59];
  xs.push(0, 1, 2, 3, NaN, 5, 1, 7, 8, 9);
  const tracked = xs.flatMap((x, frame) => (Number.isNaN(x) ? [] : [frame]));
  const track = {
    poses: Float32Array.from(tracked.flatMap((f) => [1, 0, 0, xs[f], 0, 1, 0, 0, 0, 0, 1, 0])),
    poseFrames: Int32Array.from(tracked),
    poseMaps: new Int32Array(tracked.length),
  };
  const times = Float32Array.from(xs, (_, frame) => frame);
  const loop = { ...loopsOf(twoMaps([4, 7]))[0], frame: 23, map: 0, olderMade: 2 };
  const [window] = insets([loop], times, track);
  assert.equal(window.first, 23 - INSET_BEFORE);
  assert.equal(window.first + window.matches.length - 1, Math.min(23 + INSET_AFTER, 29));
  assert.deepEqual(Array.from(window.matches), [0, 1, 2, 3, 3, 5, 5, 7, 8]);
  assert.deepEqual(insetAt([window], 24), { loop, earlier: 3 });
  assert.equal(insetAt([window], 19), null);
  assert.equal(seenText({ loop, earlier: 8 }, 28, times), "seen 20 s earlier · keyframe 1");
});

test("a frame before the window's first earlier frame has none, and the later window wins", () => {
  const loop = loopsOf(twoMaps([4, 7]))[0];
  const first = { loop, first: 10, matches: Int32Array.from([-1, 3, 4]) };
  const second = { loop: { ...loop, frame: 14 }, first: 11, matches: Int32Array.from([6, 7]) };
  assert.equal(insetAt([first, second], 10), null);
  assert.deepEqual(insetAt([first, second], 11), { loop: second.loop, earlier: 6 });
  assert.deepEqual(insetAt([first, second], 13), null);
  assert.deepEqual(insetAt([first], 12), { loop, earlier: 4 });
});

test("a room's loop shows its earlier pass, nearer its place than the window's start", async () => {
  for (const { name, run, tracked } of await rooms()) {
    if (tracked === null) {
      continue; // its poses and times are not a frame each
    }
    const windows = insets(loopsOf(run), tracked.times, tracked);
    const centre = (/** @type {number} */ f) => [3, 7, 11].map((i) => tracked.poses[f * 12 + i]);
    const apart = (/** @type {number} */ a, /** @type {number} */ b) =>
      Math.hypot(...centre(a).map((v, i) => v - centre(b)[i]));
    for (const { loop, first, matches } of windows) {
      const earlier = Array.from(matches);
      assert.ok(
        earlier.every((frame, i) => frame >= 0 && (i === 0 || frame >= earlier[i - 1])),
        `${name}: every frame has an earlier frame, never stepping back`,
      );
      // The earlier pass's frames: before the cutoff, midway between the matched keyframe's
      // making and the window's first frame.
      const cutoff = (tracked.times[loop.olderMade] + tracked.times[first]) / 2;
      assert.ok(
        earlier.every((frame) => tracked.times[frame] < cutoff),
        name,
      );
      // At the loop's frame, the earlier pass stands nearer it than the window's first frame.
      const at = earlier[loop.frame - first];
      assert.ok(apart(loop.frame, at) < apart(loop.frame, first), name);
    }
  }
});

test("no loops, or a loop whose map has no tracked poses, have no earlier frames", () => {
  const times = Float32Array.from({ length: 30 }, (_, frame) => frame);
  const track = {
    poses: Float32Array.from({ length: 30 * 12 }, (_, i) =>
      i % 12 === 3 ? Math.floor(i / 12) : 0,
    ),
    poseFrames: Int32Array.from({ length: 30 }, (_, frame) => frame),
    poseMaps: new Int32Array(30),
  };
  assert.deepEqual(insets([], times, track), []);
  const loop = { ...loopsOf(twoMaps([4, 7]))[0], frame: 23, olderMade: 2 }; // map 1: no poses
  const [window] = insets([loop], times, track);
  assert.ok(window.matches.length > 0);
  assert.ok(window.matches.every((frame) => frame === -1));
  assert.equal(insetAt([window], 23), null);
});

test("a loop's cycle is its map's trajectory from its matched keyframe's making to its frame", () => {
  // Tracked frames 0 to 9 in map 0, then 12 to 19 in map 1 (10 and 11 lost).
  const poseFrames = Int32Array.from([
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19,
  ]);
  const poseMaps = Int32Array.from(poseFrames, (frame) => (frame < 10 ? 0 : 1));
  const loop = { ...loopsOf(twoMaps([4, 7]))[0], frame: 17, map: 1, olderMade: 13 };
  assert.deepEqual(cycleRuns(loop, poseFrames, poseMaps), [[11, 16]]); // frames 13 to 17
  // Made before the map began: its cycle starts with the map.
  assert.deepEqual(cycleRuns({ ...loop, olderMade: 8 }, poseFrames, poseMaps), [[10, 16]]);
  // In the map before: its cycle stops where that map does.
  assert.deepEqual(cycleRuns({ ...loop, map: 0, olderMade: 3 }, poseFrames, poseMaps), [[3, 10]]);
  assert.deepEqual(cycleRuns({ ...loop, map: 2 }, poseFrames, poseMaps), []);
  // Map 0 taken up again from frame 16: its loop's cycle leaves map 1's frames out.
  const revisited = Int32Array.from(poseFrames, (frame) => (frame < 10 || frame >= 16 ? 0 : 1));
  const again = { ...loop, map: 0, olderMade: 3, frame: 18 };
  assert.deepEqual(cycleRuns(again, poseFrames, revisited), [
    [3, 10],
    [14, 17],
  ]);
});

test("the loop effect holds, then fades, on its own clock", () => {
  assert.equal(effectOpacity(0), 1);
  assert.equal(effectOpacity(-1), 1);
  assert.equal(effectOpacity(EFFECT_HOLD), 1);
  assert.equal(effectOpacity(EFFECT_HOLD + EFFECT_FADE / 2), 0.5);
  assert.equal(effectOpacity(EFFECT_HOLD + EFFECT_FADE), 0);
  assert.equal(effectOpacity(10), 0);
  assert.equal(effectOver(EFFECT_HOLD + EFFECT_FADE - 0.01), false);
  assert.equal(effectOver(EFFECT_HOLD + EFFECT_FADE), true);
});

test("during the effect the inset plays the earlier pass from the matched keyframe on", () => {
  const times = Float32Array.from({ length: 40 }, (_, frame) => frame / 4); // 4 frames a second
  const loop = { ...loopsOf(twoMaps([4, 7]))[0], frame: 30, olderMade: 4 };
  assert.equal(effectFrame(loop, times, 0), 4);
  assert.equal(effectFrame(loop, times, 0.3), 5);
  assert.equal(effectFrame(loop, times, 1), 8);
  assert.equal(effectFrame(loop, times, 2.5), 14);
  // Never into the loop: a short one holds the frame before it.
  assert.equal(effectFrame({ ...loop, frame: 10 }, times, 2.5), 9);
});

test("arriving at a loop starts its effect; a seek elsewhere ends it, playing on does not", () => {
  const loops = [100, 250, 260].map((frame) => ({ ...loopsOf(twoMaps([4, 7]))[0], frame }));
  // Playing onto a loop, and across two (frames skipped): the last passed.
  assert.equal(effectChange(null, loops, 99, 100, true), loops[0]);
  assert.equal(effectChange(null, loops, 240, 265, true), loops[2]);
  assert.equal(effectChange(null, loops, 100, 101, true), null);
  // A seek onto a loop (the slider, a button, a step back to it), however often.
  assert.equal(effectChange(null, loops, 10, 250, false), loops[1]);
  assert.equal(effectChange(250, loops, 251, 250, false), loops[1]);
  // A seek across a loop is no arrival; showing the same frame again is nothing.
  assert.equal(effectChange(null, loops, 10, 120, false), null);
  assert.equal(effectChange(100, loops, 100, 100, false), null);
  // While an effect runs: a seek elsewhere ends it; playing on, or a seek with none, does not.
  assert.equal(effectChange(100, loops, 100, 140, false), "end");
  assert.equal(effectChange(100, loops, 100, 101, true), null);
  assert.equal(effectChange(null, loops, 100, 140, false), null);
  // A new loop over a running effect replaces it.
  assert.equal(effectChange(250, loops, 255, 260, true), loops[2]);
});
