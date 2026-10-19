// The loops a run closed and the "seen before" inset's earlier frames: on the loop rooms' runs
// (tests/fixtures/edge/room_runs_cpp.json, each closing one loop) and on runs made up here.

import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { test } from "node:test";

import {
  INSET_AFTER,
  INSET_BEFORE,
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
 * One of the loop rooms' C++ runs, as a bundle gives a run: every frame tracked in it.
 * @param {string} name The case: "loop/tum/rgbd" or "loop/euroc/stereo".
 */
async function room(name) {
  const doc = JSON.parse(await readFile(ROOMS, "utf8"));
  const c = doc.cases[name];
  const frames = c.summaries.length;
  assert.equal(c.poses.length, frames, "every frame tracked: a frame's pose is its row");
  return {
    run: {
      records: {
        frame: Array.from({ length: frames }, (_, i) => i),
        map: c.maps,
        loop: c.summaries.map((/** @type {{loop: unknown}} */ s) => s.loop),
      },
      keyframeMaps: Int32Array.from(c.keyframe_maps),
      keyframeLives: rows(c.keyframe_lives),
      keyframeLoops: rows(c.keyframe_loops),
      poses: Float32Array.from(c.poses.flatMap((/** @type {number[]} */ p) => p.slice(0, 12))),
      poseFrames: Int32Array.from({ length: frames }, (_, i) => i),
      poseMaps: Int32Array.from(c.pose_maps),
    },
    times: Float32Array.from(c.timestamps, (/** @type {number} */ t) => t - c.timestamps[0]),
  };
}

test("a loop is the frame whose record corrects it, its keyframes the loop the map holds", async () => {
  for (const [name, frame, older, newer, olderMade] of [
    ["loop/tum/rgbd", 261, 0, 28, 0],
    ["loop/euroc/stereo", 260, 2, 30, 13],
  ]) {
    const { run, times } = await room(String(name));
    assert.deepEqual(
      loopsOf(run),
      [
        {
          frame,
          map: 0,
          older,
          newer,
          matched: older,
          current: newer,
          olderMade,
          newerMade: frame,
        },
      ],
      String(name),
    );
    const [loop] = loopsOf(run);
    const apart = (times[frame] - times[Number(olderMade)]).toFixed(1);
    assert.equal(
      loopHint(loop, times),
      `loop closed in map 0: keyframes ${older} ↔ ${newer}, ${apart} s apart`,
    );
  }
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

test("the room's loop shows its start until the loop, then the first pass playing on", async () => {
  const { run, times } = await room("loop/tum/rgbd");
  const [window] = insets(loopsOf(run), times, run);
  const matches = Array.from(window.matches);
  assert.ok(
    matches.every((earlier, i) => earlier >= 0 && (i === 0 || earlier >= matches[i - 1])),
    "every frame has an earlier frame, never stepping back",
  );
  // Its earlier frames are the first pass's: nearer the start than the window.
  assert.ok(matches.every((earlier) => times[earlier] < times[window.first] / 2));
  const centre = (/** @type {number} */ f) => [3, 7, 11].map((i) => run.poses[f * 12 + i]);
  const at = 261;
  const earlier = matches[at - window.first];
  const gap = Math.hypot(...centre(at).map((v, i) => v - centre(earlier)[i]));
  assert.ok(gap < 0.15, `the loop's frame stands ${gap} m from its earlier frame`);
});
