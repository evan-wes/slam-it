// The loops a run closed: on the loop rooms' runs (tests/fixtures/edge/room_runs_cpp.json, each
// closing one loop) and on runs made up here.

import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { test } from "node:test";

import { loopBeside, loopHint, loopsOf } from "../src/loops.js";

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
 * One of the loop rooms' C++ runs, as a bundle gives a run.
 * @param {string} name The case: "loop/tum/rgbd" or "loop/euroc/stereo".
 */
async function room(name) {
  const doc = JSON.parse(await readFile(ROOMS, "utf8"));
  const c = doc.cases[name];
  const frames = c.summaries.length;
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
