// What a run shows at a frame, on the golden bundle: lives, keypoints and their fates, records,
// poses; checked against what the run's other files say, not against fixed numbers.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBackend, loadBundle, loadTrack } from "../src/bundle.js";
import {
  FATES,
  alive,
  cameraAt,
  countAlive,
  keypointRows,
  keypointsAt,
  mapAt,
  mapsShown,
  placed,
  posesUpTo,
  recordIndex,
  sharesFrame,
  truthRange,
} from "../src/frames.js";
import { fetchGolden } from "./golden.js";

const [UNMATCHED, , TRACKED, MADE] = [0, 1, 2, 3];

/** @param {string} name A backend. */
async function golden(name) {
  const bundle = await loadBundle(fetchGolden);
  return {
    bundle,
    run: await loadBackend(fetchGolden, await loadTrack(fetchGolden, bundle, name)),
  };
}

test("a life spans the frames from the one that made it to before the one that removed it", () => {
  assert.equal(alive(3, -1, 2), false);
  assert.equal(alive(3, -1, 3), true);
  assert.equal(alive(3, 5, 4), true);
  assert.equal(alive(3, 5, 5), false);
  assert.equal(countAlive(new Int32Array([0, -1, -1, 1, 1, 0, 2, -1, -1]), 1), 1);
});

test("after a run's last frame, its lives leave its final map", async () => {
  for (const name of ["cpp", "rust", "python"]) {
    const { run } = await golden(name);
    const last = run.info.frames[1] - 1;
    assert.equal(countAlive(run.pointLives, last), run.info.points, name);
    assert.equal(countAlive(run.keyframeLives, last), run.info.keyframes, name);
    assert.equal(posesUpTo(run.poseFrames, last), run.info.tracked, name);
  }
});

test("each frame's keypoints are its rows; their fates count what its record says", async () => {
  const { run } = await golden("cpp");
  assert.ok(run.features !== null);
  const tracked = /** @type {number[]} */ (run.records.tracked);
  const placed = /** @type {(string | null)[]} */ (run.records.placed_by);
  let next = 0;
  for (const frame of run.records.frame) {
    const [start, end] = keypointRows(run.features.records, frame);
    assert.equal(start, next);
    next = end;
    const { xy, fate, counts } = keypointsAt(run.features, frame);
    assert.equal(xy.length, (end - start) * 2);
    assert.equal(fate.length, end - start);
    assert.equal(
      counts.reduce((a, b) => a + b),
      end - start,
    );
    const k = recordIndex(run.records, frame);
    if (placed[k] === "initialization") {
      assert.equal(counts[MADE], tracked[k]);
    } else if (!run.records.keyframe[k]) {
      assert.equal(counts[TRACKED], tracked[k]);
    }
  }
  assert.equal(next, run.features.records.length / 4);
  assert.equal(FATES[UNMATCHED], "unmatched");
});

test("a frame outside a run's span has no record", async () => {
  const { bundle, run } = await golden("python");
  assert.equal(
    recordIndex(run.records, run.info.frames[1] - 1),
    run.info.frames[1] - 1 - run.info.frames[0],
  );
  assert.equal(recordIndex(run.records, run.info.frames[1]), -1);
  assert.ok(run.info.frames[1] < bundle.info.frames);
});

test("the camera shown is the frame's own pose, or the last before it", async () => {
  const { run } = await golden("cpp");
  assert.equal(cameraAt(run.poses, run.poseFrames, -1), null);
  const first = cameraAt(run.poses, run.poseFrames, run.poseFrames[0]);
  assert.ok(first !== null && first.tracked);
  assert.deepEqual(Array.from(first.pose), Array.from(run.poses.subarray(0, 12)));
  const gaps = new Int32Array([0, 2]);
  const held = cameraAt(new Float32Array(24).fill(1, 12), gaps, 1);
  assert.ok(held !== null && !held.tracked && held.pose[0] === 0);
  // Within a map's poses: none of map 1's before its first.
  assert.equal(cameraAt(new Float32Array(24), gaps, 1, [[1, 2]]), null);
  assert.ok(cameraAt(new Float32Array(24), gaps, 5, [[0, 1]])?.tracked === false);
  // A map in two runs of rows, another map's between: the last of its poses up to the frame.
  const poses = Float32Array.from({ length: 5 * 12 }, (_, i) => Math.floor(i / 12));
  const frames = new Int32Array([0, 1, 2, 3, 4]);
  const runs = /** @type {[number, number][]} */ ([
    [0, 2],
    [4, 5],
  ]);
  assert.equal(cameraAt(poses, frames, 3, runs)?.pose[0], 1); // map 1's rows 2-3 left out
  assert.ok(cameraAt(poses, frames, 4, runs)?.tracked === true);
});

test("the map shown at a frame is the frame's own, else the last before it", () => {
  const records = { frame: [10, 11, 12, 13, 14, 15], map: [null, 0, 0, null, null, 1] };
  const shown = mapsShown(records);
  assert.deepEqual(Array.from(shown), [-1, 0, 0, 0, 0, 1]);
  assert.equal(mapAt(shown, records, 9), null); // before the run's first frame
  assert.equal(mapAt(shown, records, 10), null); // before its first map
  assert.equal(mapAt(shown, records, 13), 0); // the lost map, until the next starts
  assert.equal(mapAt(shown, records, 15), 1);
  assert.equal(mapAt(shown, records, 40), 1); // after its last frame, its last map
  assert.deepEqual(Array.from(mapsShown({ frame: [0, 1] })), [-1, -1]);
});

test("another backend's map is drawn where it shares the shown map's frame", () => {
  const info = {
    maps: [
      { id: 0, score: null },
      { id: 1, score: { ate_rmse: 0.1 } },
    ],
  };
  assert.deepEqual(
    [placed(info, 0), placed(info, 1), placed(info, 2), placed(info, null)],
    [false, true, false, false],
  );
  // Both placed on the truth: drawn whatever their numbers.
  assert.equal(sharesFrame(1, 3, true, true), true);
  // Both in their own frames: only the same map shares one.
  assert.equal(sharesFrame(0, 0, false, false), true);
  assert.equal(sharesFrame(0, 1, false, false), false);
  // One placed and one not: never.
  assert.equal(sharesFrame(1, 1, false, true), false);
  assert.equal(sharesFrame(0, 0, true, false), false);
  // No map on either side: never.
  assert.equal(sharesFrame(null, 0, false, false), false);
  assert.equal(sharesFrame(0, null, false, false), false);
});

test("the truth is drawn over the segments whose two frames are both in a stretch", () => {
  // Segments ending at frames 1, 2, 4 and 5 (frames 0-1, 1-2, 3-4 and 4-5; 2-3 has no truth).
  const ends = new Int32Array([1, 2, 4, 5]);
  // Two vertices a segment: [start, count].
  assert.deepEqual(truthRange(ends, 0, 5), [0, 8]);
  assert.deepEqual(truthRange(ends, 0, 2), [0, 4]);
  // From frame 1: the segment 0-1 begins before it.
  assert.deepEqual(truthRange(ends, 1, 5), [2, 6]);
  assert.deepEqual(truthRange(ends, 3, 4), [4, 2]);
  // A single frame, or a stretch inside the gap: no segment.
  assert.deepEqual(truthRange(ends, 4, 4), [6, 0]);
  assert.deepEqual(truthRange(ends, 2, 3), [4, 0]);
});
