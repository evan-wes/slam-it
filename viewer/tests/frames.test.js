// What a run shows at a frame, on the golden bundle: lives, keypoints and their fates, records,
// poses; checked against what the run's other files say, not against fixed numbers.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBackend, loadBundle, loadTrack } from "../src/bundle.js";
import { FATES, alive, cameraAt, countAlive, keypointRows, keypointsAt, posesUpTo, recordIndex } from "../src/frames.js";
import { fetchGolden } from "./golden.js";

const [UNMATCHED, , TRACKED, MADE] = [0, 1, 2, 3];

/** @param {string} name A backend. */
async function golden(name) {
  const bundle = await loadBundle(fetchGolden);
  return { bundle, run: await loadBackend(fetchGolden, await loadTrack(fetchGolden, bundle, name)) };
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

test("each frame's keypoints are its rows, and their fates count what its record says", async () => {
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
    assert.equal(counts.reduce((a, b) => a + b), end - start);
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
  assert.equal(recordIndex(run.records, run.info.frames[1] - 1), run.info.frames[1] - 1 - run.info.frames[0]);
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
});
