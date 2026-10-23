// The map shown at each frame, on the merge and revisit bundles: a frame shows the map it ends
// in, which holds its pose, whether its map was merged into another later or taken up again
// after another (B4's review panel, A1).

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBundle, loadMap, loadTrack } from "../src/bundle.js";
import { cameraAt, mapAt, mapsShown } from "../src/frames.js";
import { centres, mapDepths, runsOf } from "../src/geometry.js";
import { fetchMerge, fetchRevisit } from "./golden.js";

/**
 * A golden bundle's C++ run: its track and map.
 * @param {import("../src/bundle.js").Fetch} fetch How to fetch its files.
 * @returns {Promise<import("../src/bundle.js").Backend>} The run.
 */
async function cppRun(fetch) {
  const track = await loadTrack(fetch, await loadBundle(fetch), "cpp");
  return { ...track, ...(await loadMap(fetch, track)) };
}

/**
 * Check that every tracked frame shows a map that holds its pose, as the camera's own.
 * @param {import("../src/bundle.js").Backend} run The run.
 */
function assertEachFrameShowsItsPose(run) {
  const shown = mapsShown(run.records);
  run.poseFrames.forEach((frame, row) => {
    const map = mapAt(shown, run.records, frame);
    assert.equal(map, run.poseMaps[row], `frame ${frame}: the map its pose ends in`);
    const at = cameraAt(run.poses, run.poseFrames, frame, runsOf(run.poseMaps, map ?? -1));
    assert.ok(at !== null && at.tracked, `frame ${frame}: its own pose, in the map shown`);
    assert.deepEqual(Array.from(at.pose), Array.from(run.poses.subarray(row * 12, row * 12 + 12)));
  });
}

test("a merged map's frames show the map they were merged into, never an empty one", async () => {
  const run = await cppRun(fetchMerge);
  assert.ok(run.keyframeMerges.length > 0);
  // Every frame ends in map 0; the second map, merged into it, is not a map of the run's.
  assert.deepEqual(
    run.info.maps.map((map) => map.id),
    [0],
  );
  assert.ok(run.records.map.every((map) => map === null || map === 0));
  assert.equal(run.info.maps[0].frames.length, 2); // the black frames between, in no map
  assertEachFrameShowsItsPose(run);
});

test("a map taken up again is shown again, its poses in two runs with the other map's between", async () => {
  const run = await cppRun(fetchRevisit);
  const [first, second] = run.info.maps;
  assert.deepEqual([first.id, second.id], [0, 1]);
  assert.equal(first.frames.length, 2);
  assert.equal(second.frames.length, 1);
  // Map 1's stretch lies between map 0's two.
  assert.ok(first.frames[0][1] <= second.frames[0][0]);
  assert.ok(second.frames[0][1] <= first.frames[1][0]);
  const shown = mapsShown(run.records);
  const [during, after] = [second.frames[0][0], first.frames[1][0]];
  assert.deepEqual([mapAt(shown, run.records, during), mapAt(shown, run.records, after)], [1, 0]);
  assert.equal(runsOf(run.poseMaps, 0).length, 2);
  assertEachFrameShowsItsPose(run);
  // Each map's frustums are as deep as a share of all its runs' extent: one depth a map.
  const depths = mapDepths(
    centres(run.poses),
    run.poseMaps,
    centres(run.keyframePoses),
    run.keyframeMaps,
  );
  assert.deepEqual([...depths.keys()], [0, 1]);
});
