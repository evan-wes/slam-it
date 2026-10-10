// Comparing the golden bundle's backends: where their decisions differ, and their step times.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBundle, loadTrack } from "../src/bundle.js";
import { DECISIONS, STEPS, differences, stepsAt } from "../src/compare.js";
import { fetchGolden } from "./golden.js";

async function tracks() {
  const bundle = await loadBundle(fetchGolden);
  const names = Object.keys(bundle.info.backends);
  return { bundle, tracks: await Promise.all(names.map((name) => loadTrack(fetchGolden, bundle, name))) };
}

test("the golden backends decide alike on every frame they share", async () => {
  const { bundle, tracks: all } = await tracks();
  assert.equal(all.length, 3);
  assert.deepEqual(differences(all, bundle.info.frames), []);
});

test("a frame where one backend decided otherwise is found, and only where both ran it", async () => {
  const { bundle, tracks: all } = await tracks();
  const python = all.find((track) => track.name === "python");
  assert.ok(python !== undefined);
  const tracked = /** @type {number[]} */ (python.records.tracked);
  const changed = { ...python, records: { ...python.records, tracked: tracked.map((t, k) => (k === 2 ? t + 1 : t)) } };
  assert.deepEqual(differences([...all.filter((track) => track !== python), changed], bundle.info.frames), [2]);
  assert.deepEqual(differences([changed], bundle.info.frames), []);
  assert.deepEqual(DECISIONS, ["state", "tracked", "keyframe", "placed_by"]);
});

test("a backend's step times at a frame are its record's, none where it did not run", async () => {
  const { bundle, tracks: all } = await tracks();
  for (const track of all) {
    const steps = stepsAt(track, 1);
    assert.deepEqual(steps, STEPS.map((step) => track.records[step][1]));
    assert.equal(stepsAt(track, track.info.frames[1]), null);
    assert.ok(track.info.frames[1] <= bundle.info.frames);
  }
});
