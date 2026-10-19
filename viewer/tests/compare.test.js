// Comparing the golden bundle's backends: where their decisions differ, and their step times.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBundle, loadTrack } from "../src/bundle.js";
import { DECISIONS, STEPS, differences, ran, shownAt, spanText, stepsAt } from "../src/compare.js";
import { fetchGolden } from "./golden.js";

async function tracks() {
  const bundle = await loadBundle(fetchGolden);
  const names = Object.keys(bundle.info.backends);
  return {
    bundle,
    tracks: await Promise.all(names.map((name) => loadTrack(fetchGolden, bundle, name))),
  };
}

test("the golden backends decide alike on every frame they share", async () => {
  const { bundle, tracks: all } = await tracks();
  assert.equal(all.length, 3);
  assert.deepEqual(differences(all, bundle.info.frames), []);
});

test("a frame where one backend decided otherwise is found, only where both ran it", async () => {
  const { bundle, tracks: all } = await tracks();
  const python = all.find((track) => track.name === "python");
  assert.ok(python !== undefined);
  const tracked = /** @type {number[]} */ (python.records.tracked);
  const changed = {
    ...python,
    records: { ...python.records, tracked: tracked.map((t, k) => (k === 2 ? t + 1 : t)) },
  };
  assert.deepEqual(
    differences([...all.filter((track) => track !== python), changed], bundle.info.frames),
    [2],
  );
  assert.deepEqual(differences([changed], bundle.info.frames), []);
  assert.deepEqual(DECISIONS, ["state", "map", "tracked", "keyframe", "placed_by"]);
});

test("a backend's step times at a frame are its record's, none where it did not run", async () => {
  const { bundle, tracks: all } = await tracks();
  for (const track of all) {
    const steps = stepsAt(track, 1);
    assert.deepEqual(
      steps,
      STEPS.map((step) => track.records[step][1]),
    );
    assert.equal(stepsAt(track, track.info.frames[1]), null);
    assert.ok(track.info.frames[1] <= bundle.info.frames);
  }
});

test("a backend shows the frames it ran; elsewhere the first that ran them is shown", async () => {
  const { tracks: all } = await tracks();
  const order = ["cpp", "rust", "python"].map((name) => all.find((track) => track.name === name));
  const shown = /** @type {import("../src/bundle.js").Track[]} */ (order);
  const python = shown[2];
  const [first, end] = python.info.frames;
  assert.ok(end < shown[0].info.frames[1], "the golden python run is the shorter");
  assert.equal(ran(python, first), true);
  assert.equal(ran(python, end - 1), true);
  assert.equal(ran(python, end), false);
  assert.equal(shownAt(shown, "python", end - 1), "python");
  assert.equal(shownAt(shown, "python", end), "cpp");
  assert.equal(shownAt(shown, "rust", end), "rust");
  assert.equal(spanText(python), `python: frames ${first}–${end - 1}`);
});

test("with no backend running a frame, the chosen one stays; a run of no frames says so", () => {
  const none = { name: "python", info: { frames: /** @type {[number, number]} */ ([0, 0]) } };
  const short = { name: "cpp", info: { frames: /** @type {[number, number]} */ ([0, 3]) } };
  assert.equal(shownAt([short, none], "python", 5), "python");
  assert.equal(spanText(none), "python: no frames");
});
