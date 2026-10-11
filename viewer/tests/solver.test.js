// The solver panel's content, on the golden bundle's records.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBundle, loadTrack } from "../src/bundle.js";
import { STAGES, latency, series, summaryAt, valueText } from "../src/solver.js";
import { fetchGolden } from "./golden.js";

async function records() {
  const bundle = await loadBundle(fetchGolden);
  return (await loadTrack(fetchGolden, bundle, "cpp")).records;
}

test("the frame that starts the map shows its initialisation alone", async () => {
  const summary = summaryAt(await records(), 0);
  assert.equal(summary.placedBy, "initialization");
  assert.deepEqual(
    summary.stages.map((s) => s.stage),
    ["init"],
  );
  assert.ok(summary.stages[0].rows.every((row) => row.hint.length > 0));
});

test("a tracked frame shows the stages that ran, each field its record's", async () => {
  const columns = await records();
  const summary = summaryAt(columns, 1);
  assert.equal(summary.placedBy, "motion");
  assert.deepEqual(
    summary.stages.map((s) => s.stage),
    ["motion", "local_map"],
  );
  const local = /** @type {Record<string, unknown>} */ (columns.local_map[1]);
  const shown = summary.stages[1].rows.find((row) => row.field === "inliers");
  assert.equal(shown?.value, String(local.inliers));
});

test("every stage's fields are what the pipeline records", async () => {
  const columns = await records();
  for (const { stage, hints } of STAGES) {
    const recorded = columns[stage].find((value) => value !== null);
    if (recorded !== undefined) {
      assert.deepEqual(
        Object.keys(hints).sort(),
        Object.keys(/** @type {object} */ (recorded)).sort(),
        stage,
      );
    }
  }
});

test("values read as the panel shows them", () => {
  assert.equal(valueText("matches", null), "–");
  assert.equal(valueText("widened", true), "yes");
  assert.equal(valueText("ba", []), "none");
  const solve = { initial_cost: 12.5, final_cost: 0.75, iterations: 5, termination: "function" };
  assert.equal(
    valueText("ba", [solve, solve]),
    "12.50 → 0.7500 in 5 (function); 12.50 → 0.7500 in 5 (function)",
  );
});

test("a series is a number per frame, NaN where its stage did not run", async () => {
  const columns = await records();
  const inliers = series(columns, "local_map.inliers");
  assert.ok(Number.isNaN(inliers[0]));
  assert.equal(inliers[1], /** @type {number[]} */ (columns.tracked)[1]);
  const build = /** @type {number[]} */ (columns.build_seconds);
  const track = /** @type {number[]} */ (columns.track_seconds);
  assert.deepEqual(
    Array.from(latency(columns)),
    build.map((b, i) => b + track[i]),
  );
});
