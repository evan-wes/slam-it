// The solver panel's content, on the golden bundle's records.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBundle, loadTrack } from "../src/bundle.js";
import { STAGES, latency, series, summaryAt, summaryLayout, valueText } from "../src/solver.js";
import { fetchGolden, foundRoomRecords } from "./golden.js";

async function records() {
  const bundle = await loadBundle(fetchGolden);
  return (await loadTrack(fetchGolden, bundle, "cpp")).records;
}

/**
 * The stages that ran at a frame.
 * @param {import("../src/solver.js").StageLayout[]} layout The run's rows.
 * @param {{stages: import("../src/solver.js").StageValues[]}} summary The frame's summary.
 * @returns {string[]} Those stages.
 */
function ranAt(layout, summary) {
  return layout.filter((_, s) => summary.stages[s].ran).map(({ stage }) => stage);
}

/**
 * A field's value in a frame's summary.
 * @param {import("../src/solver.js").StageLayout[]} layout The run's rows.
 * @param {{stages: import("../src/solver.js").StageValues[]}} summary The frame's summary.
 * @param {string} stage The stage.
 * @param {string} field The field.
 * @returns {string | undefined} Its value as shown; undefined if the layout has no such row.
 */
function valueAt(layout, summary, stage, field) {
  const s = layout.findIndex((each) => each.stage === stage);
  const f = layout[s]?.fields.findIndex((each) => each.field === field) ?? -1;
  return f < 0 ? undefined : summary.stages[s].values[f];
}

test("a run's summary rows are every stage it ever ran, in order, each with all its fields", () => {
  const motion = { matches: 120, widened: false, inliers: 110 };
  const local = {
    keyframes: 3,
    points: 400,
    reference: 1,
    in_view: 300,
    matches: 250,
    inliers: 240,
  };
  // Columns in no particular order; pnp never ran, init is null throughout, mapping is missing.
  const columns = {
    frame: [0, 1, 2],
    placed_by: ["motion", null, "motion"],
    local_map: [null, null, local],
    pnp: [null, null, null],
    motion: [motion, null, motion],
    init: [null, null, null],
  };
  const layout = summaryLayout(columns);
  assert.deepEqual(
    layout.map(({ stage }) => stage),
    ["motion", "local_map"],
  );
  for (const { stage, title, fields } of layout) {
    const known = /** @type {(typeof STAGES)[number]} */ (STAGES.find((s) => s.stage === stage));
    assert.equal(title, known.title);
    assert.deepEqual(
      fields,
      Object.entries(known.hints).map(([field, hint]) => ({ field, hint })),
    );
  }
  // Each frame has every row: "–" where its stage did not run.
  const first = summaryAt(columns, 0, layout);
  assert.equal(first.placedBy, "motion");
  assert.deepEqual(first.stages, [
    { ran: true, values: ["120", "no", "110"] },
    { ran: false, values: ["–", "–", "–", "–", "–", "–"] },
  ]);
  assert.deepEqual(summaryAt(columns, 2, layout).stages[1].values, [
    "3",
    "400",
    "1",
    "300",
    "250",
    "240",
  ]);
  for (const k of [1, -1]) {
    const none = summaryAt(columns, k, layout);
    assert.equal(none.placedBy, "–");
    assert.deepEqual(
      none.stages.map(({ ran, values }) => [ran, values.every((value) => value === "–")]),
      [
        [false, true],
        [false, true],
      ],
      `record ${k}`,
    );
  }
});

test("every frame of a run, and a frame it did not run, has the run's rows", async () => {
  const columns = await foundRoomRecords();
  const layout = summaryLayout(columns);
  assert.deepEqual(
    layout.map(({ stage }) => stage),
    STAGES.map(({ stage }) => stage),
  );
  for (let k = -1; k < columns.frame.length; k++) {
    const { stages } = summaryAt(columns, k, layout);
    assert.deepEqual(
      stages.map(({ values }) => values.length),
      layout.map(({ fields }) => fields.length),
      `record ${k}`,
    );
  }
});

test("the frame that starts the map shows its initialisation alone", async () => {
  const columns = await records();
  const layout = summaryLayout(columns);
  const summary = summaryAt(columns, 0, layout);
  assert.equal(summary.placedBy, "initialization");
  assert.deepEqual(ranAt(layout, summary), ["init"]);
  assert.ok(layout.every(({ fields }) => fields.every(({ hint }) => hint.length > 0)));
});

test("a tracked frame shows the stages that ran, each field its record's", async () => {
  const columns = await records();
  const layout = summaryLayout(columns);
  const summary = summaryAt(columns, 1, layout);
  assert.equal(summary.placedBy, "motion");
  assert.deepEqual(ranAt(layout, summary), ["motion", "local_map"]);
  const local = /** @type {Record<string, unknown>} */ (columns.local_map[1]);
  assert.equal(valueAt(layout, summary, "local_map", "inliers"), String(local.inliers));
  assert.equal(valueAt(layout, summary, "init", "outcome"), "–");
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

test("a relocalised frame shows its relocalisation, each field the pipeline's", async () => {
  const columns = await foundRoomRecords();
  const layout = summaryLayout(columns);
  const k = columns.placed_by.indexOf("relocalization");
  const summary = summaryAt(columns, k, layout);
  assert.equal(summary.placedBy, "relocalization");
  assert.deepEqual(ranAt(layout, summary), ["motion", "pnp", "relocalization", "local_map"]);
  const recorded = /** @type {Record<string, unknown>} */ (columns.relocalization[k]);
  const { hints } = /** @type {(typeof STAGES)[number]} */ (
    STAGES.find((s) => s.stage === "relocalization")
  );
  assert.deepEqual(Object.keys(hints).sort(), Object.keys(recorded).sort());
  assert.equal(valueAt(layout, summary, "relocalization", "placed"), "yes");
  const [first] = /** @type {{keyframe: number, found: number}[]} */ (recorded.candidates);
  const tried = valueAt(layout, summary, "relocalization", "candidates") ?? "";
  assert.ok(tried.startsWith(`keyframe ${first.keyframe} (score `), tried);
  assert.ok(tried.endsWith(`, ${first.found} found`), tried);
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
  // Relocalisation's candidates, each as far as it got; the PnP fallback's are a count.
  const candidate = {
    keyframe: 4,
    score: 0.13934,
    matches: 131,
    iterations: 2,
    termination: "confidence",
    inliers: 130,
    refined: 120,
    found: 110,
  };
  const none = { iterations: null, termination: null, inliers: null, refined: null, found: null };
  assert.equal(
    valueText("candidates", [
      candidate,
      { ...candidate, ...none, keyframe: 9, score: 0.1, matches: 8 },
      { ...candidate, ...none, keyframe: 2, termination: "degenerate" },
    ]),
    "keyframe 4 (score 0.139): 131 matched, 130 inliers, 120 refined, 110 found; " +
      "keyframe 9 (score 0.100): 8 matched; keyframe 2 (score 0.139): 131 matched, no pose",
  );
  assert.equal(valueText("candidates", []), "none");
  assert.equal(valueText("candidates", 641), "641");
});

test("loop closing's search and correction read as the panel shows them", () => {
  const candidate = {
    keyframe: 0,
    score: 0.125,
    near: false,
    matches: 64,
    best: 0,
    inliers: 40,
    projected: 90,
    refined: 35,
    reprojected: 120,
    confirmations: 3,
  };
  const near = { ...candidate, keyframe: 7, near: true, matches: null, best: null };
  const unreached = { inliers: null, projected: null, refined: null, reprojected: null };
  const detection = {
    outcome: "confirmed",
    confirmation: { matched: 2, projected: 210, refined: 120, reprojected: null, confirmed: true },
    sharing: 4,
    scored: 2,
    candidates: [candidate, { ...near, ...unreached, confirmations: null }],
    confirmations: 2,
  };
  assert.equal(
    valueText("detection", detection),
    "confirmed, 2 confirmations; carried to keyframe 2's loop: 210 projected, 120 refined; " +
      "keyframe 0 (score 0.125): 64 matches, 40 inliers, 90 projected, 35 refined, " +
      "120 reprojected, 3 confirmations; keyframe 7 (score 0.125): linked: not tried",
  );
  const solve = { initial_error: 0.5, final_error: 0.0625, iterations: 4, termination: "function" };
  const correction = {
    current: 9,
    matched: 2,
    similarity: [
      [0, -1.5, 0, 0.5],
      [1.5, 0, 0, 0],
      [0, 0, 1.5, 0.25],
      [0, 0, 0, 1],
    ],
    corrected: 3,
    moved: 30,
    replaced: 4,
    fused: 12,
    links: 5,
    edges: 40,
    essential: solve,
    global_ba: null,
  };
  assert.equal(
    valueText("correction", correction),
    "keyframe 2, scale 1.500; 30 moved, 4 replaced, 12 fused, 5 new links; essential graph " +
      "(40 edges) 0.5000 → 0.06250 in 4 (function); global BA none",
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
