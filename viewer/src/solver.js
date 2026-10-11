// The solver panel's content, worked out from a backend's records: each stage's summary at a
// frame as rows with a one-line explanation each, and the series the plots draw. No drawing.

/**
 * A summary field: its value as shown, and what it means.
 * @typedef {{field: string, value: string, hint: string}} Row
 */

/**
 * A stage's part of a frame's summary.
 * @typedef {{stage: string, title: string, rows: Row[]}} StageRows
 */

/** Each stage the diagnostics summarise, its title, and its fields' one-line explanations. */
export const STAGES = [
  {
    stage: "init",
    title: "Initialisation",
    hints: {
      matches: "keypoints matched to the monocular reference frame",
      outcome: "how the attempt to start the map ended",
      ba:
        "the initial bundle adjustment's solves: cost before → after, iterations, why it " +
        "stopped",
    },
  },
  {
    stage: "motion",
    title: "Tracking the last frame",
    hints: {
      matches: "keypoints matched to the last frame's points, in the window the velocity predicts",
      widened: "whether the window was doubled because too few matched",
      inliers: "the matches pose refinement kept",
    },
  },
  {
    stage: "pnp",
    title: "PnP fallback",
    hints: {
      candidates: "the local map's points",
      matches: "keypoints matched to them by descriptor, with a bearing in front",
      iterations: "RANSAC's samples",
      termination: "why RANSAC stopped (degenerate: it found no pose)",
      inliers: "RANSAC's inliers",
      refined: "the inliers pose refinement kept",
    },
  },
  {
    stage: "local_map",
    title: "Tracking the local map",
    hints: {
      in_view: "the local map's points in the camera's view",
      matches: "keypoints matched to them, before the pose was refined",
      inliers: "the matches the final pose kept: the frame's tracked points",
    },
  },
  {
    stage: "mapping",
    title: "Local mapping",
    hints: {
      from_depth: "points made from the keyframe's close depth",
      culled_points: "recent points culled: tracking found them too rarely",
      triangulated: "points triangulated with the window's keyframes",
      fused: "points fused into another",
      ba:
        "the local bundle adjustment's solves: cost before → after, iterations, why it " +
        "stopped; the second solve is over the first's inliers, so it starts lower",
      outliers: "observations the adjustment dropped",
      weak_points: "points erased, left seen too little by the observations dropped",
      culled_keyframes: "redundant keyframes culled",
    },
  },
];

/** What `placed_by` means. */
export const PLACED_HINT =
  "how the frame was placed before the local map refined it: initialization, motion (the last " +
  "frame's points) or local_pnp (the fallback); none if it was not";

/**
 * A solve, as shown: cost before → after, iterations, and why it stopped.
 * @param {unknown} solve A solve's record.
 * @returns {string} The text.
 */
function solveText(solve) {
  const s =
    /** @type {{initial_cost: number, final_cost: number, iterations: number,
     *   termination: string}} */ (solve);
  const costs = `${s.initial_cost.toPrecision(4)} → ${s.final_cost.toPrecision(4)}`;
  return `${costs} in ${s.iterations} (${s.termination})`;
}

/**
 * A field's value, as shown.
 * @param {string} field The field.
 * @param {unknown} value Its value.
 * @returns {string} The text: none for null, each solve on its own for `ba`.
 */
export function valueText(field, value) {
  if (value === null || value === undefined) {
    return "–";
  }
  if (field === "ba") {
    const solves = /** @type {unknown[]} */ (value);
    return solves.length === 0 ? "none" : solves.map(solveText).join("; ");
  }
  if (typeof value === "boolean") {
    return value ? "yes" : "no";
  }
  return String(value);
}

/**
 * A frame's summary: how it was placed, and each stage that ran, as rows.
 * @param {Record<string, unknown[]>} records A backend's records.
 * @param {number} k The record's index (not the bundle's frame).
 * @returns {{placedBy: string, stages: StageRows[]}} How it was placed (none: "–"), and the
 *     stages that ran, in the pipeline's order.
 */
export function summaryAt(records, k) {
  /** @type {StageRows[]} */
  const stages = [];
  for (const { stage, title, hints } of STAGES) {
    const summary = /** @type {Record<string, unknown> | null} */ (records[stage]?.[k] ?? null);
    if (summary === null) {
      continue;
    }
    const rows = Object.entries(hints).map(([field, hint]) => ({
      field,
      value: valueText(field, summary[field]),
      hint,
    }));
    stages.push({ stage, title, rows });
  }
  return { placedBy: valueText("placed_by", records.placed_by?.[k]), stages };
}

/**
 * A number per record from a column, or a nested field of one: NaN where it is null.
 * @param {Record<string, unknown[]>} records A backend's records.
 * @param {string} path A column (`tracked`), or a stage's field (`local_map.inliers`).
 * @returns {Float64Array} One value per record.
 */
export function series(records, path) {
  const [column, field] = path.split(".");
  const values = records[column] ?? [];
  return Float64Array.from(values, (value) => {
    const picked =
      field === undefined ? value : /** @type {Record<string, unknown> | null} */ (value)?.[field];
    return typeof picked === "number" ? picked : typeof picked === "boolean" ? Number(picked) : NaN;
  });
}

/**
 * Each frame's latency: building it and tracking it, what the deadline holds
 * (slam_it_all.eval.budgets).
 * @param {Record<string, unknown[]>} records A backend's records.
 * @returns {Float64Array} Seconds, one per record.
 */
export function latency(records) {
  const build = series(records, "build_seconds");
  const track = series(records, "track_seconds");
  return build.map((b, i) => b + track[i]);
}
