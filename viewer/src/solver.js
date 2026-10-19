// The solver panel's content, worked out from a backend's records: the summary's rows, each
// with a one-line explanation, the same at every frame of a run, each stage's values at a frame,
// and the series the plots draw. No drawing.

/**
 * A stage's rows in a run's summary: its title, and each field with what it means.
 * @typedef {{stage: string, title: string, fields: {field: string, hint: string}[]}} StageLayout
 */

/**
 * A stage's part of a frame's summary, in its layout's order.
 * @typedef {{ran: boolean, values: string[]}} StageValues
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
    stage: "relocalization",
    title: "Relocalisation",
    hints: {
      words: "the frame's visual words, from the run's vocabulary",
      sharing: "the map's keyframes that share a word with it",
      scored: "those that share enough of its words to be scored",
      candidates:
        "the keyframes tried, best first: each one's score, its points matched by word, and " +
        "the inliers of RANSAC, pose refinement and the searches by projection, as far as it got",
      placed: "whether a candidate placed the frame: it was found again in its map",
    },
  },
  {
    stage: "local_map",
    title: "Tracking the local map",
    hints: {
      keyframes:
        "the local map's keyframes: those seeing the frame's matched points, and their " +
        "neighbours in the covisibility graph",
      points: "the points they observe",
      reference: "the frame's reference keyframe: the one sharing the most of its points",
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
      triangulated: "points triangulated with the keyframe's best links",
      fused: "points fused into another",
      ba:
        "the local bundle adjustment's solves: cost before → after, iterations, why it " +
        "stopped; the second solve is over the first's inliers, so it starts lower",
      free: "the keyframes it adjusted: the new one and those linked to it",
      fixed: "the keyframes it held fixed: the map's first, and the others seeing their points",
      outliers: "observations the adjustment dropped",
      weak_points: "points erased, left seen too little by the observations dropped",
      culled_keyframes: "redundant keyframes culled",
    },
  },
  {
    stage: "loop",
    title: "Loop closing",
    hints: {
      detection:
        "the search for a loop: what came of it, the loop being confirmed carried to the " +
        "keyframe, the candidates tried as far as each got, and the confirmations it has",
      correction:
        "the loop corrected: the keyframe it matched and the scale it found, the points moved, " +
        "replaced and fused, the links new across it, the essential graph's solve and global BA's",
    },
  },
];

/** What `placed_by` means. */
export const PLACED_HINT =
  "how the frame was placed before the local map refined it: initialization, motion (the last " +
  "frame's points), local_pnp (the fallback) or relocalization (found again through the map's " +
  "keyframe database, once both failed); none if it was not";

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
 * A relocalisation candidate, as shown: as far as it got, its keyframe, score and counts.
 * @param {unknown} candidate A candidate's record.
 * @returns {string} The text.
 */
function candidateText(candidate) {
  const c =
    /** @type {{keyframe: number, score: number, matches: number, termination: string | null,
     *   inliers: number | null, refined: number | null, found: number | null}} */ (candidate);
  const said = [`${c.matches} matched`];
  if (c.termination === "degenerate") {
    said.push("no pose");
  }
  for (const [count, name] of /** @type {[number | null, string][]} */ ([
    [c.inliers, "inliers"],
    [c.refined, "refined"],
    [c.found, "found"],
  ])) {
    if (count !== null) {
      said.push(`${count} ${name}`);
    }
  }
  return `keyframe ${c.keyframe} (score ${c.score.toPrecision(3)}): ${said.join(", ")}`;
}

/**
 * The counts a loop candidate or confirmation reached, as shown: each that is not null.
 * @param {Record<string, unknown>} record The record.
 * @param {string[]} names The counts, in order.
 * @returns {string[]} Each as "<count> <name>".
 */
function counts(record, names) {
  return names.filter((name) => record[name] !== null).map((name) => `${record[name]} ${name}`);
}

/**
 * A loop search, as shown: what came of it, the loop carried, the candidates tried.
 * @param {unknown} detection A detection's record.
 * @returns {string} The text.
 */
function detectionText(detection) {
  const d =
    /** @type {{outcome: string, confirmation: Record<string, unknown> | null,
     *   candidates: Record<string, unknown>[], confirmations: number}} */ (detection);
  const said = [`${d.outcome}, ${d.confirmations} confirmations`];
  if (d.confirmation !== null) {
    const carried = counts(d.confirmation, ["projected", "refined", "reprojected"]);
    said.push(`carried to keyframe ${d.confirmation.matched}'s loop: ${carried.join(", ")}`);
  }
  for (const c of d.candidates) {
    const reached = c.near
      ? ["linked: not tried"]
      : counts(c, ["matches", "inliers", "projected", "refined", "reprojected", "confirmations"]);
    const score = /** @type {number} */ (c.score).toPrecision(3);
    said.push(`keyframe ${c.keyframe} (score ${score}): ${reached.join(", ")}`);
  }
  return said.join("; ");
}

/**
 * A graph solve, as shown: error before → after, iterations, and why it stopped.
 * @param {unknown} solve A solve's record.
 * @returns {string} The text.
 */
function graphSolveText(solve) {
  const s =
    /** @type {{initial_error: number, final_error: number, iterations: number,
     *   termination: string}} */ (solve);
  const errors = `${s.initial_error.toPrecision(4)} → ${s.final_error.toPrecision(4)}`;
  return `${errors} in ${s.iterations} (${s.termination})`;
}

/**
 * A loop's correction, as shown.
 * @param {unknown} correction A correction's record.
 * @returns {string} The text.
 */
function correctionText(correction) {
  const c =
    /** @type {{matched: number, similarity: number[][], moved: number, replaced: number,
     *   fused: number, links: number, edges: number, essential: unknown,
     *   global_ba: unknown}} */ (correction);
  const [x, y, z] = c.similarity.slice(0, 3).map((row) => row[0]);
  const scale = Math.hypot(x, y, z).toPrecision(4);
  const ba = c.global_ba === null ? "none" : graphSolveText(c.global_ba);
  return (
    `keyframe ${c.matched}, scale ${scale}; ${c.moved} moved, ${c.replaced} replaced, ` +
    `${c.fused} fused, ${c.links} new links; essential graph (${c.edges} edges) ` +
    `${graphSolveText(c.essential)}; global BA ${ba}`
  );
}

/**
 * A field's value, as shown.
 * @param {string} field The field.
 * @param {unknown} value Its value.
 * @returns {string} The text: none for null, each solve on its own for `ba`, each
 *     relocalisation candidate for a list of `candidates`, and loop closing's search and
 *     correction for `detection` and `correction`.
 */
export function valueText(field, value) {
  if (value === null || value === undefined) {
    return "–";
  }
  if (field === "detection") {
    return detectionText(value);
  }
  if (field === "correction") {
    return correctionText(value);
  }
  if (field === "ba") {
    const solves = /** @type {unknown[]} */ (value);
    return solves.length === 0 ? "none" : solves.map(solveText).join("; ");
  }
  if (field === "candidates" && Array.isArray(value)) {
    return value.length === 0 ? "none" : value.map(candidateText).join("; ");
  }
  if (typeof value === "boolean") {
    return value ? "yes" : "no";
  }
  return String(value);
}

/**
 * A run's summary rows: every stage its records summarise at any frame, in the pipeline's order,
 * each with all its fields. The same at every frame, so the panel's table keeps its shape.
 * @param {Record<string, unknown[]>} records A backend's records.
 * @returns {StageLayout[]} The stages.
 */
export function summaryLayout(records) {
  return STAGES.filter(({ stage }) =>
    (records[stage] ?? []).some((summary) => summary !== null),
  ).map(({ stage, title, hints }) => ({
    stage,
    title,
    fields: Object.entries(hints).map(([field, hint]) => ({ field, hint })),
  }));
}

/**
 * A frame's summary, laid out as its run's: how it was placed, and each stage's values.
 * @param {Record<string, unknown[]>} records A backend's records.
 * @param {number} k The record's index (not the bundle's frame); -1 for a frame not run.
 * @param {StageLayout[]} layout The run's rows (`summaryLayout`).
 * @returns {{placedBy: string, stages: StageValues[]}} How it was placed (none: "–"), and each
 *     of the layout's stages: whether it ran at the frame, and its fields' values ("–" each if
 *     it did not).
 */
export function summaryAt(records, k, layout) {
  const stages = layout.map(({ stage, fields }) => {
    const summary = /** @type {Record<string, unknown> | null} */ (records[stage]?.[k] ?? null);
    return {
      ran: summary !== null,
      values: fields.map(({ field }) =>
        summary === null ? "–" : valueText(field, summary[field]),
      ),
    };
  });
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
 * (slam_it.eval.budgets).
 * @param {Record<string, unknown[]>} records A backend's records.
 * @returns {Float64Array} Seconds, one per record.
 */
export function latency(records) {
  const build = series(records, "build_seconds");
  const track = series(records, "track_seconds");
  return build.map((b, i) => b + track[i]);
}
