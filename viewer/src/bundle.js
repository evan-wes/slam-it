// Loading a bundle (slam_it_all.orchestration.bundles, whose docstring states its files): its
// description and frames first; every backend's track (its records and poses), to compare them;
// and a backend's map and features when the page first shows it.
//
// The loader is given how to fetch a file, so the page fetches over HTTP and the tests read the
// golden bundle from disk.

import { STEPS } from "./compare.js";
import { FATES } from "./frames.js";
import { readFloat32, readInt32 } from "./npy.js";

/**
 * Fetches one of the bundle's files by its path in the bundle.
 * @typedef {(path: string) => Promise<ArrayBuffer>} Fetch
 */

/**
 * @typedef {object} Camera
 * @property {string} model The camera model.
 * @property {number} width The image width, in pixels.
 * @property {number} height The image height, in pixels.
 * @property {number[]} intrinsics fx, fy, cx, cy, then any distortion.
 * @property {number} baseline A stereo or RGB-D camera's baseline, in metres (0 otherwise).
 */

/**
 * @typedef {object} Latency
 * @property {number} deadline The frame period, in seconds.
 * @property {number} median The median latency, in seconds.
 * @property {number} p95 Its 95th percentile.
 * @property {number} max Its largest.
 * @property {number} late The frames past the deadline.
 */

/**
 * @typedef {object} Score
 * @property {number} matched The frames scored against the truth.
 * @property {number} ate_rmse The ATE, in metres.
 * @property {number} rpe_translation_rmse The RPE's translation, in metres.
 * @property {number} rpe_rotation_rmse The RPE's rotation, in radians.
 * @property {number} scale The alignment's scale.
 */

/**
 * What the header shows of a backend's run (bundle.json's `backends`).
 * @typedef {object} BackendInfo
 * @property {[number, number]} frames The bundle's frames it ran, `[first, end)`.
 * @property {number} tracked The frames it tracked.
 * @property {number} keyframes The final map's keyframes.
 * @property {number} points The final map's points.
 * @property {{keyframes: number, points: number}} made How many it made.
 * @property {boolean} features Whether it recorded its features.
 * @property {string | null} alignment The alignment it was moved onto the ground truth by
 *     ("sim3" or "se3"); null if it stays in its own frame: without ground truth, or a run its
 *     ATE alignment cannot place.
 * @property {Score | null} score Its score; null unless aligned.
 * @property {Latency | null} latency Its latency, for fewer than two frames null.
 * @property {number | null} peak_memory Its program's peak memory, in bytes, if known.
 */

/**
 * The whole sequence (bundle.json's `extent`): the bundle's frames are the part its runs ran.
 * @typedef {object} Extent
 * @property {number} frames The sequence's frames.
 * @property {number} start Its first frame's time since the bundle's first, in seconds.
 * @property {number} end Its last frame's time since the bundle's first, in seconds.
 */

/**
 * bundle.json.
 * @typedef {object} BundleInfo
 * @property {string} format "slam-it-all-bundle".
 * @property {number} version 1.
 * @property {string} dataset The dataset.
 * @property {string} sequence The sequence.
 * @property {string} mode "monocular", "stereo" or "rgbd".
 * @property {Camera} camera The pipeline's camera.
 * @property {number} frames The bundle's frames.
 * @property {number} first_time The first frame's time, in seconds.
 * @property {Extent} extent The whole sequence, of which the bundle's frames are the part run.
 * @property {{chunk: number, quality: number, colour: boolean}} images How the frames are stored.
 * @property {boolean} truth Whether the sequence has ground truth (`truth.npy`).
 * @property {string[]} fates The fates' names, by code (slam_it_all.eval.runs.FATES).
 * @property {string[]} steps The steps each frame times (slam_it_all.eval.runs.STEPS).
 * @property {Record<string, BackendInfo>} backends Each backend's header.
 */

/**
 * A backend's per-frame records: one column per diagnostics key, plus `frame`.
 * @typedef {Record<string, unknown[]> & {frame: number[]}} Records
 */

/**
 * A backend's track: what the page compares across backends.
 * @typedef {object} Track
 * @property {string} name The backend.
 * @property {BackendInfo} info Its header.
 * @property {Records} records Its per-frame records.
 * @property {Float32Array} poses Each tracked frame's `T_world_cam`, its top three rows, (T, 3, 4).
 * @property {Int32Array} poseFrames Their frames, (T,).
 */

/**
 * A backend's whole run, as the bundle holds it: its track, map and features.
 * @typedef {object} Backend
 * @property {string} name The backend.
 * @property {BackendInfo} info Its header.
 * @property {Records} records Its per-frame records.
 * @property {Float32Array} poses Each tracked frame's `T_world_cam`, its top three rows, (T, 3, 4).
 * @property {Int32Array} poseFrames Their frames, (T,).
 * @property {Float32Array} keyframePoses Every keyframe's last pose, (K, 3, 4).
 * @property {Int32Array} keyframeLives Their lives, (K, 3): made, removed (-1 if never), why.
 * @property {Float32Array} points Every point's last position, (N, 3).
 * @property {Int32Array} pointLives Their lives, (N, 3).
 * @property {{pixels: Float32Array, records: Int32Array} | null} features Each frame's
 *     keypoints (pixels (M, 2); frame, level, fate and point (M, 4)), if recorded.
 */

/**
 * A bundle as the page holds it.
 * @typedef {object} Bundle
 * @property {BundleInfo} info bundle.json.
 * @property {Float32Array} times Each frame's time since the first, in seconds.
 * @property {Float32Array | null} truth The true camera centre at each frame, (F, 3), NaN
 *     where none; null without ground truth.
 * @property {Int32Array} spans Each frame's JPEG's byte range in its chunk, (F, 2).
 */

export const FORMAT = "slam-it-all-bundle";
export const VERSION = 1;

/** The backends a bundle may hold, in the order the page offers them. */
export const BACKENDS = ["cpp", "rust", "python"];

/**
 * The bundle's address: the page's ``bundle`` parameter, else ``bundle/`` beside the page, on
 * this site alone, so that no link can show another site's bundle as this site's.
 * @param {string | null} given The page's ``bundle`` parameter, if it has one.
 * @param {string} page The page's address.
 * @returns {string} The bundle's address, ending in "/".
 */
export function bundleAddress(given, page) {
  const base = new URL(given ?? "bundle/", page);
  if (base.origin !== new URL(page).origin) {
    throw new Error("the bundle must be on this site");
  }
  return base.href.endsWith("/") ? base.href : `${base.href}/`;
}

/**
 * Decode a fetched file as JSON.
 * @param {ArrayBuffer} buffer The file's bytes.
 * @returns {unknown} Its value.
 */
function json(buffer) {
  return JSON.parse(new TextDecoder().decode(buffer));
}

/**
 * Check that an array has the shape expected.
 * @param {string} name The array's file, for the message.
 * @param {number[]} shape Its shape.
 * @param {number[]} trailing The dimensions expected after the first.
 * @param {number} [rows] The first dimension expected, if known.
 * @throws {Error} If the shape differs.
 */
function expectShape(name, shape, trailing, rows) {
  const ok =
    shape.length === trailing.length + 1 &&
    trailing.every((d, i) => shape[i + 1] === d) &&
    (rows === undefined || shape[0] === rows);
  if (!ok) {
    const wanted = [rows ?? "N", ...trailing].join(", ");
    throw new Error(`${name}: expected (${wanted}), got (${shape.join(", ")})`);
  }
}

/**
 * Whether bundle.json's `extent` is one: the sequence's frames, at least the bundle's, and its
 * first and last frames' times, in order.
 * @param {unknown} extent The value.
 * @param {number} frames The bundle's frames.
 * @returns {extent is Extent} Whether it is.
 */
function isExtent(extent, frames) {
  if (typeof extent !== "object" || extent === null) {
    return false;
  }
  const { frames: whole, start, end } = /** @type {Record<string, unknown>} */ (extent);
  return (
    Number.isInteger(whole) &&
    Number(whole) >= frames &&
    Number.isFinite(start) &&
    Number.isFinite(end) &&
    Number(start) <= Number(end)
  );
}

/**
 * Load a bundle's description and frames.
 * @param {Fetch} fetch How to fetch one of its files.
 * @returns {Promise<Bundle>} The bundle.
 * @throws {Error} If bundle.json is of another format or version, lacks the sequence's extent,
 *     names fates, steps or backends the page does not know, or an array is mis-shaped.
 */
export async function loadBundle(fetch) {
  const info = /** @type {BundleInfo} */ (json(await fetch("bundle.json")));
  if (info.format !== FORMAT || info.version !== VERSION) {
    throw new Error(`bundle.json: expected ${FORMAT} version ${VERSION}`);
  }
  if (!isExtent(info.extent, info.frames)) {
    throw new Error("bundle.json: expected the sequence's extent, {frames, start, end}");
  }
  // The page's copies of the run format's tables must be the ones the bundle was baked with.
  /** @type {[string, readonly string[], readonly string[]][]} */
  const tables = [
    ["fates", info.fates, FATES],
    ["steps", info.steps, STEPS],
  ];
  for (const [name, theirs, ours] of tables) {
    if (JSON.stringify(theirs) !== JSON.stringify(ours)) {
      throw new Error(`bundle.json: its ${name} are not the page's (${ours.join(", ")})`);
    }
  }
  const unknown = Object.keys(info.backends).filter((name) => !BACKENDS.includes(name));
  if (unknown.length > 0) {
    throw new Error(`bundle.json: the page does not know the backend ${unknown.join(", ")}`);
  }
  const times = readFloat32(await fetch("times.npy"));
  expectShape("times.npy", times.shape, [], info.frames);
  const spans = readInt32(await fetch("images/spans.npy"));
  expectShape("images/spans.npy", spans.shape, [2], info.frames);
  let truth = null;
  if (info.truth) {
    const read = readFloat32(await fetch("truth.npy"));
    expectShape("truth.npy", read.shape, [3], info.frames);
    truth = read.data;
  }
  return { info, times: times.data, truth, spans: spans.data };
}

/**
 * Readers of a backend's arrays, each checked for its shape.
 * @param {Fetch} fetch How to fetch one of the bundle's files.
 * @param {string} name The backend.
 */
function readers(fetch, name) {
  return {
    /** @param {string} file @param {number[]} trailing @param {number} [rows] */
    float32: async (file, trailing, rows) => {
      const read = readFloat32(await fetch(`${name}/${file}`));
      expectShape(`${name}/${file}`, read.shape, trailing, rows);
      return read.data;
    },
    /** @param {string} file @param {number[]} trailing @param {number} [rows] */
    int32: async (file, trailing, rows) => {
      const read = readInt32(await fetch(`${name}/${file}`));
      expectShape(`${name}/${file}`, read.shape, trailing, rows);
      return read.data;
    },
  };
}

/**
 * Load a backend's track: its records and poses.
 * @param {Fetch} fetch How to fetch one of the bundle's files.
 * @param {Bundle} bundle The bundle.
 * @param {string} name The backend.
 * @returns {Promise<Track>} Its track.
 * @throws {Error} If the bundle has no such backend, or an array is mis-shaped.
 */
export async function loadTrack(fetch, bundle, name) {
  const info = bundle.info.backends[name];
  if (info === undefined) {
    throw new Error(`the bundle has no ${name} run`);
  }
  const { float32, int32 } = readers(fetch, name);
  const records = /** @type {Records} */ (json(await fetch(`${name}/records.json`)));
  const poseFrames = await int32("pose_frames.npy", []);
  return {
    name,
    info,
    records,
    poses: await float32("poses.npy", [3, 4], poseFrames.length),
    poseFrames,
  };
}

/**
 * Load the rest of a backend's run: its map and, if recorded, its features.
 * @param {Fetch} fetch How to fetch one of the bundle's files.
 * @param {Track} track Its track.
 * @returns {Promise<Backend>} Its whole run.
 * @throws {Error} If an array is mis-shaped.
 */
export async function loadBackend(fetch, track) {
  const { float32, int32 } = readers(fetch, track.name);
  const keyframeLives = await int32("keyframe_lives.npy", [3]);
  const pointLives = await int32("point_lives.npy", [3]);
  let features = null;
  if (track.info.features) {
    const records = await int32("feature_records.npy", [4]);
    features = { pixels: await float32("feature_pixels.npy", [2], records.length / 4), records };
  }
  return {
    ...track,
    keyframePoses: await float32("keyframe_poses.npy", [3, 4], keyframeLives.length / 3),
    keyframeLives,
    points: await float32("points.npy", [3], pointLives.length / 3),
    pointLives,
    features,
  };
}
