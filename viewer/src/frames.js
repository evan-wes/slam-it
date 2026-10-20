// What a backend's run shows at a frame, worked out from its arrays: the points and keyframes
// alive, its keypoints and their fates, its record and pose. No drawing: the views draw what
// these return, and Node tests them.

import { lastRow } from "./geometry.js";

/** The fates a keypoint can have, by code (slam_it.eval.runs.FATES). */
export const FATES = /** @type {const} */ (["unmatched", "rejected", "tracked", "made"]);

/** What each fate means, in a line (the legend's hover text). */
export const FATE_HINTS = {
  unmatched: "not matched by the frame's last tracking stage, and no point made of it",
  rejected: "matched by the frame's last tracking stage, then not tracked",
  tracked: "an inlier of the frame's final pose",
  made: "a new map point was made of it on this frame",
};

/**
 * Whether a life spans the end of a frame: made at it or before, not removed by its end.
 * @param {number} made The frame that made it.
 * @param {number} removed The frame that removed it, or -1.
 * @param {number} frame The frame.
 * @returns {boolean} Whether it is in the map after the frame.
 */
export function alive(made, removed, frame) {
  return made <= frame && (removed < 0 || removed > frame);
}

/**
 * Count the lives that span the end of a frame.
 * @param {Int32Array} lives Lives, (N, 3): made, removed, why.
 * @param {number} frame The frame.
 * @returns {number} How many are in the map after it.
 */
export function countAlive(lives, frame) {
  let count = 0;
  for (let i = 0; i < lives.length; i += 3) {
    if (alive(lives[i], lives[i + 1], frame)) {
      count += 1;
    }
  }
  return count;
}

/**
 * The first row whose value is at least a target, in a sorted strided column.
 * @param {ArrayLike<number>} values The values: a row's is its first, ascending by row.
 * @param {number} stride Values per row (1 for a plain column).
 * @param {number} target The target.
 * @returns {number} The first row whose value is at least `target` (the row count if none).
 */
export function lowerBound(values, stride, target) {
  let lo = 0;
  let hi = values.length / stride;
  while (lo < hi) {
    const mid = (lo + hi) >> 1;
    if (values[mid * stride] < target) {
      lo = mid + 1;
    } else {
      hi = mid;
    }
  }
  return lo;
}

/**
 * The rows of a frame's keypoints in a run's feature records (which are in frame order).
 * @param {Int32Array} records Feature records, (M, 4): frame, level, fate, point.
 * @param {number} frame The frame.
 * @returns {[number, number]} The rows `[start, end)`.
 */
export function keypointRows(records, frame) {
  return [lowerBound(records, 4, frame), lowerBound(records, 4, frame + 1)];
}

/**
 * A frame's keypoints: their pixels and fates, and how many of each fate.
 * @param {{pixels: Float32Array, records: Int32Array}} features A run's feature records.
 * @param {number} frame The frame.
 * @returns {{xy: Float32Array, fate: Int32Array, counts: number[]}} Pixels (n, 2), fate codes
 *     (n,), and the count of each fate, by code.
 */
export function keypointsAt(features, frame) {
  const [start, end] = keypointRows(features.records, frame);
  const fate = new Int32Array(end - start);
  const counts = FATES.map(() => 0);
  for (let i = start; i < end; i++) {
    fate[i - start] = features.records[i * 4 + 2];
    counts[fate[i - start]] += 1;
  }
  return { xy: features.pixels.subarray(start * 2, end * 2), fate, counts };
}

/**
 * The index of a backend's record of a frame, if it ran the frame.
 * @param {{frame: number[]}} records Its records.
 * @param {number} frame The bundle's frame.
 * @returns {number} The record's index, or -1.
 */
export function recordIndex(records, frame) {
  const first = records.frame[0];
  const k = frame - first;
  return records.frame.length > 0 && k >= 0 && k < records.frame.length ? k : -1;
}

/**
 * How many of a run's tracked poses are of frames up to one: its trajectory so far.
 * @param {Int32Array} poseFrames The tracked frames, ascending.
 * @param {number} frame The frame.
 * @returns {number} The poses of frames up to and including `frame`.
 */
export function posesUpTo(poseFrames, frame) {
  return lowerBound(poseFrames, 1, frame + 1);
}

/**
 * The truth's segments over a stretch of frames, as a draw range: those whose two frames are
 * both in it.
 * @param {Int32Array} ends The frame each segment ends at, ascending (`truthSegments`).
 * @param {number} first The stretch's first frame.
 * @param {number} last Its last frame.
 * @returns {[number, number]} The segments' vertices, `[start, count]`.
 */
export function truthRange(ends, first, last) {
  const start = lowerBound(ends, 1, first + 1);
  const stop = lowerBound(ends, 1, last + 1);
  return [start * 2, Math.max(stop - start, 0) * 2];
}

/**
 * The map a backend shows at each of its frames: the frame's own, else the last before it, so
 * that between a lost map and the next the lost one stays in view. A record's map is the map
 * the frame ends in (the bundle's), so a frame of a map merged later shows the merged map, and
 * the maps shown need not rise: a kept map taken up again is shown again.
 * @param {{frame: number[], map?: unknown[]}} records Its records.
 * @returns {Int32Array} One map per record, -1 before its first.
 */
export function mapsShown(records) {
  const out = new Int32Array(records.frame.length).fill(-1);
  const maps = records.map ?? [];
  let last = -1;
  for (let k = 0; k < out.length; k++) {
    const map = maps[k];
    last = typeof map === "number" ? map : last;
    out[k] = last;
  }
  return out;
}

/**
 * The map a backend shows at a frame, from `mapsShown`: after its last frame, its last map.
 * @param {Int32Array} shown Its maps shown, one per record.
 * @param {{frame: number[]}} records Its records.
 * @param {number} frame The bundle's frame.
 * @returns {number | null} The map; null before its first map, or its first frame.
 */
export function mapAt(shown, records, frame) {
  if (shown.length === 0 || frame < records.frame[0]) {
    return null;
  }
  const map = shown[Math.min(frame - records.frame[0], shown.length - 1)];
  return map < 0 ? null : map;
}

/**
 * Whether one of a backend's maps was placed on the truth.
 * @param {{maps: {id: number, score: object | null}[]}} info The backend's header.
 * @param {number | null} map The map.
 * @returns {boolean} Whether it was: it has a score.
 */
export function placed(info, map) {
  return map !== null && (info.maps.find((m) => m.id === map)?.score ?? null) !== null;
}

/**
 * Whether another backend's map is drawn beside the map shown: both are placed on the truth,
 * or both are in their own frames and the same map (the backends decide alike).
 * @param {number | null} mine The map shown, null if none.
 * @param {number | null} theirs The other backend's map at the frame, null if none.
 * @param {boolean} theirsPlaced Whether theirs was placed on the truth (`placed`).
 * @param {boolean} aligned Whether the map shown was, so the scene is the truth's.
 * @returns {boolean} Whether the two share a frame.
 */
export function sharesFrame(mine, theirs, theirsPlaced, aligned) {
  if (mine === null || theirs === null || theirsPlaced !== aligned) {
    return false;
  }
  return aligned || theirs === mine;
}

/**
 * The camera's pose to show at a frame: the frame's own if tracked, else the last before it,
 * of a map's poses if given.
 * @param {Float32Array} poses Each tracked frame's pose, (T, 3, 4).
 * @param {Int32Array} poseFrames Their frames, ascending.
 * @param {number} frame The frame.
 * @param {[number, number][]} [runs] The runs of the map's poses' rows (`runsOf`); every pose's
 *     if not given.
 * @returns {{pose: Float32Array, tracked: boolean} | null} The pose and whether it is the
 *     frame's own; null before the first tracked frame (of the map).
 */
export function cameraAt(poses, poseFrames, frame, runs = [[0, poseFrames.length]]) {
  const k = lastRow(runs, posesUpTo(poseFrames, frame));
  if (k < 0) {
    return null;
  }
  return { pose: poses.subarray(k * 12, k * 12 + 12), tracked: poseFrames[k] === frame };
}
