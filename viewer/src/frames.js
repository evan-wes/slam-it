// What a backend's run shows at a frame, worked out from its arrays: the points and keyframes
// alive, its keypoints and their fates, its record and pose. No drawing: the views draw what
// these return, and Node tests them.

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
 * The first index whose value is at least a target, in a sorted strided column.
 * @param {Int32Array} values The values.
 * @param {number} stride Values per row.
 * @param {number} target The target.
 * @returns {number} The first row whose value is at least `target` (the row count if none).
 */
function lowerBound(values, stride, target) {
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
 * The camera's pose to show at a frame: the frame's own if tracked, else the last before it.
 * @param {Float32Array} poses Each tracked frame's pose, (T, 3, 4).
 * @param {Int32Array} poseFrames Their frames.
 * @param {number} frame The frame.
 * @returns {{pose: Float32Array, tracked: boolean} | null} The pose and whether it is the
 *     frame's own; null before the first tracked frame.
 */
export function cameraAt(poses, poseFrames, frame) {
  const k = posesUpTo(poseFrames, frame) - 1;
  if (k < 0) {
    return null;
  }
  return { pose: poses.subarray(k * 12, k * 12 + 12), tracked: poseFrames[k] === frame };
}
