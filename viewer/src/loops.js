// The loops a run closed, and the earlier pass each loop's frames came back to: each loop's frame
// and keyframes, the loop nearest a frame, and, for the frames around a loop, the earlier frame
// that stood at the same place, for the "seen before" inset. No drawing.

import { seconds } from "./format.js";
import { centres } from "./geometry.js";

/** The "seen before" inset's window: from this long before a loop's frame, in seconds. */
export const INSET_BEFORE = 3;

/** The inset's window: to this long after a loop's frame, in seconds. */
export const INSET_AFTER = 5;

/** The earlier pass is searched for within this long either side of the frame that made the
 *  loop's matched keyframe, in seconds. */
export const INSET_SEARCH = 10;

/**
 * A loop closed.
 * @typedef {object} Loop
 * @property {number} frame The bundle's frame whose record shows the loop's correction.
 * @property {number} map The map it closed in.
 * @property {number} older The keyframe it matched, by id (the order the run made them).
 * @property {number} newer The keyframe that closed it, by id.
 * @property {number} matched The keyframe it matched as the record names it: counted in its map.
 * @property {number} current The keyframe that closed it, as the record names it.
 * @property {number} olderMade The frame that made the matched keyframe.
 * @property {number} newerMade The frame that made the closing keyframe.
 */

/**
 * The frames around a loop, each with the earlier frame at the same place.
 * @typedef {object} Inset
 * @property {Loop} loop The loop.
 * @property {number} first The window's first frame.
 * @property {Int32Array} matches Each window frame's earlier frame, from `first`; -1 before the
 *     first that has one.
 */

/**
 * The loops a run closed: each frame whose record shows a loop's correction, its keyframes
 * checked against the loops the run's map holds.
 * @param {{records: Record<string, unknown[]> & {frame: number[]}, keyframeMaps: Int32Array,
 *     keyframeLives: Int32Array, keyframeLoops: Int32Array}} run The run: its records, and its
 *     keyframes' maps, lives and loops (`Backend`).
 * @returns {Loop[]} The loops, in the order the frames closed them.
 * @throws {Error} If a correction names keyframes that are not a loop of `keyframeLoops` in its
 *     frame's map, made before it.
 */
export function loopsOf({ records, keyframeMaps, keyframeLives, keyframeLoops }) {
  // A record counts a map's keyframes from 0; the run's ids count every map's, in order.
  /** @type {Map<number, number>} */
  const firsts = new Map();
  keyframeMaps.forEach((map, id) => {
    if (!firsts.has(map)) {
      firsts.set(map, id);
    }
  });
  const pairs = new Set();
  for (let e = 0; e < keyframeLoops.length; e += 2) {
    pairs.add(`${keyframeLoops[e]} ${keyframeLoops[e + 1]}`);
  }
  /** @type {Loop[]} */
  const out = [];
  records.frame.forEach((frame, k) => {
    const loop = /** @type {{correction?: Record<string, number> | null} | null} */ (
      records.loop?.[k] ?? null
    );
    const correction = loop?.correction ?? null;
    if (correction === null) {
      return;
    }
    const { matched, current } = correction;
    const map = records.map?.[k];
    const first = typeof map === "number" ? firsts.get(map) : undefined;
    const [older, newer] = first === undefined ? [-1, -1] : [first + matched, first + current];
    const [olderMade, newerMade] = [keyframeLives[older * 3], keyframeLives[newer * 3]];
    const closed =
      first !== undefined &&
      pairs.has(`${Math.min(older, newer)} ${Math.max(older, newer)}`) &&
      keyframeMaps[older] === map &&
      keyframeMaps[newer] === map &&
      olderMade < newerMade &&
      newerMade <= frame;
    if (!closed) {
      throw new Error(
        `frame ${frame}: its loop's keyframes ${matched} and ${current} of map ${map} are ` +
          "not a loop the run closed (keyframe_loops.npy)",
      );
    }
    out.push({
      frame,
      map: /** @type {number} */ (map),
      older,
      newer,
      matched,
      current,
      olderMade,
      newerMade,
    });
  });
  return out;
}

/**
 * A loop, as its mark on the timeline says it.
 * @param {Loop} loop The loop.
 * @param {ArrayLike<number>} times The bundle's frames' times, in seconds.
 * @returns {string} "loop closed in map 0: keyframes 0 ↔ 28, 8.7 s apart", say: the
 *     keyframes as the records and the Solver panel name them.
 */
export function loopHint(loop, times) {
  const apart = times[loop.newerMade] - times[loop.olderMade];
  return (
    `loop closed in map ${loop.map}: keyframes ${loop.matched} ↔ ${loop.current}, ` +
    `${seconds(apart)} apart`
  );
}

/**
 * The loop nearest a frame, before it or after it.
 * @param {Loop[]} loops The loops, in frame order.
 * @param {number} frame The frame.
 * @param {-1 | 1} way -1 for the last loop before the frame, 1 for the first after it.
 * @returns {Loop | null} The loop; null if there is none that way.
 */
export function loopBeside(loops, frame, way) {
  const beside = loops.filter((loop) => (way < 0 ? loop.frame < frame : loop.frame > frame));
  return (way < 0 ? beside[beside.length - 1] : beside[0]) ?? null;
}

/**
 * The first frame at or after a time, or after it.
 * @param {ArrayLike<number>} times The frames' times, ascending.
 * @param {number} time The time.
 * @param {boolean} [after] Whether a frame at the time is passed over.
 * @returns {number} The frame; the frames' count if there is none.
 */
function firstFrom(times, time, after = false) {
  let [low, high] = [0, times.length];
  while (low < high) {
    const middle = (low + high) >> 1;
    if (times[middle] < time || (after && times[middle] === time)) {
      low = middle + 1;
    } else {
      high = middle;
    }
  }
  return low;
}

/**
 * Each loop's window of frames, from `INSET_BEFORE` before its frame to `INSET_AFTER` after it,
 * each with the earlier frame whose final pose's centre is nearest its own: among the tracked
 * frames of the loop's map within `INSET_SEARCH` of the frame that made the matched keyframe,
 * and nearer it in time than the window (on a short loop, the frames leading into the window
 * would otherwise be their own earlier pass). A frame without a final pose in the map takes the
 * frame before's, and the earlier frames never step back as the window's go on, so the inset
 * plays forward.
 * @param {Loop[]} loops The loops.
 * @param {ArrayLike<number>} times The bundle's frames' times, in seconds, ascending.
 * @param {{poses: Float32Array, poseFrames: Int32Array, poseMaps: Int32Array}} track The run's
 *     final poses, their frames (ascending) and their maps.
 * @returns {Inset[]} Each loop's window, in the loops' order.
 */
export function insets(loops, times, { poses, poseFrames, poseMaps }) {
  const path = centres(poses);
  /** @type {Map<number, number>} */
  const rowOf = new Map();
  poseFrames.forEach((frame, row) => rowOf.set(frame, row));
  /**
   * @param {number} a A pose's row. @param {number} b Another's.
   * @returns {number} The squared distance between their centres.
   */
  const apart = (a, b) =>
    (path[a * 3] - path[b * 3]) ** 2 +
    (path[a * 3 + 1] - path[b * 3 + 1]) ** 2 +
    (path[a * 3 + 2] - path[b * 3 + 2]) ** 2;
  return loops.map((loop) => {
    const at = times[loop.frame];
    const first = firstFrom(times, at - INSET_BEFORE);
    const end = firstFrom(times, at + INSET_AFTER, true);
    const made = times[loop.olderMade];
    const before = (made + times[first]) / 2;
    /** @type {number[]} */
    const candidates = [];
    poseFrames.forEach((frame, row) => {
      if (
        times[frame] < before &&
        poseMaps[row] === loop.map &&
        Math.abs(times[frame] - made) <= INSET_SEARCH
      ) {
        candidates.push(row);
      }
    });
    const matches = new Int32Array(end - first).fill(-1);
    let last = -1;
    for (let frame = first; frame < end; frame++) {
      const row = rowOf.get(frame);
      if (row !== undefined && poseMaps[row] === loop.map && candidates.length > 0) {
        let [nearest, least] = [candidates[0], Infinity];
        for (const candidate of candidates) {
          const distance = apart(row, candidate);
          if (distance < least) {
            [nearest, least] = [candidate, distance];
          }
        }
        last = Math.max(last, poseFrames[nearest]);
      }
      matches[frame - first] = last;
    }
    return { loop, first, matches };
  });
}

/**
 * The earlier frame the inset shows at a frame: the latest window's that holds the frame.
 * @param {Inset[]} windows The loops' windows (`insets`).
 * @param {number} frame The frame shown.
 * @returns {{loop: Loop, earlier: number} | null} The loop and the earlier frame; null outside
 *     every window, or before a window's first match.
 */
export function insetAt(windows, frame) {
  /** @type {{loop: Loop, earlier: number} | null} */
  let out = null;
  for (const { loop, first, matches } of windows) {
    const earlier = frame >= first && frame < first + matches.length ? matches[frame - first] : -1;
    if (earlier >= 0) {
      out = { loop, earlier };
    }
  }
  return out;
}

/**
 * The inset's label.
 * @param {{loop: Loop, earlier: number}} shown The loop and the earlier frame (`insetAt`).
 * @param {number} frame The frame shown.
 * @param {ArrayLike<number>} times The bundle's frames' times, in seconds.
 * @returns {string} "seen 8 s earlier · keyframe 0", say: to the whole second, the matched
 *     keyframe as the records name it.
 */
export function seenText({ loop, earlier }, frame, times) {
  const ago = Math.round(times[frame] - times[earlier]);
  return `seen ${ago} s earlier · keyframe ${loop.matched}`;
}
