// The loops a run closed: each loop's frame and keyframes, and the loop nearest a frame. No
// drawing.

import { seconds } from "./format.js";

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
