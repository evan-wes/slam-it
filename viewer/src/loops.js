// The loops a run closed, and the earlier pass each loop's frames came back to: each loop's frame
// and keyframes, the loop nearest a frame, and, for the frames around a loop, the earlier frame
// that stood at the same place, for the "seen before" inset; and the loop effect the playhead
// sets off on arriving at a loop: its cycle, its brightness over time and the frames the inset
// plays meanwhile. No drawing.

import { seconds } from "./format.js";
import { lowerBound, posesUpTo } from "./frames.js";
import { centres, runsOf } from "./geometry.js";

/** The "seen before" inset's window: from this long before a loop's frame, in seconds. */
export const INSET_BEFORE = 3;

/** The inset's window: to this long after a loop's frame, in seconds. */
export const INSET_AFTER = 5;

/** The earlier pass is searched for within this long either side of the frame that made the
 *  loop's matched keyframe, in seconds. */
export const INSET_SEARCH = 10;

/** The loop effect: at full brightness for this long, in seconds, on its own clock... */
export const EFFECT_HOLD = 1.5;

/** ...then fading to nothing over this long, in seconds. */
export const EFFECT_FADE = 1;

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
 * The last frame at or before a time.
 * @param {ArrayLike<number>} times The frames' times, ascending.
 * @param {number} time The time.
 * @returns {number} The frame; -1 if every frame is later.
 */
function lastUpTo(times, time) {
  let after = lowerBound(times, 1, time);
  while (after < times.length && times[after] === time) {
    after += 1; // a frame at the time itself is reached
  }
  return after - 1;
}

/**
 * The tracked frames a loop's window may take as its earlier pass: those of the loop's map
 * within `INSET_SEARCH` of the frame that made the matched keyframe, and before a cutoff midway
 * between the matched keyframe's making and the window's first frame (on a short loop, the
 * frames leading into the window would otherwise be their own earlier pass).
 * @param {Loop} loop The loop.
 * @param {ArrayLike<number>} times The bundle's frames' times, in seconds.
 * @param {Int32Array} poseFrames The run's tracked frames, ascending.
 * @param {Int32Array} poseMaps Their maps.
 * @param {number} first The window's first frame.
 * @returns {number[]} Their poses' rows, ascending.
 */
function candidateRows(loop, times, poseFrames, poseMaps, first) {
  const made = times[loop.olderMade];
  const cutoff = (made + times[first]) / 2;
  /** @type {number[]} */
  const out = [];
  poseFrames.forEach((frame, row) => {
    const time = times[frame];
    if (poseMaps[row] === loop.map && time < cutoff && Math.abs(time - made) <= INSET_SEARCH) {
      out.push(row);
    }
  });
  return out;
}

/**
 * The candidate pose whose centre is nearest a pose's.
 * @param {Float32Array} path The poses' centres, (T, 3).
 * @param {number} row The pose's row.
 * @param {number[]} candidates The candidates' rows, at least one.
 * @returns {number} The nearest's row; of candidates as near, the first.
 */
function nearestRow(path, row, candidates) {
  let [nearest, least] = [candidates[0], Infinity];
  for (const candidate of candidates) {
    const distance =
      (path[row * 3] - path[candidate * 3]) ** 2 +
      (path[row * 3 + 1] - path[candidate * 3 + 1]) ** 2 +
      (path[row * 3 + 2] - path[candidate * 3 + 2]) ** 2;
    if (distance < least) {
      [nearest, least] = [candidate, distance];
    }
  }
  return nearest;
}

/**
 * Each loop's window of frames, from `INSET_BEFORE` before its frame to `INSET_AFTER` after it,
 * each with the earlier frame whose final pose's centre is nearest its own, among the window's
 * candidates (`candidateRows`). A frame without a final pose in the map takes the frame before's,
 * and the earlier frames never step back as the window's go on, so the inset plays forward.
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
  return loops.map((loop) => {
    const at = times[loop.frame];
    const first = lowerBound(times, 1, at - INSET_BEFORE);
    const end = lastUpTo(times, at + INSET_AFTER) + 1;
    const candidates = candidateRows(loop, times, poseFrames, poseMaps, first);
    const matches = new Int32Array(end - first).fill(-1);
    let last = -1;
    for (let frame = first; frame < end; frame++) {
      const row = rowOf.get(frame);
      if (row !== undefined && poseMaps[row] === loop.map && candidates.length > 0) {
        last = Math.max(last, poseFrames[nearestRow(path, row, candidates)]);
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

/**
 * The trajectory's rows of a loop's cycle: its map's poses from the frame that made the matched
 * keyframe to the loop's frame, in the runs of rows the map's poses come in (`runsOf`), so that
 * another map's frames between are left out.
 * @param {Loop} loop The loop.
 * @param {Int32Array} poseFrames The run's tracked frames, ascending.
 * @param {Int32Array} poseMaps Their maps.
 * @returns {[number, number][]} The runs' rows, each `[start, end)`, in order; none if it has
 *     none.
 */
export function cycleRuns(loop, poseFrames, poseMaps) {
  const from = lowerBound(poseFrames, 1, loop.olderMade);
  const to = posesUpTo(poseFrames, loop.frame);
  return runsOf(poseMaps, loop.map)
    .map(
      ([start, end]) =>
        /** @type {[number, number]} */ ([Math.max(start, from), Math.min(end, to)]),
    )
    .filter(([start, end]) => end > start);
}

/**
 * The loop effect's brightness, a time into it.
 * @param {number} time Seconds since it began.
 * @returns {number} The opacity: 1 for `EFFECT_HOLD`, then falling evenly to 0 over
 *     `EFFECT_FADE`, and 0 after.
 */
export function effectOpacity(time) {
  return Math.min(1, Math.max(0, 1 - (time - EFFECT_HOLD) / EFFECT_FADE));
}

/**
 * Whether the loop effect is over, a time into it.
 * @param {number} time Seconds since it began.
 * @returns {boolean} Whether it has faded out.
 */
export function effectOver(time) {
  return time >= EFFECT_HOLD + EFFECT_FADE;
}

/**
 * The frame the inset plays a time into a loop's effect: the earlier pass, from the frame that
 * made the matched keyframe on, at the frames' own speed.
 * @param {Loop} loop The loop.
 * @param {ArrayLike<number>} times The bundle's frames' times, in seconds, ascending.
 * @param {number} time Seconds since the effect began.
 * @returns {number} The last frame whose time, from that frame's, is reached; before the loop's
 *     frame, so the earlier pass never runs into the loop.
 */
export function effectFrame(loop, times, time) {
  const reached = lastUpTo(times, times[loop.olderMade] + Math.max(time, 0));
  return Math.max(loop.olderMade, Math.min(reached, loop.frame - 1));
}

/**
 * What the playhead moving does to the loop effect. Arriving at a loop starts its effect, a new
 * one over a running one: playing onto it or across it (the last loop passed, as playback may
 * skip frames), or a seek onto its frame (the slider, a button, a key), again each time. A seek
 * elsewhere ends a running effect; playing on leaves it to finish.
 * @param {number | null} running The frame of the loop whose effect runs; null if none.
 * @param {Loop[]} loops The loops, in frame order.
 * @param {number} from The frame shown before.
 * @param {number} to The frame shown now.
 * @param {boolean} played Whether playback moved the playhead, rather than a seek.
 * @returns {Loop | "end" | null} The loop to start the effect of; "end" to end the running one;
 *     null to leave things as they are.
 */
export function effectChange(running, loops, from, to, played) {
  if (to === from) {
    return null;
  }
  const passed = played && to > from;
  const arrived = loops.filter((loop) =>
    passed ? from < loop.frame && loop.frame <= to : loop.frame === to,
  );
  if (arrived.length > 0) {
    return arrived[arrived.length - 1];
  }
  return running !== null && !played ? "end" : null;
}
