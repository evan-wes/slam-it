// The frame slider's timeline: where the bundle's frames sit in the whole sequence, what no run
// reached, and each backend's states and maps along its frames. No drawing.

import { seconds } from "./format.js";

/** The state of the frame a run lost the track on: its map closed there, and the next frame
 *  starts a new one (slam_it.edge.pipeline.State.LOST). */
export const LOST = "lost";

/** The state before a map, the first or the next after a lost one
 *  (slam_it.edge.pipeline.State.NOT_INITIALIZED). */
export const NOT_INITIALIZED = "not_initialized";

/** A frame's `placed_by` where relocalisation placed it: the track failed, and was found again
 *  in its map through the keyframe database (slam_it.eval.runs.RELOCALIZED). */
export const RELOCALIZED = "relocalization";

/** What a state is called on the page; a state not named here is shown as tracking. */
export const STATE_WORDS = {
  [NOT_INITIALIZED]: "no map yet",
  tracking: "tracking",
  [LOST]: "lost the track",
};

/**
 * A run of frames in one state and one map.
 * @typedef {object} StateSpan
 * @property {string} state The state.
 * @property {number | null} map The map its records name; null where there was none.
 * @property {boolean} reset Whether that map was reset (lost while too small to keep).
 * @property {number} first The first frame, the bundle's.
 * @property {number} end The frame after the last.
 */

/**
 * Where the bundle's frames sit in the whole sequence, in seconds.
 * @typedef {object} Placement
 * @property {number} before The sequence's time before the bundle's first frame, which no run
 *     reached; 0 if under half a frame.
 * @property {number} ran From the bundle's first frame to its last.
 * @property {number} after The sequence's time after the bundle's last frame; 0 if under half a
 *     frame.
 */

/**
 * A backend's frames, in runs of one state and one map.
 * @param {{frame: number[], state?: unknown[], map?: unknown[]}} records Its records.
 * @param {{id: number, reset: boolean}[]} [maps] Its maps (`MapInfo`), for which were reset.
 * @returns {StateSpan[]} Its frames in runs of one state and map, in order.
 */
export function stateSpans(records, maps = []) {
  /** @type {StateSpan[]} */
  const out = [];
  const states = records.state ?? [];
  const named = records.map ?? [];
  const reset = new Set(maps.filter((m) => m.reset).map((m) => m.id));
  records.frame.forEach((frame, k) => {
    const state = String(states[k]);
    const map = typeof named[k] === "number" ? /** @type {number} */ (named[k]) : null;
    const last = out.at(-1);
    if (last !== undefined && last.state === state && last.map === map && last.end === frame) {
      last.end = frame + 1;
    } else {
      const wasReset = map !== null && reset.has(map);
      out.push({ state, map, reset: wasReset, first: frame, end: frame + 1 });
    }
  });
  return out;
}

/**
 * A run of a backend's frames in one state, as its hint says it.
 * @param {string} name The backend.
 * @param {StateSpan} span The run of frames.
 * @returns {string} "cpp: tracking in map 1, frames 61–118", say.
 */
export function spanHint(name, { state, map, reset, first, end }) {
  const words = STATE_WORDS[/** @type {keyof typeof STATE_WORDS} */ (state)] ?? state;
  const frames = end - first > 1 ? `frames ${first}–${end - 1}` : `frame ${first}`;
  const where =
    map === null ? "" : ` ${state === LOST ? "of" : "in"} map ${map}${reset ? ", reset" : ""}`;
  return `${name}: ${words}${where}, ${frames}`;
}

/**
 * A backend's frames found again: each where its track failed and relocalisation placed it.
 * @param {{frame: number[], placed_by?: unknown[], map?: unknown[]}} records Its records.
 * @returns {{frame: number, map: number | null}[]} Each such frame, the bundle's, and the map
 *     it was found in, in order.
 */
export function foundFrames(records) {
  const placed = records.placed_by ?? [];
  const named = records.map ?? [];
  return records.frame.flatMap((frame, k) =>
    placed[k] === RELOCALIZED
      ? [{ frame, map: typeof named[k] === "number" ? /** @type {number} */ (named[k]) : null }]
      : [],
  );
}

/**
 * A frame found again, as its hint says it.
 * @param {string} name The backend.
 * @param {{frame: number, map: number | null}} found The frame and its map.
 * @returns {string} "cpp: found again in map 0, frame 34", say.
 */
export function foundHint(name, { frame, map }) {
  return `${name}: found again${map === null ? "" : ` in map ${map}`}, frame ${frame}`;
}

/**
 * Where the bundle's frames sit in the whole sequence: a run goes on past a lost track, so the
 * parts no run reached are the parts no run was given.
 * @param {import("./bundle.js").Extent} extent The whole sequence.
 * @param {ArrayLike<number>} times The bundle's frames' times since its first, in seconds.
 * @returns {Placement} The parts before, of and after the bundle's frames.
 */
export function placement(extent, times) {
  const last = times.length > 0 ? times[times.length - 1] : 0;
  // Below half a frame period, a part is the times' rounding, not a frame no run reached.
  const half = (extent.end - extent.start) / Math.max(extent.frames - 1, 1) / 2;
  return {
    before: -extent.start > half ? -extent.start : 0,
    ran: last,
    after: extent.end - last > half ? extent.end - last : 0,
  };
}

/**
 * What the timeline says of the parts no run reached, its times in the sequence's.
 * @param {Placement} placed Where the bundle's frames sit.
 * @returns {{before: [string, string], after: [string, string]}} The part before the bundle's
 *     frames and the part after them: each one's label and hint.
 */
export function notRunText(placed) {
  const first = placed.before;
  const last = first + placed.ran;
  const whole = last + placed.after;
  return {
    before: ["not run", `the sequence's first ${seconds(first)}: no run started there`],
    after: ["not run", `${seconds(last)} to ${seconds(whole)}: no run went further`],
  };
}
