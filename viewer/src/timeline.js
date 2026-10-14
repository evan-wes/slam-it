// The frame slider's timeline: where the bundle's frames sit in the whole sequence, what no run
// reached, and each backend's states and maps along its frames. No drawing.

import { seconds } from "./format.js";

/** The state that ends a run (slam_it.edge.pipeline.State.LOST). */
export const LOST = "lost";

/** The state before the first map (slam_it.edge.pipeline.State.NOT_INITIALIZED). */
export const NOT_INITIALIZED = "not_initialized";

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
 * @property {boolean} lost Whether every run that reached the bundle's last frame lost the track
 *     there, which ends a run.
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
 * Where the bundle's frames sit in the whole sequence.
 * @param {import("./bundle.js").Extent} extent The whole sequence.
 * @param {ArrayLike<number>} times The bundle's frames' times since its first, in seconds.
 * @param {{info: {frames: [number, number]}, records: Record<string, unknown[]>}[]} tracks The
 *     backends' tracks.
 * @returns {Placement} The parts before, of and after the bundle's frames.
 */
export function placement(extent, times, tracks) {
  const last = times.length > 0 ? times[times.length - 1] : 0;
  // Below half a frame period, a part is the times' rounding, not a frame no run reached.
  const half = (extent.end - extent.start) / Math.max(extent.frames - 1, 1) / 2;
  const reaching = tracks.filter((track) => track.info.frames[1] === times.length);
  return {
    before: -extent.start > half ? -extent.start : 0,
    ran: last,
    after: extent.end - last > half ? extent.end - last : 0,
    lost: reaching.length > 0 && reaching.every((track) => track.records.state?.at(-1) === LOST),
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
    after: placed.lost
      ? [
          "not processed: tracking was lost",
          `tracking was lost at ${seconds(last)} of ${seconds(whole)}, which ends a run, so the ` +
            "rest was not processed",
        ]
      : ["not run", `${seconds(last)} to ${seconds(whole)}: no run went further`],
  };
}
