// What the page shows while its parts load: over the frame, a spinner until the frame shown is
// drawn, the frame drawn before it dimmed while it stands in for it; over the map, a spinner
// until a run's trajectory can be drawn, then a small one in its corner while the run's
// keyframes and points load. None on a failure: the page's message says what failed. No
// drawing: the views draw what these return, and Node tests them.

/** How long the frame shown may wait for its image, in milliseconds, before the spinner shows
 *  over the frame drawn before it and that frame is dimmed: long enough that decoding a frame
 *  whose chunk is in, as during playback, never shows it. */
export const LATE = 250;

/**
 * A panel's loading mark: a spinner over its box, a small one in its corner, or none.
 * @typedef {"spinner" | "corner" | "none"} Mark
 */

/**
 * What the frame panel shows while the frame shown waits for its image: the spinner at once if
 * no frame is drawn yet, else once the wait is late, the frame drawn then dimmed as stale.
 * @param {object} image Where the frame panel's image stands.
 * @param {number} image.drawn The frame whose image is drawn; -1 for none yet.
 * @param {number} image.shown The frame shown; -1 before any.
 * @param {boolean} image.failed Whether the frame shown's image failed to load.
 * @param {number} image.waited How long the frame shown has waited for its image (since the
 *     frame drawn stopped being the frame shown), in milliseconds.
 * @returns {{mark: Mark, stale: boolean, recheck: number | null}} The mark; whether the frame
 *     drawn is dimmed; and in how many milliseconds the mark changes if nothing else does (the
 *     wait turning late), or null.
 */
export function imageLoading({ drawn, shown, failed, waited }) {
  if (failed || (drawn >= 0 && drawn === shown)) {
    return { mark: "none", stale: false, recheck: null };
  }
  if (drawn < 0) {
    return { mark: "spinner", stale: false, recheck: null };
  }
  if (waited < LATE) {
    return { mark: "none", stale: false, recheck: LATE - waited };
  }
  return { mark: "spinner", stale: true, recheck: null };
}

/**
 * What the map panel shows while a run loads: the spinner until a run's track is shown (its
 * trajectory, the first thing the panel draws), then the small mark in its corner while the
 * run's map loads.
 * @param {boolean} track Whether a run's track is shown.
 * @param {"loading" | "loaded" | "failed"} map Where the run's map stands (`Held`).
 * @returns {Mark} The mark.
 */
export function mapLoading(track, map) {
  if (!track) {
    return "spinner";
  }
  return map === "loading" ? "corner" : "none";
}
