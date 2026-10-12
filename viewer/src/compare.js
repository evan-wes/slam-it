// Comparing a bundle's backends: the frames where their decisions differ, and each one's step
// times at a frame. No drawing.

import { recordIndex } from "./frames.js";

/** The columns that say what a frame decided: compared across backends exactly. */
export const DECISIONS = ["state", "tracked", "keyframe", "placed_by"];

/** The steps each frame times, in seconds (slam_it.eval.runs.STEPS). */
export const STEPS = ["read_seconds", "build_seconds", "track_seconds", "map_seconds"];

/**
 * The frames where any two backends that both ran them decided differently.
 * @param {{records: Record<string, unknown[]> & {frame: number[]}}[]} tracks The backends'
 *     tracks.
 * @param {number} frames The bundle's frames.
 * @returns {number[]} Those frames, ascending.
 */
export function differences(tracks, frames) {
  /** @type {number[]} */
  const out = [];
  for (let frame = 0; frame < frames; frame++) {
    /** @type {string | null} */
    let first = null;
    for (const { records } of tracks) {
      const k = recordIndex(records, frame);
      if (k < 0) {
        continue;
      }
      const decided = JSON.stringify(DECISIONS.map((column) => records[column]?.[k] ?? null));
      if (first === null) {
        first = decided;
      } else if (decided !== first) {
        out.push(frame);
        break;
      }
    }
  }
  return out;
}

/**
 * A backend's step times at a frame.
 * @param {{records: Record<string, unknown[]> & {frame: number[]}}} track Its track.
 * @param {number} frame The bundle's frame.
 * @returns {number[] | null} Each of `STEPS`, in seconds; null if it did not run the frame.
 */
export function stepsAt(track, frame) {
  const k = recordIndex(track.records, frame);
  if (k < 0) {
    return null;
  }
  return STEPS.map((step) => {
    const value = track.records[step]?.[k];
    return typeof value === "number" ? value : NaN;
  });
}

/**
 * Whether a backend ran a frame.
 * @param {{info: {frames: [number, number]}}} track Its track.
 * @param {number} frame The bundle's frame.
 * @returns {boolean} Whether the frame is in its span, `[first, end)`.
 */
export function ran(track, frame) {
  const [first, end] = track.info.frames;
  return first <= frame && frame < end;
}

/**
 * The backend to show at a frame: the one chosen if it ran the frame, else the first that did.
 * @param {{name: string, info: {frames: [number, number]}}[]} tracks The backends' tracks, in the
 *     page's order.
 * @param {string} chosen The backend chosen.
 * @param {number} frame The bundle's frame.
 * @returns {string} The backend to show: the chosen one if no backend ran the frame.
 */
export function shownAt(tracks, chosen, frame) {
  const current = tracks.find((track) => track.name === chosen);
  if (current !== undefined && ran(current, frame)) {
    return chosen;
  }
  return tracks.find((track) => ran(track, frame))?.name ?? chosen;
}

/**
 * A backend's span, as the page states it.
 * @param {{name: string, info: {frames: [number, number]}}} track Its track.
 * @returns {string} "python: frames 0–299", say, or that it ran none.
 */
export function spanText(track) {
  const [first, end] = track.info.frames;
  return end > first ? `${track.name}: frames ${first}–${end - 1}` : `${track.name}: no frames`;
}
