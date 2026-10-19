// The 3D panel's sliders: each one's range, and its setting from what it or the page's storage
// holds. No drawing.

/**
 * A slider's range: its least and greatest values, and the step between them.
 * @typedef {readonly [number, number, number]} Range
 */

/** The keyframe size slider's steps per doubling of the frustums' depth. */
export const SIZE_STEPS = 10;

/** The keyframe size slider's range, in steps from each map's own depth (step 0): 2^-4.3, about
 *  1/20, to 2. */
export const SIZE_RANGE = /** @type {const} */ ([-43, 10, 1]);

/** The map point opacity slider's range, in percent: from faint to the points' own look. */
export const OPACITY_RANGE = /** @type {const} */ ([5, 100, 5]);

/**
 * A slider's value from text, held to its range.
 * @param {string | null} text The value as text; null if none was stored.
 * @param {Range} range The slider's range.
 * @param {number} fallback Its default.
 * @returns {number} The value on the slider's nearest step, held to its ends; the default for
 *     none, or text that is not a finite number.
 */
function onSlider(text, [least, most, step], fallback) {
  const value = text === null || text.trim() === "" ? NaN : Number(text);
  if (!Number.isFinite(value)) {
    return fallback;
  }
  const snapped = least + Math.round((value - least) / step) * step;
  return Math.min(Math.max(snapped, least), most);
}

/**
 * A step of the keyframe size slider, from what it or the page's storage holds.
 * @param {string | null} text The step as text; null if none was stored.
 * @returns {number} The step, rounded and held to `SIZE_RANGE`; 0 (each map's own depth) for
 *     none, or text that is not a number.
 */
export function sizeStep(text) {
  return onSlider(text, SIZE_RANGE, 0);
}

/**
 * The frustums' depth at a step of the keyframe size slider, as a factor of each map's own.
 * @param {number} step The step (`sizeStep`).
 * @returns {number} The factor, 2^(step / `SIZE_STEPS`): logarithmic, so each step scales alike.
 */
export function sizeFactor(step) {
  return 2 ** (step / SIZE_STEPS);
}

/**
 * The map points' opacity on the opacity slider, from what it or the page's storage holds.
 * @param {string | null} text The percentage as text; null if none was stored.
 * @returns {number} The percentage, on the slider's nearest step and held to `OPACITY_RANGE`;
 *     100 (the points' own look, opaque) for none, or text that is not a number.
 */
export function opacityPercent(text) {
  return onSlider(text, OPACITY_RANGE, 100);
}
