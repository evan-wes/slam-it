// The numbers the pages show, as text: lengths, durations, deadlines, scale factors, shares,
// times, sizes and angles. No drawing.

/**
 * A length.
 * @param {number} metres The length, in metres.
 * @returns {string} In centimetres below a metre, else metres; three significant digits.
 */
export function length(metres) {
  return Math.abs(metres) < 1
    ? `${(metres * 100).toPrecision(3)} cm`
    : `${metres.toPrecision(3)} m`;
}

/**
 * A duration.
 * @param {number} seconds The duration, in seconds.
 * @returns {string} In milliseconds, to a tenth.
 */
export function duration(seconds) {
  return `${(seconds * 1000).toFixed(1)} ms`;
}

/**
 * A number to a tenth, or whole within 0.05 of a whole number.
 * @param {number} value The number.
 * @returns {string} "12.5" or "25", say.
 */
function tenth(value) {
  const whole = Math.round(value);
  return Math.abs(value - whole) <= 0.05 ? String(whole) : value.toFixed(1);
}

/**
 * A frame deadline: the frame period (slam_it.eval.budgets), in milliseconds and as the frame
 * rate it holds.
 * @param {number} seconds The deadline, in seconds.
 * @returns {string} "deadline 40 ms (25 fps)", say: each to a tenth, or whole within 0.05 of a
 *     whole number.
 */
export function deadline(seconds) {
  return `deadline ${tenth(seconds * 1000)} ms (${tenth(1 / seconds)} fps)`;
}

/**
 * A scale factor.
 * @param {number} factor The factor.
 * @returns {string} "×0.25", say: two significant digits, trailing zeros dropped.
 */
export function factor(factor) {
  return `×${Number(factor.toPrecision(2))}`;
}

/**
 * A share, as a percentage.
 * @param {number} fraction The share, from 0 to 1.
 * @returns {string} "80%", say: to a whole percent.
 */
export function percent(fraction) {
  return `${Math.round(fraction * 100)}%`;
}

/**
 * A time in a clip or sequence.
 * @param {number} seconds The time, in seconds.
 * @returns {string} In seconds, to a tenth.
 */
export function seconds(seconds) {
  return `${seconds.toFixed(1)} s`;
}

/**
 * A size.
 * @param {number | null} bytes The size, in bytes, if known.
 * @returns {string} In mebibytes, to a tenth; a dash if unknown.
 */
export function size(bytes) {
  return bytes === null ? "–" : `${(bytes / 2 ** 20).toFixed(1)} MiB`;
}

/**
 * An angle.
 * @param {number} radians The angle, in radians.
 * @returns {string} In degrees, three significant digits.
 */
export function angle(radians) {
  return `${((radians * 180) / Math.PI).toPrecision(3)}°`;
}
