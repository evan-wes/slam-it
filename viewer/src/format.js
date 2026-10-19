// The numbers the pages show, as text: lengths, durations, times, sizes and angles. No drawing.

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
 * A scale factor.
 * @param {number} factor The factor.
 * @returns {string} "×0.25", say: two significant digits, trailing zeros dropped.
 */
export function factor(factor) {
  return `×${Number(factor.toPrecision(2))}`;
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
