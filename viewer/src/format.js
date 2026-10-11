// The header's numbers, as text: lengths, durations, sizes and angles. No drawing.

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
