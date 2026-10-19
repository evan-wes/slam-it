// The plots' value axis: ticks from 0 to a plot's top, a "nice" step apart, and their labels.
// No drawing.

/** A step's leading digits: it is one of these times a power of ten. */
const MANTISSAS = [1, 2, 2.5, 5];

/** How far a value may sit past a tick and still reach it: a top on a tick keeps the tick. */
const REACH = 1e-9;

/**
 * The fewest decimals that write a step exactly.
 * @param {number} step The step: a mantissa times a power of ten.
 * @returns {number} The decimals, 0 for a whole number.
 */
function decimalsOf(step) {
  let decimals = 0;
  while (
    decimals < 20 &&
    Math.abs(Math.round(step * 10 ** decimals) / 10 ** decimals - step) > step * REACH
  ) {
    decimals += 1;
  }
  return decimals;
}

/**
 * The ticks above 0 that a step reaches under a top.
 * @param {number} top The top.
 * @param {number} step The step.
 * @returns {number} Their count.
 */
function reached(top, step) {
  return Math.floor((top / step) * (1 + REACH));
}

/**
 * A value axis's ticks: from 0, a step of 1, 2, 2.5 or 5 times a power of ten apart, and none
 * above the top. The step is the one whose count of ticks above 0 is nearest `count`; of steps
 * as near, the one with the fewest decimals, then the largest (the fewest labels).
 * @param {number} top The axis's top, in the unit its labels are in.
 * @param {number} [count] The ticks above 0 to aim for.
 * @param {number} [least] The finest step worth labelling: 1 for counts, say. A top too small for
 *     it has the ticks it reaches, 0 alone if none.
 * @returns {{values: number[], labels: string[]}} The ticks, ascending, and their labels: each
 *     with the step's decimals, thousands grouped ("1,250"). 0 alone for a top that is not a
 *     positive, finite number.
 */
export function ticks(top, count = 5, least = 0) {
  if (!(Number.isFinite(top) && top > 0)) {
    return { values: [0], labels: ["0"] };
  }
  const decade = Math.floor(Math.log10(top / count));
  let step = least;
  let miss = Infinity;
  for (let exponent = decade - 1; exponent <= decade + 1; exponent++) {
    for (const mantissa of MANTISSAS) {
      // Divided by a power of ten, not times its inverse: 2.5 / 10 is exactly 0.25.
      const candidate = exponent < 0 ? mantissa / 10 ** -exponent : mantissa * 10 ** exponent;
      const off = Math.abs(reached(top, candidate) - count);
      const plainer = off === miss && decimalsOf(candidate) <= decimalsOf(step);
      if (candidate >= least && (off < miss || plainer)) {
        [step, miss] = [candidate, off];
      }
    }
  }
  const decimals = decimalsOf(step);
  const values = Array.from({ length: reached(top, step) + 1 }, (_, i) =>
    Number((i * step).toFixed(decimals)),
  );
  const labels = values.map((value) =>
    value.toLocaleString("en-US", {
      minimumFractionDigits: decimals,
      maximumFractionDigits: decimals,
    }),
  );
  return { values, labels };
}
