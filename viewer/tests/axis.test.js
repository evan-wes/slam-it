// The plots' value axes: their ticks and labels, and the deadline's label.

import assert from "node:assert/strict";
import { test } from "node:test";

import { ticks } from "../src/axis.js";
import * as format from "../src/format.js";

test("ticks run from 0 a nice step apart, about five above it, none past the top", () => {
  // 250 reaches five; 200 would reach six.
  assert.deepEqual(ticks(1337), {
    values: [0, 250, 500, 750, 1000, 1250],
    labels: ["0", "250", "500", "750", "1,000", "1,250"],
  });
  // A latency's top of 0.0612 s, in milliseconds: 10 reaches six, 20 only three.
  assert.deepEqual(ticks(0.0612 * 1000).values, [0, 10, 20, 30, 40, 50, 60]);
  // A top on a tick keeps it.
  assert.deepEqual(ticks(1000).values, [0, 200, 400, 600, 800, 1000]);
  // As near with 1 (seven) as with 2 or 2.5 (three): the whole step, then the larger.
  assert.deepEqual(ticks(7.5).values, [0, 2, 4, 6]);
});

test("a fractional step's labels all carry its decimals", () => {
  assert.deepEqual(ticks(0.9), {
    values: [0, 0.2, 0.4, 0.6, 0.8],
    labels: ["0.0", "0.2", "0.4", "0.6", "0.8"],
  });
  assert.deepEqual(ticks(15).labels, ["0.0", "2.5", "5.0", "7.5", "10.0", "12.5", "15.0"]);
  assert.deepEqual(ticks(1.3).values, [0, 0.25, 0.5, 0.75, 1, 1.25]);
});

test("a top that is not a positive number has 0 alone, and a step holds its least", () => {
  for (const top of [0, -3, NaN, Infinity]) {
    assert.deepEqual(ticks(top), { values: [0], labels: ["0"] }, String(top));
  }
  // Counts: a top of 1.1 has no fractional ticks, and a vanishing top only 0.
  assert.deepEqual(ticks(1.1, 5, 1).values, [0, 1]);
  assert.deepEqual(ticks(1.1e-9, 5, 1), { values: [0], labels: ["0"] });
  assert.deepEqual(ticks(3, 5, 1).values, [0, 1, 2, 3]);
  assert.deepEqual(ticks(0.35, 5, 0.1).labels, ["0.0", "0.1", "0.2", "0.3"]);
});

test("the deadline reads in milliseconds and as the frame rate it holds", () => {
  assert.equal(format.deadline(0.04), "deadline 40 ms (25 fps)");
  assert.equal(format.deadline(0.05), "deadline 50 ms (20 fps)");
  assert.equal(format.deadline(0.08), "deadline 80 ms (12.5 fps)");
  // Whole within 0.05 of a whole number, else to a tenth: NTSC's 29.97 fps reads as 30.
  assert.equal(format.deadline(1001 / 30000), "deadline 33.4 ms (30 fps)");
  assert.equal(format.deadline(0.0499999), "deadline 50 ms (20 fps)");
  assert.equal(format.deadline(1 / 14.9), "deadline 67.1 ms (14.9 fps)");
});
