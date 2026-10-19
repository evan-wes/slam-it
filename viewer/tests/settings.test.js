// The 3D panel's sliders: their ranges, and their settings from what the page stores.

import assert from "node:assert/strict";
import { test } from "node:test";

import * as format from "../src/format.js";
import {
  OPACITY_RANGE,
  SIZE_RANGE,
  opacityPercent,
  sizeFactor,
  sizeStep,
} from "../src/settings.js";

test("the keyframe size slider spans about 1/20 to 2 of a map's own depth, logarithmically", () => {
  const [least, most] = SIZE_RANGE;
  assert.equal(sizeFactor(0), 1);
  assert.equal(sizeFactor(most), 2);
  assert.ok(Math.abs(sizeFactor(least) - 1 / 20) < 0.002, String(sizeFactor(least)));
  // Each step scales alike: ten steps are a doubling, wherever they start.
  assert.equal(sizeFactor(-20), 0.25);
  assert.ok(Math.abs(sizeFactor(-17) / sizeFactor(-27) - 2) < 1e-12);
  // What storage or the slider holds: rounded, held to the ends, and 0 for nothing usable.
  assert.equal(sizeStep("-20"), -20);
  assert.equal(sizeStep("3.6"), 4);
  assert.equal(sizeStep("-99"), least);
  assert.equal(sizeStep("99"), most);
  for (const nothing of [null, "", "large", "Infinity"]) {
    assert.equal(sizeStep(nothing), 0, String(nothing));
  }
  assert.equal(format.factor(sizeFactor(0)), "×1");
  assert.equal(format.factor(sizeFactor(-20)), "×0.25");
  assert.equal(format.factor(sizeFactor(least)), "×0.051");
  assert.equal(format.factor(sizeFactor(5)), "×1.4");
});

test("the point opacity slider runs from 5% to the points' own look, opaque, by default", () => {
  assert.deepEqual(OPACITY_RANGE, [5, 100, 5]);
  assert.equal(opacityPercent("80"), 80);
  // On the slider's nearest step, held to its ends; 100 for nothing usable.
  assert.equal(opacityPercent("73"), 75);
  assert.equal(opacityPercent("2"), 5);
  assert.equal(opacityPercent("0"), 5);
  assert.equal(opacityPercent("250"), 100);
  for (const nothing of [null, "", " ", "faint", "NaN", "-Infinity"]) {
    assert.equal(opacityPercent(nothing), 100, String(nothing));
  }
  assert.equal(format.percent(0.8), "80%");
  assert.equal(format.percent(1), "100%");
  assert.equal(format.percent(0.05), "5%");
});
