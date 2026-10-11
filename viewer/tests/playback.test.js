// The playhead: seeking within the frames, and playback following the frames' own times.

import assert from "node:assert/strict";
import { test } from "node:test";

import { Playhead } from "../src/playback.js";

/** A playhead over frames at 0, 0.1, 0.2 and 0.3 s, and the frames it showed. */
function playhead() {
  /** @type {number[]} */
  const shown = [];
  const head = new Playhead(new Float32Array([0, 0.1, 0.2, 0.3]), (frame) => shown.push(frame));
  return { head, shown };
}

test("seeking stays within the frames, and shows the frame", () => {
  const { head, shown } = playhead();
  head.seek(2);
  head.seek(-1);
  head.seek(9);
  assert.deepEqual(shown, [2, 0, 3]);
  assert.equal(head.frame, 3);
  assert.equal(head.clock, head.times[3]);
});

test("playing moves to the last frame whose time is reached, at the speed chosen", () => {
  const { head, shown } = playhead();
  head.start(1000);
  assert.equal(head.advance(1050, 1), true); // 0.05 s: still frame 0
  assert.equal(head.advance(1150, 1), true); // 0.15 s: frame 1
  assert.equal(head.advance(1200, 2), true); // 0.25 s: frame 2
  assert.deepEqual(shown, [1, 2]);
});

test("playing stops at the last frame, and starts again from the first", () => {
  const { head, shown } = playhead();
  head.start(0);
  assert.equal(head.advance(1000, 1), false);
  assert.equal(head.playing, false);
  assert.equal(head.frame, 3);
  head.start(2000);
  assert.equal(head.frame, 0);
  assert.equal(head.playing, true);
  assert.deepEqual(shown, [3, 0]);
});

test("a stopped playhead does not move", () => {
  const { head, shown } = playhead();
  head.start(0);
  head.stop();
  assert.equal(head.advance(1000, 1), false);
  assert.equal(head.frame, 0);
  assert.deepEqual(shown, []);
});
