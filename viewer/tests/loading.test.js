// What the page shows while its parts load: the frame panel's spinner and its stale frame, and
// the map panel's spinner and corner mark.

import assert from "node:assert/strict";
import { test } from "node:test";

import { LATE, imageLoading, mapLoading } from "../src/loading.js";

test("the frame panel's spinner shows until a first frame is drawn, at once", () => {
  for (const shown of [-1, 0, 120]) {
    assert.deepEqual(imageLoading({ drawn: -1, shown, failed: false, waited: 0 }), {
      mark: "spinner",
      stale: false,
      recheck: null,
    });
  }
});

test("the frame drawn stands in for the frame shown undimmed until the wait is late", () => {
  const waiting = { drawn: 40, shown: 2000, failed: false };
  // A decode of a frame whose chunk is in: no spinner, no dimming, a look again when it would
  // be late.
  assert.deepEqual(imageLoading({ ...waiting, waited: 10 }), {
    mark: "none",
    stale: false,
    recheck: LATE - 10,
  });
  // A chunk still on its way: the spinner, the frame drawn dimmed.
  for (const waited of [LATE, 10_000]) {
    assert.deepEqual(imageLoading({ ...waiting, waited }), {
      mark: "spinner",
      stale: true,
      recheck: null,
    });
  }
});

test("the frame panel shows no loading once its frame is drawn, or its image failed", () => {
  const none = { mark: "none", stale: false, recheck: null };
  assert.deepEqual(imageLoading({ drawn: 7, shown: 7, failed: false, waited: 10_000 }), none);
  // A failure: the page's message says why, and nothing stays marked as loading.
  for (const drawn of [-1, 7]) {
    assert.deepEqual(imageLoading({ drawn, shown: 9, failed: true, waited: 10_000 }), none);
  }
});

test("the map panel's spinner shows until a track is, then its corner mark until its map is in", () => {
  for (const map of /** @type {const} */ (["loading", "loaded", "failed"])) {
    assert.equal(mapLoading(false, map), "spinner");
  }
  assert.equal(mapLoading(true, "loading"), "corner");
  assert.equal(mapLoading(true, "loaded"), "none");
  assert.equal(mapLoading(true, "failed"), "none"); // the page's message says why
});
