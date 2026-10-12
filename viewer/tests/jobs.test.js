// The upload and status pages' logic: a file's checks, and how a job is described.

import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { test } from "node:test";

import {
  INTRINSICS,
  MAX_BYTES,
  MAX_SECONDS,
  RETENTION_DAYS,
  STAGES,
  UPLOADS,
  describe,
  keptFor,
  refusal,
  summaryRows,
  trackingText,
  uploadFate,
  viewerLink,
} from "../src/jobs.js";

test("a file within the caps passes; over them, or the wrong kind, says why", () => {
  assert.equal(refusal("video", { name: "a.mov", size: 1 }, 30), null);
  assert.equal(refusal("video", { name: "a.mov", size: MAX_BYTES }, MAX_SECONDS), null);
  assert.equal(refusal("video", { name: "a.mov", size: 1 }, null), null); // length unread
  assert.equal(refusal("video", { name: "a.mov", size: 0 }, 1), "the file is empty");
  assert.equal(refusal("video", { name: "a", size: MAX_BYTES + 1 }, 1), "the file is over 500 MB");
  assert.match(refusal("video", { name: "a.mov", size: 1 }, MAX_SECONDS + 0.1) ?? "", /60 seconds/);
  assert.equal(refusal("stray", { name: "Room.ZIP", size: 1 }, null), null);
  assert.match(refusal("stray", { name: "rgb.mp4", size: 1 }, null) ?? "", /\.zip/);
});

test("the pages' lists match the server's", () => {
  assert.deepEqual(Object.keys(UPLOADS), ["video", "stray"]);
  assert.deepEqual(
    STAGES.map(([state]) => state),
    ["downloading", "importing", "running", "baking", "uploading"],
  );
  assert.deepEqual(Object.keys(INTRINSICS), ["stray-scanner", "profile", "35mm", "nominal"]);
});

/**
 * A status, the given fields over a waiting job's.
 * @param {Partial<import("../src/jobs.js").Status>} fields The fields.
 * @returns {import("../src/jobs.js").Status} The status.
 */
function status(fields) {
  return { state: "waiting", upload: "video", created: 0, reason: null, ...fields };
}

test("each state is described, a stage by its step", () => {
  assert.deepEqual(describe(status({ state: "running" })), {
    headline: "tracking the camera…",
    detail: "step 3 of 5",
    step: 2,
    ended: false,
    failed: false,
  });
  assert.equal(describe(status({})).step, -1);
  assert.equal(describe(status({ state: "queued", ahead: 0 })).detail, "next in line");
  assert.equal(describe(status({ state: "queued", ahead: 1 })).detail, "1 job ahead");
  assert.equal(describe(status({ state: "queued", ahead: 3 })).detail, "3 jobs ahead");
  const done = describe(status({ state: "done" }));
  assert.deepEqual([done.step, done.ended, done.failed], [STAGES.length, true, false]);
  const failed = describe(status({ state: "failed", reason: "not a readable video" }));
  assert.deepEqual(
    [failed.headline, failed.detail, failed.ended, failed.failed],
    ["failed", "not a readable video", true, true],
  );
  assert.equal(describe(status({ state: "expired", reason: "x" })).headline, "expired");
});

test("the viewer's link carries the bundle's address, escaped", () => {
  assert.equal(viewerLink("/r/a-b_c/bundle/"), "/index.html?bundle=%2Fr%2Fa-b_c%2Fbundle%2F");
});

test("how long a result is kept is said to the nearest day, then hour", () => {
  const now = 1000;
  assert.equal(keptFor(now + 7 * 86400 - 5, now), "kept for about 7 more days"); // just done
  assert.equal(keptFor(now + 1.5 * 86400, now), "kept for about 2 more days");
  assert.equal(keptFor(now + 1.4 * 86400, now), "kept for about 34 more hours");
  assert.equal(keptFor(now + 3600, now), "kept for about 1 more hour");
  assert.equal(keptFor(now + 1000, now), "deleted soon");
  assert.equal(keptFor(now - 10, now), "deleted soon");
});

test("a summary is listed, its intrinsics source named", () => {
  const rows = summaryRows({
    frames: 200,
    tracked: 150,
    keyframes: 12,
    points: 900,
    intrinsics: "profile",
  });
  assert.deepEqual(rows, [
    ["frames", "200"],
    ["tracked", "150 (75%)"],
    ["keyframes", "12"],
    ["map points", "900"],
    ["intrinsics", INTRINSICS.profile],
  ]);
  assert.deepEqual(summaryRows({ intrinsics: "elsewhere" }).at(-1), ["intrinsics", "elsewhere"]);
  assert.deepEqual(summaryRows({}).slice(0, 2), [
    ["frames", "0"],
    ["tracked", "0"],
  ]);
});

test("what of the clip was tracked is said, and why a result can be shorter than the clip", () => {
  /** @param {Record<string, unknown>} fields @returns {Record<string, unknown>} */
  const summary = (fields) => ({ frames: 120, tracked: 54, lost: false, ...fields });
  const lost = summary({
    clip_seconds: 12.0,
    tracked_from: 2.5,
    tracked_to: 4.9,
    ran_to: 4.965,
    lost: true,
  });
  assert.equal(
    trackingText(lost),
    "Tracked from 2.5 s to 4.9 s of your 12.0 s clip. Tracking was lost at 5.0 s, which ends a " +
      "run, so the rest was not processed. The first 2.5 s went to finding a first map; moving " +
      "sideways at the start shortens it.",
  );
  const whole = summary({ clip_seconds: 10, tracked_from: 0, tracked_to: 10, ran_to: 10 });
  assert.equal(trackingText(whole), "Tracked from 0.0 s to 10.0 s of your 10.0 s clip.");
  const last = summary({
    clip_seconds: 6,
    tracked_from: 0,
    tracked_to: 5.9,
    ran_to: 6,
    lost: true,
  });
  assert.equal(
    trackingText(last),
    "Tracked from 0.0 s to 5.9 s of your 6.0 s clip. Tracking was lost at 6.0 s, its last frame.",
  );
  const never = summary({ clip_seconds: 8, tracked_from: null, tracked_to: null, ran_to: 8 });
  assert.equal(
    trackingText(never),
    "No first map was made in your 8.0 s clip, so nothing was tracked. Moving sideways at the " +
      "start, with something textured in view, helps make one.",
  );
});

test("the upload page says what becomes of an upload: a deployment's words, or the demo's", async () => {
  const page = await readFile(new URL("../upload.html", import.meta.url), "utf8");
  const said = page
    .match(/<li id="upload-fate">([^<]*)<\/li>/)?.[1]
    .replace(/\s+/g, " ")
    .trim();
  assert.equal(said, uploadFate(false));
  assert.equal(
    uploadFate(true),
    `This demo keeps your upload with its result for ${RETENTION_DAYS} days, then deletes both. ` +
      "It is never shown.",
  );
});
