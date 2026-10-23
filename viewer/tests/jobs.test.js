// The upload and status pages' logic: a file's checks, and how a job is described.

import assert from "node:assert/strict";
import { readFile } from "node:fs/promises";
import { test } from "node:test";

import {
  INTRINSICS,
  MAX_BYTES,
  MAX_SECONDS,
  MODES,
  RETENTION_DAYS,
  SHORT_SIDE,
  STAGES,
  UPLOADS,
  describe,
  keptFor,
  refusal,
  sizeWarning,
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

test("a summary is listed, its mode and intrinsics source named", () => {
  const rows = summaryRows({
    frames: 200,
    tracked: 150,
    keyframes: 12,
    points: 900,
    mode: "rgbd",
    intrinsics: "profile",
  });
  assert.deepEqual(rows, [
    ["frames", "200"],
    ["tracked", "150 (75%)"],
    ["keyframes", "12"],
    ["map points", "900"],
    ["mode", MODES.rgbd],
    ["intrinsics", INTRINSICS.profile],
  ]);
  assert.deepEqual(summaryRows({ intrinsics: "elsewhere" }).at(-1), ["intrinsics", "elsewhere"]);
  assert.deepEqual(summaryRows({}).slice(0, 2), [
    ["frames", "0"],
    ["tracked", "0"],
  ]);
  // The maps the run ends with, where a lost track started more than one, or one was reset; not
  // its stretches: a map taken up again is tracked in three stretches, its maps 0, 1, 0.
  assert.deepEqual(summaryRows({ maps: 3, resets: 0 })[2], ["maps", "3"]);
  assert.deepEqual(summaryRows({ maps: 1, resets: 2 })[2], ["maps", "1 kept, 2 reset"]);
  assert.deepEqual(summaryRows({ maps: 2, segment_count: 3, resets: 0 })[2], ["maps", "2"]);
  // A map merged into another: two stretches, one map, no row.
  assert.ok(!summaryRows({ maps: 1, segment_count: 2 }).some(([label]) => label === "maps"));
  // The frames found again in their map, where there were any.
  assert.deepEqual(summaryRows({ relocalizations: 3 })[2], ["frames found again", "3"]);
  assert.ok(!summaryRows({ relocalizations: 0 }).some(([label]) => label === "frames found again"));
});

test("what of the clip was tracked is said, in stretches, and why there are several", () => {
  /** @param {Record<string, unknown>} fields @returns {Record<string, unknown>} */
  const summary = (fields) => ({ frames: 120, tracked: 54, losses: 0, resets: 0, ...fields });
  const one = [[2.5, 4.9]];
  const late = summary({ clip_seconds: 12, tracked_from: 2.5, tracked_to: 4.9, segments: one });
  assert.equal(
    trackingText({ ...late, segment_count: 1 }),
    "Tracked from 2.5 s to 4.9 s of your 12.0 s clip. The first 2.5 s went to finding a first " +
      "map; moving sideways at the start shortens it.",
  );
  const two = summary({
    clip_seconds: 27.1,
    tracked_from: 0,
    tracked_to: 27.1,
    segments: [
      [0, 15.9],
      [17.0, 27.1],
    ],
    segment_count: 2,
    losses: 2,
    resets: 1,
  });
  assert.equal(
    trackingText(two),
    "Tracked 0.0–15.9 s and 17.0–27.1 s of your 27.1 s clip. The track was lost twice, and " +
      "each time a new map was started, so the rest of the clip was still processed; one map " +
      "was lost too soon to keep.",
  );
  const three = {
    ...two,
    segments: [
      [0, 5],
      [6, 9],
      [12, 27.1],
    ],
    segment_count: 3,
    resets: 0,
  };
  assert.ok(trackingText(three).startsWith("Tracked 0.0–5.0 s, 6.0–9.0 s and 12.0–27.1 s of"));
  // More than a few, or more than the summary lists: how many, and from when to when.
  const many = { ...two, segments: [[0, 1]], segment_count: 7, losses: 6, resets: 0 };
  assert.equal(
    trackingText(many),
    "Tracked in 7 stretches, from 0.0 s to 27.1 s of your 27.1 s clip. The track was lost 6 " +
      "times, and each time a new map was started, so the rest of the clip was still processed.",
  );
  // Frames that tracking failed and relocalisation found again in their map: frames, not drops.
  assert.equal(
    trackingText({ ...two, relocalizations: 1 }),
    "Tracked 0.0–15.9 s and 17.0–27.1 s of your 27.1 s clip. Tracking failed on one frame and " +
      "found it again in the same map, by recognising a place it had seen. The track was lost " +
      "twice, and each time a new map was started, so the rest of the clip was still " +
      "processed; one map was lost too soon to keep.",
  );
  assert.match(
    trackingText({ ...two, relocalizations: 3 }),
    /Tracking failed on 3 frames and found each again in the same map/,
  );
  // Frames found in a map kept from earlier, which tracking went on in: said apart.
  const revisit = {
    ...three,
    segments: [
      [0, 9],
      [10, 20],
      [21, 27.1],
    ],
    relocalizations: 1,
  };
  assert.equal(
    trackingText({ ...revisit, taken_up: 1, losses: 1 }),
    "Tracked 0.0–9.0 s, 10.0–20.0 s and 21.0–27.1 s of your 27.1 s clip. Tracking failed on " +
      "one frame and found it in a map kept from earlier in the clip, by recognising a place " +
      "seen there, and went on in the map found. The track was lost once, and each time a new " +
      "map was started, so the rest of the clip was still processed.",
  );
  assert.match(
    trackingText({ ...two, relocalizations: 3, taken_up: 2 }),
    new RegExp(
      "Tracking failed on one frame and found it again in the same map, by recognising a " +
        "place it had seen. It also failed on 2 frames and found each in a map kept from " +
        "earlier in the clip",
    ),
  );
  const whole = summary({ clip_seconds: 10, tracked_from: 0, tracked_to: 10, segment_count: 1 });
  assert.equal(trackingText(whole), "Tracked from 0.0 s to 10.0 s of your 10.0 s clip.");
  const never = summary({ clip_seconds: 8, tracked_from: null, tracked_to: null });
  assert.equal(
    trackingText(never),
    "No first map was made in your 8.0 s clip, so nothing was tracked. Moving sideways at the " +
      "start, with something textured in view, helps make one.",
  );
  const dropped = summary({ clip_seconds: 8, tracked_from: null, losses: 3, resets: 3 });
  assert.equal(
    trackingText(dropped),
    "Maps were started in your 8.0 s clip, but each lost the track too soon to keep, so " +
      "nothing was tracked. Moving more slowly, with something textured in view, helps a map last.",
  );
});

test("an RGB-D run's first map is explained by its depth, not by moving sideways", () => {
  /** @param {Record<string, unknown>} fields @returns {Record<string, unknown>} */
  const summary = (fields) => ({ frames: 120, tracked: 54, losses: 0, mode: "rgbd", ...fields });
  const late = summary({ clip_seconds: 10, tracked_from: 1.2, tracked_to: 10, ran_to: 10 });
  assert.equal(
    trackingText(late),
    "Tracked from 1.2 s to 10.0 s of your 10.0 s clip. The first 1.2 s went to finding a first " +
      "map; starting within a few metres of something textured shortens it.",
  );
  const never = summary({ clip_seconds: 8, tracked_from: null, tracked_to: null, ran_to: 8 });
  assert.equal(
    trackingText(never),
    "No first map was made in your 8.0 s clip, so nothing was tracked: no frame had enough " +
      "textured points with LiDAR depth. Starting within a few metres of something textured " +
      "helps make one.",
  );
});

test("the upload page says what becomes of an upload once the server says, the demo's or not", async () => {
  const page = await readFile(new URL("../upload.html", import.meta.url), "utf8");
  assert.match(page, /<li id="upload-fate" hidden><\/li>/); // no promise before the answer
  assert.equal(
    uploadFate(false),
    "Your upload is deleted as soon as its frames have been imported. It is never shown.",
  );
  assert.equal(
    uploadFate(true),
    "This demo keeps your upload with its result on the computer it runs on, so that it can be " +
      `run again; it is never shown. Both are deleted once ${RETENTION_DAYS} days old, while the ` +
      "demo is running.",
  );
});

test("frames that arrived under the import's short side are warned of, never refused", () => {
  const warning = sizeWarning({ size: [360, 480] });
  assert.ok(warning?.startsWith("Your video arrived at 360x480, under the 480 pixels"));
  assert.ok(warning?.includes("Camera app"));
  assert.equal(sizeWarning({ size: [480, 854] }), null);
  assert.equal(sizeWarning({ size: [640, SHORT_SIDE] }), null);
  assert.equal(sizeWarning({}), null);
});
