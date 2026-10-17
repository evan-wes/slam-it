// The images' chunks and cache, the 3D view's geometry, and the header's formats.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBundle } from "../src/bundle.js";
import * as format from "../src/format.js";
import {
  FRUSTUM_SEGMENTS,
  MIN_DEPTH,
  NEVER,
  centres,
  frustums,
  keyframeSegments,
  lifeAttributes,
  linkPairs,
  mapAttribute,
  mapDepths,
  mapFrustums,
  treePairs,
  truthSegments,
} from "../src/geometry.js";
import {
  ChunkCache,
  chunkOf,
  chunkPath,
  chunksAround,
  frameBytes,
  onCanvas,
} from "../src/images.js";
import { fetchGolden } from "./golden.js";

test("each golden frame's bytes are a whole JPEG cut from its chunk", async () => {
  const bundle = await loadBundle(fetchGolden);
  const chunk = bundle.info.images.chunk;
  for (let frame = 0; frame < bundle.info.frames; frame++) {
    const bytes = frameBytes(
      await fetchGolden(chunkPath(chunkOf(frame, chunk))),
      bundle.spans,
      frame,
    );
    assert.deepEqual(Array.from(bytes.subarray(0, 2)), [0xff, 0xd8], `frame ${frame} starts`);
    assert.deepEqual(Array.from(bytes.subarray(-2)), [0xff, 0xd9], `frame ${frame} ends`);
  }
});

test("a frame's chunk, and the next one while there is one, are at hand", () => {
  assert.equal(chunkPath(7), "images/chunk_0007.bin");
  assert.deepEqual(chunksAround(150, 3682, 100), [1, 2]);
  assert.deepEqual(chunksAround(3681, 3682, 100), [36]);
  assert.deepEqual(chunksAround(0, 6, 100), [0]);
});

test("the cache fetches each chunk once and drops the least recently used", async () => {
  /** @type {string[]} */
  const fetched = [];
  const cache = new ChunkCache(async (path) => {
    fetched.push(path);
    return new ArrayBuffer(1);
  }, 2);
  await Promise.all([
    cache.get(0),
    cache.get(1),
    cache.get(0),
    cache.get(2),
    cache.get(0),
    cache.get(1),
  ]);
  assert.deepEqual(fetched, [chunkPath(0), chunkPath(1), chunkPath(2), chunkPath(1)]);
});

test("a keypoint is drawn at its pixel's centre, which a canvas puts half a pixel in", () => {
  assert.equal(onCanvas(0), 0.5);
  assert.equal(onCanvas(639), 639.5);
});

test("a frustum reaches from the centre to the image's corners at its depth", () => {
  const identity = new Float32Array([1, 0, 0, 5, 0, 1, 0, 6, 0, 0, 1, 7]);
  const camera = { width: 640, height: 480, intrinsics: [500, 500, 320, 240] };
  const lines = frustums(identity, camera, 2);
  assert.equal(lines.length, FRUSTUM_SEGMENTS * 2 * 3);
  // The top-left corner: the image's edge, half a pixel beyond the first pixel's centre.
  const corner = [5 + ((-0.5 - 320) / 500) * 2, 6 + ((-0.5 - 240) / 500) * 2, 9];
  assert.deepEqual(
    Array.from(lines.subarray(0, 6)),
    Array.from(new Float32Array([5, 6, 7, ...corner])),
  );
  assert.deepEqual(Array.from(centres(identity)), [5, 6, 7]);
});

test("the truth's segments join only frames that both have a truth", () => {
  const truth = new Float32Array([0, 0, 0, 1, 0, 0, NaN, NaN, NaN, 3, 0, 0, 4, 0, 0]);
  const { positions, ends } = truthSegments(truth);
  assert.deepEqual(Array.from(positions), [0, 0, 0, 1, 0, 0, 3, 0, 0, 4, 0, 0]);
  // Each segment by the frame it ends at: frames 0-1 and 3-4, frame 2 having none.
  assert.deepEqual(Array.from(ends), [1, 4]);
});

test("each vertex carries its item's life, a life without an end ending never", () => {
  const { made, removed } = lifeAttributes(new Int32Array([2, -1, -1, 0, 4, 1]), 2);
  assert.deepEqual(Array.from(made), [2, 2, 0, 0]);
  assert.deepEqual(Array.from(removed), [NEVER, NEVER, 4, 4]);
});

test("each vertex carries its item's map", () => {
  assert.deepEqual(Array.from(mapAttribute(new Int32Array([0, 2]), 3)), [0, 0, 0, 2, 2, 2]);
});

test("the graph's lines join keyframes' centres, alive while both keyframes are", () => {
  // Three keyframes of map 0 at x = 0, 1 and 2; the second culled at frame 6, the third made at 4.
  const poses = new Float32Array(36);
  for (let k = 0; k < 3; k++) {
    poses.set([1, 0, 0, k, 0, 1, 0, 0, 0, 0, 1, 0], k * 12);
  }
  const lives = new Int32Array([0, -1, -1, 2, 6, 0, 4, -1, -1]);
  const maps = new Int32Array([0, 0, 0]);
  const tree = treePairs(new Int32Array([-1, 0, 1]));
  assert.deepEqual(Array.from(tree), [1, 0, 2, 1]);
  const links = linkPairs(new Int32Array([0, 2, 120]));
  assert.deepEqual(Array.from(links), [0, 2]);
  const lines = keyframeSegments(poses, lives, maps, tree);
  assert.deepEqual(Array.from(lines.positions), [1, 0, 0, 0, 0, 0, 2, 0, 0, 1, 0, 0]);
  // 1-0 lives from 2 to 6, as keyframe 1; 2-1 from 4, until keyframe 1 goes at 6.
  assert.deepEqual(Array.from(lines.lives), [2, 6, 0, 4, 6, 0]);
  assert.deepEqual(Array.from(lines.maps), [0, 0]);
  const strong = keyframeSegments(poses, lives, maps, links);
  assert.deepEqual(Array.from(strong.lives), [4, -1, -1]);
});

test("each map's frustums are as deep as a share of its own trajectory's extent", () => {
  // Map 0 spans 4 m along x, map 1 a millimetre: its frustums are the shallowest.
  const path = new Float32Array([0, 0, 0, 4, 0, 0, 0, 0, 0, 0.001, 0, 0]);
  const depths = mapDepths(path, new Int32Array([0, 0, 1, 1]));
  assert.deepEqual(
    [...depths],
    [
      [0, 4 / 40],
      [1, MIN_DEPTH],
    ],
  );
  const camera = { width: 2, height: 2, intrinsics: [1, 1, 0.5, 0.5] };
  const pose = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0];
  const poses = new Float32Array([...pose, ...pose, ...pose]);
  const lines = mapFrustums(poses, new Int32Array([0, 1, 5]), camera, depths, 0.3);
  const each = FRUSTUM_SEGMENTS * 2 * 3;
  for (const [k, depth] of [0.1, MIN_DEPTH, 0.3].entries()) {
    const alone = frustums(poses.subarray(k * 12, k * 12 + 12), camera, depth);
    assert.deepEqual(Array.from(lines.subarray(k * each, (k + 1) * each)), Array.from(alone));
  }
});

test("the header's numbers read in their units", () => {
  assert.equal(format.length(0.0425), "4.25 cm");
  assert.equal(format.length(1.5), "1.50 m");
  assert.equal(format.duration(0.00975), "9.8 ms");
  assert.equal(format.size(27_594_752), "26.3 MiB");
  assert.equal(format.size(null), "–");
  assert.equal(format.angle(Math.PI / 180), "1.00°");
});
