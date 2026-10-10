// The images' chunks and cache, the 3D view's geometry, and the header's formats.

import assert from "node:assert/strict";
import { test } from "node:test";

import { loadBundle } from "../src/bundle.js";
import * as format from "../src/format.js";
import { FRUSTUM_SEGMENTS, NEVER, centres, frustums, lifeAttributes, truthSegments } from "../src/geometry.js";
import { ChunkCache, chunkOf, chunkPath, chunksAround, frameBytes } from "../src/images.js";
import { fetchGolden } from "./golden.js";

test("each golden frame's bytes are a whole JPEG cut from its chunk", async () => {
  const bundle = await loadBundle(fetchGolden);
  const chunk = bundle.info.images.chunk;
  for (let frame = 0; frame < bundle.info.frames; frame++) {
    const bytes = frameBytes(await fetchGolden(chunkPath(chunkOf(frame, chunk))), bundle.spans, frame);
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
  await Promise.all([cache.get(0), cache.get(1), cache.get(0), cache.get(2), cache.get(0), cache.get(1)]);
  assert.deepEqual(fetched, [chunkPath(0), chunkPath(1), chunkPath(2), chunkPath(1)]);
});

test("a frustum reaches from the centre to the image's corners at its depth", () => {
  const identity = new Float32Array([1, 0, 0, 5, 0, 1, 0, 6, 0, 0, 1, 7]);
  const camera = { width: 640, height: 480, intrinsics: [500, 500, 320, 240] };
  const lines = frustums(identity, camera, 2);
  assert.equal(lines.length, FRUSTUM_SEGMENTS * 2 * 3);
  assert.deepEqual(Array.from(lines.subarray(0, 6)), Array.from(new Float32Array([5, 6, 7, 5 - 1.28, 6 - 0.96, 9])));
  assert.deepEqual(Array.from(centres(identity)), [5, 6, 7]);
});

test("the truth's segments join only frames that both have a truth", () => {
  const truth = new Float32Array([0, 0, 0, 1, 0, 0, NaN, NaN, NaN, 3, 0, 0, 4, 0, 0]);
  assert.deepEqual(Array.from(truthSegments(truth)), [0, 0, 0, 1, 0, 0, 3, 0, 0, 4, 0, 0]);
});

test("each vertex carries its item's life, a life without an end ending never", () => {
  const { made, removed } = lifeAttributes(new Int32Array([2, -1, -1, 0, 4, 1]), 2);
  assert.deepEqual(Array.from(made), [2, 2, 0, 0]);
  assert.deepEqual(Array.from(removed), [NEVER, NEVER, 4, 4]);
});

test("the header's numbers read in their units", () => {
  assert.equal(format.length(0.0425), "4.25 cm");
  assert.equal(format.length(1.5), "1.50 m");
  assert.equal(format.duration(0.00975), "9.8 ms");
  assert.equal(format.size(27_594_752), "26.3 MiB");
  assert.equal(format.size(null), "–");
  assert.equal(format.angle(Math.PI / 180), "1.00°");
});
