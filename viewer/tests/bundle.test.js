// Reading bundles: the golden bundle's files, and the refusals of the .npy reader and the loader.

import assert from "node:assert/strict";
import { test } from "node:test";

import { bundleAddress, loadBundle, loadFeatures, loadMap, loadTrack } from "../src/bundle.js";
import { readFloat32, readHeader, readInt32 } from "../src/npy.js";
import { fetchGolden, fetchMerge, loadRun } from "./golden.js";

/**
 * A .npy file's bytes, written as numpy writes a version-1 header.
 * @param {string} descr The dtype.
 * @param {string} shape The shape, as Python writes the tuple.
 * @param {ArrayBufferView} data The data.
 * @param {string} [fortran] "True" or "False".
 * @returns {ArrayBuffer} The bytes.
 */
function npy(descr, shape, data, fortran = "False") {
  let header = `{'descr': '${descr}', 'fortran_order': ${fortran}, 'shape': (${shape}), }`;
  header += " ".repeat(63 - ((10 + header.length) % 64)) + "\n";
  const out = new Uint8Array(10 + header.length + data.byteLength);
  out.set([0x93, 0x4e, 0x55, 0x4d, 0x50, 0x59, 1, 0, header.length & 0xff, header.length >> 8]);
  out.set(new TextEncoder().encode(header), 10);
  out.set(new Uint8Array(data.buffer, data.byteOffset, data.byteLength), 10 + header.length);
  return out.buffer;
}

test("a .npy file's data is viewed in place after its header", () => {
  const read = readFloat32(npy("<f4", "2, 3", new Float32Array([1, 2, 3, 4, 5, 6.5])));
  assert.deepEqual(read.shape, [2, 3]);
  assert.deepEqual(Array.from(read.data), [1, 2, 3, 4, 5, 6.5]);
  assert.deepEqual(readInt32(npy("<i4", "3,", new Int32Array([-1, 0, 7]))).shape, [3]);
  assert.equal(readHeader(npy("<i4", "0, 4", new Int32Array([]))).offset % 64, 0);
});

test("a .npy file of another dtype, order or format is refused", () => {
  assert.throws(
    () => readFloat32(npy("<f8", "1,", new Float64Array([1]))),
    /expected <f4, got <f8/,
  );
  assert.throws(
    () => readInt32(npy("<i8", "1,", new BigInt64Array([1n]))),
    /expected <i4, got <i8/,
  );
  assert.throws(
    () => readFloat32(npy("<f4", "1, 1", new Float32Array([1]), "True")),
    /Fortran-order/,
  );
  assert.throws(
    () => readHeader(new Uint8Array([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]).buffer),
    /not a .npy file/,
  );
});

test("the golden bundle's frames, images and truth agree in count", async () => {
  const bundle = await loadBundle(fetchGolden);
  const frames = bundle.info.frames;
  assert.equal(bundle.times.length, frames);
  assert.equal(bundle.spans.length, frames * 2);
  assert.equal(bundle.truth?.length, frames * 3);
  assert.equal(bundle.times[0], 0);
  assert.ok(bundle.times.every((t, i) => i === 0 || t > bundle.times[i - 1]));
});

test("each golden backend's arrays agree with its header", async () => {
  const bundle = await loadBundle(fetchGolden);
  for (const [name, info] of Object.entries(bundle.info.backends)) {
    const run = await loadRun(fetchGolden, bundle, name);
    assert.equal(run.poseFrames.length, info.tracked, name);
    assert.equal(run.poseMaps.length, info.tracked, name);
    assert.equal(run.keyframeLives.length / 3, info.made.keyframes, name);
    assert.equal(run.keyframeMaps.length, info.made.keyframes, name);
    assert.equal(run.keyframeParents.length, info.made.keyframes, name);
    // The spanning tree joins keyframes of one map; so does each strong link, the lower first.
    run.keyframeParents.forEach((parent, child) => {
      assert.ok(parent === -1 || run.keyframeMaps[parent] === run.keyframeMaps[child], name);
    });
    for (let l = 0; l < run.keyframeLinks.length; l += 3) {
      const [a, b, weight] = run.keyframeLinks.subarray(l, l + 3);
      assert.ok(a < b && weight >= 1 && run.keyframeMaps[a] === run.keyframeMaps[b], name);
    }
    // The room closes no loop and makes no merge: both are read, empty.
    assert.deepEqual([run.keyframeLoops.length, run.keyframeMerges.length], [0, 0], name);
    assert.equal(run.pointLives.length / 3, info.made.points, name);
    assert.equal(run.pointMaps.length, info.made.points, name);
    // Each map's tracked frames are its poses'.
    for (const map of info.maps) {
      assert.equal(run.poseMaps.filter((m) => m === map.id).length, map.tracked, name);
    }
    assert.deepEqual(
      run.records.frame,
      Array.from({ length: info.frames[1] - info.frames[0] }, (_, i) => info.frames[0] + i),
    );
    assert.equal(run.features !== null, info.features, name);
  }
});

test("the loader refuses another format, a mis-shaped array, an unknown backend or keyframe", async () => {
  /**
   * @param {Record<string, ArrayBuffer>} files Files that replace the golden bundle's.
   * @returns {(path: string) => Promise<ArrayBuffer>} The fetcher.
   */
  const over = (files) => async (path) => files[path] ?? (await fetchGolden(path));
  const info = JSON.parse(new TextDecoder().decode(await fetchGolden("bundle.json")));
  const older = new TextEncoder().encode(JSON.stringify({ ...info, version: 0 })).buffer;
  await assert.rejects(
    loadBundle(over({ "bundle.json": older })),
    /expected slam-it-bundle version 5/,
  );
  const short = npy("<f4", "2,", new Float32Array([0, 1]));
  await assert.rejects(
    loadBundle(over({ "times.npy": short })),
    /times.npy: expected \(6\), got \(2\)/,
  );
  const bundle = await loadBundle(fetchGolden);
  await assert.rejects(loadTrack(fetchGolden, bundle, "java"), /the bundle has no java run/);
  const [name, run] = Object.entries(bundle.info.backends)[0];
  const k = run.made.keyframes;
  const parents = new Int32Array(k).fill(-1);
  parents[k - 1] = k;
  const past = npy("<i4", `${k},`, parents);
  const track = await loadTrack(fetchGolden, bundle, name);
  await assert.rejects(
    loadMap(over({ [`${name}/keyframe_parents.npy`]: past }), track),
    new RegExp(`keyframe_parents.npy: ${k} names none of the run's ${k} keyframes`),
  );
  const link = npy("<i4", "1, 3", new Int32Array([0, k, 120]));
  await assert.rejects(
    loadMap(over({ [`${name}/keyframe_links.npy`]: link }), track),
    /keyframe_links.npy: \d+ names none/,
  );
  // Below the least id: a parent under -1 (none), a link's keyframe under 0.
  parents[k - 1] = -2;
  const below = npy("<i4", `${k},`, parents);
  await assert.rejects(
    loadMap(over({ [`${name}/keyframe_parents.npy`]: below }), track),
    /keyframe_parents.npy: -2 names none/,
  );
  const unnamed = npy("<i4", "1, 3", new Int32Array([-1, 0, 120]));
  await assert.rejects(
    loadMap(over({ [`${name}/keyframe_links.npy`]: unnamed }), track),
    /keyframe_links.npy: -1 names none/,
  );
  // A loop's or a merge's two keyframes are both ids: one past the last, or under 0, names none.
  for (const file of ["keyframe_loops.npy", "keyframe_merges.npy"]) {
    for (const ends of [
      [0, k],
      [-1, 0],
    ]) {
      const pair = npy("<i4", "1, 2", new Int32Array(ends));
      await assert.rejects(
        loadMap(over({ [`${name}/${file}`]: pair }), track),
        new RegExp(`${file}: ${ends[0] < 0 ? -1 : k} names none`),
      );
    }
  }
  // A merge of two of the run's keyframes is read as baked.
  const merge = npy("<i4", "1, 2", new Int32Array([0, k - 1]));
  const merged = await loadMap(over({ [`${name}/keyframe_merges.npy`]: merge }), track);
  assert.deepEqual(Array.from(merged.keyframeMerges), [0, k - 1]);
});

/** A map's files, as the page asks for them: its keyframes', their graph's and its points'. */
const MAP_FILES = [
  "keyframe_lives.npy",
  "keyframe_poses.npy",
  "keyframe_maps.npy",
  "keyframe_parents.npy",
  "keyframe_links.npy",
  "keyframe_loops.npy",
  "keyframe_merges.npy",
  "point_lives.npy",
  "points.npy",
  "point_maps.npy",
];

/**
 * A fetcher of a bundle's files that answers no request until released, and records each asked
 * for: a loader that waited for one file before asking for the next would ask for one alone.
 * @param {(path: string) => Promise<ArrayBuffer>} fetch The bundle's fetcher, which answers.
 * @returns {{fetch: (path: string) => Promise<ArrayBuffer>, asked: string[],
 *     release: () => void}} The fetcher, the paths asked for in order, and what answers them.
 */
function gated(fetch) {
  /** @type {string[]} */
  const asked = [];
  /** @type {(() => void)[]} */
  const waiting = [];
  return {
    asked,
    fetch: (path) => {
      asked.push(path);
      return new Promise((resolve, reject) => {
        waiting.push(() => fetch(path).then(resolve, reject));
      });
    },
    release: () => waiting.splice(0).forEach((answer) => answer()),
  };
}

/** @returns {Promise<void>} Once every callback already due has run. */
const settled = () => new Promise((resolve) => setImmediate(resolve));

test("a map's arrays are all asked for before any arrives, and none of its features", async () => {
  const bundle = await loadBundle(fetchGolden);
  const track = await loadTrack(fetchGolden, bundle, "cpp");
  assert.ok(track.info.features, "the run recorded features, which its map must not fetch");
  const gate = gated(fetchGolden);
  const loading = loadMap(gate.fetch, track);
  await settled();
  assert.deepEqual([...gate.asked].sort(), MAP_FILES.map((file) => `cpp/${file}`).sort());
  gate.release();
  const map = await loading;
  assert.equal(map.keyframeLives.length / 3, track.info.made.keyframes);
  assert.equal(map.pointLives.length / 3, track.info.made.points);
  assert.equal(gate.asked.length, MAP_FILES.length);
});

test("a run's two feature arrays are asked for together; a run that recorded none, never", async () => {
  const bundle = await loadBundle(fetchGolden);
  const track = await loadTrack(fetchGolden, bundle, "cpp");
  const gate = gated(fetchGolden);
  const loading = loadFeatures(gate.fetch, track);
  await settled();
  assert.deepEqual([...gate.asked].sort(), ["cpp/feature_pixels.npy", "cpp/feature_records.npy"]);
  gate.release();
  const features = await loading;
  assert.ok(features !== null);
  assert.ok(features.records.length > 0);
  assert.equal(features.pixels.length / 2, features.records.length / 4);
  // The merge room's run recorded none: its features are null, and nothing is fetched.
  const merge = await loadTrack(fetchMerge, await loadBundle(fetchMerge), "cpp");
  assert.equal(merge.info.features, false);
  const none = gated(fetchMerge);
  assert.equal(await loadFeatures(none.fetch, merge), null);
  assert.deepEqual(none.asked, []);
});

test("features that fail to load, or are mis-shaped, fail their own load alone", async () => {
  const bundle = await loadBundle(fetchGolden);
  /** @param {string} path */
  const failing = async (path) => {
    if (path.includes("/feature_")) {
      throw new Error(`${path}: 503 Service Unavailable`);
    }
    return fetchGolden(path);
  };
  const track = await loadTrack(failing, bundle, "cpp");
  const [map, features] = await Promise.allSettled([
    loadMap(failing, track),
    loadFeatures(failing, track),
  ]);
  assert.equal(map.status, "fulfilled");
  assert.ok(features.status === "rejected");
  assert.match(String(features.reason), /cpp\/feature_\w+\.npy: 503 Service Unavailable/);
  // A pixel short of the records: refused by the features' load, the map's untouched.
  const rows = (await loadFeatures(fetchGolden, track))?.records.length ?? 0;
  const short = npy("<f4", `${rows / 4 - 1}, 2`, new Float32Array(rows / 2 - 2));
  /** @param {string} path */
  const misshaped = async (path) =>
    path === "cpp/feature_pixels.npy" ? short : await fetchGolden(path);
  await assert.rejects(
    loadFeatures(misshaped, track),
    new RegExp(`cpp/feature_pixels.npy: expected \\(${rows / 4}, 2\\)`),
  );
  await loadMap(misshaped, track);
});

/**
 * The golden bundle with fields of its bundle.json changed.
 * @param {object} changed The fields.
 * @returns {(path: string) => Promise<ArrayBuffer>} The fetcher.
 */
function baked(changed) {
  return async (path) => {
    if (path !== "bundle.json") {
      return fetchGolden(path);
    }
    const info = JSON.parse(new TextDecoder().decode(await fetchGolden(path)));
    return new TextEncoder().encode(JSON.stringify({ ...info, ...changed })).buffer;
  };
}

test("the loader refuses other tables, or a backend the page does not know", async () => {
  const info = (await loadBundle(fetchGolden)).info;
  assert.deepEqual(info.fates, ["unmatched", "rejected", "tracked", "made"]);
  const reordered = [...info.fates].reverse();
  await assert.rejects(loadBundle(baked({ fates: reordered })), /its fates are not the page's/);
  await assert.rejects(
    loadBundle(baked({ steps: ["read_seconds"] })),
    /its steps are not the page's/,
  );
  const java = { ...info.backends, java: info.backends.cpp };
  await assert.rejects(loadBundle(baked({ backends: java })), /does not know the backend java/);
});

test("the loader refuses a bundle without the sequence's extent, or one fewer than its frames", async () => {
  const refused = /expected the sequence's extent, \{frames, start, end\}/;
  await assert.rejects(loadBundle(baked({ extent: undefined })), refused);
  await assert.rejects(loadBundle(baked({ extent: { frames: 5, start: 0, end: 1 } })), refused);
  await assert.rejects(loadBundle(baked({ extent: { frames: 40, start: 1, end: 0 } })), refused);
  await assert.rejects(loadBundle(baked({ extent: { frames: 40, start: "0", end: 1 } })), refused);
});

test("a bundle's address is on the page's own site, or refused", () => {
  const page = "https://slam.example/index.html?bundle=x";
  assert.equal(bundleAddress(null, page), "https://slam.example/bundle/");
  assert.equal(bundleAddress("/r/tok/bundle", page), "https://slam.example/r/tok/bundle/");
  assert.equal(bundleAddress("runs/a/", page), "https://slam.example/runs/a/");
  assert.throws(() => bundleAddress("https://elsewhere.example/b/", page), /on this site/);
  assert.throws(() => bundleAddress("//elsewhere.example/b/", page), /on this site/);
});
