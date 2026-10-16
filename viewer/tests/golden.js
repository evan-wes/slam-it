// The golden bundles the Python tests hold the bake to (tests/fixtures/orchestration/bundle/
// and found_bundle/), fetched from disk as the page fetches a bundle over HTTP: the room run
// briefly by every backend, and the found room, whose track is found again.

import { readFile } from "node:fs/promises";

import { loadBundle, loadTrack } from "../src/bundle.js";

const GOLDEN = new URL("../../tests/fixtures/orchestration/bundle/", import.meta.url);

/**
 * Fetch one of the golden bundle's files.
 * @param {string} path Its path in the bundle.
 * @returns {Promise<ArrayBuffer>} Its bytes, in a buffer of their own.
 */
export async function fetchGolden(path) {
  const bytes = await readFile(new URL(path, GOLDEN));
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}

const FOUND = new URL("../../tests/fixtures/orchestration/found_bundle/", import.meta.url);

/**
 * Fetch one of the found bundle's files.
 * @param {string} path Its path in the bundle.
 * @returns {Promise<ArrayBuffer>} Its bytes, in a buffer of their own.
 */
async function fetchFound(path) {
  const bytes = await readFile(new URL(path, FOUND));
  return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
}

/**
 * The found room's TUM RGB-D run, as the found bundle holds it (Python's run, baked by
 * tests/fixtures/orchestration/generate_bundle.py): every frame's state, map, placement and
 * stages' summaries. Its track fails on the blank frames and is relocalised in map 0 on the
 * frame after them.
 * @returns {Promise<import("../src/bundle.js").Records>} The columns, a value per frame.
 */
export async function foundRoomRecords() {
  const bundle = await loadBundle(fetchFound);
  return (await loadTrack(fetchFound, bundle, "python")).records;
}
