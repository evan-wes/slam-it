// The golden bundles the Python tests hold the bake to (tests/fixtures/orchestration/bundle/,
// found_bundle/, merge_bundle/ and revisit_bundle/), fetched from disk as the page fetches a
// bundle over HTTP: the room run briefly by every backend; the found room, whose track is found
// again; the merge room, whose second map is merged into its first; and the revisit room, whose
// first map is taken up again. The last two hold no frame's JPEG, only the images' spans
// (tests/fixtures/orchestration/generate_bundle.py).

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

/**
 * A fetcher of one of the golden bundles' files.
 * @param {string} folder The bundle's folder under tests/fixtures/orchestration/, ending in "/".
 * @returns {(path: string) => Promise<ArrayBuffer>} Fetches a file by its path in the bundle,
 *     its bytes in a buffer of their own.
 */
function fetcher(folder) {
  const base = new URL(`../../tests/fixtures/orchestration/${folder}`, import.meta.url);
  return async (path) => {
    const bytes = await readFile(new URL(path, base));
    return bytes.buffer.slice(bytes.byteOffset, bytes.byteOffset + bytes.byteLength);
  };
}

/** Fetch one of the merge bundle's files: the merge room run by C++, its two maps merged. */
export const fetchMerge = fetcher("merge_bundle/");

/** Fetch one of the revisit bundle's files: the revisit room run by C++, map 0 taken up again. */
export const fetchRevisit = fetcher("revisit_bundle/");
