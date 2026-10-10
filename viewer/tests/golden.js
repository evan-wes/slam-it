// The golden bundle the Python tests hold the bake to (tests/fixtures/orchestration/bundle/),
// fetched from disk as the page fetches a bundle over HTTP.

import { readFile } from "node:fs/promises";

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
