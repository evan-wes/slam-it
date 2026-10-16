// The golden bundle the Python tests hold the bake to (tests/fixtures/orchestration/bundle/),
// fetched from disk as the page fetches a bundle over HTTP; and the golden record's found room
// (tests/fixtures/edge/room_runs.json), whose track is found again, as a backend's records.

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

/** The golden record (tests/fixtures/edge/room_runs.json). */
const RECORD = new URL("../../tests/fixtures/edge/room_runs.json", import.meta.url);

/**
 * The found room's TUM RGB-D run, as a backend's records: every frame's state, map, placement
 * and stages' summaries, as the bake writes them (slam_it.orchestration.bundles). Its track
 * fails on the blank frames and is relocalised in map 0 on the frame after them.
 * @returns {Promise<Record<string, unknown[]>>} The columns, a value per frame.
 */
export async function foundRoomRecords() {
  const record = JSON.parse(await readFile(RECORD, "utf8"));
  const run = record.cases["found/tum/rgbd"];
  const summaries = /** @type {Record<string, unknown>[]} */ (run.summaries);
  /** @type {Record<string, unknown[]>} */
  const columns = {
    frame: summaries.map((_, k) => k),
    state: String(run.states).split(" "),
    map: run.maps,
  };
  for (const key of Object.keys(summaries[0])) {
    columns[key] = summaries.map((summary) => summary[key]);
  }
  return columns;
}
