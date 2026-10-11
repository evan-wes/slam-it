// The upload and status pages' logic: checking a file before it is sent, and saying how a job
// is going. No page, no network: upload_view.js and status_view.js do those.

/** The largest upload, in bytes (slam_it_all_server.config.MAX_BYTES). */
export const MAX_BYTES = 500e6;

/** The longest video, in seconds (slam_it_all_server.config.MAX_SECONDS). */
export const MAX_SECONDS = 60;

/** What may be uploaded, and the files each one is (an `accept` attribute's value). */
export const UPLOADS = {
  video: "video/*",
  stray: ".zip,application/zip",
};

/** The worker's stages, in order (slam_it_all_server.jobs.STAGES), each as the page says it. */
export const STAGES = [
  ["downloading", "fetching the upload"],
  ["importing", "importing the frames"],
  ["running", "tracking the camera"],
  ["baking", "baking the result"],
  ["uploading", "saving the result"],
];

/** Where a run's intrinsics came from (slam_it_all.orchestration.worker), as the page says it. */
export const INTRINSICS = {
  "stray-scanner": "Stray Scanner's camera matrix: exact",
  profile: "a Gyroflow lens profile of this phone",
  "35mm": "the video's 35mm-equivalent focal length",
  nominal: "a typical phone's field of view: a guess",
};

/**
 * Why a file cannot be uploaded, or null if it can.
 * @param {string} upload What it is: "video" or "stray".
 * @param {{name: string, size: number}} file The file.
 * @param {number | null} seconds A video's length, if the browser could read it.
 * @returns {string | null} The reason, fit to show.
 */
export function refusal(upload, file, seconds) {
  if (file.size <= 0) {
    return "the file is empty";
  }
  if (file.size > MAX_BYTES) {
    return `the file is over ${MAX_BYTES / 1e6} MB`;
  }
  if (upload === "stray" && !file.name.toLowerCase().endsWith(".zip")) {
    return "a Stray Scanner recording is a .zip, from its Share button";
  }
  if (upload === "video" && seconds !== null && seconds > MAX_SECONDS) {
    return `the video is over ${MAX_SECONDS} seconds: trim it first`;
  }
  return null;
}

/**
 * A job's status, as the server gives it (GET /api/jobs/<token>).
 * @typedef {object} Status
 * @property {string} state Where the job is.
 * @property {string} upload What was uploaded.
 * @property {number} created When it was made, in seconds since the epoch.
 * @property {string | null} reason Why it failed or expired.
 * @property {number} [ahead] For a queued job, how many go first.
 * @property {number} [deleted] For an ended job, when it is deleted, in seconds since the epoch.
 * @property {Record<string, unknown>} [summary] For a done job, the worker's summary.
 * @property {string} [bundle] For a done job, its bundle's address.
 */

/**
 * What the status page says of a job.
 * @param {Status} status The job's status.
 * @returns {{headline: string, detail: string, step: number, ended: boolean, failed: boolean}}
 *     A headline and a line under it; the stage reached, -1 before the first and the number of
 *     stages once done; whether the job has ended, and whether badly.
 */
export function describe(status) {
  const step = STAGES.findIndex(([state]) => state === status.state);
  if (step >= 0) {
    return {
      headline: `${STAGES[step][1]}…`,
      detail: `step ${step + 1} of ${STAGES.length}`,
      step,
      ended: false,
      failed: false,
    };
  }
  switch (status.state) {
    case "waiting":
      return {
        headline: "waiting for the upload…",
        detail: "if the upload was stopped, start again from the upload page",
        step: -1,
        ended: false,
        failed: false,
      };
    case "queued": {
      const ahead = status.ahead ?? 0;
      return {
        headline: "queued…",
        detail: ahead === 0 ? "next in line" : `${ahead} ${ahead === 1 ? "job" : "jobs"} ahead`,
        step: -1,
        ended: false,
        failed: false,
      };
    }
    case "done":
      return { headline: "done", detail: "", step: STAGES.length, ended: true, failed: false };
    default:
      return {
        headline: status.state === "expired" ? "expired" : "failed",
        detail: status.reason ?? "",
        step: -1,
        ended: true,
        failed: true,
      };
  }
}

/**
 * The run viewer's address for a done job's bundle.
 * @param {string} bundle The bundle's address.
 * @returns {string} The viewer's.
 */
export function viewerLink(bundle) {
  return `/index.html?bundle=${encodeURIComponent(bundle)}`;
}

/**
 * How long until an ended job is deleted, as the page says it, to the nearest day or hour.
 * @param {number} deleted When it is deleted, in seconds since the epoch.
 * @param {number} now The time, in seconds since the epoch.
 * @returns {string} The words.
 */
export function keptFor(deleted, now) {
  const days = Math.round((deleted - now) / 86400);
  if (days >= 2) {
    return `kept for about ${days} more days`;
  }
  const hours = Math.round((deleted - now) / 3600);
  if (hours >= 1) {
    return `kept for about ${hours} more ${hours === 1 ? "hour" : "hours"}`;
  }
  return "deleted soon";
}

/**
 * A done job's summary, as label and value pairs.
 * @param {Record<string, unknown>} summary The worker's summary.
 * @returns {[string, string][]} The pairs, in the page's order.
 */
export function summaryRows(summary) {
  /** @type {[string, string][]} */
  const rows = [];
  const frames = Number(summary.frames ?? 0);
  const tracked = Number(summary.tracked ?? 0);
  rows.push(["frames", String(frames)]);
  rows.push([
    "tracked",
    frames > 0 ? `${tracked} (${Math.round((100 * tracked) / frames)}%)` : "0",
  ]);
  rows.push(["keyframes", String(summary.keyframes ?? 0)]);
  rows.push(["map points", String(summary.points ?? 0)]);
  const source = String(summary.intrinsics ?? "");
  rows.push(["intrinsics", INTRINSICS[/** @type {keyof typeof INTRINSICS} */ (source)] ?? source]);
  return rows;
}
