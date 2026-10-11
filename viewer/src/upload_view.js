// The upload page: checks the file, makes a job with the invite code, sends the file straight
// to storage on the link the server signed, confirms it, and goes to the job's status page.

import { UPLOADS, refusal } from "./jobs.js";
import { element } from "./page_view.js";

/**
 * A video's length, in seconds, if the browser can read it.
 * @param {File} file The video.
 * @returns {Promise<number | null>} Its length, or null.
 */
function videoSeconds(file) {
  return new Promise((resolve) => {
    const video = document.createElement("video");
    const url = URL.createObjectURL(file);
    const done = (/** @type {number | null} */ seconds) => {
      URL.revokeObjectURL(url);
      resolve(seconds);
    };
    video.preload = "metadata";
    video.onloadedmetadata = () => done(Number.isFinite(video.duration) ? video.duration : null);
    video.onerror = () => done(null);
    setTimeout(() => done(null), 5000);
    video.src = url;
  });
}

/**
 * Send a file to a signed link with PUT, reporting how much has gone.
 * @param {string} url The link.
 * @param {File} file The file.
 * @param {(sent: number) => void} progress Called with the fraction sent.
 * @returns {Promise<void>} Once storage has taken it.
 */
function put(url, file, progress) {
  return new Promise((resolve, reject) => {
    const request = new XMLHttpRequest();
    request.open("PUT", url);
    request.upload.onprogress = (event) => {
      if (event.lengthComputable) {
        progress(event.loaded / event.total);
      }
    };
    request.onload = () =>
      request.status >= 200 && request.status < 300
        ? resolve()
        : reject(new Error(`storage refused the upload (${request.status})`));
    request.onerror = () => reject(new Error("the upload was cut off"));
    request.send(file);
  });
}

/**
 * POST JSON to the server; the answer's JSON, or an error with the server's reason.
 * @param {string} path The route.
 * @param {unknown} body The request.
 * @returns {Promise<Record<string, string>>} The answer.
 */
async function post(path, body) {
  const response = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const answer = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(typeof answer.detail === "string" ? answer.detail : response.statusText);
  }
  return answer;
}

function main() {
  const form = element("form", HTMLFormElement);
  const file = element("file", HTMLInputElement);
  const code = element("code", HTMLInputElement);
  const send = element("send", HTMLButtonElement);
  const bar = element("progress", HTMLProgressElement);
  const state = element("state", HTMLElement);
  const message = element("message", HTMLElement);
  const chosen = () => {
    const checked = form.querySelector('input[name="upload"]:checked');
    return checked instanceof HTMLInputElement ? checked.value : "video";
  };
  const fail = (/** @type {string} */ why) => {
    message.textContent = why;
    message.hidden = false;
    send.disabled = false;
    state.textContent = "";
  };

  form.addEventListener("change", (event) => {
    if (event.target instanceof HTMLInputElement && event.target.name === "upload") {
      file.accept = UPLOADS[/** @type {keyof typeof UPLOADS} */ (chosen())];
      file.value = "";
    }
  });
  form.addEventListener("submit", async (event) => {
    event.preventDefault();
    message.hidden = true;
    const picked = file.files?.[0];
    if (picked === undefined) {
      fail("choose a file first");
      return;
    }
    const upload = chosen();
    send.disabled = true;
    state.textContent = "checking the file…";
    const seconds = upload === "video" ? await videoSeconds(picked) : null;
    const why = refusal(upload, picked, seconds);
    if (why !== null) {
      fail(why);
      return;
    }
    try {
      state.textContent = "making the job…";
      const job = await post("/api/jobs", { code: code.value, upload, bytes: picked.size });
      state.textContent = "uploading…";
      bar.hidden = false;
      await put(job.upload_url, picked, (sent) => {
        bar.value = sent;
      });
      await post(`/api/jobs/${job.token}/uploaded`, {});
      location.assign(job.link);
    } catch (error) {
      bar.hidden = true;
      fail(error instanceof Error ? error.message : String(error));
    }
  });
}

main();
