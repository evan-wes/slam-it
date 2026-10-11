// The status page, at /r/<token>: asks how the job is going every few seconds until it ends,
// then shows its summary and the link to the run viewer, or why it did not finish.

import { STAGES, describe, keptFor, summaryRows, viewerLink } from "./jobs.js";
import { element } from "./page_view.js";

/** How often the page asks, in milliseconds. */
const POLL = 2000;

/**
 * Show a job's status.
 * @param {import("./jobs.js").Status} status The status.
 * @returns {boolean} Whether the job has ended.
 */
function show(status) {
  const said = describe(status);
  element("headline", HTMLElement).textContent = said.headline;
  const detail = element("detail", HTMLElement);
  detail.textContent = said.detail;
  detail.classList.toggle("fail-text", said.failed);
  const stages = element("stages", HTMLOListElement);
  stages.replaceChildren(
    ...STAGES.map(([, words], k) => {
      const item = document.createElement("li");
      item.textContent = words;
      item.className = k < said.step ? "past" : k === said.step ? "now" : "";
      return item;
    }),
  );
  stages.hidden = said.failed;
  if (status.state === "done" && status.summary && status.bundle) {
    const summary = element("summary", HTMLElement);
    summary.replaceChildren(
      ...summaryRows(status.summary).map(([label, value]) => {
        const row = document.createElement("div");
        const term = document.createElement("dt");
        const data = document.createElement("dd");
        term.textContent = label;
        data.textContent = value;
        row.append(term, data);
        return row;
      }),
    );
    summary.hidden = false;
    element("viewer", HTMLAnchorElement).href = viewerLink(status.bundle);
    element("open", HTMLElement).hidden = false;
  }
  if (status.deleted !== undefined) {
    element("kept", HTMLElement).textContent = keptFor(status.deleted, Date.now() / 1000);
  }
  return said.ended;
}

async function main() {
  const token = location.pathname.split("/")[2] ?? "";
  for (;;) {
    const response = await fetch(`/api/jobs/${encodeURIComponent(token)}`).catch(() => null);
    if (response !== null && response.status === 404) {
      element("headline", HTMLElement).textContent = "no such run";
      element("detail", HTMLElement).textContent =
        "its link is wrong, or the run has been deleted (results are kept 7 days)";
      return;
    }
    if (response !== null && response.ok && show(await response.json())) {
      return;
    }
    await new Promise((resolve) => setTimeout(resolve, POLL));
  }
}

void main();
