// What every page's DOM code shares: finding the page's elements by id, remembering a viewer's
// settings in their browser, and a panel's loading mark.

/**
 * An element of the page, by id.
 * @template {HTMLElement} T
 * @param {string} id Its id.
 * @param {new () => T} type Its type.
 * @returns {T} The element.
 */
export function element(id, type) {
  const found = document.getElementById(id);
  if (!(found instanceof type)) {
    throw new Error(`the page has no ${id}`);
  }
  return found;
}

/**
 * A setting the page remembered in this browser.
 * @param {string} key Its key.
 * @returns {string | null} Its value; null if none was remembered, or the browser keeps no site
 *     data for the page (a private window, data blocked or cleared), which it then works without.
 */
export function remembered(key) {
  try {
    return localStorage.getItem(key);
  } catch {
    return null; // storage refused: the page works with its defaults
  }
}

/**
 * Remember a setting in this browser, where it keeps site data for the page.
 * @param {string} key Its key.
 * @param {string} value Its value.
 */
export function remember(key, value) {
  try {
    localStorage.setItem(key, value);
  } catch {
    // storage refused or full: the setting holds until the page is left, and is not remembered
  }
}

/**
 * A panel's loading mark: its `.loading` element, a spinner over the panel's box or a small one
 * in its corner (its `data-mark`, styled by app.css: still text under reduced motion); its
 * visually hidden status (`role="status"`), which says "Loading…" while the mark shows; and the
 * panel, busy (`aria-busy`) meanwhile.
 */
export class LoadingMark {
  /**
   * @param {HTMLElement} panel The panel: its first `.loading` and `[role="status"]` are its
   *     mark's.
   * @throws {Error} If the panel has either missing.
   */
  constructor(panel) {
    const mark = panel.querySelector(".loading");
    const status = panel.querySelector('[role="status"]');
    if (!(mark instanceof HTMLElement) || !(status instanceof HTMLElement)) {
      throw new Error(`the panel ${panel.id} has no loading mark`);
    }
    this.panel = panel;
    this.mark = mark;
    this.status = status;
  }

  /**
   * Show a mark, or none.
   * @param {import("./loading.js").Mark} mark The mark.
   */
  set(mark) {
    this.mark.dataset.mark = mark;
    this.panel.setAttribute("aria-busy", String(mark !== "none"));
    const text = mark === "none" ? "" : "Loading…";
    if (this.status.textContent !== text) {
      this.status.textContent = text;
    }
  }
}

/**
 * Take down every panel's loading mark: on a failure that stops the page, its message says what
 * failed.
 */
export function stopLoading() {
  for (const panel of document.querySelectorAll(".panel[aria-busy]")) {
    if (panel instanceof HTMLElement) {
      new LoadingMark(panel).set("none");
    }
  }
}
