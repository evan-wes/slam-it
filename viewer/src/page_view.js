// What every page's DOM code shares: finding the page's elements by id, and remembering a
// viewer's settings in their browser.

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
