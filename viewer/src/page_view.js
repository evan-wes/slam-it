// What every page's DOM code shares: finding the page's elements by id.

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
