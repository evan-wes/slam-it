// The loops the chosen run closed, on the page: a tick at each on the timeline, under the
// backends' rows; buttons that jump to the loop before or after the frame shown; and around each
// loop, the "seen before" inset: the earlier pass at the same place, in the Frame panel's corner,
// playing as the frames do, larger on a click, and off on its toggle (remembered).

import { FrameImages } from "./image_view.js";
import { insetAt, insets, loopBeside, loopHint, loopsOf, seenText } from "./loops.js";
import { remember, remembered } from "./page_view.js";

/** The height of the loops' row under the backends' rows on the timeline, in pixels. */
const LOOP_ROW = 6;

/** Where the browser remembers whether the inset is on. */
const SEEN_KEY = "slam-it.seen-before";

/** The inset's chunks kept: its frame's and the next. */
const INSET_CHUNKS = 2;

/** The inset's decoded frames kept: a couple of seconds about its frame. */
const INSET_DECODED = 60;

/**
 * The page's parts the loops are shown in.
 * @typedef {object} LoopParts
 * @property {HTMLElement} spans The timeline's rows, under the frame slider.
 * @property {number} top The height of the backends' rows in it, in pixels: the loops' row goes
 *     under them.
 * @property {HTMLButtonElement} previous The button to the loop before the frame shown.
 * @property {HTMLButtonElement} next The button to the loop after it.
 * @property {HTMLButtonElement} toggle The inset's toggle.
 * @property {HTMLButtonElement} inset The inset, which a click enlarges.
 * @property {HTMLCanvasElement} picture Where the inset's frame is drawn.
 * @property {HTMLElement} label The inset's label.
 */

/** The chosen run's loops: their ticks, the buttons between them, and the inset. */
export class LoopView {
  /**
   * @param {LoopParts} parts The page's parts.
   * @param {import("./bundle.js").Bundle} bundle The bundle.
   * @param {import("./bundle.js").Fetch} fetch How to fetch one of its files.
   * @param {(frame: number) => void} seek Show a frame.
   * @param {(error: unknown) => void} report Show what went wrong drawing the inset.
   */
  constructor(parts, bundle, fetch, seek, report) {
    this.parts = parts;
    this.bundle = bundle;
    this.images = new FrameImages(bundle, fetch, INSET_CHUNKS, INSET_DECODED);
    /** @type {import("./loops.js").Loop[]} */
    this.loops = [];
    /** @type {import("./loops.js").Inset[]} */
    this.windows = [];
    this.frame = 0;
    this.on = remembered(SEEN_KEY) !== "off";
    this.drawn = -1; // the earlier frame last asked for: an older one's late decode is not drawn
    parts.picture.width = bundle.info.camera.width;
    parts.picture.height = bundle.info.camera.height;
    parts.toggle.setAttribute("aria-pressed", String(this.on));
    parts.toggle.addEventListener("click", () => {
      this.on = !this.on;
      parts.toggle.setAttribute("aria-pressed", String(this.on));
      remember(SEEN_KEY, this.on ? "on" : "off");
      this.showInset().catch(report);
    });
    parts.inset.addEventListener("click", () => {
      const large = parts.inset.getAttribute("aria-pressed") !== "true";
      parts.inset.setAttribute("aria-pressed", String(large));
    });
    for (const [button, way] of /** @type {[HTMLButtonElement, -1 | 1][]} */ ([
      [parts.previous, -1],
      [parts.next, 1],
    ])) {
      button.addEventListener("click", () => {
        const loop = loopBeside(this.loops, this.frame, way);
        if (loop !== null) {
          seek(loop.frame);
        }
      });
    }
  }

  /**
   * Show a run's loops: its ticks and the inset's windows, worked out once.
   * @param {import("./bundle.js").Backend} run The run.
   * @throws {Error} If its records name a loop its map does not hold (`loopsOf`): it is then
   *     shown with none.
   */
  setRun(run) {
    this.loops = [];
    this.windows = [];
    try {
      this.loops = loopsOf(run);
      this.windows = insets(this.loops, this.bundle.times, run);
    } finally {
      this.mark();
    }
  }

  /** Tick each loop on the timeline, in a row of its own, and offer the inset if there are any. */
  mark() {
    const { spans, top, toggle } = this.parts;
    const frames = this.bundle.info.frames;
    for (const old of spans.querySelectorAll(".loop")) {
      old.remove();
    }
    spans.append(
      ...this.loops.map((loop) => {
        const tick = document.createElement("div");
        tick.className = "span loop";
        tick.title = loopHint(loop, this.bundle.times);
        tick.style.left = `${(100 * loop.frame) / frames}%`;
        tick.style.width = `${100 / frames}%`;
        tick.style.top = `${top}px`;
        return tick;
      }),
    );
    spans.style.height = `${top + (this.loops.length > 0 ? LOOP_ROW : 0)}px`;
    toggle.hidden = this.loops.length === 0;
  }

  /**
   * Show a frame: the buttons to the loops beside it, and the inset if it is in a loop's window.
   * @param {number} frame The frame.
   * @returns {Promise<void>} Once the inset is drawn.
   */
  show(frame) {
    this.frame = frame;
    this.parts.previous.disabled = loopBeside(this.loops, frame, -1) === null;
    this.parts.next.disabled = loopBeside(this.loops, frame, 1) === null;
    return this.showInset();
  }

  /**
   * Draw the inset at the frame shown: the earlier frame at its place, labelled; hidden when it
   * is off or the frame is in no loop's window. Its frames load apart from the main frame's.
   * @returns {Promise<void>} Once drawn.
   */
  async showInset() {
    const { inset, picture, label } = this.parts;
    const shown = this.on ? insetAt(this.windows, this.frame) : null;
    if (shown === null) {
      if (!inset.hidden) {
        inset.hidden = true;
        this.drawn = -1;
        /** @type {CanvasRenderingContext2D} */ (picture.getContext("2d")).clearRect(
          0,
          0,
          picture.width,
          picture.height,
        );
      }
      return;
    }
    inset.hidden = false;
    label.textContent = seenText(shown, this.frame, this.bundle.times);
    const earlier = shown.earlier;
    if (earlier === this.drawn) {
      return;
    }
    this.drawn = earlier;
    this.images.ahead(earlier);
    const bitmap = await this.images.image(earlier);
    if (this.drawn === earlier) {
      const context = /** @type {CanvasRenderingContext2D} */ (picture.getContext("2d"));
      context.drawImage(bitmap, 0, 0, picture.width, picture.height);
    }
  }
}
