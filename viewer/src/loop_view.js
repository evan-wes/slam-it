// The loops the chosen run closed, on the page: a tick at each on the timeline, under the
// backends' rows, and buttons that jump to the loop before or after the frame shown.

import { loopBeside, loopHint, loopsOf } from "./loops.js";

/** The height of the loops' row under the backends' rows on the timeline, in pixels. */
const LOOP_ROW = 6;

/**
 * The page's parts the loops are shown in.
 * @typedef {object} LoopParts
 * @property {HTMLElement} spans The timeline's rows, under the frame slider.
 * @property {number} top The height of the backends' rows in it, in pixels: the loops' row goes
 *     under them.
 * @property {HTMLButtonElement} previous The button to the loop before the frame shown.
 * @property {HTMLButtonElement} next The button to the loop after it.
 */

/** The chosen run's loops: their ticks, and the buttons between them. */
export class LoopView {
  /**
   * @param {LoopParts} parts The page's parts.
   * @param {import("./bundle.js").Bundle} bundle The bundle.
   * @param {(frame: number) => void} seek Show a frame.
   */
  constructor(parts, bundle, seek) {
    this.parts = parts;
    this.bundle = bundle;
    /** @type {import("./loops.js").Loop[]} */
    this.loops = [];
    this.frame = 0;
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
   * Show a run's loops: their ticks, worked out once.
   * @param {import("./bundle.js").Backend} run The run.
   * @throws {Error} If its records name a loop its map does not hold (`loopsOf`): it is then
   *     shown with none.
   */
  setRun(run) {
    this.loops = [];
    try {
      this.loops = loopsOf(run);
    } finally {
      this.mark();
    }
  }

  /** Tick each loop on the timeline, in a row of its own. */
  mark() {
    const { spans, top } = this.parts;
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
  }

  /**
   * Show a frame: the buttons to the loops beside it, each disabled if there is none.
   * @param {number} frame The frame.
   */
  show(frame) {
    this.frame = frame;
    this.parts.previous.disabled = loopBeside(this.loops, frame, -1) === null;
    this.parts.next.disabled = loopBeside(this.loops, frame, 1) === null;
  }
}
