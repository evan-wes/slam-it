// The loops the chosen run closed, on the page: a tick at each on the timeline, under the
// backends' rows; buttons that jump to the loop before or after the frame shown; around each
// loop, the "seen before" inset: the earlier pass at the same place, in the Frame panel's corner,
// playing as the frames do, larger on a click, and off on its toggle (remembered); and the loop
// effect, when the playhead arrives at a loop: the loop drawn bright over the map and the inset
// playing the loop's beginning, then fading, on a clock of its own.

import { FrameImages } from "./image_view.js";
import {
  effectChange,
  effectFrame,
  effectOpacity,
  effectOver,
  insetAt,
  insets,
  loopBeside,
  loopHint,
  loopsOf,
  seenText,
} from "./loops.js";
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

/**
 * A loop effect running: its loop, its clock's start (the time of the first animation frame
 * with the loop on screen, in milliseconds; null before it), its next animation frame's request,
 * its opacity now, and the earlier frame the inset plays now.
 * @typedef {{loop: import("./loops.js").Loop, start: number | null, request: number,
 *     opacity: number, earlier: number}} Effect
 */

/** The chosen run's loops: their ticks, the buttons between them, the inset and the effect. */
export class LoopView {
  /**
   * @param {LoopParts} parts The page's parts.
   * @param {import("./bundle.js").Bundle} bundle The bundle.
   * @param {import("./bundle.js").Fetch} fetch How to fetch one of its files.
   * @param {import("./scene_view.js").SceneView} scene The 3D view, which draws a loop's effect.
   * @param {(frame: number) => void} seek Show a frame.
   * @param {(error: unknown) => void} report Show what went wrong drawing the inset.
   */
  constructor(parts, bundle, fetch, scene, seek, report) {
    this.parts = parts;
    this.bundle = bundle;
    this.scene = scene;
    this.report = report;
    this.images = new FrameImages(bundle, fetch, INSET_CHUNKS, INSET_DECODED);
    /** @type {Effect | null} The loop effect running, if any. */
    this.effect = null;
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
    this.endEffect();
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
   * Follow the playhead to a frame, before the frame is shown: arriving at a loop sets off its
   * effect, over any running; a seek elsewhere ends a running one (`effectChange`).
   * @param {number} to The frame.
   * @param {boolean} played Whether playback moved the playhead there, rather than a seek.
   */
  moved(to, played) {
    const running = this.effect === null ? null : this.effect.loop.frame;
    const change = effectChange(running, this.loops, this.frame, to, played);
    if (change === "end") {
      this.endEffect();
    } else if (change !== null) {
      this.startEffect(change);
    }
  }

  /**
   * Set off a loop's effect: the loop drawn bright over the map, and the inset playing the
   * earlier pass from the matched keyframe on, both fading out on the effect's own clock.
   * @param {import("./loops.js").Loop} loop The loop.
   */
  startEffect(loop) {
    this.endEffect();
    this.scene.highlightLoop(loop);
    /** @type {Effect} */
    const effect = { loop, start: null, request: 0, opacity: 1, earlier: loop.olderMade };
    // The frame shown next paints the loop; the effect's clock starts at the frame after it, once
    // the loop is on screen, so a slow first paint does not eat into it.
    effect.request = requestAnimationFrame(() => {
      effect.request = requestAnimationFrame((now) => this.tick(now));
    });
    this.effect = effect;
  }

  /**
   * Move the effect on to an animation frame's time, and draw it; end it once it has faded.
   * @param {number} now The animation frame's time, in milliseconds.
   */
  tick(now) {
    const effect = this.effect;
    if (effect === null) {
      return;
    }
    effect.start ??= now;
    const time = (now - effect.start) / 1000;
    if (effectOver(time)) {
      this.endEffect();
      return;
    }
    const opacity = effectOpacity(time);
    if (opacity !== effect.opacity) {
      effect.opacity = opacity; // the map is drawn again only while the loop fades
      this.scene.setLoopOpacity(opacity);
    }
    effect.earlier = effectFrame(effect.loop, this.bundle.times, time);
    this.showInset().catch(this.report);
    effect.request = requestAnimationFrame((later) => this.tick(later));
  }

  /** End the effect running, if any: the map and the inset go back to their own look. */
  endEffect() {
    if (this.effect === null) {
      return;
    }
    cancelAnimationFrame(this.effect.request);
    this.effect = null;
    this.scene.clearLoop();
    this.showInset().catch(this.report);
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
   * Draw the inset at the frame shown: the earlier frame at its place, labelled, or during a
   * loop's effect the earlier pass it plays, as faded as the effect; hidden when it is off or
   * there is neither. Its frames load apart from the main frame's.
   * @returns {Promise<void>} Once drawn.
   */
  async showInset() {
    const { inset, picture, label } = this.parts;
    const effect = this.effect;
    /** @type {{loop: import("./loops.js").Loop, earlier: number} | null} */
    let shown = null;
    if (this.on) {
      shown = effect === null ? insetAt(this.windows, this.frame) : effect;
    }
    inset.style.opacity = effect === null ? "" : String(effect.opacity);
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
