// The image panel: the frame as the pipeline saw it, its keypoints drawn over it in the colour of
// their fate. Frames are decoded once and kept while near the playhead (`FrameImages`, which the
// "seen before" inset loads its own frames through). While the frame shown waits for its image,
// a spinner says so, and the frame drawn before it is dimmed once the wait is late
// (`imageLoading`).

import { FATES, FATE_HINTS } from "./frames.js";
import { ChunkCache, chunkOf, chunksAround, frameBytes, onCanvas } from "./images.js";
import { imageLoading } from "./loading.js";

/** Decoded frames kept: a few seconds either side of the playhead. */
const DECODED = 120;

/** Chunks kept: the playhead's, the next, and a few behind for scrubbing back. */
const CHUNKS = 4;

/**
 * A CSS custom property's colour.
 * @param {string} name The property, `--fate-tracked` say.
 * @returns {string} Its value.
 */
function token(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

/** A bundle's frames' images: each fetched with its chunk and decoded once, kept while near the
 *  frame last asked for. */
export class FrameImages {
  /**
   * @param {import("./bundle.js").Bundle} bundle The bundle.
   * @param {import("./bundle.js").Fetch} fetch How to fetch one of its files.
   * @param {number} chunks The chunks kept.
   * @param {number} kept The decoded frames kept.
   */
  constructor(bundle, fetch, chunks, kept) {
    this.bundle = bundle;
    this.chunks = new ChunkCache(fetch, chunks);
    this.kept = kept;
    /** @type {Map<number, Promise<ImageBitmap>>} */
    this.decoded = new Map();
  }

  /**
   * Fetch a frame's chunk, and the next, ahead of drawing it.
   * @param {number} frame The frame.
   */
  ahead(frame) {
    for (const chunk of chunksAround(
      frame,
      this.bundle.info.frames,
      this.bundle.info.images.chunk,
    )) {
      this.chunks.get(chunk).catch(() => {}); // fetched ahead; a failure shows when drawn
    }
  }

  /**
   * A frame's decoded image, decoding it (and fetching its chunk) the first time.
   * @param {number} frame The frame.
   * @returns {Promise<ImageBitmap>} The image.
   */
  image(frame) {
    let held = this.decoded.get(frame);
    if (held === undefined) {
      const chunk = chunkOf(frame, this.bundle.info.images.chunk);
      held = this.chunks
        .get(chunk)
        .then((buffer) =>
          createImageBitmap(
            new Blob([frameBytes(buffer, this.bundle.spans, frame)], { type: "image/jpeg" }),
          ),
        );
      held.catch(() => this.decoded.delete(frame));
      this.decoded.set(frame, held);
      for (const [old] of this.decoded) {
        if (this.decoded.size <= this.kept) {
          break;
        }
        if (Math.abs(old - frame) > this.kept / 2) {
          this.decoded.get(old)?.then((bitmap) => bitmap.close());
          this.decoded.delete(old);
        }
      }
    }
    return held;
  }
}

/** The frame as the pipeline saw it, its keypoints in their fates' colours, and the fates'
 *  counts in the legend; while the frame shown waits for its image, the panel's loading mark,
 *  and the frame drawn before it and its counts dimmed (the panel's `stale` class). */
export class ImageView {
  /**
   * @param {HTMLCanvasElement} canvas Where the frame is drawn.
   * @param {HTMLElement} legend Where the fates' colours are listed.
   * @param {HTMLElement} tag The label over the frame.
   * @param {import("./page_view.js").LoadingMark} loading The panel's loading mark.
   * @param {import("./bundle.js").Bundle} bundle The bundle.
   * @param {import("./bundle.js").Fetch} fetch How to fetch one of its files.
   */
  constructor(canvas, legend, tag, loading, bundle, fetch) {
    this.canvas = canvas;
    this.tag = tag;
    this.loading = loading;
    this.images = new FrameImages(bundle, fetch, CHUNKS, DECODED);
    this.shown = -1; // the frame last asked for: an older one's late decode is not drawn
    this.drawn = -1; // the frame whose image is drawn
    this.failed = false; // whether the frame shown's image failed to load
    this.waiting = 0; // when the frame drawn stopped being the frame shown (performance.now)
    this.recheck = 0; // the timer that looks again once the wait would be late
    canvas.width = bundle.info.camera.width;
    canvas.height = bundle.info.camera.height;
    /** @type {HTMLElement[]} */
    this.counts = FATES.map((fate) => {
      const item = document.createElement("span");
      item.title = FATE_HINTS[fate];
      const swatch = document.createElement("i");
      swatch.style.background = `var(--fate-${fate})`;
      const count = document.createElement("b");
      item.append(swatch, fate, " ", count);
      legend.append(item);
      return count;
    });
  }

  /**
   * Draw a frame and, if the run recorded them, its keypoints.
   * @param {number} frame The frame.
   * @param {{xy: Float32Array, fate: Int32Array, counts: number[]} | null} keypoints Its
   *     keypoints, or null.
   * @param {string} note The label over the frame (why there are no keypoints, say).
   * @returns {Promise<void>} Once drawn.
   */
  async show(frame, keypoints, note) {
    if (frame !== this.shown) {
      if (this.shown === this.drawn) {
        this.waiting = performance.now(); // the frame drawn stands in from now on
      }
      this.shown = frame;
    }
    this.failed = false;
    this.mark();
    this.images.ahead(frame);
    /** @type {ImageBitmap} */
    let bitmap;
    try {
      bitmap = await this.images.image(frame);
    } catch (error) {
      if (this.shown === frame) {
        this.failed = true; // the page's message says why; the next frame shown tries again
        this.mark();
      }
      throw error;
    }
    if (this.shown !== frame) {
      return;
    }
    const context = /** @type {CanvasRenderingContext2D} */ (this.canvas.getContext("2d"));
    context.drawImage(bitmap, 0, 0, this.canvas.width, this.canvas.height);
    this.tag.textContent = note;
    this.counts.forEach((count, code) => {
      count.textContent = keypoints === null ? "–" : String(keypoints.counts[code]);
    });
    this.drawn = frame;
    this.mark();
    if (keypoints === null) {
      return;
    }
    const colours = FATES.map((fate) => token(`--fate-${fate}`));
    const radius = Math.max(1.5, this.canvas.width / 320);
    for (let i = 0; i < keypoints.fate.length; i++) {
      context.beginPath();
      const [x, y] = [onCanvas(keypoints.xy[i * 2]), onCanvas(keypoints.xy[i * 2 + 1])];
      context.arc(x, y, radius, 0, 2 * Math.PI);
      context.fillStyle = colours[keypoints.fate[i]];
      context.fill();
    }
  }

  /**
   * Show the panel's loading mark as the frame's image stands (`imageLoading`), dim the frame
   * drawn while it stands in late, and look again when the wait would turn late.
   */
  mark() {
    clearTimeout(this.recheck);
    const { mark, stale, recheck } = imageLoading({
      drawn: this.drawn,
      shown: this.shown,
      failed: this.failed,
      waited: performance.now() - this.waiting,
    });
    this.loading.set(mark);
    this.loading.panel.classList.toggle("stale", stale);
    if (recheck !== null) {
      this.recheck = setTimeout(() => this.mark(), recheck);
    }
  }
}
