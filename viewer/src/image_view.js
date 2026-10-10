// The image panel: the frame as the pipeline saw it, its keypoints drawn over it in the colour of
// their fate. Frames are decoded once and kept while near the playhead.

import { FATES, FATE_HINTS } from "./frames.js";
import { ChunkCache, chunkOf, chunksAround, frameBytes } from "./images.js";

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

export class ImageView {
  /**
   * @param {HTMLCanvasElement} canvas Where the frame is drawn.
   * @param {HTMLElement} legend Where the fates' colours are listed.
   * @param {HTMLElement} tag The label over the frame.
   * @param {import("./bundle.js").Bundle} bundle The bundle.
   * @param {import("./bundle.js").Fetch} fetch How to fetch one of its files.
   */
  constructor(canvas, legend, tag, bundle, fetch) {
    this.canvas = canvas;
    this.tag = tag;
    this.bundle = bundle;
    this.chunks = new ChunkCache(fetch, CHUNKS);
    /** @type {Map<number, Promise<ImageBitmap>>} */
    this.decoded = new Map();
    this.shown = -1; // the frame last asked for: an older one's late decode is not drawn
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
        .then((buffer) => createImageBitmap(new Blob([frameBytes(buffer, this.bundle.spans, frame)], { type: "image/jpeg" })));
      held.catch(() => this.decoded.delete(frame));
      this.decoded.set(frame, held);
      for (const [old] of this.decoded) {
        if (this.decoded.size <= DECODED) {
          break;
        }
        if (Math.abs(old - frame) > DECODED / 2) {
          this.decoded.get(old)?.then((bitmap) => bitmap.close());
          this.decoded.delete(old);
        }
      }
    }
    return held;
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
    this.shown = frame;
    for (const chunk of chunksAround(frame, this.bundle.info.frames, this.bundle.info.images.chunk)) {
      this.chunks.get(chunk).catch(() => {}); // fetched ahead; a failure shows when drawn
    }
    const bitmap = await this.image(frame);
    if (this.shown !== frame) {
      return;
    }
    const context = /** @type {CanvasRenderingContext2D} */ (this.canvas.getContext("2d"));
    context.drawImage(bitmap, 0, 0, this.canvas.width, this.canvas.height);
    this.tag.textContent = note;
    this.counts.forEach((count, code) => {
      count.textContent = keypoints === null ? "–" : String(keypoints.counts[code]);
    });
    if (keypoints === null) {
      return;
    }
    const colours = FATES.map((fate) => token(`--fate-${fate}`));
    const radius = Math.max(1.5, this.canvas.width / 320);
    for (let i = 0; i < keypoints.fate.length; i++) {
      context.beginPath();
      context.arc(keypoints.xy[i * 2], keypoints.xy[i * 2 + 1], radius, 0, 2 * Math.PI);
      context.fillStyle = colours[keypoints.fate[i]];
      context.fill();
    }
  }
}
