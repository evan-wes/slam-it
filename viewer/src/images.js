// The bundle's images: which chunk holds a frame, its JPEG's bytes in it, and a cache of the
// chunks around the playhead, so scrubbing fetches a few files rather than thousands. No
// decoding: the image view decodes the bytes these return.

/**
 * A chunk file's path in the bundle.
 * @param {number} index The chunk.
 * @returns {string} `images/chunk_NNNN.bin`.
 */
export function chunkPath(index) {
  return `images/chunk_${String(index).padStart(4, "0")}.bin`;
}

/**
 * The chunk that holds a frame.
 * @param {number} frame The frame.
 * @param {number} chunk Frames per chunk.
 * @returns {number} The chunk's index.
 */
export function chunkOf(frame, chunk) {
  return Math.floor(frame / chunk);
}

/**
 * A frame's JPEG, cut from its chunk.
 * @param {ArrayBuffer} buffer The chunk's bytes.
 * @param {Int32Array} spans Each frame's byte range in its chunk, (F, 2).
 * @param {number} frame The frame.
 * @returns {Uint8Array<ArrayBuffer>} The JPEG's bytes (a view into the chunk).
 */
export function frameBytes(buffer, spans, frame) {
  return new Uint8Array(buffer, spans[frame * 2], spans[frame * 2 + 1] - spans[frame * 2]);
}

/**
 * The chunks to have at hand for a frame: its own, then the next one, while there is one.
 * @param {number} frame The frame.
 * @param {number} frames The bundle's frames.
 * @param {number} chunk Frames per chunk.
 * @returns {number[]} Chunk indices, the frame's first.
 */
export function chunksAround(frame, frames, chunk) {
  const own = chunkOf(frame, chunk);
  const last = chunkOf(Math.max(frames - 1, 0), chunk);
  return own < last ? [own, own + 1] : [own];
}

/** A cache of fetched chunks: at most `limit`, the least recently used dropped first. */
export class ChunkCache {
  /**
   * @param {(path: string) => Promise<ArrayBuffer>} fetch How to fetch a bundle's file.
   * @param {number} limit The chunks kept.
   */
  constructor(fetch, limit) {
    this.fetch = fetch;
    this.limit = limit;
    /** @type {Map<number, Promise<ArrayBuffer>>} */
    this.chunks = new Map();
  }

  /**
   * A chunk's bytes, fetched once while it stays in the cache.
   * @param {number} index The chunk.
   * @returns {Promise<ArrayBuffer>} Its bytes.
   */
  get(index) {
    let held = this.chunks.get(index);
    if (held === undefined) {
      held = this.fetch(chunkPath(index));
      held.catch(() => this.chunks.delete(index)); // a failed fetch is tried again
    } else {
      this.chunks.delete(index); // re-inserted below, as the most recently used
    }
    this.chunks.set(index, held);
    while (this.chunks.size > this.limit) {
      const oldest = this.chunks.keys().next().value;
      if (oldest === undefined) {
        break;
      }
      this.chunks.delete(oldest);
    }
    return held;
  }
}
