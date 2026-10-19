// Reading a bundle's .npy files: the header numpy writes, then the data as a typed array.
//
// A bundle's arrays are little-endian float32 and int32 in C order (slam_it.orchestration
// .bundles); numpy pads the header so the data starts on a 64-byte boundary, so the data is
// viewed in place, not copied.

/** @typedef {{data: Float32Array, shape: number[]}} Float32Npy */
/** @typedef {{data: Int32Array, shape: number[]}} Int32Npy */

const MAGIC = [0x93, 0x4e, 0x55, 0x4d, 0x50, 0x59]; // "\x93NUMPY"

/**
 * Read a .npy file's header: its dtype, its shape, and where its data starts.
 * @param {ArrayBuffer} buffer The file's bytes.
 * @returns {{descr: string, shape: number[], offset: number}} The header.
 * @throws {Error} If the bytes are not a C-order .npy file of version 1, 2 or 3.
 */
export function readHeader(buffer) {
  const bytes = new Uint8Array(buffer);
  if (bytes.length < 10 || MAGIC.some((b, i) => bytes[i] !== b)) {
    throw new Error("not a .npy file");
  }
  const major = bytes[6];
  const view = new DataView(buffer);
  let length;
  let start;
  if (major === 1) {
    length = view.getUint16(8, true);
    start = 10;
  } else if (major === 2 || major === 3) {
    length = view.getUint32(8, true);
    start = 12;
  } else {
    throw new Error(`unknown .npy version ${major}`);
  }
  const text = new TextDecoder("latin1").decode(bytes.subarray(start, start + length));
  const descr = /'descr':\s*'([^']+)'/.exec(text);
  const fortran = /'fortran_order':\s*(True|False)/.exec(text);
  const shape = /'shape':\s*\(([^)]*)\)/.exec(text);
  if (!descr || !fortran || !shape) {
    throw new Error(`not a .npy header: ${text.trim()}`);
  }
  if (fortran[1] === "True") {
    throw new Error("a Fortran-order array");
  }
  const dims = shape[1]
    .split(",")
    .map((d) => d.trim())
    .filter((d) => d !== "")
    .map(Number);
  return { descr: descr[1], shape: dims, offset: start + length };
}

/**
 * The number of items an array of a shape holds.
 * @param {number[]} shape The shape.
 * @returns {number} Their product (1 for a scalar).
 */
export function size(shape) {
  return shape.reduce((a, b) => a * b, 1);
}

/**
 * Read a float32 .npy file.
 * @param {ArrayBuffer} buffer The file's bytes.
 * @returns {Float32Npy} Its data, viewed in place, and its shape.
 * @throws {Error} If it is not little-endian float32.
 */
export function readFloat32(buffer) {
  const header = readHeader(buffer);
  if (header.descr !== "<f4") {
    throw new Error(`expected <f4, got ${header.descr}`);
  }
  return { data: new Float32Array(buffer, header.offset, size(header.shape)), shape: header.shape };
}

/**
 * Read an int32 .npy file.
 * @param {ArrayBuffer} buffer The file's bytes.
 * @returns {Int32Npy} Its data, viewed in place, and its shape.
 * @throws {Error} If it is not little-endian int32.
 */
export function readInt32(buffer) {
  const header = readHeader(buffer);
  if (header.descr !== "<i4") {
    throw new Error(`expected <i4, got ${header.descr}`);
  }
  return { data: new Int32Array(buffer, header.offset, size(header.shape)), shape: header.shape };
}
