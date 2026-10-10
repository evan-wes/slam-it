// The 3D view's geometry, as flat arrays the view hands to three.js: camera frustums, the
// truth's polyline, and the lives each vertex carries. No drawing.

/** A frustum's line segments: four rays from the centre, four edges of the image's rectangle. */
export const FRUSTUM_SEGMENTS = 8;

/** The removal frame of an item never removed: past any bundle's frames, and finite, as a GPU's
 *  shader compares it reliably. */
export const NEVER = 2 ** 30;

/**
 * The vertices of frustums: line segments, each a pair of points, for each pose.
 * @param {Float32Array} poses `T_world_cam` top rows, (K, 3, 4).
 * @param {{width: number, height: number, intrinsics: number[]}} camera The camera (fx, fy, cx,
 *     cy first).
 * @param {number} depth How far the frustum reaches, in the poses' units.
 * @returns {Float32Array} (K * 16, 3): each frustum's 8 segments.
 */
export function frustums(poses, camera, depth) {
  const [fx, fy, cx, cy] = camera.intrinsics;
  const corners = [
    [0, 0],
    [camera.width, 0],
    [camera.width, camera.height],
    [0, camera.height],
  ].map(([u, v]) => [((u - cx) / fx) * depth, ((v - cy) / fy) * depth, depth]);
  const count = poses.length / 12;
  const out = new Float32Array(count * FRUSTUM_SEGMENTS * 2 * 3);
  for (let k = 0; k < count; k++) {
    const p = poses.subarray(k * 12, k * 12 + 12);
    /** @param {number[]} x A point in the camera @returns {number[]} It in the world */
    const world = (x) => [0, 1, 2].map((r) => p[r * 4] * x[0] + p[r * 4 + 1] * x[1] + p[r * 4 + 2] * x[2] + p[r * 4 + 3]);
    const centre = [p[3], p[7], p[11]];
    const ends = corners.map(world);
    const segments = [
      ...ends.map((end) => [centre, end]),
      ...ends.map((end, i) => [end, ends[(i + 1) % 4]]),
    ];
    segments.flat().forEach((point, j) => out.set(point, (k * FRUSTUM_SEGMENTS * 2 + j) * 3));
  }
  return out;
}

/**
 * The centres of poses.
 * @param {Float32Array} poses `T_world_cam` top rows, (T, 3, 4).
 * @returns {Float32Array} (T, 3).
 */
export function centres(poses) {
  const count = poses.length / 12;
  const out = new Float32Array(count * 3);
  for (let k = 0; k < count; k++) {
    out.set([poses[k * 12 + 3], poses[k * 12 + 7], poses[k * 12 + 11]], k * 3);
  }
  return out;
}

/**
 * The truth's polyline as segments between consecutive frames that both have a truth.
 * @param {Float32Array} truth The true centre at each frame, (F, 3), NaN where none.
 * @returns {Float32Array} (S * 2, 3): each segment's two ends.
 */
export function truthSegments(truth) {
  /** @type {number[]} */
  const out = [];
  for (let i = 3; i < truth.length; i += 3) {
    const pair = truth.subarray(i - 3, i + 3);
    if (pair.every((v) => !Number.isNaN(v))) {
      out.push(...pair);
    }
  }
  return Float32Array.from(out);
}

/**
 * Each vertex's life, for a view that hides what is not alive at the frame shown.
 * @param {Int32Array} lives Lives, (N, 3): made, removed (-1 if never), why.
 * @param {number} vertices The vertices each life's item has.
 * @returns {{made: Float32Array, removed: Float32Array}} Per vertex: the frame that made its
 *     item, and the frame that removed it (`NEVER` if none).
 */
export function lifeAttributes(lives, vertices) {
  const count = lives.length / 3;
  const made = new Float32Array(count * vertices);
  const removed = new Float32Array(count * vertices);
  for (let i = 0; i < count; i++) {
    made.fill(lives[i * 3], i * vertices, (i + 1) * vertices);
    removed.fill(lives[i * 3 + 1] < 0 ? NEVER : lives[i * 3 + 1], i * vertices, (i + 1) * vertices);
  }
  return { made, removed };
}
