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
  // The image's edges: half a pixel beyond the outer pixels' centres (at 0 and width - 1).
  const [left, top, right, bottom] = [-0.5, -0.5, camera.width - 0.5, camera.height - 0.5];
  const corners = [
    [left, top],
    [right, top],
    [right, bottom],
    [left, bottom],
  ].map(([u, v]) => [((u - cx) / fx) * depth, ((v - cy) / fy) * depth, depth]);
  const count = poses.length / 12;
  const out = new Float32Array(count * FRUSTUM_SEGMENTS * 2 * 3);
  for (let k = 0; k < count; k++) {
    const p = poses.subarray(k * 12, k * 12 + 12);
    /** @param {number[]} x A point in the camera @returns {number[]} It in the world */
    const world = (x) =>
      [0, 1, 2].map(
        (r) => p[r * 4] * x[0] + p[r * 4 + 1] * x[1] + p[r * 4 + 2] * x[2] + p[r * 4 + 3],
      );
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
 * The vertices of frustums, each pose's as deep as its map's (`frustums`, map by map).
 * @param {Float32Array} poses `T_world_cam` top rows, (K, 3, 4).
 * @param {Int32Array} maps Each pose's map, (K,): a map's poses are consecutive.
 * @param {{width: number, height: number, intrinsics: number[]}} camera The camera.
 * @param {Map<number, number>} depths Each map's frustum depth (`mapDepths`).
 * @param {number} fallback The depth of a map without one.
 * @returns {Float32Array} (K * 16, 3): each frustum's 8 segments.
 */
export function mapFrustums(poses, maps, camera, depths, fallback) {
  const count = poses.length / 12;
  const out = new Float32Array(count * FRUSTUM_SEGMENTS * 2 * 3);
  for (const [start, end] of mapRuns(maps)) {
    const depth = depths.get(maps[start]) ?? fallback;
    const lines = frustums(poses.subarray(start * 12, end * 12), camera, depth);
    out.set(lines, start * FRUSTUM_SEGMENTS * 2 * 3);
  }
  return out;
}

/** A frustum's depth, as a share of its map's trajectory's extent. */
export const DEPTH_SHARE = 1 / 40;

/** A frustum's deepest, as a multiple of the median distance between its map's consecutive
 *  keyframes: on a long run with dense keyframes, the extent's share would overlap them. */
export const SPACING_SHARE = 2;

/** The shallowest frustum, in the trajectory's units. */
export const MIN_DEPTH = 0.02;

/** The keyframe size slider's steps per doubling of the frustums' depth. */
export const SIZE_STEPS = 10;

/** The slider's ends, in steps from each map's own depth (step 0): 2^-4.3, about 1/20, and 2. */
export const SIZE_LIMITS = /** @type {const} */ ([-43, 10]);

/**
 * A step of the keyframe size slider, from what it or the page's storage holds.
 * @param {string | null} text The step as text; null if none was stored.
 * @returns {number} The step, rounded and held to `SIZE_LIMITS`; 0 (each map's own depth) for
 *     none, or text that is not a number.
 */
export function sizeStep(text) {
  const step = Math.round(Number(text ?? 0));
  return Number.isFinite(step) ? Math.min(Math.max(step, SIZE_LIMITS[0]), SIZE_LIMITS[1]) : 0;
}

/**
 * The frustums' depth at a step of the keyframe size slider, as a factor of each map's own.
 * @param {number} step The step (`sizeStep`).
 * @returns {number} The factor, 2^(step / `SIZE_STEPS`): logarithmic, so each step scales alike.
 */
export function sizeFactor(step) {
  return 2 ** (step / SIZE_STEPS);
}

/**
 * The runs of consecutive equal maps.
 * @param {ArrayLike<number>} maps Each item's map.
 * @returns {[number, number][]} Each run's rows `[start, end)`, in order.
 */
function mapRuns(maps) {
  /** @type {[number, number][]} */
  const out = [];
  for (let start = 0; start < maps.length;) {
    let end = start + 1;
    while (end < maps.length && maps[end] === maps[start]) {
      end += 1;
    }
    out.push([start, end]);
    start = end;
  }
  return out;
}

/**
 * The middle of some values.
 * @param {number[]} values The values, at least one; sorted in place.
 * @returns {number} The middle one, or the mean of the middle two.
 */
function median(values) {
  values.sort((a, b) => a - b);
  const middle = values.length >> 1;
  return values.length % 2 === 1 ? values[middle] : (values[middle - 1] + values[middle]) / 2;
}

/**
 * Each map's median distance between consecutive keyframes' centres.
 * @param {Float32Array} keyframes The keyframes' centres, by id, (K, 3).
 * @param {Int32Array} maps Their maps, (K,).
 * @returns {Map<number, number>} The median of each map with at least two keyframes.
 */
function keyframeSpacings(keyframes, maps) {
  /** @type {Map<number, number[]>} */
  const gaps = new Map();
  for (let k = 1; k < maps.length; k++) {
    if (maps[k] !== maps[k - 1]) {
      continue;
    }
    const [a, b] = [keyframes.subarray(k * 3 - 3, k * 3), keyframes.subarray(k * 3, k * 3 + 3)];
    const each = gaps.get(maps[k]) ?? [];
    each.push(Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]));
    gaps.set(maps[k], each);
  }
  return new Map([...gaps].map(([map, each]) => [map, median(each)]));
}

/**
 * Each map's frustum depth: a share of the extent of its trajectory, whose scale is its own,
 * capped at `SPACING_SHARE` times the median distance between its consecutive keyframes (where
 * it has two and that distance is not 0), so that dense keyframes along a long run do not
 * overlap.
 * @param {Float32Array} path The tracked frames' centres, (T, 3).
 * @param {Int32Array} maps Their maps, (T,): a map's are consecutive.
 * @param {Float32Array} keyframes The keyframes' centres, by id, (K, 3).
 * @param {Int32Array} keyframeMaps Their maps, (K,).
 * @returns {Map<number, number>} Each map's depth, at least `MIN_DEPTH`.
 */
export function mapDepths(path, maps, keyframes, keyframeMaps) {
  const spacings = keyframeSpacings(keyframes, keyframeMaps);
  /** @type {Map<number, number>} */
  const out = new Map();
  for (const [start, end] of mapRuns(maps)) {
    const low = [Infinity, Infinity, Infinity];
    const high = [-Infinity, -Infinity, -Infinity];
    for (let i = start; i < end; i++) {
      for (let axis = 0; axis < 3; axis++) {
        low[axis] = Math.min(low[axis], path[i * 3 + axis]);
        high[axis] = Math.max(high[axis], path[i * 3 + axis]);
      }
    }
    const extent = Math.hypot(...high.map((h, axis) => h - low[axis]));
    const spacing = spacings.get(maps[start]) ?? 0;
    const cap = spacing > 0 ? SPACING_SHARE * spacing : Infinity;
    out.set(maps[start], Math.max(Math.min(extent * DEPTH_SHARE, cap), MIN_DEPTH));
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
 * Segments between keyframes' centres, as the covisibility graph joins them: each from one
 * keyframe to another of its map, alive while both are (made when the later is, removed when
 * the first goes).
 * @param {Float32Array} poses Every keyframe's last pose, (K, 3, 4).
 * @param {Int32Array} lives Their lives, (K, 3): made, removed (-1 if never), why.
 * @param {Int32Array} maps Their maps, (K,).
 * @param {ArrayLike<number>} pairs The keyframes each segment joins, two ids a segment, (S * 2,).
 * @returns {{positions: Float32Array, lives: Int32Array, maps: Int32Array}} Each segment's two
 *     ends, (S * 2, 3); its life, (S, 3), why it ended 0 (culled) when it did; and its map, (S,).
 */
export function keyframeSegments(poses, lives, maps, pairs) {
  const count = pairs.length / 2;
  const at = centres(poses);
  const positions = new Float32Array(count * 6);
  const segmentLives = new Int32Array(count * 3);
  const segmentMaps = new Int32Array(count);
  for (let s = 0; s < count; s++) {
    const [a, b] = [pairs[s * 2], pairs[s * 2 + 1]];
    positions.set(at.subarray(a * 3, a * 3 + 3), s * 6);
    positions.set(at.subarray(b * 3, b * 3 + 3), s * 6 + 3);
    const ends = [lives[a * 3 + 1], lives[b * 3 + 1]].filter((end) => end >= 0);
    const removed = ends.length ? Math.min(...ends) : -1;
    segmentLives.set([Math.max(lives[a * 3], lives[b * 3]), removed, removed < 0 ? -1 : 0], s * 3);
    segmentMaps[s] = maps[a];
  }
  return { positions, lives: segmentLives, maps: segmentMaps };
}

/**
 * The spanning tree's edges: each keyframe with a parent, to it.
 * @param {Int32Array} parents Each keyframe's last parent, by id, or -1.
 * @returns {Int32Array} The pairs (child, parent), (S * 2,).
 */
export function treePairs(parents) {
  /** @type {number[]} */
  const out = [];
  parents.forEach((parent, child) => {
    if (parent >= 0) {
      out.push(child, parent);
    }
  });
  return Int32Array.from(out);
}

/**
 * The strong links' pairs, their weights left out.
 * @param {Int32Array} links The links, (L, 3): two ids and the points they share.
 * @returns {Int32Array} The pairs, (L * 2,).
 */
export function linkPairs(links) {
  const out = new Int32Array((links.length / 3) * 2);
  for (let l = 0; l < links.length / 3; l++) {
    out.set([links[l * 3], links[l * 3 + 1]], l * 2);
  }
  return out;
}

/**
 * The truth's polyline as segments between consecutive frames that both have a truth.
 * @param {Float32Array} truth The true centre at each frame, (F, 3), NaN where none.
 * @returns {{positions: Float32Array, ends: Int32Array}} Each segment's two ends, (S * 2, 3),
 *     and the frame each segment ends at, (S,), ascending: it begins at the frame before.
 */
export function truthSegments(truth) {
  /** @type {number[]} */
  const out = [];
  /** @type {number[]} */
  const ends = [];
  for (let i = 3; i < truth.length; i += 3) {
    const pair = truth.subarray(i - 3, i + 3);
    if (pair.every((v) => !Number.isNaN(v))) {
      out.push(...pair);
      ends.push(i / 3);
    }
  }
  return { positions: Float32Array.from(out), ends: Int32Array.from(ends) };
}

/**
 * Each vertex's map, for a view that shows one map at a time.
 * @param {Int32Array} maps Each item's map, (N,).
 * @param {number} vertices The vertices each item has.
 * @returns {Float32Array} Per vertex: its item's map.
 */
export function mapAttribute(maps, vertices) {
  const out = new Float32Array(maps.length * vertices);
  maps.forEach((map, i) => out.fill(map, i * vertices, (i + 1) * vertices));
  return out;
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
