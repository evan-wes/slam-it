// The 3D panel: one map of the run at a time, the map of the frame shown: its trajectory as
// tracked, the other backends' beside it, the truth if the map is placed on it (over the frames
// its trajectory is drawn over, or whole), its keyframes and points alive at the frame, and the
// camera. Every point and keyframe sits in one buffer with the frames of its life and its map;
// a shader hides each outside them, so scrubbing changes two numbers and rebuilds nothing.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

import {
  cameraAt,
  mapAt,
  mapRows,
  mapsShown,
  placed,
  posesUpTo,
  sharesFrame,
  truthRange,
} from "./frames.js";
import {
  FRUSTUM_SEGMENTS,
  centres,
  frustums,
  lifeAttributes,
  mapAttribute,
  mapDepths,
  mapFrustums,
  truthSegments,
} from "./geometry.js";

const VERTEX = `
attribute float made;
attribute float removed;
attribute float map;
uniform float frame;
uniform float shownMap;
uniform float size;
varying float shown;
void main() {
  shown = made <= frame && removed > frame && abs(map - shownMap) < 0.5 ? 1.0 : 0.0;
  gl_PointSize = size;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}`;

const FRAGMENT = `
uniform vec3 colour;
varying float shown;
void main() {
  if (shown < 0.5) discard;
  gl_FragColor = vec4(colour, 1.0);
}`;

/**
 * A CSS custom property's colour.
 * @param {string} name The property.
 * @returns {THREE.Color} The colour.
 */
function colour(name) {
  return new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue(name).trim());
}

/**
 * A material that draws only what is alive at its `frame` uniform, of its `shownMap`.
 * @param {string} token The colour's CSS property.
 * @param {number} size A point's size, in pixels.
 * @returns {THREE.ShaderMaterial} The material.
 */
function livingMaterial(token, size) {
  return new THREE.ShaderMaterial({
    uniforms: {
      frame: { value: 0 },
      shownMap: { value: -1 },
      size: { value: size },
      colour: { value: colour(token) },
    },
    vertexShader: VERTEX,
    fragmentShader: FRAGMENT,
  });
}

/**
 * A geometry of positions, each with its life and map.
 * @param {Float32Array} positions (V, 3).
 * @param {Int32Array} lives The items' lives, (N, 3).
 * @param {Int32Array} maps The items' maps, (N,).
 * @param {number} vertices Each item's vertices.
 * @returns {THREE.BufferGeometry} The geometry.
 */
function livingGeometry(positions, lives, maps, vertices) {
  const { made, removed } = lifeAttributes(lives, vertices);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("made", new THREE.BufferAttribute(made, 1));
  geometry.setAttribute("removed", new THREE.BufferAttribute(removed, 1));
  geometry.setAttribute("map", new THREE.BufferAttribute(mapAttribute(maps, vertices), 1));
  return geometry;
}

/** A frustum's depth before any map is shown, in the trajectory's units. */
const DEFAULT_DEPTH = 0.1;

/** The truth's opacity: faint beside the trajectories it is read against. */
const TRUTH_OPACITY = 0.6;

/** The way up in a truth's world, gravity-aligned. */
const TRUTH_UP = new THREE.Vector3(0, 0, 1);

/** The way up in a run's own world, its first camera's (y down). */
const CAMERA_UP = new THREE.Vector3(0, -1, 0);

/** The map shown after the frame shown: its trajectory so far, the other backends' and the
 *  truth's (or the whole truth), its keyframes and points alive, and the camera, followed or
 *  orbited. */
export class SceneView {
  /**
   * @param {HTMLElement} box The element the view fills.
   * @param {import("./bundle.js").Bundle} bundle The bundle.
   */
  constructor(box, bundle) {
    this.box = box;
    this.bundle = bundle;
    this.follow = true;
    /** Whether the truth is drawn whole, rather than over the frames the trajectory is. */
    this.wholeTruth = false;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    box.append(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.background = colour("--scene");
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 1000);
    this.camera.up.copy(bundle.info.truth ? TRUTH_UP : CAMERA_UP);
    this.controls = this.orbit();
    /** @type {THREE.Object3D[]} */
    this.runObjects = [];
    /** @type {THREE.ShaderMaterial[]} */
    this.living = [];
    /** @type {THREE.LineSegments | null} */
    this.truth = null;
    /** @type {Int32Array} The frame each of the truth's segments ends at (`truthSegments`). */
    this.truthEnds = new Int32Array(0);
    if (bundle.truth !== null) {
      const { positions, ends } = truthSegments(bundle.truth);
      const truth = new THREE.BufferGeometry();
      truth.setAttribute("position", new THREE.BufferAttribute(positions, 3));
      const material = new THREE.LineBasicMaterial({
        color: colour("--truth"),
        transparent: true,
        opacity: TRUTH_OPACITY,
      });
      this.truth = new THREE.LineSegments(truth, material);
      this.truthEnds = ends;
      this.scene.add(this.truth);
    }
    this.trajectory = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
    this.current = new THREE.LineSegments(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: colour("--camera") }),
    );
    // Their positions change after three.js has bounded them: never cull them by stale bounds.
    this.trajectory.frustumCulled = false;
    this.current.frustumCulled = false;
    this.scene.add(this.trajectory, this.current);
    /** @type {import("./bundle.js").Backend | null} */
    this.run = null;
    /** @type {{track: import("./bundle.js").Track, line: THREE.Line, shown: Int32Array}[]} */
    this.others = [];
    this.depth = DEFAULT_DEPTH;
    /** @type {Int32Array} The run's map shown at each of its records (`mapsShown`). */
    this.shown = new Int32Array(0);
    /** @type {number | null | undefined} The map shown; undefined until the first is set. */
    this.shownMap = undefined;
    /** @type {[number, number]} The rows of the map's poses. */
    this.rows = [0, 0];
    /** Whether the map shown is placed on the truth. */
    this.aligned = false;
    /** @type {Float32Array} The run's tracked frames' centres, (T, 3). */
    this.path = new Float32Array(0);
    /** @type {Map<number, number>} Each map's frustum depth. */
    this.depths = new Map();
    new ResizeObserver(() => this.resize()).observe(box);
  }

  /**
   * Show a backend's run: its trajectory, keyframes and points, built once; the map shown is
   * chosen at the next frame shown.
   * @param {import("./bundle.js").Backend} run The run.
   */
  setRun(run) {
    for (const object of this.runObjects) {
      this.scene.remove(object);
    }
    this.run = run;
    this.shown = mapsShown(run.records);
    this.shownMap = undefined;
    /** @type {THREE.LineBasicMaterial} */ (this.trajectory.material).color = colour(
      `--backend-${run.name}`,
    );
    this.path = centres(run.poses);
    this.trajectory.geometry.setAttribute("position", new THREE.BufferAttribute(this.path, 3));
    this.depths = mapDepths(this.path, run.poseMaps);
    const camera = this.bundle.info.camera;
    const keyframes = livingMaterial("--keyframe", 1);
    const points = livingMaterial("--point", 2);
    this.living = [keyframes, points];
    const frames = mapFrustums(
      run.keyframePoses,
      run.keyframeMaps,
      camera,
      this.depths,
      DEFAULT_DEPTH,
    );
    this.runObjects = [
      new THREE.LineSegments(
        livingGeometry(frames, run.keyframeLives, run.keyframeMaps, FRUSTUM_SEGMENTS * 2),
        keyframes,
      ),
      new THREE.Points(livingGeometry(run.points, run.pointLives, run.pointMaps, 1), points),
    ];
    this.scene.add(...this.runObjects);
  }

  /**
   * Show one of the run's maps: its keyframes, points and trajectory, the truth only if the map
   * is placed on it. A map left in its own frame is in its first camera's world, not the
   * truth's: while it is shown, the truth is hidden and the view is the camera's way up.
   * @param {import("./bundle.js").Backend} run The run.
   * @param {number | null} map The map; null for none.
   */
  showMap(run, map) {
    this.shownMap = map;
    for (const material of this.living) {
      material.uniforms.shownMap.value = map ?? -1;
    }
    this.rows = map === null ? [0, 0] : mapRows(run.poseMaps, map);
    this.aligned = placed(run.info, map);
    if (this.truth !== null) {
      this.truth.visible = this.aligned;
    }
    const up = this.aligned ? TRUTH_UP : CAMERA_UP;
    if (!this.camera.up.equals(up)) {
      this.camera.up.copy(up);
      this.controls.dispose();
      this.controls = this.orbit(); // the controls take the camera's up when made
    }
    this.depth = map === null ? DEFAULT_DEPTH : (this.depths.get(map) ?? DEFAULT_DEPTH);
    this.fit();
  }

  /**
   * Show the other backends' trajectories beside the run's, each in its backend's colour.
   * @param {import("./bundle.js").Track[]} tracks Their tracks.
   */
  setOthers(tracks) {
    for (const { line } of this.others) {
      this.scene.remove(line);
    }
    this.others = tracks.map((track) => {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute("position", new THREE.BufferAttribute(centres(track.poses), 3));
      const material = new THREE.LineBasicMaterial({
        color: colour(`--backend-${track.name}`),
        transparent: true,
        opacity: 0.6,
      });
      const line = new THREE.Line(geometry, material);
      line.frustumCulled = false; // its draw range changes with the frame
      this.scene.add(line);
      return { track, line, shown: mapsShown(track.records) };
    });
  }

  /**
   * Orbit controls for the camera, as it is pointed up now.
   * @returns {OrbitControls} The controls.
   */
  orbit() {
    const controls = new OrbitControls(this.camera, this.renderer.domElement);
    controls.addEventListener("change", () => this.render());
    return controls;
  }

  /** Frame the map's trajectory and the truth, if shown. */
  fit() {
    const [start, end] = this.rows;
    const bounds = new THREE.Box3();
    if (end > start) {
      bounds.setFromArray(this.path.subarray(start * 3, end * 3));
    }
    if (this.truth !== null && this.truth.visible) {
      bounds.expandByObject(this.truth);
    }
    if (bounds.isEmpty()) {
      bounds.set(new THREE.Vector3(-1, -1, -1), new THREE.Vector3(1, 1, 1));
    }
    const centre = bounds.getCenter(new THREE.Vector3());
    const reach = Math.max(bounds.getSize(new THREE.Vector3()).length(), 0.5);
    this.controls.target.copy(centre);
    this.camera.position
      .copy(centre)
      .add(this.camera.up.clone().multiplyScalar(reach * 0.6))
      .add(new THREE.Vector3(reach * 0.6, reach * 0.3, 0));
    this.camera.near = reach / 1000;
    this.camera.far = reach * 100;
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  /**
   * Show the map of a frame as it was after the frame: the frame's map, else the last before it.
   * @param {number} frame The frame.
   * @returns {number | null} The map shown; null before the run's first.
   */
  show(frame) {
    for (const material of this.living) {
      material.uniforms.frame.value = frame;
    }
    if (this.run !== null) {
      const map = mapAt(this.shown, this.run.records, frame);
      if (map !== this.shownMap) {
        this.showMap(this.run, map);
      }
      const [start, end] = this.rows;
      const upTo = Math.min(posesUpTo(this.run.poseFrames, frame), end);
      this.trajectory.geometry.setDrawRange(start, Math.max(upTo - start, 0));
      this.showTruth(this.run.poseFrames, start, upTo);
      const at = cameraAt(this.run.poses, this.run.poseFrames, frame, this.rows);
      this.current.visible = at !== null;
      if (at !== null) {
        const lines = frustums(at.pose, this.bundle.info.camera, this.depth * 1.5);
        this.current.geometry.setAttribute("position", new THREE.BufferAttribute(lines, 3));
        if (this.follow) {
          const centre = new THREE.Vector3(at.pose[3], at.pose[7], at.pose[11]);
          this.camera.position.add(centre.clone().sub(this.controls.target));
          this.controls.target.copy(centre);
          this.controls.update();
        }
      }
    }
    this.showOthers(frame);
    this.render();
    return this.shownMap ?? null;
  }

  /**
   * Draw the truth whole, or over the frames the map's trajectory is drawn over: from its first
   * tracked frame to its last so far, growing with it.
   * @param {Int32Array} poseFrames The run's tracked frames.
   * @param {number} start The map's first pose's row.
   * @param {number} upTo The row after its last pose so far.
   */
  showTruth(poseFrames, start, upTo) {
    if (this.truth === null) {
      return;
    }
    if (this.wholeTruth) {
      this.truth.geometry.setDrawRange(0, Infinity);
    } else if (upTo > start) {
      const [first, count] = truthRange(this.truthEnds, poseFrames[start], poseFrames[upTo - 1]);
      this.truth.geometry.setDrawRange(first, count);
    } else {
      this.truth.geometry.setDrawRange(0, 0);
    }
  }

  /**
   * Show each other backend's map of a frame where it shares the shown map's frame: both placed
   * on the truth, or both in their own frames and the same map (the backends decide alike).
   * @param {number} frame The frame.
   */
  showOthers(frame) {
    const mine = this.shownMap ?? null;
    for (const { track, line, shown } of this.others) {
      const theirs = mapAt(shown, track.records, frame);
      line.visible = sharesFrame(mine, theirs, placed(track.info, theirs), this.aligned);
      if (line.visible && theirs !== null) {
        const [start, end] = mapRows(track.poseMaps, theirs);
        const upTo = Math.min(posesUpTo(track.poseFrames, frame), end);
        line.geometry.setDrawRange(start, Math.max(upTo - start, 0));
      }
    }
  }

  /** Fit the drawing to the panel's size. */
  resize() {
    const { clientWidth: width, clientHeight: height } = this.box;
    if (width === 0 || height === 0) {
      return;
    }
    this.renderer.setSize(width, height, false);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.render();
  }

  /** Draw. */
  render() {
    this.renderer.render(this.scene, this.camera);
  }
}
