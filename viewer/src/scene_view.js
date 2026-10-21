// The 3D panel: one map of the run at a time, the map of the frame shown (the map the frame ends
// in): its final trajectory, drawn through the runs of rows its poses come in, the other
// backends' beside it, the truth if the map is placed on it (over the frames its trajectory is
// drawn over, or whole), its keyframes and points alive at the frame, the covisibility graph if
// asked (each keyframe's line to its parent, the strong links, the loops closed and the merges
// made, each in its own colour), and the camera. Every point and keyframe sits in one buffer
// with the frames of its life and its map; a shader hides each outside them, so scrubbing
// changes two numbers and rebuilds nothing. A loop closed can be drawn over it all, bright and
// thick, for the loop effect (`highlightLoop`).

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { Line2 } from "three/addons/lines/Line2.js";
import { LineGeometry } from "three/addons/lines/LineGeometry.js";
import { LineMaterial } from "three/addons/lines/LineMaterial.js";
import { LineSegments2 } from "three/addons/lines/LineSegments2.js";
import { LineSegmentsGeometry } from "three/addons/lines/LineSegmentsGeometry.js";

import {
  cameraAt,
  mapAt,
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
  keyframeSegments,
  lastRow,
  lifeAttributes,
  linkPairs,
  mapAttribute,
  mapDepths,
  mapFrustums,
  runSegments,
  runSegmentsBefore,
  runsOf,
  treePairs,
  truthSegments,
} from "./geometry.js";
import { cycleRuns } from "./loops.js";

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
uniform float alpha;
varying float shown;
void main() {
  if (shown < 0.5) discard;
  gl_FragColor = vec4(colour, alpha);
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
 * A material that draws only what is alive at its `frame` uniform, of its `shownMap`, opaque
 * until its `alpha` uniform is set lower (`SceneView.setPointOpacity`).
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
      alpha: { value: 1 },
    },
    vertexShader: VERTEX,
    fragmentShader: FRAGMENT,
  });
}

/**
 * The lines of some of the covisibility graph's edges, each alive while both its keyframes are.
 * @param {import("./bundle.js").Backend} run The run.
 * @param {Int32Array} pairs The keyframes each line joins, two ids a line.
 * @param {THREE.ShaderMaterial} material Its material (`livingMaterial`).
 * @param {boolean} visible Whether it is drawn at first.
 * @returns {THREE.LineSegments} The lines.
 */
function graphLine(run, pairs, material, visible) {
  const segments = keyframeSegments(run.keyframePoses, run.keyframeLives, run.keyframeMaps, pairs);
  const line = new THREE.LineSegments(
    livingGeometry(segments.positions, segments.lives, segments.maps, 2),
    material,
  );
  line.visible = visible;
  return line;
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

/** A highlighted loop's cycle's width, in CSS pixels: the trajectory's is 1. */
const CYCLE_WIDTH = 4;

/** A highlighted loop's edge's and its keyframes' frustums' width, in CSS pixels. */
const LOOP_WIDTH = 2.5;

/**
 * A screen-space thick line's material, drawn over everything else and able to fade.
 * @param {string} token The colour's CSS property.
 * @param {number} width The line's width, in CSS pixels.
 * @returns {LineMaterial} The material.
 */
function thickMaterial(token, width) {
  return new LineMaterial({
    color: colour(token),
    linewidth: width,
    transparent: true,
    depthTest: false,
  });
}

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
    /** Whether the covisibility graph is drawn: the tree, the strong links, the loops, merges. */
    this.graph = false;
    /** @type {THREE.Object3D[]} The graph's lines, shown with `graph`. */
    this.graphObjects = [];
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
    // The map's trajectory: segments between consecutive rows of each run of its poses' rows,
    // indexed (`runSegments`), so a map whose poses come in several runs is drawn without
    // joining them across another map's.
    this.trajectory = new THREE.LineSegments(
      new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial(),
    );
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
    /**
     * The other backends' trajectories: each one's track, line, maps shown, and the map its line
     * is indexed for, with the runs of that map's poses' rows.
     * @type {{track: import("./bundle.js").Track, line: THREE.LineSegments, shown: Int32Array,
     *     map: number | null, runs: [number, number][]}[]}
     */
    this.others = [];
    this.depth = DEFAULT_DEPTH;
    /** @type {Int32Array} The run's map shown at each of its records (`mapsShown`). */
    this.shown = new Int32Array(0);
    /** @type {number | null | undefined} The map shown; undefined until the first is set. */
    this.shownMap = undefined;
    /** @type {[number, number][]} The runs of the map's poses' rows (`runsOf`). */
    this.runs = [];
    /** Whether the map shown is placed on the truth. */
    this.aligned = false;
    /** @type {Float32Array} The run's tracked frames' centres, (T, 3). */
    this.path = new Float32Array(0);
    /** @type {Map<number, number>} Each map's frustum depth. */
    this.depths = new Map();
    /** The frustums' depth, as a factor of each map's own (the keyframe size slider's). */
    this.size = 1;
    /** @type {THREE.BufferGeometry | null} The run's keyframes' frustums. */
    this.keyframeGeometry = null;
    /** The map points' opacity (the point opacity slider's), from 0 to 1. */
    this.pointOpacity = 1;
    /** @type {THREE.ShaderMaterial | null} The run's map points' material. */
    this.pointMaterial = null;
    /** @type {(Line2 | LineSegments2)[]} The loop highlighted, if any (`highlightLoop`). */
    this.loopLines = [];
    /** The highlighted loop's map: it is drawn only while that map is shown. */
    this.loopMap = -1;
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
    this.depths = mapDepths(this.path, run.poseMaps, centres(run.keyframePoses), run.keyframeMaps);
    const keyframes = livingMaterial("--keyframe", 1);
    const points = livingMaterial("--point", 2);
    const tree = livingMaterial("--graph-tree", 1);
    const links = livingMaterial("--graph-link", 1);
    const loops = livingMaterial("--graph-loop", 1);
    const merges = livingMaterial("--graph-merge", 1);
    this.living = [keyframes, points, tree, links, loops, merges];
    this.pointMaterial = points;
    this.setPointOpacity(this.pointOpacity);
    this.keyframeGeometry = livingGeometry(
      this.keyframeFrustums(run),
      run.keyframeLives,
      run.keyframeMaps,
      FRUSTUM_SEGMENTS * 2,
    );
    this.runObjects = [
      new THREE.LineSegments(this.keyframeGeometry, keyframes),
      new THREE.Points(livingGeometry(run.points, run.pointLives, run.pointMaps, 1), points),
    ];
    this.graphObjects = [
      graphLine(run, treePairs(run.keyframeParents), tree, this.graph),
      graphLine(run, linkPairs(run.keyframeLinks), links, this.graph),
      graphLine(run, run.keyframeLoops, loops, this.graph),
      graphLine(run, run.keyframeMerges, merges, this.graph),
    ];
    this.runObjects.push(...this.graphObjects);
    this.scene.add(...this.runObjects);
  }

  /**
   * A run's keyframes' frustums, each as deep as its map's depth times the size factor.
   * @param {import("./bundle.js").Backend} run The run.
   * @returns {Float32Array} Their vertices (`mapFrustums`).
   */
  keyframeFrustums(run) {
    const depths = new Map([...this.depths].map(([map, depth]) => [map, depth * this.size]));
    return mapFrustums(
      run.keyframePoses,
      run.keyframeMaps,
      this.bundle.info.camera,
      depths,
      DEFAULT_DEPTH * this.size,
    );
  }

  /**
   * Draw the keyframes' frustums, and the camera's from the next frame shown, at a factor of
   * each map's own depth: the keyframes' vertices rebuilt in place, their lives and maps kept.
   * @param {number} factor The factor (the keyframe size slider's).
   */
  setKeyframeSize(factor) {
    this.size = factor;
    if (this.run === null || this.keyframeGeometry === null) {
      return;
    }
    const position = /** @type {THREE.BufferAttribute} */ (
      this.keyframeGeometry.getAttribute("position")
    );
    position.copyArray(this.keyframeFrustums(this.run));
    position.needsUpdate = true;
    this.keyframeGeometry.computeBoundingSphere(); // three.js culls the lines by it
  }

  /**
   * Draw the map points at an opacity: below 1 they blend over what is behind them, and write no
   * depth, so that a faint point never hides another or what is drawn after it.
   * @param {number} opacity The opacity, from 0 to 1 (the point opacity slider's).
   */
  setPointOpacity(opacity) {
    this.pointOpacity = opacity;
    const material = this.pointMaterial;
    if (material === null) {
      return;
    }
    material.uniforms.alpha.value = opacity;
    const transparent = opacity < 1;
    if (material.transparent !== transparent) {
      material.transparent = transparent;
      material.depthWrite = !transparent;
      material.needsUpdate = true;
    }
  }

  /**
   * Draw the covisibility graph, or not: the spanning tree, the strong links, the loops closed and
   * the merges made of the map shown.
   * @param {boolean} shown Whether to draw it.
   */
  showGraph(shown) {
    this.graph = shown;
    for (const line of this.graphObjects) {
      line.visible = shown;
    }
    this.render();
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
    this.runs = map === null ? [] : runsOf(run.poseMaps, map);
    this.trajectory.geometry.setIndex(new THREE.BufferAttribute(runSegments(this.runs), 1));
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
      const line = new THREE.LineSegments(geometry, material);
      line.frustumCulled = false; // its draw range changes with the frame
      this.scene.add(line);
      /** @type {[number, number][]} */
      const runs = [];
      return { track, line, shown: mapsShown(track.records), map: null, runs };
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

  /** Frame the map's trajectory, every run of it, and the truth, if shown. */
  fit() {
    const bounds = new THREE.Box3();
    for (const [start, end] of this.runs) {
      bounds.union(new THREE.Box3().setFromArray(this.path.subarray(start * 3, end * 3)));
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
      const upTo = posesUpTo(this.run.poseFrames, frame);
      this.trajectory.geometry.setDrawRange(0, 2 * runSegmentsBefore(this.runs, upTo));
      this.showTruth(this.run.poseFrames, this.runs, upTo);
      const at = cameraAt(this.run.poses, this.run.poseFrames, frame, this.runs);
      this.current.visible = at !== null;
      if (at !== null) {
        const depth = this.depth * 1.5 * this.size;
        const lines = frustums(at.pose, this.bundle.info.camera, depth);
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
    for (const line of this.loopLines) {
      line.visible = this.shownMap === this.loopMap;
    }
    this.render();
    return this.shownMap ?? null;
  }

  /**
   * Draw a loop over the map until `clearLoop`, bright and thick in the loops' colour: its cycle
   * (the trajectory from the frame that made its matched keyframe to the loop's frame, in the
   * runs of rows the map's poses come in, `cycleRuns`), the edge between its two keyframes, and
   * their frustums. It is drawn from the next frame shown (`show`), which the caller draws: a
   * render here would only be drawn over.
   * @param {import("./loops.js").Loop} loop The loop, of the run shown.
   */
  highlightLoop(loop) {
    this.clearLoop();
    const run = this.run;
    if (run === null) {
      return;
    }
    for (const [start, end] of cycleRuns(loop, run.poseFrames, run.poseMaps)) {
      if (end - start >= 2) {
        const cycle = new LineGeometry();
        cycle.setPositions(this.path.subarray(start * 3, end * 3));
        this.loopLines.push(new Line2(cycle, thickMaterial("--graph-loop", CYCLE_WIDTH)));
      }
    }
    const poses = new Float32Array(24);
    poses.set(run.keyframePoses.subarray(loop.older * 12, loop.older * 12 + 12));
    poses.set(run.keyframePoses.subarray(loop.newer * 12, loop.newer * 12 + 12), 12);
    const depth = (this.depths.get(loop.map) ?? DEFAULT_DEPTH) * this.size;
    const edge = new LineSegmentsGeometry();
    edge.setPositions(
      Float32Array.from([...centres(poses), ...frustums(poses, this.bundle.info.camera, depth)]),
    );
    this.loopLines.push(new LineSegments2(edge, thickMaterial("--graph-loop", LOOP_WIDTH)));
    this.loopMap = loop.map;
    for (const line of this.loopLines) {
      line.frustumCulled = false; // its bounds are not kept up to date
      line.renderOrder = 1; // over the transparent lines and points too
      line.visible = this.shownMap === loop.map;
      this.scene.add(line);
    }
  }

  /**
   * Fade the loop highlighted.
   * @param {number} opacity Its opacity, from 0 to 1.
   */
  setLoopOpacity(opacity) {
    for (const line of this.loopLines) {
      line.material.opacity = opacity;
    }
    this.render();
  }

  /** Stop drawing the loop highlighted, if any, and free its lines. */
  clearLoop() {
    if (this.loopLines.length === 0) {
      return;
    }
    for (const line of this.loopLines) {
      this.scene.remove(line);
      line.geometry.dispose();
      line.material.dispose();
    }
    this.loopLines = [];
    this.loopMap = -1;
    this.render();
  }

  /**
   * Draw the truth whole, or over the frames the map's trajectory is drawn over: from its first
   * tracked frame to its last so far, growing with it (across another map's frames between its
   * runs too: the truth is one).
   * @param {Int32Array} poseFrames The run's tracked frames.
   * @param {[number, number][]} runs The runs of the map's poses' rows.
   * @param {number} upTo The row after the last pose of the frames so far.
   */
  showTruth(poseFrames, runs, upTo) {
    if (this.truth === null) {
      return;
    }
    const last = lastRow(runs, upTo);
    if (this.wholeTruth) {
      this.truth.geometry.setDrawRange(0, Infinity);
    } else if (last >= 0) {
      const [first, count] = truthRange(this.truthEnds, poseFrames[runs[0][0]], poseFrames[last]);
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
    for (const other of this.others) {
      const { track, line, shown } = other;
      const theirs = mapAt(shown, track.records, frame);
      line.visible = sharesFrame(mine, theirs, placed(track.info, theirs), this.aligned);
      if (line.visible && theirs !== null) {
        if (theirs !== other.map) {
          other.map = theirs;
          other.runs = runsOf(track.poseMaps, theirs);
          line.geometry.setIndex(new THREE.BufferAttribute(runSegments(other.runs), 1));
        }
        const upTo = posesUpTo(track.poseFrames, frame);
        line.geometry.setDrawRange(0, 2 * runSegmentsBefore(other.runs, upTo));
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
