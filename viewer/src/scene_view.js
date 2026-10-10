// The 3D panel: the trajectory as tracked, the other backends' beside it, the truth, the
// keyframes and points alive at the frame shown, and the camera. Every point and keyframe sits in
// one buffer with the frames of its life; a shader hides each outside it, so scrubbing changes
// one number and rebuilds nothing.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";

import { cameraAt, posesUpTo } from "./frames.js";
import { FRUSTUM_SEGMENTS, centres, frustums, lifeAttributes, truthSegments } from "./geometry.js";

const VERTEX = `
attribute float made;
attribute float removed;
uniform float frame;
uniform float size;
varying float shown;
void main() {
  shown = made <= frame && removed > frame ? 1.0 : 0.0;
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
 * A material that draws only what is alive at its `frame` uniform.
 * @param {string} token The colour's CSS property.
 * @param {number} size A point's size, in pixels.
 * @returns {THREE.ShaderMaterial} The material.
 */
function livingMaterial(token, size) {
  return new THREE.ShaderMaterial({
    uniforms: { frame: { value: 0 }, size: { value: size }, colour: { value: colour(token) } },
    vertexShader: VERTEX,
    fragmentShader: FRAGMENT,
  });
}

/**
 * A geometry of positions, each with its life.
 * @param {Float32Array} positions (V, 3).
 * @param {Int32Array} lives The items' lives, (N, 3).
 * @param {number} vertices Each item's vertices.
 * @returns {THREE.BufferGeometry} The geometry.
 */
function livingGeometry(positions, lives, vertices) {
  const { made, removed } = lifeAttributes(lives, vertices);
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute("made", new THREE.BufferAttribute(made, 1));
  geometry.setAttribute("removed", new THREE.BufferAttribute(removed, 1));
  return geometry;
}

export class SceneView {
  /**
   * @param {HTMLElement} box The element the view fills.
   * @param {import("./bundle.js").Bundle} bundle The bundle.
   */
  constructor(box, bundle) {
    this.box = box;
    this.bundle = bundle;
    this.follow = true;
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    box.append(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.scene.background = colour("--scene");
    this.camera = new THREE.PerspectiveCamera(50, 1, 0.01, 1000);
    // A truth's world is gravity-aligned, z up; without one, a run's world is its first camera's.
    if (bundle.info.truth === null) {
      this.camera.up.set(0, -1, 0);
    } else {
      this.camera.up.set(0, 0, 1);
    }
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.addEventListener("change", () => this.render());
    /** @type {THREE.Object3D[]} */
    this.runObjects = [];
    /** @type {THREE.ShaderMaterial[]} */
    this.living = [];
    if (bundle.truth !== null) {
      const truth = new THREE.BufferGeometry();
      truth.setAttribute("position", new THREE.BufferAttribute(truthSegments(bundle.truth), 3));
      this.scene.add(new THREE.LineSegments(truth, new THREE.LineBasicMaterial({ color: colour("--truth") })));
    }
    this.trajectory = new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
    this.current = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({ color: colour("--camera") }));
    // Their positions change after three.js has bounded them: never cull them by stale bounds.
    this.trajectory.frustumCulled = false;
    this.current.frustumCulled = false;
    this.scene.add(this.trajectory, this.current);
    /** @type {import("./bundle.js").Backend | null} */
    this.run = null;
    /** @type {{track: import("./bundle.js").Track, line: THREE.Line}[]} */
    this.others = [];
    this.depth = 0.1;
    new ResizeObserver(() => this.resize()).observe(box);
  }

  /**
   * Show a backend's run: its trajectory, keyframes and points, built once.
   * @param {import("./bundle.js").Backend} run The run.
   */
  setRun(run) {
    for (const object of this.runObjects) {
      this.scene.remove(object);
    }
    this.run = run;
    /** @type {THREE.LineBasicMaterial} */ (this.trajectory.material).color = colour(`--backend-${run.name}`);
    const path = centres(run.poses);
    this.trajectory.geometry.setAttribute("position", new THREE.BufferAttribute(path, 3));
    const box = new THREE.Box3().setFromArray(path.length > 0 ? path : new Float32Array(3));
    this.depth = Math.max(box.getSize(new THREE.Vector3()).length() / 40, 0.02);
    const camera = this.bundle.info.camera;
    const keyframes = livingMaterial("--keyframe", 1);
    const points = livingMaterial("--point", 2);
    this.living = [keyframes, points];
    this.runObjects = [
      new THREE.LineSegments(
        livingGeometry(frustums(run.keyframePoses, camera, this.depth), run.keyframeLives, FRUSTUM_SEGMENTS * 2),
        keyframes,
      ),
      new THREE.Points(livingGeometry(run.points, run.pointLives, 1), points),
    ];
    this.scene.add(...this.runObjects);
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
      const material = new THREE.LineBasicMaterial({ color: colour(`--backend-${track.name}`), transparent: true, opacity: 0.6 });
      const line = new THREE.Line(geometry, material);
      line.frustumCulled = false; // its draw range changes with the frame
      this.scene.add(line);
      return { track, line };
    });
  }

  /** Frame the trajectory and the truth. */
  fit() {
    const bounds = new THREE.Box3();
    for (const object of [this.trajectory, ...this.scene.children]) {
      if (object instanceof THREE.Line && object.geometry.getAttribute("position")) {
        bounds.expandByObject(object);
      }
    }
    if (bounds.isEmpty()) {
      bounds.set(new THREE.Vector3(-1, -1, -1), new THREE.Vector3(1, 1, 1));
    }
    const centre = bounds.getCenter(new THREE.Vector3());
    const reach = Math.max(bounds.getSize(new THREE.Vector3()).length(), 0.5);
    this.controls.target.copy(centre);
    this.camera.position.copy(centre).add(this.camera.up.clone().multiplyScalar(reach * 0.6)).add(new THREE.Vector3(reach * 0.6, reach * 0.3, 0));
    this.camera.near = reach / 1000;
    this.camera.far = reach * 100;
    this.camera.updateProjectionMatrix();
    this.controls.update();
  }

  /**
   * Show the map as it was after a frame.
   * @param {number} frame The frame.
   */
  show(frame) {
    for (const material of this.living) {
      material.uniforms.frame.value = frame;
    }
    for (const { track, line } of this.others) {
      line.geometry.setDrawRange(0, posesUpTo(track.poseFrames, frame));
    }
    if (this.run !== null) {
      this.trajectory.geometry.setDrawRange(0, posesUpTo(this.run.poseFrames, frame));
      const at = cameraAt(this.run.poses, this.run.poseFrames, frame);
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
    this.render();
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
