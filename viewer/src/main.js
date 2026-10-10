// The run viewer's page: loads a bundle (the URL's `bundle`, else `bundle/` beside the page),
// and shows one frame at a time in the image, map and solver panels, for the backend chosen,
// beside the other backends' trajectories and step times, the frames where they decided
// differently marked.

import { loadBackend, loadBundle, loadTrack } from "./bundle.js";
import { differences } from "./compare.js";
import * as format from "./format.js";
import { keypointsAt, recordIndex } from "./frames.js";
import { ImageView } from "./image_view.js";
import { SceneView } from "./scene_view.js";
import { SolverView } from "./solver_view.js";

/** The order backends are offered in, and the first one present is shown. */
const BACKENDS = ["cpp", "rust", "python"];

/**
 * An element of the page, by id.
 * @template {HTMLElement} T
 * @param {string} id Its id.
 * @param {new () => T} type Its type.
 * @returns {T} The element.
 */
function element(id, type) {
  const found = document.getElementById(id);
  if (!(found instanceof type)) {
    throw new Error(`the page has no ${id}`);
  }
  return found;
}

/**
 * How to fetch a bundle's files, from where it is served.
 * @param {string} base The bundle's URL, ending in "/".
 * @returns {import("./bundle.js").Fetch} The fetcher.
 */
function fetcher(base) {
  return async (path) => {
    const response = await fetch(new URL(path, base));
    if (!response.ok) {
      throw new Error(`${path}: ${response.status} ${response.statusText}`);
    }
    return response.arrayBuffer();
  };
}

/**
 * Fill the header's numbers for a run.
 * @param {HTMLElement} numbers The header's list.
 * @param {import("./bundle.js").BackendInfo} info The run's header.
 */
function showNumbers(numbers, info) {
  /** @type {[string, string, string][]} */
  const items = [];
  if (info.score !== null) {
    items.push(["ATE", format.length(info.score.ate_rmse), "absolute trajectory error, RMS, after alignment"]);
    items.push([
      "RPE",
      `${format.length(info.score.rpe_translation_rmse)} · ${format.angle(info.score.rpe_rotation_rmse)}`,
      "relative pose error over one frame period, RMS: translation and rotation",
    ]);
  }
  const ran = info.frames[1] - info.frames[0];
  items.push(["tracked", `${info.tracked} / ${ran}`, "frames tracked, of the frames run"]);
  items.push(["keyframes", String(info.keyframes), `in the final map (${info.made.keyframes} made)`]);
  items.push(["points", String(info.points), `in the final map (${info.made.points} made)`]);
  if (info.latency !== null) {
    const { median, p95, deadline, late } = info.latency;
    items.push([
      "latency",
      `${format.duration(median)} · p95 ${format.duration(p95)} / ${format.duration(deadline)}`,
      `building and tracking a frame, median and 95th percentile, against the frame period; ${late} late`,
    ]);
  }
  items.push(["memory", format.size(info.peak_memory), "the program's peak resident memory"]);
  numbers.replaceChildren(
    ...items.map(([name, value, hint]) => {
      const item = document.createElement("div");
      item.title = hint;
      const term = document.createElement("dt");
      term.textContent = name;
      const description = document.createElement("dd");
      description.textContent = value;
      item.append(term, description);
      return item;
    }),
  );
}

async function main() {
  const message = element("message", HTMLParagraphElement);
  const base = new URL(new URLSearchParams(location.search).get("bundle") ?? "bundle/", location.href);
  const fetchFile = fetcher(base.href.endsWith("/") ? base.href : `${base.href}/`);
  const bundle = await loadBundle(fetchFile);
  const info = bundle.info;
  element("sequence", HTMLElement).textContent = `${info.sequence} · ${info.dataset} · ${info.mode}`;
  const scrubber = element("scrubber", HTMLInputElement);
  scrubber.max = String(info.frames - 1);
  const frameLabel = element("frame-label", HTMLElement);
  const numbers = element("numbers", HTMLElement);
  const image = new ImageView(element("image", HTMLCanvasElement), element("legend", HTMLElement), element("image-tag", HTMLElement), bundle, fetchFile);
  const scene = new SceneView(element("scene", HTMLElement), bundle);
  const solver = new SolverView(
    element("summary", HTMLElement),
    element("steps", HTMLElement),
    element("placed-by", HTMLElement),
    element("plot-tracked", HTMLCanvasElement),
    element("plot-latency", HTMLCanvasElement),
  );
  const mapCounts = element("map-counts", HTMLElement);

  const present = BACKENDS.filter((name) => name in info.backends);
  const tracks = await Promise.all(present.map((name) => loadTrack(fetchFile, bundle, name)));
  const differ = differences(tracks, info.frames);
  const agreement = element("agreement", HTMLElement);
  if (tracks.length > 1) {
    agreement.textContent =
      differ.length === 0 ? "decisions agree" : `decisions differ at ${differ.length} frames`;
    agreement.title =
      differ.length === 0
        ? "every frame two backends ran, they decided alike: state, tracked points, keyframe, placement"
        : `first at frame ${differ[0]}; each is ticked on the tracked-points plot`;
    agreement.classList.toggle("differ", differ.length > 0);
  }
  /** @type {Map<string, Promise<import("./bundle.js").Backend>>} */
  const loaded = new Map();
  /** @type {import("./bundle.js").Backend | null} */
  let run = null;
  let frame = 0;

  /** Show the frame, for the run chosen. */
  function show() {
    scrubber.value = String(frame);
    frameLabel.textContent = `frame ${frame} · ${bundle.times[frame].toFixed(2)} s`;
    if (run === null) {
      return;
    }
    const k = recordIndex(run.records, frame);
    let note = k < 0 ? `${run.name} did not run this frame` : "";
    let keypoints = null;
    if (k >= 0 && run.features !== null) {
      keypoints = keypointsAt(run.features, frame);
    } else if (k >= 0) {
      note = "no feature records";
    }
    image.show(frame, keypoints, note).catch((error) => fail(error));
    scene.show(frame);
    solver.show(frame, k);
    const state = k < 0 ? "" : ` · ${String(run.records.state[k])}`;
    mapCounts.textContent = `${run.name}${state}`;
  }

  /**
   * Show a backend's run.
   * @param {string} name The backend.
   */
  async function choose(name) {
    let held = loaded.get(name);
    const track = tracks[present.indexOf(name)];
    if (held === undefined) {
      held = loadBackend(fetchFile, track);
      loaded.set(name, held);
    }
    run = await held;
    for (const button of backends.querySelectorAll("button")) {
      button.setAttribute("aria-pressed", String(button.dataset.backend === name));
    }
    showNumbers(numbers, run.info);
    scene.setRun(run);
    scene.setOthers(tracks.filter((other) => other.name !== name));
    solver.setRun(run, tracks, info.frames, differ);
    show();
  }

  const backends = element("backends", HTMLElement);
  for (const name of present) {
    const button = document.createElement("button");
    button.type = "button";
    button.textContent = name;
    button.style.setProperty("--swatch", `var(--backend-${name})`);
    button.dataset.backend = name;
    button.addEventListener("click", () => choose(name).catch((error) => fail(error)));
    backends.append(button);
  }

  // Playback follows the frames' own times, scaled by the speed chosen.
  const play = element("play", HTMLButtonElement);
  const speed = element("speed", HTMLSelectElement);
  let playing = false;
  let clock = 0; // the sequence's time the playhead is at, in seconds since the first frame
  let last = 0;
  /** @param {number} now The animation frame's time, in milliseconds. */
  function tick(now) {
    if (!playing) {
      return;
    }
    clock += ((now - last) / 1000) * Number(speed.value);
    last = now;
    let next = frame;
    while (next + 1 < info.frames && bundle.times[next + 1] <= clock) {
      next += 1;
    }
    if (next !== frame) {
      frame = next;
      show();
    }
    if (frame + 1 >= info.frames) {
      setPlaying(false);
      return;
    }
    requestAnimationFrame(tick);
  }
  /** @param {boolean} on Whether to play. */
  function setPlaying(on) {
    playing = on;
    play.textContent = on ? "❚❚ pause" : "▶ play";
    play.setAttribute("aria-label", on ? "pause" : "play");
    if (on) {
      if (frame + 1 >= info.frames) {
        frame = 0;
      }
      clock = bundle.times[frame];
      last = performance.now();
      requestAnimationFrame(tick);
    }
  }
  /** @param {number} to The frame to show. */
  function seek(to) {
    frame = Math.min(Math.max(to, 0), info.frames - 1);
    clock = bundle.times[frame];
    show();
  }
  play.addEventListener("click", () => setPlaying(!playing));
  element("prev", HTMLButtonElement).addEventListener("click", () => seek(frame - 1));
  element("next", HTMLButtonElement).addEventListener("click", () => seek(frame + 1));
  scrubber.addEventListener("input", () => seek(Number(scrubber.value)));
  document.addEventListener("keydown", (event) => {
    if (event.target instanceof HTMLInputElement && event.target !== scrubber) {
      return;
    }
    if (event.key === " ") {
      event.preventDefault();
      setPlaying(!playing);
    } else if (event.key === "ArrowLeft" || event.key === "ArrowRight") {
      event.preventDefault();
      seek(frame + (event.key === "ArrowLeft" ? -1 : 1));
    }
  });
  const follow = element("follow", HTMLButtonElement);
  follow.addEventListener("click", () => {
    scene.follow = !scene.follow;
    follow.setAttribute("aria-pressed", String(scene.follow));
    show();
  });
  element("fit", HTMLButtonElement).addEventListener("click", () => {
    scene.fit();
    scene.render();
  });

  /** @param {unknown} error What went wrong. */
  function fail(error) {
    message.hidden = false;
    message.textContent = error instanceof Error ? error.message : String(error);
  }
  message.hidden = true;
  if (present.length === 0) {
    fail("the bundle holds no runs");
    return;
  }
  await choose(present[0]);
}

main().catch((error) => {
  const message = document.getElementById("message");
  if (message !== null) {
    message.hidden = false;
    message.textContent = `could not load the bundle: ${error instanceof Error ? error.message : String(error)}`;
  }
});
