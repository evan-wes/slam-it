// The run viewer's page: loads a bundle (the URL's `bundle`, else `bundle/` beside the page),
// and shows one frame at a time in the image, map and solver panels, for the backend chosen,
// beside the other backends' trajectories and step times, the frames where they decided
// differently marked. This module only wires the views and controls to the page.

import { BACKENDS, loadBackend, loadBundle, loadTrack } from "./bundle.js";
import { differences, ran, shownAt, spanText } from "./compare.js";
import * as format from "./format.js";
import { keypointsAt, recordIndex } from "./frames.js";
import { ImageView } from "./image_view.js";
import { element } from "./page_view.js";
import { Playhead } from "./playback.js";
import { SceneView } from "./scene_view.js";
import { SolverView } from "./solver_view.js";

/** The height of each backend's row under the frame slider, in pixels. */
const SPAN_ROW = 4;

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
 * @param {boolean} truth Whether the bundle has ground truth.
 */
function showNumbers(numbers, info, truth) {
  /** @type {[string, string, string][]} */
  const items = [];
  if (info.score !== null) {
    items.push([
      "ATE",
      format.length(info.score.ate_rmse),
      "absolute trajectory error, RMS, after alignment",
    ]);
    items.push([
      "RPE",
      `${format.length(info.score.rpe_translation_rmse)} · ` +
        format.angle(info.score.rpe_rotation_rmse),
      "relative pose error over one frame period, RMS: translation and rotation",
    ]);
  } else if (truth) {
    items.push([
      "ATE",
      "not aligned",
      "the run could not be aligned to the ground truth (too few frames tracked, or their " +
        "centres on a line), so it is shown in its own frame and not scored",
    ]);
  }
  const ran = info.frames[1] - info.frames[0];
  items.push(["tracked", `${info.tracked} / ${ran}`, "frames tracked, of the frames run"]);
  items.push([
    "keyframes",
    String(info.keyframes),
    `in the final map (${info.made.keyframes} made)`,
  ]);
  items.push(["points", String(info.points), `in the final map (${info.made.points} made)`]);
  if (info.latency !== null) {
    const { median, p95, deadline, late } = info.latency;
    items.push([
      "latency",
      `${format.duration(median)} · p95 ${format.duration(p95)} / ${format.duration(deadline)}`,
      "building and tracking a frame, median and 95th percentile, against the frame period; " +
        `${late} late`,
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

/**
 * Say whether the backends decided alike.
 * @param {HTMLElement} agreement The header's note.
 * @param {number} backends How many backends the bundle holds.
 * @param {number[]} differ The frames where they decided differently.
 */
function showAgreement(agreement, backends, differ) {
  if (backends < 2) {
    return;
  }
  agreement.textContent =
    differ.length === 0 ? "decisions agree" : `decisions differ at ${differ.length} frames`;
  agreement.title =
    differ.length === 0
      ? "every frame two backends ran, they decided alike: state, tracked points, keyframe, " +
        "placement"
      : `first at frame ${differ[0]}; each is ticked on the tracked-points plot`;
  agreement.classList.toggle("differ", differ.length > 0);
}

/**
 * Draw each backend's span under the frame slider: a bar in its colour over the frames it ran.
 * @param {HTMLElement} box The spans' box.
 * @param {import("./bundle.js").Track[]} tracks The backends' tracks.
 * @param {number} frames The bundle's frames.
 */
function showSpans(box, tracks, frames) {
  box.replaceChildren(
    ...tracks.map((track, row) => {
      const [first, end] = track.info.frames;
      const bar = document.createElement("div");
      bar.className = "span";
      bar.title = spanText(track);
      bar.style.left = `${(100 * first) / frames}%`;
      bar.style.width = `${(100 * (end - first)) / frames}%`;
      bar.style.top = `${row * SPAN_ROW}px`;
      bar.style.background = `var(--backend-${track.name})`;
      return bar;
    }),
  );
  box.style.height = `${tracks.length * SPAN_ROW}px`;
}

/**
 * Bind the playback controls and keys to the playhead.
 * @param {Playhead} playhead The playhead.
 * @param {HTMLInputElement} scrubber The frame slider.
 */
function bindPlayback(playhead, scrubber) {
  const play = element("play", HTMLButtonElement);
  const speed = element("speed", HTMLSelectElement);
  let pending = false; // an animation frame is requested
  /** @param {boolean} on Whether the button shows playing. */
  function showPlaying(on) {
    play.textContent = on ? "❚❚ pause" : "▶ play";
    play.setAttribute("aria-label", on ? "pause" : "play");
  }
  /** @param {number} now The animation frame's time, in milliseconds. */
  function tick(now) {
    pending = false;
    if (playhead.advance(now, Number(speed.value))) {
      schedule();
    } else {
      showPlaying(false);
    }
  }
  /** Request the next animation frame, unless one is already requested. */
  function schedule() {
    if (!pending) {
      pending = true;
      requestAnimationFrame(tick);
    }
  }
  /** @param {boolean} on Whether to play. */
  function setPlaying(on) {
    if (on) {
      playhead.start(performance.now());
      schedule();
    } else {
      playhead.stop();
    }
    showPlaying(on);
  }
  play.addEventListener("click", () => setPlaying(!playhead.playing));
  element("prev", HTMLButtonElement).addEventListener("click", () => {
    playhead.seek(playhead.frame - 1);
  });
  element("next", HTMLButtonElement).addEventListener("click", () => {
    playhead.seek(playhead.frame + 1);
  });
  scrubber.addEventListener("input", () => playhead.seek(Number(scrubber.value)));
  document.addEventListener("keydown", (event) => {
    if (event.target instanceof HTMLInputElement && event.target !== scrubber) {
      return;
    }
    if (event.key === " ") {
      event.preventDefault();
      setPlaying(!playhead.playing);
    } else if (event.key === "ArrowLeft" || event.key === "ArrowRight") {
      event.preventDefault();
      playhead.seek(playhead.frame + (event.key === "ArrowLeft" ? -1 : 1));
    }
  });
}

/**
 * Load the bundle and run the page: its views, the backends' buttons and playback.
 * @returns {Promise<void>} Once the first backend is shown.
 */
async function main() {
  const message = element("message", HTMLParagraphElement);
  const given = new URLSearchParams(location.search).get("bundle") ?? "bundle/";
  const base = new URL(given, location.href);
  const fetchFile = fetcher(base.href.endsWith("/") ? base.href : `${base.href}/`);
  const bundle = await loadBundle(fetchFile);
  const info = bundle.info;
  element("sequence", HTMLElement).textContent =
    `${info.sequence} · ${info.dataset} · ${info.mode}`;
  const scrubber = element("scrubber", HTMLInputElement);
  scrubber.max = String(info.frames - 1);
  const frameLabel = element("frame-label", HTMLElement);
  const numbers = element("numbers", HTMLElement);
  const image = new ImageView(
    element("image", HTMLCanvasElement),
    element("legend", HTMLElement),
    element("image-tag", HTMLElement),
    bundle,
    fetchFile,
  );
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
  showAgreement(element("agreement", HTMLElement), tracks.length, differ);
  showSpans(element("spans", HTMLElement), tracks, info.frames);
  /**
   * @param {string | undefined} name A backend.
   * @returns {import("./bundle.js").Track} Its track.
   */
  const trackOf = (name) => tracks[present.indexOf(name ?? "")];
  /** @type {Map<string, Promise<import("./bundle.js").Backend>>} */
  const loaded = new Map();
  /** @type {import("./bundle.js").Backend | null} */
  let run = null;
  let chosen = ""; // the backend last asked for, which a slower earlier load must not replace
  const playhead = new Playhead(bundle.times, show);

  /** Show the playhead's frame, for the run chosen. */
  function show() {
    const frame = playhead.frame;
    scrubber.value = String(frame);
    frameLabel.textContent = `frame ${frame} · ${bundle.times[frame].toFixed(2)} s`;
    for (const button of backends.querySelectorAll("button")) {
      button.disabled = !ran(trackOf(button.dataset.backend), frame);
    }
    if (run === null) {
      return;
    }
    // A backend that did not run this frame gives way to the first that did, once loaded.
    const shown = shownAt(tracks, run.name, frame);
    if (shown !== run.name) {
      if (chosen !== shown) {
        choose(shown).catch((error) => fail(error));
      }
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
    chosen = name;
    let held = loaded.get(name);
    const track = tracks[present.indexOf(name)];
    if (held === undefined) {
      held = loadBackend(fetchFile, track);
      loaded.set(name, held);
    }
    const ready = await held;
    if (chosen !== name) {
      return;
    }
    run = ready;
    for (const button of backends.querySelectorAll("button")) {
      button.setAttribute("aria-pressed", String(button.dataset.backend === name));
    }
    showNumbers(numbers, run.info, info.truth);
    scene.setRun(run);
    // Only runs in the same frame as this one: aligned to the truth, or each in its own.
    const aligned = run.info.alignment !== null;
    scene.setOthers(
      tracks.filter((o) => o.name !== name && (o.info.alignment !== null) === aligned),
    );
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
    button.title = spanText(trackOf(name));
    button.addEventListener("click", () => choose(name).catch((error) => fail(error)));
    backends.append(button);
  }
  bindPlayback(playhead, scrubber);
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
    const what = error instanceof Error ? error.message : String(error);
    message.textContent = `could not load the bundle: ${what}`;
  }
});
