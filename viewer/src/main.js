// The run viewer's page: loads a bundle (the URL's `bundle` on this site, else `bundle/` beside it),
// and shows one frame at a time in the image, map and solver panels, for the backend chosen,
// beside the other backends' trajectories and step times, the frames where they decided
// differently marked. This module only wires the views and controls to the page.

import { BACKENDS, bundleAddress, loadBundle, loadFeatures, loadMap, loadTrack } from "./bundle.js";
import { differences, ran, shownAt, spanText } from "./compare.js";
import * as format from "./format.js";
import { keypointsAt, recordIndex } from "./frames.js";
import { ImageView } from "./image_view.js";
import { LoopView } from "./loop_view.js";
import { element, remember, remembered } from "./page_view.js";
import { Playhead } from "./playback.js";
import { SceneView } from "./scene_view.js";
import { OPACITY_RANGE, SIZE_RANGE, opacityPercent, sizeFactor, sizeStep } from "./settings.js";
import { SolverView } from "./solver_view.js";
import {
  LOST,
  NOT_INITIALIZED,
  foundFrames,
  foundHint,
  notRunText,
  placement,
  spanHint,
  stateSpans,
} from "./timeline.js";

/** The height of each backend's row under the frame slider, in pixels. */
const SPAN_ROW = 4;

/** Where the browser remembers the keyframe size slider's step. */
const SIZE_KEY = "slam-it.keyframe-size";

/** Where the browser remembers the point opacity slider's percentage. */
const OPACITY_KEY = "slam-it.point-opacity";

/**
 * One of the 3D panel's sliders: its input's id (its value shown in `<id>-value`), where the
 * browser remembers it, its range, its value from text, the text it is shown as, and what it
 * sets.
 * @typedef {object} Slider
 * @property {string} id The input's id.
 * @property {string} key Its storage key.
 * @property {import("./settings.js").Range} range Its least and greatest values, and its step.
 * @property {(text: string | null) => number} read Its value from text (`settings.js`).
 * @property {(value: number) => string} text Its value as shown.
 * @property {(value: number) => void} apply What a value sets.
 */

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
  const several = info.maps.length > 1 ? ", each map aligned on its own" : "";
  if (info.score !== null) {
    items.push([
      "ATE",
      format.length(info.score.ate_rmse),
      `absolute trajectory error, RMS, after alignment${several}`,
    ]);
    const { rpe_translation_rmse: drift, rpe_rotation_rmse: turn } = info.score;
    items.push(
      drift === null || turn === null
        ? ["RPE", "–", "no two matched frames a frame period apart, to measure it over"]
        : [
            "RPE",
            `${format.length(drift)} · ${format.angle(turn)}`,
            "relative pose error over one frame period, RMS: translation and rotation",
          ],
    );
  } else if (truth) {
    items.push([
      "ATE",
      "not aligned",
      "no map of the run could be aligned to the ground truth (too few frames tracked, or " +
        "their centres on a line), so each is shown in its own frame and not scored",
    ]);
  }
  const reset = info.maps.filter((map) => map.reset).length;
  if (info.maps.length > 1 || reset > 0) {
    items.push([
      "maps",
      reset > 0 ? `${info.maps.length} · ${reset} reset` : String(info.maps.length),
      "a lost track closes its map, and the next frame starts a new one; a map lost with few " +
        "keyframes is reset: its keyframes and points removed, its frames' poses dropped",
    ]);
  }
  const ran = info.frames[1] - info.frames[0];
  items.push(["tracked", `${info.tracked} / ${ran}`, "frames tracked, of the frames run"]);
  items.push([
    "keyframes",
    String(info.keyframes),
    `in the final maps (${info.made.keyframes} made)`,
  ]);
  items.push(["points", String(info.points), `in the final maps (${info.made.points} made)`]);
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
      ? "every frame two backends ran, they decided alike: state, map, tracked points, " +
        "keyframe, placement"
      : `first at frame ${differ[0]}; each is ticked on the tracked-points plot`;
  agreement.classList.toggle("differ", differ.length > 0);
}

/**
 * Draw the timeline: under the frame slider, each backend's frames, a row each, coloured by
 * state, each frame found again marked on it; beside it, the parts of the sequence no run
 * reached, each part as wide as its time.
 * @param {import("./bundle.js").Bundle} bundle The bundle.
 * @param {import("./bundle.js").Track[]} tracks The backends' tracks.
 */
function showTimeline(bundle, tracks) {
  const frames = bundle.info.frames;
  const box = element("spans", HTMLElement);
  box.replaceChildren(
    ...tracks.flatMap((track, row) =>
      stateSpans(track.records, track.info.maps).map((span) => {
        const bar = document.createElement("div");
        bar.className = "span";
        bar.classList.toggle("lost", span.state === LOST);
        bar.classList.toggle("starting", span.state === NOT_INITIALIZED);
        bar.classList.toggle("reset", span.reset && span.state !== LOST);
        bar.title = spanHint(track.name, span);
        bar.style.left = `${(100 * span.first) / frames}%`;
        bar.style.width = `${(100 * (span.end - span.first)) / frames}%`;
        bar.style.top = `${row * SPAN_ROW}px`;
        if (span.state !== LOST) {
          bar.style.background = `var(--backend-${track.name})`;
        }
        return bar;
      }),
    ),
    ...tracks.flatMap((track, row) =>
      foundFrames(track.records).map((found) => {
        const mark = document.createElement("div");
        mark.className = "span found";
        mark.title = foundHint(track.name, found);
        mark.style.left = `${(100 * found.frame) / frames}%`;
        mark.style.width = `${100 / frames}%`;
        mark.style.top = `${row * SPAN_ROW}px`;
        return mark;
      }),
    ),
  );
  box.style.height = `${tracks.length * SPAN_ROW}px`;
  const placed = placement(bundle.info.extent, bundle.times);
  const said = notRunText(placed);
  const whole = placed.before + placed.ran + placed.after;
  // Each part grows by its share of the whole, the shares summing to 1 so they fill the row.
  element("ran", HTMLElement).style.flexGrow = String(whole > 0 ? placed.ran / whole : 1);
  /** @type {[string, number, [string, string]][]} */
  const parts = [
    ["not-run-before", placed.before, said.before],
    ["not-run-after", placed.after, said.after],
  ];
  for (const [id, seconds, [label, hint]] of parts) {
    const part = element(id, HTMLElement);
    part.hidden = seconds === 0;
    part.style.flexGrow = String(whole > 0 ? seconds / whole : 0);
    part.textContent = label;
    part.title = hint;
  }
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
 * Bind one of the 3D panel's sliders: its range, its value shown beside it and remembered in
 * this browser where it can be (else its default), and what moving it sets.
 * @param {Slider} slider The slider.
 * @param {() => void} redraw Draw the 3D view again at the frame shown.
 */
function bindSlider({ id, key, range, read, text, apply }, redraw) {
  const input = element(id, HTMLInputElement);
  const shown = element(`${id}-value`, HTMLOutputElement);
  [input.min, input.max, input.step] = range.map(String);
  /** @param {number} value The slider's value. */
  function set(value) {
    shown.textContent = text(value);
    apply(value);
  }
  const value = read(remembered(key));
  input.value = String(value);
  set(value);
  input.addEventListener("input", () => {
    const value = read(input.value);
    set(value);
    remember(key, String(value));
    redraw();
  });
}

/**
 * Load the bundle and run the page: its views, the backends' buttons and playback.
 * @returns {Promise<void>} Once the first backend is shown.
 */
async function main() {
  const message = element("message", HTMLParagraphElement);
  const given = new URLSearchParams(location.search).get("bundle");
  const fetchFile = fetcher(bundleAddress(given, location.href));
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
  showTimeline(bundle, tracks);
  /**
   * @param {string | undefined} name A backend.
   * @returns {import("./bundle.js").Track} Its track.
   */
  const trackOf = (name) => tracks[present.indexOf(name ?? "")];
  /**
   * A backend's whole run: its track, map and features.
   * @typedef {import("./bundle.js").Backend & {features: import("./bundle.js").Features | null}}
   *     Run
   */
  /** @type {Map<string, Promise<Run>>} */
  const loaded = new Map();
  /** @type {Run | null} */
  let run = null;
  let chosen = ""; // the backend last asked for, which a slower earlier load must not replace
  const playhead = new Playhead(bundle.times, (frame, played) => {
    loops.moved(frame, played); // before the frame is shown: arriving at a loop sets off its effect
    show();
  });
  const loops = new LoopView(
    {
      spans: element("spans", HTMLElement),
      top: tracks.length * SPAN_ROW,
      previous: element("prev-loop", HTMLButtonElement),
      next: element("next-loop", HTMLButtonElement),
      toggle: element("seen-before", HTMLButtonElement),
      inset: element("inset", HTMLButtonElement),
      picture: element("inset-image", HTMLCanvasElement),
      label: element("inset-label", HTMLElement),
    },
    bundle,
    fetchFile,
    scene,
    (frame) => playhead.seek(frame),
    (error) => fail(error),
  );

  /** Show the playhead's frame, for the run chosen. */
  function show() {
    const frame = playhead.frame;
    scrubber.value = String(frame);
    frameLabel.textContent = `frame ${frame} · ${bundle.times[frame].toFixed(2)} s`;
    for (const button of backends.querySelectorAll("button")) {
      button.disabled = !ran(trackOf(button.dataset.backend), frame);
    }
    loops.show(frame).catch((error) => fail(error));
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
    const map = scene.show(frame);
    solver.show(frame, k);
    const state = k < 0 ? "" : ` · ${String(run.records.state[k])}`;
    mapCounts.textContent = `${run.name}${state}${map === null ? "" : ` · map ${map}`}`;
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
      held = Promise.all([loadMap(fetchFile, track), loadFeatures(fetchFile, track)]).then(
        ([map, features]) => ({ ...track, ...map, features }),
      );
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
    // Each shown beside the run's map where its map shares that map's frame (SceneView.show).
    scene.setOthers(tracks.filter((o) => o.name !== name));
    solver.setRun(run, tracks, info.frames, differ);
    try {
      loops.setRun(run);
    } catch (error) {
      fail(error); // the run is shown without its loops
    }
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
  const wholeTruth = element("whole-truth", HTMLButtonElement);
  wholeTruth.hidden = !info.truth;
  wholeTruth.addEventListener("click", () => {
    scene.wholeTruth = !scene.wholeTruth;
    wholeTruth.setAttribute("aria-pressed", String(scene.wholeTruth));
    show();
  });
  const graph = element("graph", HTMLButtonElement);
  graph.addEventListener("click", () => {
    scene.showGraph(!scene.graph);
    graph.setAttribute("aria-pressed", String(scene.graph));
  });
  element("fit", HTMLButtonElement).addEventListener("click", () => {
    scene.fit();
    scene.render();
  });
  /** Draw the 3D view again, alone, at the frame shown. */
  const redrawScene = () => scene.show(playhead.frame);
  bindSlider(
    {
      id: "keyframe-size",
      key: SIZE_KEY,
      range: SIZE_RANGE,
      read: sizeStep,
      text: (step) => format.factor(sizeFactor(step)),
      apply: (step) => scene.setKeyframeSize(sizeFactor(step)),
    },
    redrawScene,
  );
  bindSlider(
    {
      id: "point-opacity",
      key: OPACITY_KEY,
      range: OPACITY_RANGE,
      read: opacityPercent,
      text: (percent) => format.percent(percent / 100),
      apply: (percent) => scene.setPointOpacity(percent / 100),
    },
    redrawScene,
  );

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
