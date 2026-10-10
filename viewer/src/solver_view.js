// The solver panel: plots over the whole run with the frame marked (every backend's latency, and
// the frames where the backends decided differently), the backends' step times at the frame side
// by side, and the frame's stage summaries, each field explained on hover.

import { STEPS, stepsAt } from "./compare.js";
import { duration } from "./format.js";
import { PLACED_HINT, latency, series, summaryAt } from "./solver.js";

/**
 * A CSS custom property's colour.
 * @param {string} name The property.
 * @returns {string} Its value.
 */
function token(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

/** A plot of series over the bundle's frames, with the frame shown marked. */
class Plot {
  /**
   * @param {HTMLCanvasElement} canvas Where it is drawn.
   */
  constructor(canvas) {
    this.canvas = canvas;
    /** @type {{values: Float64Array, first: number, colour: string, dashed?: boolean}[]} */
    this.lines = [];
    /** @type {number[]} */
    this.marks = [];
    this.frames = 1;
    this.top = 1;
  }

  /**
   * Set what is plotted.
   * @param {number} frames The bundle's frames (the x axis).
   * @param {{values: Float64Array, first: number, colour: string, dashed?: boolean}[]} lines
   *     Each line's values, one per frame from `first`.
   * @param {number[]} [marks] Frames to tick along the top.
   * @param {number} [top] The value at the top, past which lines run off; by default the
   *     largest of them all.
   */
  set(frames, lines, marks = [], top = undefined) {
    this.frames = Math.max(frames, 2);
    this.lines = lines;
    this.marks = marks;
    const finite = lines.flatMap((line) => Array.from(line.values).filter(Number.isFinite));
    this.top = (top ?? Math.max(...finite, 1e-9)) * 1.1;
  }

  /**
   * Draw, the frame marked.
   * @param {number} frame The frame shown.
   */
  draw(frame) {
    const ratio = window.devicePixelRatio;
    const width = Math.round(this.canvas.clientWidth * ratio);
    const height = Math.round(this.canvas.clientHeight * ratio);
    if (width === 0 || height === 0) {
      return;
    }
    this.canvas.width = width;
    this.canvas.height = height;
    const context = /** @type {CanvasRenderingContext2D} */ (this.canvas.getContext("2d"));
    /** @param {number} f */
    const x = (f) => (f / (this.frames - 1)) * width;
    /** @param {number} v */
    const y = (v) => height - (v / this.top) * height;
    context.lineWidth = ratio;
    for (const line of this.lines) {
      context.strokeStyle = line.colour;
      context.setLineDash(line.dashed ? [4 * ratio, 4 * ratio] : []);
      context.beginPath();
      let drawing = false;
      line.values.forEach((value, i) => {
        if (!Number.isFinite(value)) {
          drawing = false;
          return;
        }
        const [px, py] = [x(line.first + i), y(value)];
        if (drawing) {
          context.lineTo(px, py);
        } else {
          context.moveTo(px, py);
        }
        drawing = true;
      });
      context.stroke();
    }
    context.setLineDash([]);
    context.strokeStyle = token("--fate-rejected");
    for (const mark of this.marks) {
      context.beginPath();
      context.moveTo(x(mark), 0);
      context.lineTo(x(mark), 6 * ratio);
      context.stroke();
    }
    context.strokeStyle = token("--camera");
    context.beginPath();
    context.moveTo(x(frame), 0);
    context.lineTo(x(frame), height);
    context.stroke();
  }
}

export class SolverView {
  /**
   * @param {HTMLElement} summary Where the frame's summary goes.
   * @param {HTMLElement} steps Where the backends' step times go.
   * @param {HTMLElement} placed Where how the frame was placed goes.
   * @param {HTMLCanvasElement} tracked The tracked points' plot.
   * @param {HTMLCanvasElement} timing The latency's plot.
   */
  constructor(summary, steps, placed, tracked, timing) {
    this.summary = summary;
    this.steps = steps;
    /** @type {import("./bundle.js").Track[]} */
    this.tracks = [];
    this.placed = placed;
    this.placed.title = PLACED_HINT;
    this.tracked = new Plot(tracked);
    this.timing = new Plot(timing);
    /** @type {import("./bundle.js").Backend | null} */
    this.run = null;
    this.frame = 0;
    new ResizeObserver(() => this.redraw()).observe(tracked);
  }

  /**
   * Show a backend's run: its plots, built once, beside every backend's latency.
   * @param {import("./bundle.js").Backend} run The run.
   * @param {import("./bundle.js").Track[]} tracks Every backend's track, the run's among them.
   * @param {number} frames The bundle's frames.
   * @param {number[]} differ The frames where the backends decided differently.
   */
  setRun(run, tracks, frames, differ) {
    this.run = run;
    this.tracks = tracks;
    const first = run.info.frames[0];
    this.tracked.set(
      frames,
      [
        { values: series(run.records, "tracked"), first, colour: token("--fate-tracked") },
        { values: series(run.records, "local_map.matches"), first, colour: token("--muted"), dashed: true },
      ],
      differ,
    );
    const deadline = run.info.latency?.deadline ?? NaN;
    const own = Array.from(latency(run.records)).filter(Number.isFinite);
    // Scaled to the chosen run and the deadline: a slower backend runs off the top.
    const top = Math.max(...own, Number.isFinite(deadline) ? deadline * 1.5 : 0, 1e-9);
    this.timing.set(
      frames,
      [
        ...tracks.map((track) => ({
          values: latency(track.records),
          first: track.info.frames[0],
          colour: token(`--backend-${track.name}`),
        })),
        { values: new Float64Array(frames).fill(deadline), first: 0, colour: token("--fate-rejected"), dashed: true },
      ],
      [],
      top,
    );
  }

  /**
   * Set the backends' step times at a frame side by side.
   * @param {number} frame The bundle's frame.
   */
  showSteps(frame) {
    const table = document.createElement("table");
    const head = table.insertRow();
    head.insertCell().textContent = "step";
    for (const track of this.tracks) {
      const cell = head.insertCell();
      cell.textContent = track.name;
      cell.style.color = `var(--backend-${track.name})`;
    }
    const times = this.tracks.map((track) => stepsAt(track, frame));
    STEPS.forEach((step, i) => {
      const row = table.insertRow();
      row.insertCell().textContent = step.replace("_seconds", "");
      for (const steps of times) {
        row.insertCell().textContent = steps === null ? "–" : duration(steps[i]);
      }
    });
    this.steps.replaceChildren(table);
  }

  /**
   * Show a frame's summary, and mark it on the plots.
   * @param {number} frame The bundle's frame.
   * @param {number} k The run's record of it, or -1 if the run did not run it.
   */
  show(frame, k) {
    this.frame = frame;
    this.showSteps(frame);
    this.summary.replaceChildren();
    if (this.run === null || k < 0) {
      this.placed.textContent = "";
      const none = document.createElement("p");
      none.className = "none";
      none.textContent = "this run did not run the frame";
      this.summary.append(none);
    } else {
      const { placedBy, stages } = summaryAt(this.run.records, k);
      this.placed.textContent = `placed by ${placedBy}`;
      const table = document.createElement("table");
      for (const { title, rows } of stages) {
        const head = table.insertRow();
        const cell = document.createElement("th");
        cell.colSpan = 2;
        cell.textContent = title;
        head.append(cell);
        for (const { field, value, hint } of rows) {
          const row = table.insertRow();
          const name = row.insertCell();
          name.textContent = field;
          name.title = hint;
          row.insertCell().textContent = value;
        }
      }
      this.summary.append(table);
    }
    this.redraw();
  }

  /** Draw the plots again, at their current size. */
  redraw() {
    this.tracked.draw(this.frame);
    this.timing.draw(this.frame);
  }
}
