// The solver panel: plots over the whole run with the frame marked (every backend's latency
// against the labelled deadline, and the frames where the backends decided differently), each
// with a value axis, the backends' step times at the frame side by side, and the frame's stage
// summaries, each field explained on hover. Its tables are built once a run, with every row the
// run has, and a frame shown changes only their text: the panel holds its height while the
// frames play.

import { ticks } from "./axis.js";
import { STEPS, stepsAt } from "./compare.js";
import * as format from "./format.js";
import { PLACED_HINT, latency, series, summaryAt, summaryLayout } from "./solver.js";

/** The plots' text, in CSS pixels. */
const TEXT = 10;

/** The space beside the plots' text, in CSS pixels. */
const PAD = 4;

/** The ticks above 0 a plot's axis aims for: about five fit its 90 px, a label each. */
const TICKS = 5;

/** The widest tick label the gutter holds: both plots' gutters alike, so their frames line up. */
const WIDEST = "00,000";

/** The tracked points' axis: counts, so no step below one. */
const POINTS_AXIS = { unit: "points", scale: 1, least: 1 };

/** Latency's axis: seconds plotted, milliseconds labelled, no step below a tenth of one (the
 *  step times' precision, `format.duration`). */
const LATENCY_AXIS = { unit: "ms", scale: 1000, least: 0.1 };

/**
 * A plot's value axis: its unit, the factor from the values plotted to it, and the finest step
 * labelled, in it.
 * @typedef {{unit: string, scale: number, least: number}} Axis
 */

/**
 * A plotted line: its values, one per frame from `first`, its colour, whether it is dashed, and
 * a label written over its right end.
 * @typedef {{values: Float64Array, first: number, colour: string, dashed?: boolean,
 *     label?: string}} Line
 */

/**
 * A CSS custom property's colour.
 * @param {string} name The property.
 * @returns {string} Its value.
 */
function token(name) {
  return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
}

/**
 * Set an element's text where it changed: a frame shown rewrites only what moved.
 * @param {HTMLElement} element The element.
 * @param {string} text Its text.
 */
function setText(element, text) {
  if (element.textContent !== text) {
    element.textContent = text;
  }
}

/**
 * The largest of some values, ignoring those not finite.
 * @param {Iterable<number>} values The values.
 * @param {number} floor The least it returns.
 * @returns {number} The largest finite value, or `floor` if larger.
 */
function largest(values, floor) {
  let top = floor;
  for (const value of values) {
    if (Number.isFinite(value) && value > top) {
      top = value;
    }
  }
  return top;
}

/** A plot of series over the bundle's frames, with the frame shown marked, and a value axis in a
 *  gutter on its left: its unit, and ticks from 0, each labelled, with faint lines across. */
class Plot {
  /**
   * @param {HTMLCanvasElement} canvas Where it is drawn.
   * @param {Axis} axis Its value axis.
   */
  constructor(canvas, axis) {
    this.canvas = canvas;
    this.axis = axis;
    /** @type {Line[]} */
    this.lines = [];
    /** @type {number[]} */
    this.marks = [];
    this.frames = 1;
    this.top = 1;
  }

  /**
   * Set what is plotted.
   * @param {number} frames The bundle's frames (the x axis).
   * @param {Line[]} lines The lines.
   * @param {number[]} [marks] Frames to tick along the top.
   * @param {number} [top] The value at the top, past which lines run off; by default the
   *     largest of them all.
   */
  set(frames, lines, marks = [], top = undefined) {
    this.frames = Math.max(frames, 2);
    this.lines = lines;
    this.marks = marks;
    this.top = (top ?? lines.reduce((most, line) => largest(line.values, most), 1e-9)) * 1.1;
  }

  /**
   * Draw, the frame marked, at the canvas's size.
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
    const [text, pad] = [TEXT * ratio, PAD * ratio];
    context.font = `${text}px ${token("--mono")}`;
    // The gutter: the unit turned up its left edge, then the ticks' labels.
    const gutter = Math.ceil(pad + text + pad + context.measureText(WIDEST).width + pad);
    /** @param {number} f */
    const x = (f) => gutter + (f / (this.frames - 1)) * (width - gutter);
    /** @param {number} v */
    const y = (v) => height - (v / this.top) * height;
    const axis = ticks(this.top * this.axis.scale, TICKS, this.axis.least);
    const levels = axis.values.map((value) => y(value / this.axis.scale));
    context.lineWidth = ratio;
    context.strokeStyle = token("--line");
    context.beginPath();
    for (const level of levels) {
      context.moveTo(gutter, level);
      context.lineTo(width, level);
    }
    context.moveTo(gutter, 0);
    context.lineTo(gutter, height);
    context.stroke();
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
    context.fillStyle = token("--muted");
    context.textAlign = "right";
    context.textBaseline = "middle";
    axis.labels.forEach((label, i) => {
      // Each beside its line, the top's and 0's kept inside the canvas.
      const level = Math.min(Math.max(levels[i], text / 2), height - text / 2);
      context.fillText(label, gutter - pad, level);
    });
    context.save();
    context.translate(pad + text / 2, height / 2);
    context.rotate(-Math.PI / 2);
    context.textAlign = "center";
    context.fillText(this.axis.unit, 0, 0);
    context.restore();
    for (const line of this.lines) {
      this.label(context, line, y, width - pad, text);
    }
  }

  /**
   * Write a line's label, if it has one, over its right end: just above it, or just below where
   * the top would cut it, on a backing of the plot's ground so lines under it do not blur it.
   * @param {CanvasRenderingContext2D} context The plot's context, its font set.
   * @param {Line} line The line.
   * @param {(v: number) => number} y A value's height on the canvas.
   * @param {number} right Where the label ends, on the canvas.
   * @param {number} text The text's size, on the canvas.
   */
  label(context, line, y, right, text) {
    if (line.label === undefined) {
      return;
    }
    let last = NaN;
    for (let i = line.values.length - 1; i >= 0 && !Number.isFinite(last); i--) {
      last = line.values[i];
    }
    if (!Number.isFinite(last)) {
      return;
    }
    // The label's box: a line of text and a gap around it, clear of the line by another gap.
    const gap = text / 4;
    const tall = text + 2 * gap;
    const top = y(last) - gap - tall >= 0 ? y(last) - gap - tall : y(last) + gap;
    const width = context.measureText(line.label).width;
    context.globalAlpha = 0.85;
    context.fillStyle = token("--ground");
    context.fillRect(right - width - gap, top, width + 2 * gap, tall);
    context.globalAlpha = 1;
    context.fillStyle = line.colour;
    context.textAlign = "right";
    context.textBaseline = "middle";
    context.fillText(line.label, right, top + tall / 2);
  }
}

/** How the solvers went: the tracked points' and latency's plots over the run, every backend's
 *  step times at the frame, and the frame's stage summaries. */
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
    this.tracked = new Plot(tracked, POINTS_AXIS);
    this.timing = new Plot(timing, LATENCY_AXIS);
    /** @type {import("./bundle.js").Track | null} */
    this.run = null;
    this.frame = 0;
    /** @type {HTMLTableCellElement[][]} Each step's cells, a backend's each. */
    this.stepCells = [];
    /** @type {import("./solver.js").StageLayout[]} The run's summary rows. */
    this.layout = [];
    /** @type {HTMLTableCellElement[]} Each stage's heading, in the layout's order. */
    this.stageHeads = [];
    /** @type {HTMLTableCellElement[][]} Each stage's value cells, a field's each. */
    this.valueCells = [];
    new ResizeObserver(() => this.redraw()).observe(tracked);
  }

  /**
   * Show a backend's run: its plots, built once, beside every backend's latency. Its track is
   * all they need.
   * @param {import("./bundle.js").Track} run The run's track.
   * @param {import("./bundle.js").Track[]} tracks Every backend's track, the run's among them.
   * @param {number} frames The bundle's frames.
   * @param {number[]} differ The frames where the backends decided differently.
   */
  setRun(run, tracks, frames, differ) {
    this.run = run;
    this.tracks = tracks;
    this.layout = summaryLayout(run.records);
    this.buildSteps();
    this.buildSummary();
    const first = run.info.frames[0];
    this.tracked.set(
      frames,
      [
        { values: series(run.records, "tracked"), first, colour: token("--fate-tracked") },
        {
          values: series(run.records, "local_map.matches"),
          first,
          colour: token("--muted"),
          dashed: true,
        },
      ],
      differ,
    );
    const deadline = run.info.latency?.deadline ?? NaN;
    // Scaled to the chosen run and the deadline: a slower backend runs off the top.
    const floor = Math.max(Number.isFinite(deadline) ? deadline * 1.5 : 0, 1e-9);
    const top = largest(latency(run.records), floor);
    this.timing.set(
      frames,
      [
        ...tracks.map((track) => ({
          values: latency(track.records),
          first: track.info.frames[0],
          colour: token(`--backend-${track.name}`),
        })),
        {
          values: new Float64Array(frames).fill(deadline),
          first: 0,
          colour: token("--fate-rejected"),
          dashed: true,
          label: Number.isFinite(deadline) ? format.deadline(deadline) : undefined,
        },
      ],
      [],
      top,
    );
  }

  /** Build the backends' step times' table, once a run: a row a step, a column a backend. */
  buildSteps() {
    const table = document.createElement("table");
    const head = table.insertRow();
    head.insertCell().textContent = "step";
    for (const track of this.tracks) {
      const cell = head.insertCell();
      cell.textContent = track.name;
      cell.style.color = `var(--backend-${track.name})`;
    }
    this.stepCells = STEPS.map((step) => {
      const row = table.insertRow();
      row.insertCell().textContent = step.replace("_seconds", "");
      return this.tracks.map(() => row.insertCell());
    });
    this.steps.replaceChildren(table);
  }

  /** Build the summary's table, once a run: every stage the run summarises, a row a field, each
   *  explained on hover. */
  buildSummary() {
    const table = document.createElement("table");
    const columns = document.createElement("colgroup");
    const names = document.createElement("col");
    names.className = "name";
    columns.append(names, document.createElement("col"));
    table.append(columns);
    this.stageHeads = [];
    this.valueCells = this.layout.map(({ title, fields }) => {
      const head = document.createElement("th");
      head.colSpan = 2;
      head.textContent = title;
      table.insertRow().append(head);
      this.stageHeads.push(head);
      return fields.map(({ field, hint }) => {
        const row = table.insertRow();
        const name = row.insertCell();
        name.textContent = field;
        name.title = hint;
        return row.insertCell();
      });
    });
    this.summary.replaceChildren(table);
  }

  /**
   * Set the backends' step times at a frame side by side.
   * @param {number} frame The bundle's frame.
   */
  showSteps(frame) {
    this.tracks.forEach((track, column) => {
      const steps = stepsAt(track, frame);
      this.stepCells.forEach((cells, i) => {
        setText(cells[column], steps === null ? "–" : format.duration(steps[i]));
      });
    });
  }

  /**
   * Set the run's summary at a frame: each stage's values, "–" where it did not run, its
   * heading faded.
   * @param {number} k The run's record of the frame, or -1 if the run did not run it.
   */
  showSummary(k) {
    if (this.run === null) {
      return;
    }
    const { placedBy, stages } = summaryAt(this.run.records, k, this.layout);
    setText(this.placed, k < 0 ? "this run did not run the frame" : `placed by ${placedBy}`);
    stages.forEach(({ ran, values }, s) => {
      this.stageHeads[s].classList.toggle("idle", !ran);
      values.forEach((value, f) => {
        const cell = this.valueCells[s][f];
        setText(cell, value);
        cell.title = value === "–" ? "" : value; // a value cut to its line, whole on hover
      });
    });
  }

  /**
   * Show a frame's summary, and mark it on the plots.
   * @param {number} frame The bundle's frame.
   * @param {number} k The run's record of it, or -1 if the run did not run it.
   */
  show(frame, k) {
    this.frame = frame;
    this.showSteps(frame);
    this.showSummary(k);
    this.redraw();
  }

  /** Draw the plots again, at their current size. */
  redraw() {
    this.tracked.draw(this.frame);
    this.timing.draw(this.frame);
  }
}
