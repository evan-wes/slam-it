// The playhead: which frame is shown, and how playback moves it. Playback follows the frames'
// own times, scaled by the speed chosen. No drawing and no clock of its own: the page hands it
// the animation frames' times, and Node tests it.

export class Playhead {
  /**
   * @param {Float32Array} times Each frame's time since the first, in seconds.
   * @param {(frame: number) => void} onFrame Called with the frame whenever it changes, or is
   *     sought.
   */
  constructor(times, onFrame) {
    this.times = times;
    this.onFrame = onFrame;
    /** The frame shown. */
    this.frame = 0;
    /** Whether it is playing. */
    this.playing = false;
    /** The sequence's time the playhead is at, in seconds since the first frame. */
    this.clock = 0;
    /** The last animation frame's time, in milliseconds. */
    this.last = 0;
  }

  /**
   * Show a frame: the playhead moves to it, within the frames.
   * @param {number} to The frame.
   */
  seek(to) {
    this.frame = Math.min(Math.max(to, 0), this.times.length - 1);
    this.clock = this.times[this.frame];
    this.onFrame(this.frame);
  }

  /**
   * Start playing, from the first frame if at the last.
   * @param {number} now The animation frame's time, in milliseconds.
   */
  start(now) {
    if (this.frame + 1 >= this.times.length) {
      this.seek(0);
    }
    this.playing = true;
    this.clock = this.times[this.frame];
    this.last = now;
  }

  /** Stop playing, at the frame shown. */
  stop() {
    this.playing = false;
  }

  /**
   * Move the playhead on to an animation frame's time: to the last frame whose time it has
   * reached. Playing stops at the last frame.
   * @param {number} now The animation frame's time, in milliseconds.
   * @param {number} speed How many seconds of the sequence play in a second.
   * @returns {boolean} Whether it is still playing.
   */
  advance(now, speed) {
    if (!this.playing) {
      return false;
    }
    this.clock += ((now - this.last) / 1000) * speed;
    this.last = now;
    let next = this.frame;
    while (next + 1 < this.times.length && this.times[next + 1] <= this.clock) {
      next += 1;
    }
    if (next !== this.frame) {
      this.frame = next;
      this.onFrame(next);
    }
    if (this.frame + 1 >= this.times.length) {
      this.playing = false;
    }
    return this.playing;
  }
}
